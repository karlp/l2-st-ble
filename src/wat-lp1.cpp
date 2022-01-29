/**
 * Some manual testing files to work out low power modes :|
 */
#include <cstdio>

#include <cortex_m/debug.h>
#include <exti/exti.h>
#include <gpio/gpio.h>
#include <interrupt/interrupt.h>
#include <pwr/pwr.h>
#include <rcc/flash.h>
#include <rcc/rcc.h>
#include <rtc/rtc.h>
#include <syscfg/syscfg.h>
#include <wpan/hsem.h>
#include <wpan/ipcc.h>


auto led_r = GPIOB[1];
auto led_g = GPIOB[0];
auto led_b = GPIOB[5];
auto sw_1 = GPIOC[4];
auto sw_2 = GPIOD[0];
auto sw_3 = GPIOD[1];

/* disable this with a debugger if you want to measure power sanely
 * You'll then have to watch the ITM channels to see that things do what you think.
 */
bool opt_really_use_leds = false;

/**
 * We just want to run at 32Mhz, so skip the "normal" rcc_init() full speed option
 */
void krcc_init32(void)
{
	// Prefetch and both caches, plus 1WS for 32MHz
	FLASH->ACR = 0x700 | 1;

	// Enable HSE.
	RCC->CR |= (1 << 16);
	while (!(RCC->CR & (1 << 17)));

	RCC->CFGR |= 0x2;
	while ((RCC->CFGR & (2 << 2)) != (2 << 2)); // SWS = HSE
	// Leave prescalers alone...

	PWR->CR1 |= (1 << 8); // Unlock backup domain
	RCC->BDCR |= (1 << 0); // LSE ON

	// RF wakeup clock selection
	RCC->CSR &= ~(0x3 << 14);
	RCC->CSR |= (1 << 14); // LSE

	/////// FROM HERE DOWN IS PROBABLY NOT PART OF THE "rcc32_init" step? ///////////////
	// Set SMPS clock to HSE.  (This is _ignored_ if the radio is active, and HSE is used regardless!)
	RCC->SMPSCR = (1 << 4) | (2 << 0); // HSE+div1.

	// Set RF wakeup clock to LSE
	RCC->CSR |= (1 << 14);
}

static void krtc_init(void)
{
	// Use the last backup register as a flag that we're already initialized
	if (RTC->BKP[19] == 0xcafe) {
		printf("RTC: already setup!\n");
		return;
	}
	PWR->CR1 |= (1 << 8); // Unlock backup domain

	// We've decided that it's "not setup" so go ahead and hard reset
	// The RTC.  You want to be careful with this, or you lose your
	// precious calendar.
	printf("hard reset backup domain\n");
	RCC->BDCR |= (1 << 16);
	RCC->BDCR &= ~(1 << 16);

	// make sure LSE is on, we need that for most sane uses of RTC
	RCC->BDCR |= 1;
	while (!(RCC->BDCR & (1 << 1))) {
		; // wait for LSERDY
	}

	RCC->BDCR |= (1 << 8); // RTCSEL == LSE
	RCC->BDCR |= (1 << 15); // RTCEN
	RCC.enable(rcc::RTCAPB);
	while (!(RCC->APB1ENR1 & rcc::RTCAPB)) {
		; // make sure we have access!
	}

	RTC.unlock();

	RTC->PRER = 0x7f00ff; // default, gives 1sec from 32768

	// setup calendar as well
	RTC->ISR = (1 << 7);
	while (!(RTC->ISR & (1 << 6))) {
		; // Wait for INITF
	}
	RTC->CR &= ~(1 << 6); // 24hr format.
	RTC->DR = 0x220127 | (4 << 13); // Thursday,2022-01-27.
	RTC->TR = 0x092042; // 09:20:42 AM
	RTC->ISR &= ~(1 << 7); // Clear init, start calendar

	// set wakekup clock sel to rtc/16, so ~488usecs per tick
	RTC->CR &= ~(0x7 << 0);
	RTC->CR |= (0 << 0);

	RTC.lock();

	// Set it up now, but we will turn it on later.
	// RTC Wakeup is via exti 19! only listed in the nvic section
	EXTI->RTSR1 |= (1 << 19);
	EXTI->IMR1 |= (1 << 19);
	// we want to wakeup, but we don't need a handler
	//NVIC.enable(interrupt::irq::RTC_WKUP);
	RTC->BKP[19] = 0xcafe;
}

static void print_date(void)
{
	// order of read is important!
	uint32_t s = RTC->SSR;
	uint32_t t = RTC->TR;
	uint32_t d = RTC->DR;

	printf("20%02lx:%02lx:%02lx T %02lx:%02lx:%02lx.%03ld\n",
		(d >> 16) & 0xff,
		(d >> 8) & 0x1f,
		d & 0x3f,
		(t >> 16) & 0x3f,
		(t >> 8) & 0x7f,
		t & 0x7f,
		s * 1000 / 256 // based on "default" fck_spre
		);
}

static void kexti_init(void)
{
	// sw_3.. is D1...
	RCC.enable(rcc::GPIOD);

	sw_3.set_mode(Pin::Mode::Input);
	sw_3.set_pull(Pin::Pull::PullUp);

	SYSCFG->EXTICR[sw_3.n >> 2] &= ~(0xf << ((sw_3.n & 0x3) * 4));
	SYSCFG->EXTICR[sw_3.n >> 2] |= (sw_3.get_portnum() << ((sw_3.n & 0x3) * 4));

	EXTI->IMR1 |= (1 << sw_3.n); // unmask
	EXTI->FTSR1 |= (1 << sw_3.n); // falling edge

	NVIC.enable(interrupt::irq::EXTI1);
}

static volatile bool pressed = false;


enum sleep_modes {
	STOP0,
	STOP1,
	STOP2,
	SLEEP,
	LPRUN,
	LPSLEEP,
	STANDBY_WSRAM2,
	STANDBY,
	SHUTDOWN
};


class ISleepHandler {
public:
	virtual void pre(void) = 0;
	virtual void post(void) = 0;
};

class HandleSleep : public ISleepHandler {
public:
	void pre(void) {
		// sleep needs nothing
	}
	void post(void) {
		// sleep needs nothing
	}
};

class HandleStop : public ISleepHandler {
private:
	enum sleep_modes _mode;
public:
	HandleStop(enum sleep_modes mode): _mode{mode} {};
	
	void pre(void) {
		printf("stop pre %u\n", _mode);
		PWR.set_lpms(_mode);
	}
	void post(void) {
		printf("stop post\n");
		// sleep needs nothing
	}
};

class HandleLpRun : public ISleepHandler {
private:
	uint32_t _rcc_cr;
	uint32_t _rcc_cfgr;
public:
	void pre(void) {
//		printf("lprun pre\n");
		// 1. Optional, jump to sram and power down flash...
		// 2. Drop sysclock <= 2Mhz
		// options are MSI @ 2Mhz or lower, (ranges 0..5 inclusive)
		// or.. HSE /2, sysclk = 16 cpu1_hpre)/8 abd cpu2/8?  (seems awkward)
		_rcc_cr = RCC->CR;
		_rcc_cfgr = RCC->CFGR;
		RCC->CR &= ~(0xf<<4); // Clear MSI Range;
//		uint32_t msi_range = 4; // 1Mhz
		uint32_t msi_range = 0; // 100kHz
		RCC->CR |= (msi_range << 4) | 1; // MSI on as well, just in case...
		RCC->CFGR &= ~(0x3<<0); // Sysclock as MSI
		// better actually check, so that we enter properly..
		while (RCC->CFGR & (0x3<<20)); // wait for SWS to show MSI (0)
		
		// 3. Regulator to LP mode.
		PWR->CR1 |= (1<<14);
	}
	void post(void) {
//		printf("lprun post\n");
		// 1. Regulator back to main mode
		PWR->CR1 &= ~(1<<14);
		// 2. wait for REGLPF to clear
		while (PWR->SR2 & (1<<9));
		// 3. Increase sysclock as desired
		// we're just going to to with "restore RCC->CR and RCC-CFGR....
		RCC->CR = _rcc_cr;
		RCC->CFGR = _rcc_cfgr;
	}
};


auto handleLpRun = HandleLpRun();
auto handleSleep = HandleSleep();

//const ISleepHandler *handler = &handleLpRun;

static void ksleep() {
	handleLpRun.pre();
//	handleSleep.pre();
	asm volatile ("wfi");
}

static void ksleep_exit() {
	handleLpRun.post();
//	handleSleep.post();
}

#if 0
static sleep_modes mode = sleep_modes::LPRUN;
static ISleepHandler *sleepHandler;

static void ksleep_x()
{
	// default it, so keep uninitialized warnings happy
	switch(mode) {
	case SLEEP:
	{
		auto x = HandleSleep();
		sleepHandler = &x;
		break;
	}
	case LPRUN:
	{
		auto x = HandleLpRun();
		sleepHandler = &x;
		break;
	}
	case STOP0:
	case STOP1:
	case STOP2:
	{
		auto x = HandleStop(mode);
		sleepHandler = &x;
		break;
	}
	default:
		printf("unhandled sleep mode!\n");
		// Fall through
	}
	sleepHandler->pre();
	asm volatile ("wfi");
}

static void ksleep_exit_x(void) {
	sleepHandler->post();
}
#endif

int main()
{
	// Turn on DWT_CYCNT.  We'll use it ourselves, and pc sampling needs it too.
	DWT->CTRL |= 1;

	krcc_init32();
	// We'll need the rtc for low power wakeups
	krtc_init();
	kexti_init();

	if (opt_really_use_leds) {
		RCC.enable(rcc::GPIOB);
		led_b.set_mode(Pin::Output);
	}

	printf("main preloop:  ");
	print_date();
	while (1) {

		printf("start loop\n");
		while (!pressed) {
			for (int i = 0; i < 0x80000; i++) {
				asm volatile ("nop");
			}
			if (opt_really_use_leds) {
				led_b.toggle();
			}
		}
//		for (auto i = 5; i; i--) {
//			printf("sleeping in %d!\n", i);
//			for (auto k = 0; k < 0x800000; k++) {
//				asm volatile ("nop");
//			}
//			if (opt_really_use_leds) {
//				led_b.toggle();
//			}
//		}
		// Ok, button pressed, go to sleep!
		printf("going to sleep at: ");
		print_date();
		ksleep();
		ksleep_exit();
//		asm volatile("wfi");
		pressed = false;
		printf("woke up at.. ");
		print_date();
	}

	return 0;
}

template <>
void interrupt::handler<interrupt::irq::EXTI1>()
{
	EXTI->PR1 |= (1 << sw_3.n);
	ITM->stim_blocking(0, '!');
	pressed = true;
}

