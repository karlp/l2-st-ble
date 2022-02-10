#include <cstdio>

#include <cortex_m/debug.h>
#include <exti/exti.h>
#include <gpio/gpio.h>
#include <interrupt/interrupt.h>
#include <pwr/pwr.h>
#include <rcc/flash.h>
#include <rcc/rcc.h>
#include <rtc/rtc.h>
#include <uart/uart.h>
#include <wpan/hsem.h>
#include <wpan/ipcc.h>

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "arm_math.h"

#include "t_ble.h"

auto led_r = GPIOB[1];
auto led_g = GPIOB[0];
auto led_b = GPIOB[5];

#define SAVE_TO_SECOND_BUFFER


#include <cerrno>
#include <cstdlib>
#include <unistd.h>


extern "C" int _write(int file, char* ptr, int len) {
        int i;

        if (file == STDOUT_FILENO || file == STDERR_FILENO) {
                for (i = 0; i < len; i++) {
                        if (ptr[i] == '\n') {
				LPUART1.write_blocking('\r');
                        }
		LPUART1.write_blocking(ptr[i]);
                }
                return i;
        }
        errno = EIO;
        return -1;
}


/**
 * We just want to run at 32Mhz, so skip the "normal" rcc_init() full speed option
 */
void krcc_init32(void) {
	// Prefetch and both caches, plus 1WS for 32MHz
	FLASH->ACR = 0x700 | 1;

	// Enable HSE.
	RCC->CR |= (1<<16);
	while(!(RCC->CR & (1<<17)));

	// Make sure to clear bits, we may be waking up!
	RCC->CFGR &= ~0x3;
	RCC->CFGR |= 0x2;
	while ((RCC->CFGR & (2<<2)) != (2<<2)); // SWS = HSE
	// Leave prescalers alone...
	
	// RF wakeup clock selection
	RCC->CSR &= ~(0x3<<14);
	RCC->CSR |= (1<<14); // LSE

	/////// FROM HERE DOWN IS PROBABLY NOT PART OF THE "rcc32_init" step? ///////////////
	// Set SMPS clock to HSE.  (This is _ignored_ if the radio is active, and HSE is used regardless!)
	RCC->SMPSCR = (1<<4) | (2<<0); // HSE+div1.
	
	// Set RF wakeup clock to LSE
	RCC->CSR |= (1<<14);	
}


static void krtc_init(void) {
	// always need that even if RTC itself is already setup
	PWR->CR1 |= (1 << 8); // Unlock backup domain
	
	// Use the last backup register as a flag that we're already initialized
	if (RTC->BKP[19] == 0xcafe) {
		printf("RTC: already setup!\n");
		return;
	}

	// We've decided that it's "not setup" so go ahead and hard reset
	// The RTC.  You want to be careful with this, or you lose your
	// precious calendar.
	printf("hard reset backup domain\n");
	RCC->BDCR |= (1<<16);
	RCC->BDCR &= ~(1<<16);

	// make sure LSE is on, we need that for most sane uses of RTC
	RCC->BDCR |= 1;
	while (!(RCC->BDCR & (1<<1))) {
		; // wait for LSERDY
	}

	RCC->BDCR |= (1<<8); // RTCSEL == LSE
	RCC->BDCR |= (1<<15); // RTCEN
	RCC.enable(rcc::RTCAPB);
//	while (!(RCC->APB1ENR1 & rcc::RTCAPB)) {
//		; // make sure we have access!
//	}

	RTC.unlock();

	RTC->PRER = 0x7f00ff; // default, gives 1sec from 32768

	// setup calendar as well
	RTC->ISR = (1<<7);
	while (!(RTC->ISR & (1<<6))) {
		; // Wait for INITF
	}
	RTC->CR &= ~(1<<6);  // 24hr format.
	RTC->DR = 0x220127 | (4<<13); // Thursday,2022-01-27.
	RTC->TR = 0x092042;  // 09:20:42 AM
	RTC->ISR &= ~(1<<7); // Clear init, start calendar

	// set wakekup clock sel to rtc/16, so ~488usecs per tick
	RTC->CR &= ~(0x7<<0);
	RTC->CR |= (0<<0);

	RTC.lock();

	// Set it up now, but we will turn it on later.
	// RTC Wakeup is via exti 19! only listed in the nvic section
	EXTI->RTSR1 |= (1<<19);
	EXTI->IMR1 |= (1 << 19);
	// we want to wakeup, but we don't need a handler
	//NVIC.enable(interrupt::irq::RTC_WKUP);
	RTC->BKP[19] = 0xcafe;
}

static void klpuart_init(void) {
//	RCC.enable(rcc::USART1);
	RCC.enable(rcc::LPUART1);

	// sysclock? lpuart1
	RCC->CCIPR &= ~(0x3<<10);
	RCC->CCIPR |= (1<<10);
	// usart1
	RCC->CCIPR &= ~(0x3<<0);
	RCC->CCIPR |= (1<<0);


#if defined(STM32WB) || defined(STM32G4)
	// PA2,3 for lpuart1
	RCC.enable(rcc::GPIOA);
	auto lp1_rx = GPIOA[2];
	auto lp1_tx = GPIOA[3];
#elif defined(STM32L4)
	RCC.enable(rcc::GPIOC);
	auto lp1_rx = GPIOC[0];
	auto lp1_tx = GPIOC[1];
#endif

#if defined(STM32G4)
	lp1_rx.set_af(12);
	lp1_tx.set_af(12);
#else
	lp1_rx.set_af(8);
	lp1_tx.set_af(8);
#endif
	lp1_rx.set_mode(Pin::Mode::AF);
	lp1_tx.set_mode(Pin::Mode::AF);

//	RCC.enable(rcc::GPIOB);
//	auto pb6 = GPIOB[6];
//	auto pb7 = GPIOB[7];
//	pb6.set_af(7);
//	pb7.set_af(7);
//	pb6.set_mode(Pin::Mode::AF);
//	pb7.set_mode(Pin::Mode::AF);
//
//	USART1->CR1 = (1<<29); // fifo, 8bit, no parity, no special
//	//USART1->CR1 = 0;
//	USART1->CR2 = 0; // standard
//	// this is for lpuart USART1->BRR = (256*32000000u / 115200); // 71111?
//	USART1->BRR = configCPU_CLOCK_HZ / 115200u;
//	USART1->CR1 |= (1<<3)|1; // TX only, enabled

	LPUART1->CR1 = (1<<29); // fifo, 8bit, no parity, no special
	//LPUART1->CR1 = 0;
	LPUART1->CR2 = 0; // standard
	LPUART1->BRR = (configCPU_CLOCK_HZ / 115200) * 256; // careful with overflows!
	LPUART1->CR1 |= (1<<3)|1; // TX only, enabled

}


static void print_date(void) {
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
		(1000*(255 - s)/256) // based on "default" fck_spre
		);
}


static void prvTaskBlinkGreen(void *pvParameters)
{
	(void)pvParameters;
	led_g.set_mode(Pin::Output);

	int i = 0;
	while (1) {
		i++;
		vTaskDelay(pdMS_TO_TICKS(2500));
//	        ITM->stim_blocking(0, (uint8_t)('a' + (i%26)));
		led_g.toggle();
//		printf("testing: %d\n", i);
		print_date();
	}
}


int main() {
	ble_pre();
	// Turn on DWT_CYCNT.  We'll use it ourselves, and pc sampling needs it too.
	DWT->CTRL |= 1;
	
	krcc_init32();
	klpuart_init();
	printf("Clocks and console up!\n");

	// We'll need the rtc for low power wakeups
	krtc_init();

	RCC.enable(rcc::GPIOB);
	RCC.enable(rcc::GPIOE);

	xTaskCreate(prvTaskBlinkGreen, "green.blink", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);

	xTaskCreate(task_ble, "ble", configMINIMAL_STACK_SIZE*3, &task_state_ble, tskIDLE_PRIORITY + 1, &th_ble);

	printf("Starting OS!\n"); // FIXME - print out rtc date/time here ;)
	vTaskStartScheduler();

	return 0;
}

// TODO -figure out how to give this to freertosconfig?
//#define vPortSVCHandler SVC_Handler
//#define xPortPendSVHandler PendSV_Handler
//#define xPortSysTickHandler SysTick_Handler
extern "C" {
	void vPortSVCHandler(void);
	void xPortPendSVHandler(void);
}
template <>
void interrupt::handler<interrupt::exception::SVCall>() {
	vPortSVCHandler();
}
template <>
void interrupt::handler<interrupt::exception::PendSV>() {
	xPortPendSVHandler();
}
extern void LPTIM_IRQHandler(void);
template <>
void interrupt::handler<interrupt::irq::LPTIM1>() {
	LPTIM_IRQHandler();
}