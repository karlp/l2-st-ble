#include <cstdio>

#include <adc/adc_f3.h>
#include <cal/cal.h>
#include <cortex_m/debug.h>
#include <dma/dma.h>
#include <exti/exti.h>
#include <gpio/gpio.h>
#include <interrupt/interrupt.h>
#include <pwr/pwr.h>
#include <rcc/flash.h>
#include <rcc/rcc.h>
#include <timer/timer.h>
#include <uart/uart.h>
#include <usb/usb.h>
#include <usb/descriptor.h>
#include <wpan/hsem.h>
#include <wpan/ipcc.h>

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "arm_math.h"

#include "analog.h"
#include "t_ble.h"

auto led_r = GPIOB[1];
auto led_g = GPIOB[0];
auto led_b = GPIOB[5];

#define SAVE_TO_SECOND_BUFFER

// This makes it more ~equivalent to the ST demos, that all run at 32MHz...
#define RUNNING_AT_32MHZ

#if defined(RUNNING_AT_32MHZ)
/**
 * We just want to run at 32Mhz, so skip the "normal" rcc_init() full speed option
 */
void krcc_init32(void) {
	// Prefetch and both caches, plus 1WS for 32MHz
	FLASH->ACR = 0x700 | 1;

	// Enable HSE.
	RCC->CR |= (1<<16);
	while(!(RCC->CR & (1<<17)));

	/* I initially setup PLL to 32Mhz, because I had PLL to 64, and just downtuned
	 * but if you want lower power, use HSE by itself at 32.  The PLL is useless burn
	 */
#if defined(USE_PLL_32)
	// Configure and enable PLL.
	// R=2, Q = 2, P = 2, M = 2, N = 8, src=HSE
	const auto m = 2;
	const auto n = 8;
	const auto p = 2;
	const auto q = 2;
	const auto r = 4;
	//const auto hse_pre_div2 = false;

	RCC->PLLCFGR = ((r-1)<<29) | (1<<28) | ((q-1)<<25) | ((p-1)<<17) | (n<<8) | ((m-1)<<4) | (3<<0);
	RCC->CR |= (1<<24);
	while(!(RCC->CR & (1<<25)));

	// Switch to PLL.
	RCC->CFGR |= 0x3;
	while((RCC->CFGR & (3 << 2)) != (3 << 2)); // SWS = PLL
#else
	RCC->CFGR |= 0x2;
	while ((RCC->CCIPR & (2<<2)) != (2<<2)); // SWS = HSE
#endif
	// Leave prescalers alone...
	
	PWR->CR1 |= (1<<8); // Unlock backup domain
	RCC->BDCR |= (1<<0); // LSE ON

	/////// FROM HERE DOWN IS PROBABLY NOT PART OF THE "rcc32_init" step? ///////////////
	// Set SMPS clock to HSE.  (This is _ignored_ if the radio is active, and HSE is used regardless!)
	RCC->SMPSCR = (1<<4) | (2<<0); // HSE+div1.
	
	// Set RF wakeup clock to LSE
	RCC->CSR |= (1<<14);
	
	// MX_IPCC_Init()...
	RCC.enable(rcc::IPCC);
	NVIC.enable(interrupt::irq::IPCC_C1_RX);
	NVIC.enable(interrupt::irq::IPCC_C1_TX);
	// XXX: IPCC reset? yolo! I don't see what state we should bother with this in...
//	IPCC->C1CR = 0;
//	IPCC->C1MR = 0x3f << 16 | 0x3f;
//	IPCC->C1SCR = 0x3f;
	// Ok, but seriously,
	IPCC->C1CR |= (1<<16) | (1<<0); // TXFIE | RXOIE
	
	// MX_RF_Init() is null
	// MX_RTC_Init()...
	// FIXME: XXX fuck it, I'm not sure what we're using this for, it can wait...
	// demo app seems to be using it for a wakeup timer? I'll have adc dma timer for that...
	
	
	// MX_APPE_Init()...
	// CAN this go to top of bluetooth? (appears to still do exti/smps shits..
	PWR->CR5 &= ~(7 << 4); // 80mA startup current
	// Not sure why we need to do this, but... It talks about limiting rf output power?
	int32_t now = PWR->CR5 & 0x7; // reset calibration is at 1.5V
	now -= 2; // Attempt to get 1.4V
	if (now > 0 && now < 7) {
		PWR->CR5 |= now;
	} else {
		printf("yolo smps setting?!");
	}
	
	EXTI->IMR2 |= (1 << (36-32)) | (1<<(38-32)); // IPCC and HSEM wakeup EXTIs
	
	// XXX: more RTC init here, setting wakeup clocks.
	
	// SystemPower_Config()....
	// set hsi as sysclock after wakeup from stop?
	// ->  nope, we're never going to stop...
	// init "util_lpm_..." -> nope...
	PWR->C2CR1 &= ~(0x7);
	PWR->C2CR1 |= 0x4; // LPMS == Shutdown
	
	
	// HW_TS_Init()....
	// FIXME - this one _might_ need the RTC finally?
	
	///// APPD_Init()
	// XXX: there's a step here about enabling debugger, which is a power thing... revisit.
	
	// APPD_SetCPU2GpioConfig() ? lol, no, we're not getting support from ST with this code :)
	// APPD_bleDtbCfg() ? lol, same, no ST support here bois!
	/////
	
	// UTIL_LPM modes again.  we're definitely not going to be using that code, we want it better tied into freertos..
	
	// appe_Tl_Init()...
	// XXX this is a real good sized one!
	// go straight to another file for it...
	kble_tl_init();  // this is the last thing ST does beore starting the OS...
	
}
#endif


static void prvTaskBlinkGreen(void *pvParameters)
{
	(void)pvParameters;
	led_g.set_mode(Pin::Output);

	int i = 0;
	while (1) {
		i++;
		vTaskDelay(pdMS_TO_TICKS(500));
//	        ITM->stim_blocking(0, (uint8_t)('a' + (i%26)));
		led_g.toggle();
//		printf("testing: %d\n", i);
	}
}


int main() {
	ble_pre();
	
#if defined(RUNNING_AT_32MHZ)
	krcc_init32();
#else
	rcc_init();
#endif
	// Turn on DWT_CYCNT.  We'll use it ourselves, and pc sampling needs it too.
	DWT->CTRL |= 1;

	RCC.enable(rcc::GPIOB);
	RCC.enable(rcc::GPIOE);

	xTaskCreate(prvTaskBlinkGreen, "green.blink", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);

	// Required to use FreeRTOS ISR methods!
	NVIC.set_priority(interrupt::irq::DMA1_CH1, 6<<configPRIO_BITS);

	xTaskCreate(task_ble, "ble", configMINIMAL_STACK_SIZE*3, &task_state_ble, tskIDLE_PRIORITY + 1, &th_ble);
	xTaskCreate(task_kadc, "kadc", configMINIMAL_STACK_SIZE*3, &task_state_adc, tskIDLE_PRIORITY + 1, &th_kadc);
	xTaskCreate(task_temperature, "ktemp", configMINIMAL_STACK_SIZE*3, NULL, tskIDLE_PRIORITY + 1, NULL);

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
	void xPortSysTickHandler(void);
}
template <>
void interrupt::handler<interrupt::exception::SVCall>() {
	vPortSVCHandler();
}
template <>
void interrupt::handler<interrupt::exception::PendSV>() {
	xPortPendSVHandler();
}
template <>
void interrupt::handler<interrupt::exception::SysTick>() {
	xPortSysTickHandler();
}

