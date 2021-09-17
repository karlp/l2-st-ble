#include <adc/adc_f3.h>
#include <cortex_m/debug.h>
#include <dma/dma.h>
#include <gpio/gpio.h>
#include <interrupt/interrupt.h>
#include <rcc/flash.h>
#include <rcc/rcc.h>
#include <timer/timer.h>
#include <uart/uart.h>
#include <usb/usb.h>
#include <usb/descriptor.h>

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

/**
 * We just want to run at 32Mhz, so skip the "normal" rcc_init() full speed option
 */
void krcc_init32(void) {
	// Prefetch and both caches, plus 1WS for 32MHz
	FLASH.ACR = 0x700 | 1;

	// Enable HSE.
	RCC->CR |= (1<<16);
	while(!(RCC->CR & (1<<17)));

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

	// Leave prescalers alone...

}

static volatile uint16_t adc_buf[5];
static volatile uint16_t kdata[1024];
static volatile int kindex = 0;
static volatile int kinteresting = 4;

void adc_set_sampling(unsigned channel, int sampling) {
	if (channel < 10) {
		ADC1.SMPR1 &= ~(0x7<<(3*channel));
		ADC1.SMPR1 |= (sampling<<(3*channel));
	} else {
		channel -= 10;
		ADC1.SMPR1 &= ~(0x7<<(3*channel));
		ADC1.SMPR1 |= (sampling<<(3*channel));
	}
}

void adc_set_sampling(int sampling) {
	uint32_t reg = 0;
	for (int i = 0; i < 10; i++) {
		reg |= (sampling << (3*i));
	}
	ADC1.SMPR1 = reg;
	ADC1.SMPR2 = reg;  // extra bits? yolo!
}

auto led_r = GPIOB[1];
static void prvTask_ktimer_simple(void *pvParameters)
{
	(void)pvParameters;

	led_r.set_mode(Pin::Output);

	RCC.enable(rcc::TIM2);
	const auto freq = 5;

	TIM2->ARR = (32000000 / freq) - 1;
	TIM2->CR2 = (2<<4); // Master mode update event, will be used by ADC eventually
	TIM2->CCER = 1 << 0;
	TIM2->DIER = 1 << 0; // Update interrupt plz

	interrupt_ctl.enable(interrupt::irq::TIM2);

	// Finally, start the timer that is going to do the counting....
	TIM2->CR1 = 1 << 0; // Enable;

	while (1) {
		vTaskDelay(pdMS_TO_TICKS(2000));
	}
}

static void prvTask_kadc(void *pvParameters)
{
	(void)pvParameters;

	auto led_r = GPIOB[1];
	led_r.set_mode(Pin::Output);
	uint32_t before = DWT->CYCCNT;


	RCC.enable(rcc::TIM2);
	const auto freq = 5000;

	TIM2->ARR = (32000000 / freq) - 1;
	TIM2->CR2 = (2<<4); // Master mode update event, will be used by ADC eventually
	TIM2->CCER = 1 << 0;

	// setup DMA first...
	RCC.enable(rcc::DMAMUX1);
	RCC.enable(rcc::DMA1);

	// Use DMA mux channel 0 / DMA Channel 1for ADC
	DMAMUX1->CCR[0] = 5;
	DMA1->C[0].NDTR = 5;
	DMA1->C[0].MAR = (uint32_t)&adc_buf;
	DMA1->C[0].PAR = (uint32_t)&(ADC1.DR);
	DMA1->C[0].CR = 0
		| (1<<10) // msize 16bit
		| (1<<8) // psize 16bit
		| (1<<7) // minc plz
		| (1<<5) // circ plz
		| (5<<1) // TE+TC irqs plz
		| 1 // enable, will do nothing without requests
		;
	interrupt_ctl.enable(interrupt::irq::DMA1_CH1);

	// Turn on the ADC then we'll do other things while it's waking up.
	RCC.enable(rcc::ADC1);
	// Make sure we give it a clock!
	RCC->CCIPR |= (3<<28); // Use sysclk for now. We may want to run it slower later.
	ADC1.CR = (1<<28);  // turn off deep power down (bit 29) and enables vreg
	// waiting for adc vreg is max 20 usecs, FIXME: get a shorter loop for usecs..
	// (20usecs is 640 cycles at 32MHz, fyi... so we're always going to be waiting...)
	ITM->STIM[2].u16 = DWT->CYCCNT - before;
	vTaskDelay(pdMS_TO_TICKS(1));

	// If you have calibration from "earlier" apply it, otherwise...
	before = DWT->CYCCNT;
	uint32_t calfact = 0;
	if (calfact) {
		// TODO - I think ADEN must be turned on here first.
		ADC1.CALFACT = calfact;
	} else {
		// This is meant to take about 116 adc fclock cycles, ish.
		// That's still only <4usecs at 32MHz
		ADC1.CR |= (1<<31);
		while (ADC1.CR & (1<<31))
			;
		calfact = ADC1.CALFACT; // nominally, save them
	}
	ITM->STIM[2].u16 = DWT->CYCCNT - before;
	// nominally, 4 clock cycles required between CAL finishing and before we can turn on CR, should be ok....

	// clear adcrdy flag, aden=1, wait til adcrdy flag...
	ADC1.ISR = 1;
	ADC1.CR |= 1;
	while (!(ADC1.ISR & (1<<0)))
		;

	// TODO: cube sets up OVERRUN interrupt?  is it worth handling that? Even just rebooting? flagging that we've got bad data?

	// turn on temp sensor and vrefint
	ADC_COMMON.CCR |= (1<<23) | (1<<22);

	// FIXME - recalculate based on final ADC clocks:
	// at 32Mhz, 4us = 128, 5us = 160, and 12 = 384
	adc_set_sampling(4); //4 == 47.5 clocks on all for starters.
	adc_set_sampling(0, 0b110); // that's 247, but 92 is too low
	adc_set_sampling(17, 0b110); // that's 247, but 92 is too low

	// I want ext11, which is tim2 trgo
	ADC1.CFGR = (1<<31)  // LEave JQDIS
		| (1 << 10) // EXTEN rising edge
		| (11<<6) // EXTI11 for tim2 trgo
		| (3) // DMA circular + DMA enable
		;
	// Set ADC to start when it starts getting triggers
	ADC1.CR |= (1<<2);

	// Sequences are silly, but so be it...
	ADC1.SQR1 = 5-1;  // 5 conversions first.

	ADC1.SQR1 |= (1<<(6*1));
	ADC1.SQR1 |= (2<<(6*2));
	ADC1.SQR1 |= (6<<(6*3));  // external 1,2,6
	ADC1.SQR1 |= (17<<(6*4)); // temp sensor
	ADC1.SQR2 = (18 << (6*0)); // vrefint

	// Finally, start the timer that is going to do the counting....
	TIM2->CR1 = 1 << 0; // Enable;

	int i = 0;
	while (1) {
		i++;
	        ITM->STIM[0].u8 = 'A' + (i%26);
//		led_r.toggle();
		vTaskDelay(pdMS_TO_TICKS(1000));
	}


}


template <>
void interrupt::handler<interrupt::irq::TIM2>() {
	TIM2->SR = ~(1<<0); // Clear UIF
//	led_r.toggle();
}

template <>
void interrupt::handler<interrupt::irq::DMA1_CH1>() {
	if (DMA1->ISR & (1<<1)) { // CH1 TCIF
		DMA1->IFCR = (1<<1);
		ITM->STIM[1].u16 = adc_buf[kinteresting];
		kdata[kindex++] = adc_buf[kinteresting];
		if (kindex >= 1024) {
			kindex = 0;
		}
	}
	if (DMA1->ISR & (1<<3)) {
		// Errors...
		DMA1->IFCR = (1<<3); // clear it at least.
		ITM->STIM[0].u8 = '!';
	}
}

static TimerHandle_t xBlueTimer;
auto led_b = GPIOB[5];
static void prvTimerBlue(TimerHandle_t xTimer)
{
	/* Timers can only work on globals, boo,
	 * no, (ab)using pvTimerGetTimerID doesn't sound worthwhile */
        (void) xTimer;
//        led_b.toggle();
}


static void prvTaskBlinkGreen(void *pvParameters)
{
	(void)pvParameters;
	auto led_g = GPIOB[0];
	led_g.set_mode(Pin::Output);

	int i = 0;
	while (1) {
		i++;
		vTaskDelay(pdMS_TO_TICKS(100));
	        ITM->STIM[0].u8 = 'a' + (i%26);
//		led_g.toggle();
	}
}


int main() {
	krcc_init32();
	// Turn on DWT_CYCNT.  We'll use it ourselves, and pc sampling needs it too.
	DWT->CTRL |= 1;

	RCC.enable(rcc::GPIOB);

	xTaskCreate(prvTaskBlinkGreen, "green.blink", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);

	led_b.set_mode(Pin::Output);
	xBlueTimer = xTimerCreate("blue.blink", 200 * portTICK_PERIOD_MS, true, 0, prvTimerBlue);
	if (xBlueTimer) {
		if (xTimerStart(xBlueTimer, 0) != pdTRUE) {
			/* whee */
		} else {
			// boooo
		}
	} else {
		// boooo!!!!! fixme trace?
	}

	xTaskCreate(prvTask_kadc, "kadc", configMINIMAL_STACK_SIZE*3, NULL, tskIDLE_PRIORITY + 1, NULL);
//	xTaskCreate(prvTask_ktimer_simple, "ktimer", configMINIMAL_STACK_SIZE*3, NULL, tskIDLE_PRIORITY + 1, NULL);

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

