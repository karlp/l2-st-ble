#include <cstdio>

#include <adc/adc_f3.h>
#include <cal/cal.h>
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
#include "arm_math.h"

#include "analog.h"

auto led_r = GPIOB[1];
auto led_g = GPIOB[0];
auto led_b = GPIOB[5];

#define SAVE_TO_SECOND_BUFFER


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
#endif

static TaskHandle_t th_ble;


// FIXME - place holder for bluetooth task state object...
struct ts_ble_t {
	
};
struct ts_ble_t task_state_ble;



static TimerHandle_t xBlueTimer;
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
	led_g.set_mode(Pin::Output);

	int i = 0;
	while (1) {
		i++;
		vTaskDelay(pdMS_TO_TICKS(500));
//	        ITM->stim_blocking(0, (uint8_t)('a' + (i%26)));
//		led_g.toggle();
//		printf("testing: %d\n", i);
	}
}


static void prvTask_ble(void *pvParameters)
{
	struct ts_ble_t *ts = (struct ts_ble_t*)pvParameters;
	(void)ts; // FIXME - remove when you start using it!
	TickType_t xLastWakeTime = xTaskGetTickCount();
	while (1) {
		xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(250));
	}
}


int main() {
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

	// Required to use FreeRTOS ISR methods!
	NVIC.set_priority(interrupt::irq::DMA1_CH1, 6<<configPRIO_BITS);

	xTaskCreate(prvTask_ble, "ble", configMINIMAL_STACK_SIZE*3, &task_state_ble, tskIDLE_PRIORITY + 1, &th_ble);
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

