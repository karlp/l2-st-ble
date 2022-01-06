/*
 * top level ble task code.... just to not get _toooo_ much clutter
 */

#include <cstdio>

#include <cal/cal.h>
#include <cortex_m/debug.h>
#include <dma/dma.h>
#include <gpio/gpio.h>
#include <interrupt/interrupt.h>
#include <rcc/rcc.h>
#include <timer/timer.h>

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

#include "t_ble.h"


TaskHandle_t th_ble;


// FIXME - place holder for bluetooth task state object...
struct ts_ble_t {
	
};
struct ts_ble_t task_state_ble;



void task_ble(void *pvParameters)
{
	struct ts_ble_t *ts = (struct ts_ble_t*)pvParameters;
	(void)ts; // FIXME - remove when you start using it!
	
	RCC.enable(rcc::HSEM);
	NVIC.enable(interrupt::irq::HSEM);
	
	TickType_t xLastWakeTime = xTaskGetTickCount();
	while (1) {
		xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(250));
	}
}



