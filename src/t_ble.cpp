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



//(gdb) x /64x 0x1FFF7000
//0x1fff7000:	0xe1051355	0x00170080
//(gdb) x /64b 0x1FFF7000
//0x1fff7000:	0x55	0x13	0x05	0xe1	0x80	0x00	0x17	0x00


//  typedef  PACKED_STRUCT
//  {
//    uint8_t   bd_address[6];
//    uint8_t   hse_tuning;
//    uint8_t   id;
//  } OTP_ID0_t;

/* WARNING MUST BE CALLED WITH HSE OFF! */
static void _rcc_tune_hse(uint16_t tune) {
	RCC->HSECR = 0xcafecafe; // unlock
	RCC->HSECR |= (tune & 0x3f) << 8;
}

static void _tune_hse(void) {
	// FIXME - this is wrong, they apparently re-write the struct downwards,
	// and have a function to scan backwards looking for a valid block...
	if (STM32::Calibration::OTP_ID0.hse_tuning) {
		
	}
	// on my board, it has one entry, and "id 0 therefore implies hse tuning of 0x17...
	// and this is what the ST demo app does.
	_rcc_tune_hse(0x17);
}

void ble_pre(void) {
	// ST demos reset IPCC and backup domain here, but.... let's not, and get bitten by that later.	
	_tune_hse();
}

void ble_tl_init(void) {
	
}

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



