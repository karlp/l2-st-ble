/*
 * Handles the nitty gritty of entering and exiting low power modes,
 * and when that's allowed.
 */
#include <stdio.h>
#include "FreeRTOS.h"

#include <exti/exti.h>
#include <gpio/gpio.h>
#include <interrupt/interrupt.h>
#include <pwr/pwr.h>
#include <rcc/rcc.h>
#include <uart/uart.h>
#include <wpan/hsem.h>

struct save_data_t {
	uint32_t cr;
	uint32_t cfgr;
	uint32_t smpscr;
};
static struct save_data_t save_data;


/******************************************************************************
 * Semaphores
 * THIS SHALL NO BE CHANGED AS THESE SEMAPHORES ARE USED AS WELL ON THE CM0+
 *****************************************************************************/
/**
*  The CPU2 may be configured to store the Thread persistent data either in internal NVM storage on CPU2 or in
*  SRAM2 buffer provided by the user application. This can be configured with the system command SHCI_C2_Config()
*  When the CPU2 is requested to store persistent data in SRAM2, it can write data in this buffer at any time when needed.
*  In order to read consistent data with the CPU1 from the SRAM2 buffer, the flow should be:
*  + CPU1 takes CFG_HW_THREAD_NVM_SRAM_SEMID semaphore
*  + CPU1 reads all persistent data from SRAM2 (most of the time, the goal is to write these data into an NVM managed by CPU1)
*  + CPU1 releases CFG_HW_THREAD_NVM_SRAM_SEMID semaphore
*  CFG_HW_THREAD_NVM_SRAM_SEMID semaphore makes sure CPU2 does not update the persistent data in SRAM2 at the same time CPU1 is reading them.
*  There is no timing constraint on how long this semaphore can be kept.
*/
#define CFG_HW_THREAD_NVM_SRAM_SEMID                    9

/**
*  The CPU2 may be configured to store the BLE persistent data either in internal NVM storage on CPU2 or in
*  SRAM2 buffer provided by the user application. This can be configured with the system command SHCI_C2_Config()
*  When the CPU2 is requested to store persistent data in SRAM2, it can write data in this buffer at any time when needed.
*  In order to read consistent data with the CPU1 from the SRAM2 buffer, the flow should be:
*  + CPU1 takes CFG_HW_BLE_NVM_SRAM_SEMID semaphore
*  + CPU1 reads all persistent data from SRAM2 (most of the time, the goal is to write these data into an NVM managed by CPU1)
*  + CPU1 releases CFG_HW_BLE_NVM_SRAM_SEMID semaphore
*  CFG_HW_BLE_NVM_SRAM_SEMID semaphore makes sure CPU2 does not update the persistent data in SRAM2 at the same time CPU1 is reading them.
*  There is no timing constraint on how long this semaphore can be kept.
*/
#define CFG_HW_BLE_NVM_SRAM_SEMID                    8

/**
*  Index of the semaphore used by CPU2 to prevent the CPU1 to either write or erase data in flash
*  The CPU1 shall not either write or erase in flash when this semaphore is taken by the CPU2
*  When the CPU1 needs to either write or erase in flash, it shall first get the semaphore and release it just
*  after writing a raw (64bits data) or erasing one sector.
*  Once the Semaphore has been released, there shall be at least 1us before it can be taken again. This is required
*  to give the opportunity to CPU2 to take it.
*  On v1.4.0 and older CPU2 wireless firmware, this semaphore is unused and CPU2 is using PES bit.
*  By default, CPU2 is using the PES bit to protect its timing. The CPU1 may request the CPU2 to use the semaphore
*  instead of the PES bit by sending the system command SHCI_C2_SetFlashActivityControl()
*/
#define CFG_HW_BLOCK_FLASH_REQ_BY_CPU2_SEMID                    7

/**
*  Index of the semaphore used by CPU1 to prevent the CPU2 to either write or erase data in flash
*  In order to protect its timing, the CPU1 may get this semaphore to prevent the  CPU2 to either
*  write or erase in flash (as this will stall both CPUs)
*  The PES bit shall not be used as this may stall the CPU2 in some cases.
*/
#define CFG_HW_BLOCK_FLASH_REQ_BY_CPU1_SEMID                    6

/**
*  Index of the semaphore used to manage the CLK48 clock configuration
*  When the USB is required, this semaphore shall be taken before configuring te CLK48 for USB
*  and should be released after the application switch OFF the clock when the USB is not used anymore
*  When using the RNG, it is good enough to use CFG_HW_RNG_SEMID to control CLK48.
*  More details in AN5289
*/
#define CFG_HW_CLK48_CONFIG_SEMID                               5

/* Index of the semaphore used to manage the entry Stop Mode procedure */
#define CFG_HW_ENTRY_STOP_MODE_SEMID                            4

/* Index of the semaphore used to access the RCC */
#define CFG_HW_RCC_SEMID                                        3

/* Index of the semaphore used to access the FLASH */
#define CFG_HW_FLASH_SEMID                                      2

/* Index of the semaphore used to access the PKA */
#define CFG_HW_PKA_SEMID                                        1

/* Index of the semaphore used to access the RNG */
#define CFG_HW_RNG_SEMID                                        0



// hackyhack..
extern Pin led_g;

static void _enable_hsi(void) {
	RCC->CR |= (1<<8); // Turn on HSI
	while (!(RCC->CR & (1<<10))); // Wait for HSIRDY
	RCC->CFGR &= ~(0x3<<0);
	RCC->CFGR |= (1<<0); // Switch clock to HSI
	RCC->SMPSCR &= ~(0x3<<0); // Smps clock to HSI
	while (((RCC->CFGR >> 2) & (0x3)) != 0x1); // Wait for switch

}


void pre_lpsleep(uint32_t _msi_range)
{
	save_data.cr = RCC->CR;
	save_data.cfgr = RCC->CFGR;
	save_data.smpscr = RCC->SMPSCR;
	while (!HSEM.get_lock_1step(CFG_HW_RCC_SEMID));

	// We just do this _anyway?
	if (!HSEM.get_lock_1step(CFG_HW_ENTRY_STOP_MODE_SEMID)) {
		// Check C2DS and C2SBF
		if ((PWR->EXTSCR & (1<<15))  || (PWR->EXTSCR & (1<<10))) {
			HSEM.release(CFG_HW_ENTRY_STOP_MODE_SEMID, 0);
//			printf("P1");
			//_enable_hsi();
			RCC->CR |= (1<<8); // Turn on HSI _only_ don't switch to it!  Just make sure it's available!

		} else {
			printf("MX");
			// Is this the only place I'm allowed to go to MSI?
		}
	} else {
		printf("P2");
		//_enable_hsi();
		RCC->CR |= (1<<8); // Turn on HSI _only_ don't switch to it!  Just make sure it's available!
	}

	// Ensure MSI is on, and switch down to that range...
	RCC->CR |= (1 << 3) | 1; // Also enable range selection here
	// Can't modify MSI range when it's on and not ready
	while (!(RCC->CR & (1 << 1)));
	RCC->CR &= ~(0xf << 4);
	RCC->CR |= (_msi_range << 4);

	RCC->CFGR &= ~(0x3 << 0); // Set MSI as clock source

	HSEM.release(CFG_HW_RCC_SEMID, 0);

	
	SCB->SCR &= ~(1 << 2); // SLEEPDEEP must be clear
	// LoLoLoLO no lprun/lpsleep support for radios!
//	PWR->CR1 |= (1 << 14); // Set regulator low power mode
}

void post_lpsleep(void)
{
	// Restore regulator power
//	PWR->CR1 &= ~(1 << 14);
//	while (PWR->SR2 & (1 << 9)); // Wait for REGLPF

	HSEM.release(CFG_HW_ENTRY_STOP_MODE_SEMID, 0);
	while (!HSEM.get_lock_1step(CFG_HW_RCC_SEMID));
	RCC->CR = save_data.cr;
	RCC->CFGR = save_data.cfgr;
	RCC->SMPSCR = save_data.smpscr;
	HSEM.release(CFG_HW_RCC_SEMID, 0);
}

/* handles the "requirements" of shutting down with c2, copied from ble_heartrateFreertos demo
 * Also, "When the RF system is enabled the HSI16 must be selected as system
 * clock after wakeup from Stop modes." (From RM0434rev9, section 8.2.2 HSI16 clock)
 */
static void _c2_pre_stop(void) {
	// TODO? Should I wait until I get it before reading the values?
	save_data.cr = RCC->CR;
	save_data.cfgr = RCC->CFGR;
	save_data.smpscr = RCC->SMPSCR;
	while (!HSEM.get_lock_1step(CFG_HW_RCC_SEMID));
	if (!HSEM.get_lock_1step(CFG_HW_ENTRY_STOP_MODE_SEMID)) {
		// Check C2DS and C2SBF
		if ((PWR->EXTSCR & (1<<15))  || (PWR->EXTSCR & (1<<10))) {
			HSEM.release(CFG_HW_ENTRY_STOP_MODE_SEMID, 0);
			_enable_hsi();
		}
	} else {
		_enable_hsi();
	}
	HSEM.release(CFG_HW_RCC_SEMID, 0);
}



static void _c2_post_stop(void) {
	HSEM.release(CFG_HW_ENTRY_STOP_MODE_SEMID, 0);
	while (!HSEM.get_lock_1step(CFG_HW_RCC_SEMID));
	RCC->CR = save_data.cr;
	RCC->CFGR = save_data.cfgr;
	RCC->SMPSCR = save_data.smpscr;
	HSEM.release(CFG_HW_RCC_SEMID, 0);
}

void pre_stop(uint32_t stop_mode)
{
	//		printf("stop pre %lu\n", _mode);
//	_rcc_cr = RCC->CR;
//	_rcc_cfgr = RCC->CFGR;
	_c2_pre_stop();
#if defined(STM32WB)
	// DO _not_ use PWR.set_lpms_c2 here, the wireless fw blob will do
	// that itself.  you _can_ call that ahead of time to set a system
	// startup value, (and you must do that for single core apps)
	// but don't call it here all the time!
#endif

#if 0  // TODO measure this? (but need to then restore properly for lpuart and leds and shit?
	led_g.off();
	// This section is in some demos, we're doing it for now just to try for completeness
	RCC.enable(rcc::GPIOA);
	RCC.enable(rcc::GPIOB);
	RCC.enable(rcc::GPIOC);
	RCC.enable(rcc::GPIOD);
	GPIOA->MODER = 0xffffffff;
	GPIOB->MODER = 0xffffffff;
	GPIOC->MODER = 0xffffffff;
	GPIOD->MODER = 0xffffffff;
	GPIOA->PUPDR = 0;
	GPIOB->PUPDR = 0;
	GPIOC->PUPDR = 0;
	GPIOD->PUPDR = 0;
	RCC.disable(rcc::GPIOA);
	RCC.disable(rcc::GPIOB);
	RCC.disable(rcc::GPIOC);
	RCC.disable(rcc::GPIOD);
#endif

	// This doesn't _seem_ to matter if you unplug the debugger,
	// and doesn't help even if you leave it in, but it's in the errata,
	// so maybe I just got lucky in my testsm, better leave it in.
#if defined(STM32WB) && 1
	// Separate section for errata on stop mode and debug
	EXTI->IMR2 &= ~(1<<(48-32));
	EXTI->C2IMR2 &= ~(1<<(48-32));
#endif
	
	PWR.set_lpms(stop_mode);
	// Enable SLEEPDEEP
	SCB->SCR |= (1 << 2);
}

void post_stop(void)
{
	//		printf("stop post\n");
	// 3. We wakeup with either HSI16 or MSI depending on RCC->CFGR:STOPWUCK...
	// we're just going to to with "restore RCC->CR and RCC-CFGR....
//	RCC->CR = _rcc_cr;
//	RCC->CFGR = _rcc_cfgr;
	_c2_post_stop();
}

extern "C" {

	void myPreSleepFunction(TickType_t xModifiableIdleTime)
	{
		while (!(LPUART1->ISR & (1<<6)));
//		pre_lpsleep(0);
//		pre_stop(2);
	}

	void myPostSleepFunction(const TickType_t xExpectedIdleTime)
	{
//		post_lpsleep();
//		post_stop();
	}
}
