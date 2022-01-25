/*
 * Include this file in your build, and you can have _function_ trace via SWO
 * This is in contrast to DWT pc sampling, (sampling based) or DWT exception
 * tracing, which shows interrupt calls and "return normal"
 * Designed to be compatible with orbuculum/orbstat.
 * See http://shadetail.com/blog/swo-instrumentation-first-tunes/
 */


#define TRACE_CHANNEL 30
#define DELAY_TIME 0 // mubes wat?

#include <cortex_m/debug.h>
// That header doesn't include nicely in C++?
//#include <cmsis_compiler.h>
// So, we just redeclare the few we need.

__attribute__((no_instrument_function))
static inline uint32_t __get_PRIMASK(void)
{
	uint32_t result;
	__asm volatile ("MRS %0, primask" : "=r" (result));
	return(result);
}

__attribute__((no_instrument_function))
static inline void __set_PRIMASK(uint32_t priMask)
{
	__asm volatile ("MSR primask, %0" : : "r" (priMask) : "memory");
}

__attribute__((no_instrument_function))
static inline void __disable_irq(void)
{
	__asm volatile ("cpsid i" : : : "memory");
}

// Yup, C linkage so that gcc finds them nicely.
extern "C" {
	__attribute__((no_instrument_function))
	void __cyg_profile_func_enter(void *this_fn, void *call_site)
	{
		if (!(ITM->TER[0] & (1 << TRACE_CHANNEL))) return;
		uint32_t oldIntStat = __get_PRIMASK();

		// This is not atomic, but by using the stack for
		//storing oldIntStat it doesn't matter
		__disable_irq();
		while (ITM->STIM[TRACE_CHANNEL].u32 == 0);

		// This is CYCCNT - number of cycles of the CPU clock  
		ITM->STIM[TRACE_CHANNEL].u32 = ((*((uint32_t *) 0xE0001004))&0x03FFFFFF) | 0x40000000;
		while (ITM->STIM[TRACE_CHANNEL].u32 == 0);
		ITM->STIM[TRACE_CHANNEL].u32 = (uint32_t) (call_site)&0xFFFFFFFE;
		while (ITM->STIM[TRACE_CHANNEL].u32 == 0);

		ITM->STIM[TRACE_CHANNEL].u32 = (uint32_t) this_fn & 0xFFFFFFFE;
		for (uint32_t d = 0; d < DELAY_TIME; d++) asm volatile ("NOP");

		__set_PRIMASK(oldIntStat);
	}

	__attribute__((no_instrument_function))
	void __cyg_profile_func_exit(void *this_fn, void *call_site)
	{
		if (!(ITM->TER[0] & (1 << TRACE_CHANNEL))) return;
		uint32_t oldIntStat = __get_PRIMASK();
		__disable_irq();
		while (ITM->STIM[TRACE_CHANNEL].u32 == 0);
		ITM->STIM[TRACE_CHANNEL].u32 = ((*((uint32_t *) 0xE0001004))&0x03FFFFFF) | 0x50000000;
		while (ITM->STIM[TRACE_CHANNEL].u32 == 0);
		ITM->STIM[TRACE_CHANNEL].u32 = (uint32_t) (call_site) & 0xFFFFFFFE;
		while (ITM->STIM[TRACE_CHANNEL].u32 == 0);
		ITM->STIM[TRACE_CHANNEL].u32 = (uint32_t) this_fn & 0xFFFFFFFE;
		for (uint32_t d = 0; d < DELAY_TIME; d++) asm volatile ("NOP");
		__set_PRIMASK(oldIntStat);
	}
}