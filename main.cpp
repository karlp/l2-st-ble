#include <cortex_m/debug.h>
#include <gpio/gpio.h>
#include <usart/usart.h>
#include <interrupt/interrupt.h>
#include <rcc/flash.h>
#include <rcc/rcc.h>
#include <usb/usb.h>
#include <usb/descriptor.h>
#include <timer/timer.h>

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

void setup_timer_adc_dma(void)
{

	RCC.enable(rcc::TIM2);
	const auto freq = 5;

	TIM2.ARR = (32000000 / freq) - 1;
	TIM2.CCER = 1 << 0;
	TIM2.DIER = 1 << 0; // Update interrupt plz
	TIM2.CR1 = 1 << 0; // Enable;

	interrupt_ctl.enable(interrupt::irq::TIM2);

}



int main() {
	krcc_init32();

	RCC.enable(rcc::GPIOB);

	auto led_r = GPIOB[1];
	auto led_g = GPIOB[0];
	auto led_b = GPIOB[5];

	led_r.set_mode(Pin::Output);
	led_g.set_mode(Pin::Output);
	led_b.set_mode(Pin::Output);
	
	setup_timer_adc_dma();

	int i = 0;
	while (1) {
		for (unsigned int k = 0; k < 0x70000; k++) {
			asm volatile ("nop");
		}
		i++;
	        ITM->STIM[0] = 'a' + (i%26);
		led_r.set(i & (1<<0));
		led_g.set(i & (1<<1));
		led_b.set(i & (1<<2));

	}
	return 0;
}
