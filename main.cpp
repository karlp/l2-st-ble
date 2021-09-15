#include <adc/adc_f3.h>
#include <cortex_m/debug.h>
#include <dma/dma.h>
#include <dma/dmamux.h>
#include <gpio/gpio.h>
#include <interrupt/interrupt.h>
#include <rcc/flash.h>
#include <rcc/rcc.h>
#include <timer/timer.h>
#include <usart/usart.h>
#include <usb/usb.h>
#include <usb/descriptor.h>

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

static uint16_t adc_buf[5];
static volatile uint16_t kdata[1024];
static volatile int kindex = 0;
static volatile int kinteresting = 0;

void adc_set_sampling_all(int sampling) {
	uint32_t reg = 0;
	for (int i = 0; i < 10; i++) {
		reg |= (sampling << (3*i));
	}
	ADC1.SMPR1 = reg;
	ADC1.SMPR2 = reg;  // extra bits? yolo!
}

void setup_timer_adc_dma(void)
{

	RCC.enable(rcc::TIM2);
	const auto freq = 5;

	TIM2.ARR = (32000000 / freq) - 1;
	TIM2.CR2 = (2<<4); // Master mode update event, will be used by ADC eventually
	TIM2.CCER = 1 << 0;
	// TIM2.DIER = 1 << 0; // Update interrupt plz

	interrupt_ctl.enable(interrupt::irq::TIM2);


	// setup DMA first...
	RCC.enable(rcc::DMAMUX1);
	RCC.enable(rcc::DMA1);

	// Use DMA mux channel 0 / DMA Channel 1for ADC
	DMAMUX1.CCR[0] = 5;
	DMA1.reg.C[0].CR = 0
		| (2<<10) // msize 16bit
		| (2<<8) // psize 16bit
		| (1<<7) // minc plz
		| (1<<5) // circ plz
		| (1<<1) // TC irq plz
		;
	DMA1.reg.C[0].NDTR = 5;
	DMA1.reg.C[0].MAR = (uint32_t)&adc_buf;
	DMA1.reg.C[0].PAR = (uint32_t)&(ADC1.DR);
	DMA1.reg.C[0].CR |= 1; // enable DMA  // I _believe_ this won't do anythign yet, but might need to move timer enable to the end? that's the driver of it all...


	// Setup ADC as well
	RCC.enable(rcc::ADC1);
	ADC1.CR = (1<<28);  // turn off deep power down (bit 29) and enables vreg

	// If you have calibration from "earlier" apply it, otherwise...
	uint32_t calfact = 0;
	if (calfact) {
		// TODO - I think ADEN must be turned on here first.
		ADC1.CALFACT = calfact;
	} else {
		ADC1.CR |= (1<<31);
		while (ADC1.CR & (1<<31))
			;
		calfact = ADC1.CALFACT; // nominally, save them
	}
	// nominally, 4 clock cycles required between CAL finishing and before we can turn on CR, should be ok....

	// clear adcrdy flag, aden=1, wait til adcrdy flag...
	ADC1.ISR = 1;
	ADC1.CR |= 1;
	while (!(ADC1.ISR & (1<<0)))
		;

	// turn on temp sensor and vrefint
	ADC_COMMON.CCR |= (1<<23) | (1<<22);

	adc_set_sampling_all(4); // 47.5 clocks on all for starters.

	// I want ext11, which is tim2 trgo
	ADC1.CFGR = (1<<31)  // LEave JQDIS
		| (1 << 10) // EXTEN rising edge
		| (11<<6) // EXTI11 for tim2 trgo
		| (3) // DMA circular + DMA enable
		;

	// Sequences are silly, but so be it...
	ADC1.SQR1 = 5-1;  // 5 conversions first.

	ADC1.SQR1 |= (1<<(6*1));
	ADC1.SQR1 |= (2<<(6*2));
	ADC1.SQR1 |= (6<<(6*3));  // external 1,2,6
	ADC1.SQR1 |= (17<<(6*4)); // temp sensor
	ADC1.SQR2 = (18 << (6*0)); // vrefint

	// Finally, start the timer that is going to do the counting....
	TIM2.CR1 = 1 << 0; // Enable;


}

auto led_r = GPIOB[1];
auto led_g = GPIOB[0];
auto led_b = GPIOB[5];

template <>
void interrupt::handler<interrupt::irq::TIM2>() {
	TIM2.SR = ~(1<<0); // Clear UIF 
	led_r.toggle();
}

template <>
void interrupt::handler<interrupt::irq::DMA1_CH1>() {
	if (DMA1.reg.ISR & (1<<1)) { // CH1 TCIF
		DMA1.reg.IFCR = (1<<1);
		kdata[kindex++] = adc_buf[kinteresting];
		if (kindex > 1024) {
			kindex = 0;
		}
	}
	if (DMA1.reg.ISR & (1<<3)) {
		// Errors...
		DMA1.reg.IFCR = (1<<3); // clear it at least.
		ITM->STIM[0] = '!';
	}
}


int main() {
	krcc_init32();

	RCC.enable(rcc::GPIOB);


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
//		led_r.set(i & (1<<0));
		led_g.set(i & (1<<1));
		led_b.set(i & (1<<2));

	}
	return 0;
}
