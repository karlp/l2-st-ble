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

auto led_r = GPIOB[1];
auto led_g = GPIOB[0];
auto led_b = GPIOB[5];
auto const ADC_CH_VREFINT = 0;
auto const ADC_CH_TEMPSENSOR = 17;

// 500 samples per channel, at sample rate is 100ms chunks
// Idea is that ram usage is better than running more often.
auto const ADC_DMA_LOOPS = 500;
auto const ADC_CHANNELS_FILTERED = 3;

enum task_kadc_notifications {
	dma_half,
	dma_full,
	dma_error
};

enum ADC_SAMPLING_CYCLES {
	ADC_SAMPLING_2_5,
	ADC_SAMPLING_6_5,
	ADC_SAMPLING_12_5,
	ADC_SAMPLING_24_5,
	ADC_SAMPLING_47_5,
	ADC_SAMPLING_92_5,
	ADC_SAMPLING_247_5,
	ADC_SAMPLING_640_5,
};

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

static volatile uint16_t adc_buf[ADC_CHANNELS_FILTERED*ADC_DMA_LOOPS];
static volatile uint16_t kdata[1024];
static volatile int kindex = 0;
static volatile int kinteresting = 0;
static volatile int kirq_count = 0;

static TaskHandle_t th_kadc;
static TaskHandle_t th_ble;

// >>> freq = 5000
// >>> b,a = scipy.signal.iirfilter(1, 20/freq, btype="highpass")
// >>> filter_model.dump_arm_cmsis(b,a)
float32_t filter_coeffs[5] = {
	0.99375596495365714489,
       -0.99375596495365714489,
	0.00000000000000000000,
	0.98751192990731440080,
       -0.00000000000000000000,
};

class KAdcFilter {
private:
	float32_t filter_state[4];
	arm_biquad_casd_df1_inst_f32 filter_instance;

public:
	// TODO - can this not just be a constructor?
	/// Initialize filters and internal states
	/// \param filter_coeffs pointer to an array of properly formed filter co-efficients.
	/// \param num_stages biquad stages.  coeffs must be n*5 long.
	void init(const float *filter_coeffs, int num_stages)
	{
		arm_biquad_cascade_df1_init_f32(&filter_instance, num_stages, filter_coeffs, filter_state);
	}

	float feed(const float &input) {
		float out;
		arm_biquad_cascade_df1_f32(&filter_instance, &input, &out, 1);
		return out;
	}
};

// FIXME - place holder for bluetooth task state object...
struct ts_ble_t {
	
};
struct ts_ble_t task_state_ble;


struct ts_adc_t {
	KAdcFilter filter[ADC_CHANNELS_FILTERED];

	float sum_squares[ADC_CHANNELS_FILTERED];
};

struct ts_adc_t task_state_adc;

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

static void adc_setup_with_dma(void) {
	uint32_t before = DWT->CYCCNT;
	// setup DMA first...
	RCC.enable(rcc::DMA1);
	RCC.enable(rcc::DMAMUX1);
	// Use DMA mux channel 0 / DMA Channel 1for ADC
	DMAMUX1->CCR[0] = 5;

	DMA1->C[0].NDTR = ADC_CHANNELS_FILTERED * ADC_DMA_LOOPS;
	DMA1->C[0].MAR = (uint32_t)&adc_buf;
	DMA1->C[0].PAR = (uint32_t)&(ADC1.DR);
	DMA1->C[0].CR = 0
		| (1<<10) // msize 16bit
		| (1<<8) // psize 16bit
		| (1<<7) // minc plz
		| (1<<5) // circ plz
		| (7<<1) // TE+TC+HT irqs plz
		| 1 // enable, will do nothing without requests
		;
	interrupt_ctl.enable(interrupt::irq::DMA1_CH1);

	// Turn on the ADC then we'll do other things while it's waking up.
	RCC.enable(rcc::ADC1);
	// Make sure we give it a clock!
	RCC->CCIPR |= (3<<28); // Use sysclk for now. We may want to run it slower later.
	// and prescale to 32MHZ from 64.
	ADC_COMMON1.CCR |= (1<<18);
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
	ADC_COMMON1.CCR |= (1<<23) | (1<<22);

	// FIXME - recalculate based on final ADC clocks:
	// at 32Mhz, 4us = 128, 5us = 160
	adc_set_sampling(ADC_SAMPLING_47_5); //4 == 47.5 clocks on all for starters. // 1.5usecs
	adc_set_sampling(ADC_CH_VREFINT, ADC_SAMPLING_247_5);
	adc_set_sampling(ADC_CH_TEMPSENSOR, ADC_SAMPLING_247_5); // 7.7usecs (still glitchy though :(

	// I want ext11, which is tim2 trgo
	ADC1.CFGR = (1<<31)  // Leave JQDIS
		| (1 << 10) // EXTEN rising edge
		| (11<<6) // EXTI11 for tim2 trgo
		| (3) // DMA circular + DMA enable
		;

	// Oversampling configuration, for the regular channels.
	auto ovsr = 0;
	auto ovss = 0;
	ADC1.CFGR2 = (1<<10) | (ovsr<<2) | (1<<0); // ROVSM | OVSR | ROVSE
	ADC1.CFGR2 |= (ovss<<5);

	// Set ADC to start when it starts getting triggers
	ADC1.CR |= (1<<2);

	// Sequences are silly, but so be it...
	ADC1.SQR1 = 3-1;  // 3 conversions first.

	ADC1.SQR1 |= (1<<(6*1));
	ADC1.SQR1 |= (2<<(6*2));
	ADC1.SQR1 |= (6<<(6*3));  // external 1,2,6

	// Injected afterwards
	ADC1.JSQR = 2-1;  // sequence of two injected
	ADC1.JSQR |= (ADC_CH_TEMPSENSOR<<8);
	ADC1.JSQR |= (ADC_CH_VREFINT<<14);
}

// TODO - calibrate based on vref int, we know from experience this improves things
//static float compensate_vref(uint16_t adc_count, uint16_t vref_count)
//{
//	// could read from system rom every call actually
//	float ret = adc_count * VREFINT_CAL;
//	return (ret / vref_count);
//}


void adc_process_samples(ts_adc_t* ts, auto i){
	for (int k = 0; k < ADC_CHANNELS_FILTERED; k++) {
		float f;
		uint16_t raw = adc_buf[(i * ADC_CHANNELS_FILTERED) + k];
		//f = compensate_vref(raw, adc_buf[(i*ADC_CHANNELS_FILTERED) + 4]);
		f = raw;
		float out = ts->filter[k].feed(f);
		ts->sum_squares[k] += (out * out);
		if (kinteresting >= 0 && kinteresting == k) {
			ITM->stim_blocking(1, raw);
			ITM->stim_blocking(4, out);
#if defined(SAVE_TO_SECOND_BUFFER)
			kdata[kindex++] = raw;
			if (kindex >= 1024) {
				kindex = 0;
			}
#endif
		}
	}
}

static void prvTask_kadc(void *pvParameters)
{
	struct ts_adc_t *ts = (struct ts_adc_t*)pvParameters;

	// setup adc filters
	for (auto i = 0; i < ADC_CHANNELS_FILTERED; i++) {
		// DC Cut for current channels
		ts->filter[i].init(filter_coeffs, 1);
		ts->sum_squares[i] = 0.0f;
	}

	led_r.set_mode(Pin::Output);


	RCC.enable(rcc::TIM2);
	const auto freq = 5000;

	const auto tim_clk = 64000000;
	TIM2->ARR = (tim_clk / freq) - 1;
	TIM2->CR2 = (2<<4); // Master mode update event, will be used by ADC eventually
	TIM2->CCER = 1 << 0;

	adc_setup_with_dma();

	// Finally, start the timer that is going to do the counting....
	TIM2->CR1 = 1 << 0; // Enable;

	int stats_dma_err = 0;
	uint32_t flags;
	int index = 0;
	while (1) {
		xTaskNotifyWait(0, UINT32_MAX, &flags, portMAX_DELAY);
		if (flags & (1<<dma_half)) {
			for (auto i = 0; i < ADC_DMA_LOOPS / 2; i++) {
				adc_process_samples(ts, i);
			}
			index += ADC_DMA_LOOPS / 2;

		}
		if (flags & (1<<dma_full)) {
			for (auto i = ADC_DMA_LOOPS / 2; i < ADC_DMA_LOOPS; i++) {
				adc_process_samples(ts, i);
			}
			index += ADC_DMA_LOOPS / 2;
		}
		if (flags & (1<<dma_error)) {
			stats_dma_err++;
			printf("DMA Error: %d!\n", stats_dma_err);
		}
		auto my_n = ADC_DMA_LOOPS * 2; // 100ms per entire buffer.
		if (index == my_n) {
			index = 0;
			for (auto i = 0; i < ADC_CHANNELS_FILTERED; i++) {
				float rms = sqrtf(ts->sum_squares[i] / my_n);
				ts->sum_squares[i] = 0.0f;
				if (kinteresting >= 0 && kinteresting == i) {
					ITM->stim_blocking(9, rms);
					printf("Ch%d: %ld\n", i, (int32_t)(rms * 10000));
				}
			}
		}
	}


}




//template <>
//void interrupt::handler<interrupt::irq::TIM2>() {
//	TIM2->SR = ~(1<<0); // Clear UIF
////	led_r.toggle();
//}

#if defined(TEMPLATES_ARE_COOL)
template <typename T1>
constexpr auto dma_flag_teif(T1 channel) { return (8<<(channel * 4)); }
template <typename T1>
constexpr auto dma_flag_htif(T1 channel) { return (4<<(channel * 4)); }
template <typename T1>
constexpr auto dma_flag_tcif(T1 channel) { return (2<<(channel * 4)); }
template <typename T1>
constexpr auto dma_flag_gif(T1 channel) { return (1<<(channel * 4)); }
#else
constexpr auto dma_flag_teif(auto channel) { return (8<<(channel * 4)); }
constexpr auto dma_flag_htif(auto channel) { return (4<<(channel * 4)); }
constexpr auto dma_flag_tcif(auto channel) { return (2<<(channel * 4)); }
constexpr auto dma_flag_gif(auto channel) { return (1<<(channel * 4)); }
#endif

template <>
void interrupt::handler<interrupt::irq::DMA1_CH1>() {
	uint32_t before = DWT->CYCCNT;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	kirq_count++;
	if (DMA1->ISR & dma_flag_htif(0)) {
		DMA1->IFCR = dma_flag_htif(0);
		// TODO - notify processing task to do first chunk
		xTaskNotifyFromISR(th_kadc, (1<<dma_half), eSetBits, &xHigherPriorityTaskWoken);
	}

	if (DMA1->ISR & dma_flag_tcif(0)) { // CH1 TCIF
		DMA1->IFCR = dma_flag_tcif(0);
		xTaskNotifyFromISR(th_kadc, (1<<dma_full), eSetBits, &xHigherPriorityTaskWoken);
		// Allow turning off this processing at runtime.
	}
	if (DMA1->ISR & dma_flag_teif(0)) {
		// Errors...
		DMA1->IFCR = dma_flag_teif(0); // clear it at least.
		xTaskNotifyFromISR(th_kadc, (1<<dma_error), eSetBits, &xHigherPriorityTaskWoken);
		ITM->STIM[0].u8 = '!';
	}
	ITM->STIM[2].u32 = DWT->CYCCNT - before;
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

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
		ITM->stim_blocking(3, (uint16_t)kirq_count);
		kirq_count = 0;
//		printf("testing: %d\n", i);
	}
}

static void prvTaskTemperature(void *pvParameters)
{
	(void)pvParameters;
	KAdcFilter filter_vref;
	KAdcFilter filter_temp;

//	>>> freq = 4
//	>>> b,a = signal.iirfilter(1, 2/freq/2, btype="lowpass")
//	>>> filter_model.dump_arm_cmsis(b,a)
	float32_t filter_coeffs[5] = {
		0.29289321881345248277f,
		0.29289321881345248277f,
		0.00000000000000000000f,
		0.41421356237309508996f,
		-0.00000000000000000000f,
	};
	filter_vref.init(filter_coeffs, 1);
	filter_temp.init(filter_coeffs, 1);

	// Precalculate.
	float factor = (STM32::Calibration::TS_CAL2_TEMP - STM32::Calibration::TS_CAL1_TEMP) * 1.0f /
		(STM32::Calibration::TS_CAL2 - STM32::Calibration::TS_CAL1) * 1.0f;

	TickType_t xLastWakeTime = xTaskGetTickCount();
	while (1) {
		xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(250));
		// sw trigger injected
		ADC1.CR |= (1<<3);
		// It won't be ready immediately, but it will be ok after the
		// first time, we'll use the last ones...
		uint16_t tempsens_raw = ADC1.JDR1;
		float tempsens_rawf = tempsens_raw;
		uint16_t vrefint_raw = ADC1.JDR2;
		float vrefint_rawf = vrefint_raw;
		float temp_filter = filter_temp.feed(tempsens_rawf);
		float vref_filter = filter_vref.feed(vrefint_rawf);

		const float vdd = 3.3f;  // could use vrefint to adjust this... but...
		float corrected_tdata = temp_filter * vdd / STM32::Calibration::TS_CAL_VOLTAGE;
		float temp = factor * (corrected_tdata - STM32::Calibration::TS_CAL1) + 30.0f;
		int32_t tempi = temp * 1000;
//		printf("temp: raw: %u filtered: %ld, vref: %ld\n", tempsens_raw, tempi, (int32_t)(vref_filter*1000));
		ITM->stim_blocking(6, tempsens_raw);
		ITM->stim_blocking(7, vrefint_raw);
		ITM->stim_blocking(8, temp);
		
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
	xTaskCreate(prvTask_kadc, "kadc", configMINIMAL_STACK_SIZE*3, &task_state_adc, tskIDLE_PRIORITY + 1, &th_kadc);
	xTaskCreate(prvTaskTemperature, "ktemp", configMINIMAL_STACK_SIZE*3, NULL, tskIDLE_PRIORITY + 1, NULL);

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

