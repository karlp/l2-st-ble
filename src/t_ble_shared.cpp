/*
 * Shared bluetooth code for running the middleware....
 */


#include <exti/exti.h>
#include <interrupt/interrupt.h>
#include <rcc/rcc.h>
#include <wpan/ipcc.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <ble.h>
#include <tl.h>
#include <hci_tl.h>
#include <shci_tl.h>
#include <shci.h>
#include "tl_dbg_conf.h"

#include "t_ble.h"

#define POOL_SIZE (CFG_TLBLE_EVT_QUEUE_LENGTH*4U*DIVC(( sizeof(TL_PacketHeader_t) + TL_BLE_EVENT_FRAME_SIZE ), 4U))

PLACE_IN_SECTION("MB_MEM2") ALIGN(4) static uint8_t EvtPool[POOL_SIZE];
PLACE_IN_SECTION("MB_MEM2") ALIGN(4) static TL_CmdPacket_t SystemCmdBuffer;
PLACE_IN_SECTION("MB_MEM2") ALIGN(4) static uint8_t SystemSpareEvtBuffer[sizeof(TL_PacketHeader_t) + TL_EVT_HDR_SIZE + 255U];
PLACE_IN_SECTION("MB_MEM2") ALIGN(4) static uint8_t BleSpareEvtBuffer[sizeof(TL_PacketHeader_t) + TL_EVT_HDR_SIZE + 255];

PLACE_IN_SECTION("MB_MEM1") ALIGN(4) static TL_CmdPacket_t BleCmdBuffer;

SemaphoreHandle_t MtxShciId;
SemaphoreHandle_t SemShciId;
TaskHandle_t ShciUserEvtProcessId;

SemaphoreHandle_t MtxHciId;
SemaphoreHandle_t SemHciId;
TaskHandle_t HciUserEvtProcessId;

static void _tune_hse(void)
{
	// TODO - some demos from ST do this HSE tuning....
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



	// FIXME - this is wrong, they apparently re-write the struct downwards,
	// and have a function to scan backwards looking for a valid block...
	//	if (STM32::Calibration::OTP_ID0.hse_tuning) {
	//		
	//	}
	// on my board, it has one entry, and "id 0 therefore implies hse tuning of 0x17...
	// and this is what the ST demo app does.
	/* WARNING MUST BE CALLED WITH HSE OFF! */
	int tune = 0x17;
	RCC->HSECR = 0xcafecafe; // unlock
	RCC->HSECR |= (tune & 0x3f) << 8;
}

/**
 * you probably want something better here, but it's enough for now.
 */
static void Error_Handler(void)
{
	__disable_irq();
	while (1) {
	}
}

static void APPE_SysStatusNot(SHCI_TL_CmdStatus_t status)
{
	switch (status) {
	case SHCI_TL_CmdBusy:
		xSemaphoreTake(MtxShciId, portMAX_DELAY);
		break;

	case SHCI_TL_CmdAvailable:
		xSemaphoreGive(MtxShciId);
		break;

	default:
		break;
	}
	return;
}

// shared

static void BLE_StatusNot(HCI_TL_CmdStatus_t status)
{
	switch (status) {
	case HCI_TL_CmdBusy:
		xSemaphoreTake(MtxHciId, portMAX_DELAY);
		break;

	case HCI_TL_CmdAvailable:
		xSemaphoreGive(MtxHciId);
		break;

	default:
		break;
	}
	return;
}

static void ShciUserEvtProcess(void *argument)
{
	void(*UserEvtRx)(void* pData) = (void(*)(void*))argument;
	printf("SHCI starting\n");

	TL_MM_Config_t tl_mm_config;
	SHCI_TL_HciInitConf_t SHci_Tl_Init_Conf;
	/**< Reference table initialization */
	TL_Init();

	MtxShciId = xSemaphoreCreateMutex();
	SemShciId = xSemaphoreCreateBinary();
	//
	/**< System channel initialization */
	SHci_Tl_Init_Conf.p_cmdbuffer = (uint8_t*) & SystemCmdBuffer;
	SHci_Tl_Init_Conf.StatusNotCallBack = APPE_SysStatusNot;
	shci_init(UserEvtRx, (void*) &SHci_Tl_Init_Conf);

	/**< Memory Manager channel initialization */
	tl_mm_config.p_BleSpareEvtBuffer = BleSpareEvtBuffer;
	tl_mm_config.p_SystemSpareEvtBuffer = SystemSpareEvtBuffer;
	tl_mm_config.p_AsynchEvtPool = EvtPool;
	tl_mm_config.AsynchEvtPoolSize = POOL_SIZE;
	TL_MM_Init(&tl_mm_config);

	TL_Enable();

	printf("SHCI running\n");

	while (1) {
		xTaskNotifyWait(1, 1, NULL, portMAX_DELAY);
		printf("SCHI: event!\n");
		shci_user_evt_proc();
	}
}

static void HciUserEvtProcess(void *argument)
{
	void(*UserEvtRx)(void* pData) = (void(*)(void*))argument;
	HCI_TL_HciInitConf_t Hci_Tl_Init_Conf;

	MtxHciId = xSemaphoreCreateMutex();
	SemHciId = xSemaphoreCreateBinary();

	printf("Ble_Tl_Init: making hci\n");
	Hci_Tl_Init_Conf.p_cmdbuffer = (uint8_t*) & BleCmdBuffer;
	Hci_Tl_Init_Conf.StatusNotCallBack = BLE_StatusNot;
	hci_init(UserEvtRx, (void*) &Hci_Tl_Init_Conf);

	for (;;) {
		xTaskNotifyWait(1, 1, NULL, portMAX_DELAY);
		hci_user_evt_proc();
	}
}

void APP_BLE_Init(SHCI_C2_Ble_Init_Cmd_Packet_t *ble_init_cmd_packet, void(*user_func)(void*))
{
	/**
	 * Do not allow standby in the application
	 */
	//  printf("KLPM:app:DISABLE\n");
	// FIXME kkk  UTIL_LPM_SetOffMode(1 << CFG_LPM_APP_BLE, UTIL_LPM_DISABLE);

	/**
	 * Register the hci transport layer to handle BLE User Asynchronous Events
	 */
	xTaskCreate(HciUserEvtProcess, "hci_evt", configMINIMAL_STACK_SIZE * 8, (void*) user_func, tskIDLE_PRIORITY + 1, &HciUserEvtProcessId);


	/**
	 * Starts the BLE Stack on CPU2
	 */
	if (SHCI_C2_BLE_Init(ble_init_cmd_packet) != SHCI_Success) {
		Error_Handler();
	}

	// Say here, we split and let apps take over again?
	xTaskNotify(th_ble, (1 << 0), eSetBits);
	return;
}

void ble_pre(void)
{
	// ST demos reset IPCC and backup domain here, but.... let's not, and get bitten by that later.	
	_tune_hse();

	// ST does this before even system clock config.
	RCC.enable(rcc::HSEM);
	NVIC.enable(interrupt::irq::HSEM);
	// also, sets priority of pendsv to 15 and hsem to 5...
	NVIC.set_priority(interrupt::irq::HSEM, 5 << configPRIO_BITS);
	NVIC.set_priority(interrupt::exception::PendSV, 15 << configPRIO_BITS);

	// let's just set all the priorities up front here.
	NVIC.set_priority(interrupt::irq::IPCC_C1_RX, 5 << configPRIO_BITS);
	NVIC.set_priority(interrupt::irq::IPCC_C1_TX, 5 << configPRIO_BITS);

}

void task_ble_setup(void(*user_func)(void*))
{
	// MX_IPCC_Init()...
	RCC.enable(rcc::IPCC);
	NVIC.enable(interrupt::irq::IPCC_C1_RX);
	NVIC.enable(interrupt::irq::IPCC_C1_TX);
	// XXX: IPCC reset? yolo! I don't see what state we should bother with this in...
	//	IPCC->C1CR = 0;
	//	IPCC->C1MR = 0x3f << 16 | 0x3f;
	//	IPCC->C1SCR = 0x3f;
	// Ok, but seriously,
	IPCC->C1CR |= (1 << 16) | (1 << 0); // TXFIE | RXOIE

	// MX_RF_Init() is null
	// MX_RTC_Init()...
	// FIXME: XXX fuck it, I'm not sure what we're using this for, it can wait...
	// demo app seems to be using it for a wakeup timer? I'll have adc dma timer for that...
	// correct, fuck it off for now, we're going full power to get the stack working,
	// and we _only_ "need" the RTC to get a lower power wakeup


	// MX_APPE_Init()... // stop that, MX_APPE_Init isn't in the freertos demos!
	// CAN this go to top of bluetooth? (appears to still do exti/smps shits..
	//	PWR->CR5 &= ~(7 << 4); // 80mA startup current
	//	// Not sure why we need to do this, but... It talks about limiting rf output power?
	//	int32_t now = PWR->CR5 & 0x7; // reset calibration is at 1.5V
	//	now -= 2; // Attempt to get 1.4V
	//	if (now > 0 && now < 7) {
	//		PWR->CR5 |= now;
	//	} else {
	//		printf("yolo smps setting?!");
	//	}

	EXTI->IMR2 |= (1 << (36 - 32)) | (1 << (38 - 32)); // IPCC and HSEM wakeup EXTIs

	// XXX: more RTC init here, setting wakeup clocks.

	// SystemPower_Config()....
	// set hsi as sysclock after wakeup from stop?
	// ->  nope, we're never going to stop...
	// init "util_lpm_..." -> nope...
	// nope, we're running in full power mode until the stack works!
	//	PWR->C2CR1 &= ~(0x7);
	//	PWR->C2CR1 |= 0x4; // LPMS == Shutdown


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


	// Start the lowest level task, and pass our "user" startup task...
	// this will fire up C2, start hci task, and let us know when we can continue...
	xTaskCreate(ShciUserEvtProcess, "shci_evt", configMINIMAL_STACK_SIZE * 7, (void*) user_func, tskIDLE_PRIORITY + 1, &ShciUserEvtProcessId);

}

/** API impl called from STM32_WPAN shci_tl.c */
void shci_notify_asynch_evt(void* pdata)
{
	(void) pdata;
	BaseType_t whocares;
	xTaskNotifyFromISR(ShciUserEvtProcessId, 0, eNoAction, &whocares);
	return;
}

/** API impl called from STM32_WPAN hci_tl.c */
void hci_notify_asynch_evt(void* pdata)
{
	(void) pdata;
	BaseType_t whocares;
	xTaskNotifyFromISR(HciUserEvtProcessId, 0, eNoAction, &whocares);
	return;
}


extern "C" {

#if (DEBUG_BUFS == 1)

	void DbgTraceBuffer(const void *pBuffer, uint32_t u32Length, const char *strFormat, ...)
	{
		va_list vaArgs;
		uint32_t u32Index;
		va_start(vaArgs, strFormat);
		vprintf(strFormat, vaArgs);
		va_end(vaArgs);
		for (u32Index = 0; u32Index < u32Length; u32Index++) {
			printf(" %02X", ((const uint8_t *) pBuffer)[u32Index]);
		}
	}
#endif

}