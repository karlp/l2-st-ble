/*
 * top level ble task code.... just to not get _toooo_ much clutter
 * 
 * We're ripped off large chunks of the "app_ble.c" from the HRS_FREERTOS demo from st,
 * at least getting started.
 */

#include <cstdio>
#include <limits.h>
#include <stdint.h>

#include <cal/cal.h>
#include <cortex_m/debug.h>
#include <dma/dma.h>
#include <exti/exti.h>
#include <gpio/gpio.h>
#include <interrupt/interrupt.h>
#include <rcc/rcc.h>
#include <timer/timer.h>
#include <wpan/ipcc.h>

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

#include <ble.h>
#include <tl.h>
#include <hci_tl.h>
#include <shci_tl.h>
#include <shci.h>

#include "dis_app.h"


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
//	if (STM32::Calibration::OTP_ID0.hse_tuning) {
//		
//	}
	// on my board, it has one entry, and "id 0 therefore implies hse tuning of 0x17...
	// and this is what the ST demo app does.
	_rcc_tune_hse(0x17);
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
/* Private typedef -----------------------------------------------------------*/

/**
 * security parameters structure
 */
typedef struct _tSecurityParams
{
  /**
   * IO capability of the device
   */
  uint8_t ioCapability;

  /**
   * Authentication requirement of the device
   * Man In the Middle protection required?
   */
  uint8_t mitm_mode;

  /**
   * bonding mode of the device
   */
  uint8_t bonding_mode;

  /**
   * this variable indicates whether to use a fixed pin
   * during the pairing process or a passkey has to be
   * requested to the application during the pairing process
   * 0 implies use fixed pin and 1 implies request for passkey
   */
  uint8_t Use_Fixed_Pin;

  /**
   * minimum encryption key size requirement
   */
  uint8_t encryptionKeySizeMin;

  /**
   * maximum encryption key size requirement
   */
  uint8_t encryptionKeySizeMax;

  /**
   * fixed pin to be used in the pairing process if
   * Use_Fixed_Pin is set to 1
   */
  uint32_t Fixed_Pin;

  /**
   * this flag indicates whether the host has to initiate
   * the security, wait for pairing or does not have any security
   * requirements.\n
   * 0x00 : no security required
   * 0x01 : host should initiate security by sending the slave security
   *        request command
   * 0x02 : host need not send the clave security request but it
   * has to wait for paiirng to complete before doing any other
   * processing
   */
  uint8_t initiateSecurity;
}tSecurityParams;

/**
 * global context
 * contains the variables common to all
 * services
 */
typedef struct _tBLEProfileGlobalContext
{

  /**
   * security requirements of the host
   */
  tSecurityParams bleSecurityParam;

  /**
   * gap service handle
   */
  uint16_t gapServiceHandle;

  /**
   * device name characteristic handle
   */
  uint16_t devNameCharHandle;

  /**
   * appearance characteristic handle
   */
  uint16_t appearanceCharHandle;

  /**
   * connection handle of the current active connection
   * When not in connection, the handle is set to 0xFFFF
   */
  uint16_t connectionHandle;

  /**
   * length of the UUID list to be used while advertising
   */
  uint8_t advtServUUIDlen;

  /**
   * the UUID list to be used while advertising
   */
  uint8_t advtServUUID[100];

}BleGlobalContext_t;

// This enum was originally in app_ble.h, but not sure I need that file at all?
    typedef enum
    {
      APP_BLE_IDLE,
      APP_BLE_FAST_ADV,
      APP_BLE_LP_ADV,
      APP_BLE_SCAN,
      APP_BLE_LP_CONNECTING,
      APP_BLE_CONNECTED_SERVER,
      APP_BLE_CONNECTED_CLIENT
    } APP_BLE_ConnStatus_t;
    
typedef struct
{
  BleGlobalContext_t BleApplicationContext_legacy;
  APP_BLE_ConnStatus_t Device_Connection_Status;

  /**
   * ID of the Advertising Timeout
   */
  uint8_t Advertising_mgr_timer_Id;

}BleApplicationContext_t;

#define POOL_SIZE (CFG_TLBLE_EVT_QUEUE_LENGTH*4U*DIVC(( sizeof(TL_PacketHeader_t) + TL_BLE_EVENT_FRAME_SIZE ), 4U))

PLACE_IN_SECTION("MB_MEM2") ALIGN(4) static uint8_t EvtPool[POOL_SIZE];
PLACE_IN_SECTION("MB_MEM2") ALIGN(4) static TL_CmdPacket_t SystemCmdBuffer;
PLACE_IN_SECTION("MB_MEM2") ALIGN(4) static uint8_t SystemSpareEvtBuffer[sizeof(TL_PacketHeader_t) + TL_EVT_HDR_SIZE + 255U];
PLACE_IN_SECTION("MB_MEM2") ALIGN(4) static uint8_t BleSpareEvtBuffer[sizeof(TL_PacketHeader_t) + TL_EVT_HDR_SIZE + 255];

PLACE_IN_SECTION("MB_MEM1") ALIGN(4) static TL_CmdPacket_t BleCmdBuffer;
#define APPBLE_GAP_DEVICE_NAME_LENGTH 7
#define INITIAL_ADV_TIMEOUT            (pdMS_TO_TICKS(15*1000))
#define BD_ADDR_SIZE_LOCAL    6

static uint8_t bd_addr_udn[BD_ADDR_SIZE_LOCAL];


/**
*   Identity root key used to derive LTK and CSRK
*/
static const uint8_t BLE_CFG_IR_VALUE[16] = CFG_BLE_IRK;

/**
* Encryption root key used to derive LTK and CSRK
*/
static const uint8_t BLE_CFG_ER_VALUE[16] = CFG_BLE_ERK;

/**
 * These are the two tags used to manage a power failure during OTA
 * The MagicKeywordAdress shall be mapped @0x140 from start of the binary image
 * The MagicKeywordvalue is checked in the ble_ota application
 */
PLACE_IN_SECTION("TAG_OTA_END") const uint32_t MagicKeywordValue = 0x94448A29 ;
PLACE_IN_SECTION("TAG_OTA_START") const uint32_t MagicKeywordAddress = (uint32_t)&MagicKeywordValue;

PLACE_IN_SECTION("BLE_APP_CONTEXT") static BleApplicationContext_t BleApplicationContext;
PLACE_IN_SECTION("BLE_APP_CONTEXT") static uint16_t AdvIntervalMin, AdvIntervalMax;


static const char local_name[] = { AD_TYPE_COMPLETE_LOCAL_NAME ,'H','R','S','T','M'};
uint8_t  manuf_data[14] = {
    sizeof(manuf_data)-1, AD_TYPE_MANUFACTURER_SPECIFIC_DATA,
    0x01/*SKD version */,
    0x00 /* Generic*/,
    0x00 /* GROUP A Feature  */,
    0x00 /* GROUP A Feature */,
    0x00 /* GROUP B Feature */,
    0x00 /* GROUP B Feature */,
    0x00, /* BLE MAC start -MSB */
    0x00,
    0x00,
    0x00,
    0x00,
    0x00, /* BLE MAC stop */

};

uint8_t kustom_adv_data[30] = {
		sizeof(kustom_adv_data) -1, AD_TYPE_MANUFACTURER_SPECIFIC_DATA,
		0x99,0x09, /* 0x0999, eTactica assigned number */
		0xca, /* flags */
		0xfe, /* version */
		1,1,1, /* current ch1 (24bit) */
		2,2,2,
		3,3,3,
		6,6,6, /* voltage ch1 (24bit) */
		7,7,7,
		8,8,8,
		0xa,0xa, /* pf ch 1 (16bit) */
		0xb,0xb,
		0xc,0xc,
};


SemaphoreHandle_t MtxHciId;
SemaphoreHandle_t SemHciId;
SemaphoreHandle_t MtxShciId;
SemaphoreHandle_t SemShciId;
TaskHandle_t HciUserEvtProcessId;
TaskHandle_t ShciUserEvtProcessId;
TaskHandle_t AdvUpdateProcessId;
TimerHandle_t AdvMgrTimerId;


void ble_pre(void) {
	// ST demos reset IPCC and backup domain here, but.... let's not, and get bitten by that later.	
//	_tune_hse();

	// ST does this before even system clock config.
	RCC.enable(rcc::HSEM);
	NVIC.enable(interrupt::irq::HSEM);
	// also, sets priority of pendsv to 15 and hsem to 5...
	NVIC.set_priority(interrupt::irq::HSEM, 5<<configPRIO_BITS);
	NVIC.set_priority(interrupt::exception::PendSV, 15<<configPRIO_BITS);
	
	// let's just set all the priorities up front here.
	NVIC.set_priority(interrupt::irq::IPCC_C1_RX, 5<<configPRIO_BITS);
	NVIC.set_priority(interrupt::irq::IPCC_C1_TX, 5<<configPRIO_BITS);

}

static void Adv_Request(APP_BLE_ConnStatus_t New_Status)
{
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  uint16_t Min_Inter, Max_Inter;

  if (New_Status == APP_BLE_FAST_ADV)
  {
    Min_Inter = AdvIntervalMin;
    Max_Inter = AdvIntervalMax;
  }
  else
  {
    Min_Inter = CFG_LP_CONN_ADV_INTERVAL_MIN;
    Max_Inter = CFG_LP_CONN_ADV_INTERVAL_MAX;
  }

    /**
     * Stop the timer, it will be restarted for a new shot
     * It does not hurt if the timer was not running
     */
  xTimerStop(AdvMgrTimerId, 50*portTICK_PERIOD_MS); // TODO - check return value

    APP_DBG_MSG("First index in %d state \n", BleApplicationContext.Device_Connection_Status);

    if ((New_Status == APP_BLE_LP_ADV)
        && ((BleApplicationContext.Device_Connection_Status == APP_BLE_FAST_ADV)
            || (BleApplicationContext.Device_Connection_Status == APP_BLE_LP_ADV)))
    {
      /* Connection in ADVERTISE mode have to stop the current advertising */
    	/* (must stop ads before changing them, that's all) */
      //ret = aci_gap_set_non_discoverable();
    	ret = hci_le_set_advertise_enable(0);
      if (ret == BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("Successfully Stopped Advertising \n");
      }
      else
      {
        APP_DBG_MSG("Stop Advertising Failed , result: %d \n", ret);
      }
    }

    BleApplicationContext.Device_Connection_Status = New_Status;
    /* Start Fast or Low Power Advertising */
#if 0
    /* aci level command forcible includes flags and tx power and thigns we don't want. */
    ret = aci_gap_set_discoverable(
        ADV_IND,
        Min_Inter,
        Max_Inter,
        PUBLIC_ADDR,
        NO_WHITE_LIST_USE, /* use white list */
		0, NULL, /* local name */
		0, NULL, /* uuid lists */
        0,
        0);


    /* Update Advertising data */
    //ret = aci_gap_update_adv_data(sizeof(manuf_data), (uint8_t*) manuf_data);
    //ret = aci_gap_update_adv_data(sizeof(kustom_adv_data), (uint8_t*) kustom_adv_data);
#endif
    ret = hci_le_set_advertising_parameters(Min_Inter, Max_Inter, GAP_ADV_IND, PUBLIC_ADDR, 0, NULL, 3, 0);
    if (ret != BLE_STATUS_SUCCESS) {
    	printf("K: Failed hci le adv params: %d (%#x)\n", ret, ret);
    }
    ret = hci_le_set_advertising_data(sizeof(kustom_adv_data), kustom_adv_data);
    if (ret != BLE_STATUS_SUCCESS) {
    	printf("K: Failed hci le adv data: %d (%#x)\n", ret, ret);
    }
    ret = hci_le_set_advertise_enable(1);
    if (ret != BLE_STATUS_SUCCESS) {
    	printf("K: Failed hci start adv: %d (%#x)\n", ret, ret);
    }


    if (ret == BLE_STATUS_SUCCESS)
    {
      if (New_Status == APP_BLE_FAST_ADV)
      {
        APP_DBG_MSG("Successfully Start Fast Advertising \n" );
        /* Start Timer to STOP ADV - TIMEOUT */
	xTimerStart(AdvMgrTimerId, portMAX_DELAY);  // TODO - check return type, it shouldn't really ever fail though...
      }
      else
      {
        APP_DBG_MSG("Successfully Start Low Power Advertising \n");
      }
    }
    else
    {
      if (New_Status == APP_BLE_FAST_ADV)
      {
        APP_DBG_MSG("Start Fast Advertising Failed , result: %d \n", ret);
      }
      else
      {
        APP_DBG_MSG("Start Low Power Advertising Failed , result: %d \n", ret);
      }
    }

  return;
}


/** This is a complete re-implementation as we're not using hal/ll! */
const uint8_t* BleGetBdAddress( void )
{
//  uint8_t *otp_addr;
  const uint8_t *bd_addr;
  uint32_t udn;
  uint32_t company_id;
  uint32_t device_id;

  udn = STM32::Calibration::UID64_UDN;
  company_id = STM32::Calibration::UID64_PART >> 8;
  device_id = STM32::Calibration::UID64_PART & 0xff;

  // karl: if ST delivers parts that don't have the UID64 written, they need to fix their supply chain.
  // I don't need to handle this failure...
//  if(udn != 0xFFFFFFFF)
//  {
//    company_id = LL_FLASH_GetSTCompanyID();
//    device_id = LL_FLASH_GetDeviceID();

/**
 * Public Address with the ST company ID
 * bit[47:24] : 24bits (OUI) equal to the company ID
 * bit[23:16] : Device ID.
 * bit[15:0] : The last 16bits from the UDN
 * Note: In order to use the Public Address in a final product, a dedicated
 * 24bits company ID (OUI) shall be bought.
 */
    bd_addr_udn[0] = (uint8_t)(udn & 0x000000FF);
    bd_addr_udn[1] = (uint8_t)( (udn & 0x0000FF00) >> 8 );
    bd_addr_udn[2] = (uint8_t)device_id;
    bd_addr_udn[3] = (uint8_t)(company_id & 0x000000FF);
    bd_addr_udn[4] = (uint8_t)( (company_id & 0x0000FF00) >> 8 );
    bd_addr_udn[5] = (uint8_t)( (company_id & 0x00FF0000) >> 16 );

    bd_addr = (const uint8_t *)bd_addr_udn;
//  }
//  else
//  {
//    otp_addr = OTP_Read(0);
//    if(otp_addr)
//    {
//      bd_addr = ((OTP_ID0_t*)otp_addr)->bd_address;
//    }
//    else
//    {
//      bd_addr = M_bd_addr;
//    }
//  }

  return bd_addr;
}

void hci_notify_asynch_evt(void* pdata)
{
  (void)pdata;
  BaseType_t whocares;
  xTaskNotifyFromISR(HciUserEvtProcessId, 0, eNoAction, &whocares);
  return;
}


void shci_notify_asynch_evt(void* pdata)
{
  (void)pdata;
  BaseType_t whocares;
  xTaskNotifyFromISR(ShciUserEvtProcessId, 0, eNoAction, &whocares);
  return;
}


static void APPE_SysStatusNot( SHCI_TL_CmdStatus_t status )
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

static void BLE_UserEvtRx( void * pPayload )
{
  SVCCTL_UserEvtFlowStatus_t svctl_return_status;
  tHCI_UserEvtRxParam *pParam;

  pParam = (tHCI_UserEvtRxParam *)pPayload;

  svctl_return_status = SVCCTL_UserEvtRx((void *)&(pParam->pckt->evtserial));
  if (svctl_return_status != SVCCTL_UserEvtFlowDisable)
  {
    pParam->status = HCI_TL_UserEventFlow_Enable;
  }
  else
  {
    pParam->status = HCI_TL_UserEventFlow_Disable;
  }
}


static void BLE_StatusNot( HCI_TL_CmdStatus_t status )
{
  switch (status)
  {
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


SVCCTL_UserEvtFlowStatus_t SVCCTL_App_Notification( void *pckt )
{
  hci_event_pckt *event_pckt;
  evt_le_meta_event *meta_evt;
  evt_blecore_aci *blecore_evt;
  hci_le_phy_update_complete_event_rp0 *evt_le_phy_update_complete;
  uint8_t TX_PHY, RX_PHY;
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;

  event_pckt = (hci_event_pckt*) ((hci_uart_pckt *) pckt)->data;

  /* USER CODE BEGIN SVCCTL_App_Notification */

  /* USER CODE END SVCCTL_App_Notification */

  switch (event_pckt->evt)
  {
    case HCI_DISCONNECTION_COMPLETE_EVT_CODE:
    {
      hci_disconnection_complete_event_rp0 *disconnection_complete_event;
      disconnection_complete_event = (hci_disconnection_complete_event_rp0 *) event_pckt->data;

      if (disconnection_complete_event->Connection_Handle == BleApplicationContext.BleApplicationContext_legacy.connectionHandle)
      {
        BleApplicationContext.BleApplicationContext_legacy.connectionHandle = 0;
        BleApplicationContext.Device_Connection_Status = APP_BLE_IDLE;

        APP_DBG_MSG("\r\n\r** DISCONNECTION EVENT WITH CLIENT \n");
      }

      /* restart advertising */
      printf("K: restart fast adv\n");
      Adv_Request(APP_BLE_FAST_ADV);

      /* USER CODE BEGIN EVT_DISCONN_COMPLETE */

      /* USER CODE END EVT_DISCONN_COMPLETE */
    }

    break; /* HCI_DISCONNECTION_COMPLETE_EVT_CODE */

    case HCI_LE_META_EVT_CODE:
    {
      meta_evt = (evt_le_meta_event*) event_pckt->data;
      /* USER CODE BEGIN EVT_LE_META_EVENT */

      /* USER CODE END EVT_LE_META_EVENT */
      switch (meta_evt->subevent)
      {
        case HCI_LE_CONNECTION_UPDATE_COMPLETE_SUBEVT_CODE:
          APP_DBG_MSG("\r\n\r** CONNECTION UPDATE EVENT WITH CLIENT \n");

          /* USER CODE BEGIN EVT_LE_CONN_UPDATE_COMPLETE */

          /* USER CODE END EVT_LE_CONN_UPDATE_COMPLETE */
          break;
        case HCI_LE_PHY_UPDATE_COMPLETE_SUBEVT_CODE:
          APP_DBG_MSG("EVT_UPDATE_PHY_COMPLETE \n");
          evt_le_phy_update_complete = (hci_le_phy_update_complete_event_rp0*)meta_evt->data;
          if (evt_le_phy_update_complete->Status == 0)
          {
            APP_DBG_MSG("EVT_UPDATE_PHY_COMPLETE, status ok \n");
          }
          else
          {
            APP_DBG_MSG("EVT_UPDATE_PHY_COMPLETE, status nok \n");
          }

          ret = hci_le_read_phy(BleApplicationContext.BleApplicationContext_legacy.connectionHandle,&TX_PHY,&RX_PHY);
          if (ret == BLE_STATUS_SUCCESS)
          {
            APP_DBG_MSG("Read_PHY success \n");

            if ((TX_PHY == TX_2M) && (RX_PHY == RX_2M))
            {
              APP_DBG_MSG("PHY Param  TX= %d, RX= %d \n", TX_PHY, RX_PHY);
            }
            else
            {
              APP_DBG_MSG("PHY Param  TX= %d, RX= %d \n", TX_PHY, RX_PHY);
            }
          }
          else
          {
            APP_DBG_MSG("Read conf not succeess \n");
          }
          /* USER CODE BEGIN EVT_LE_PHY_UPDATE_COMPLETE */

          /* USER CODE END EVT_LE_PHY_UPDATE_COMPLETE */
          break;
        case HCI_LE_CONNECTION_COMPLETE_SUBEVT_CODE:
        {
          hci_le_connection_complete_event_rp0 *connection_complete_event;

          /**
           * The connection is done, there is no need anymore to schedule the LP ADV
           */
          connection_complete_event = (hci_le_connection_complete_event_rp0 *) meta_evt->data;
          printf("K: Turning off advertising, as we got a connection?! no thanks!\n");
	  xTimerStop(AdvMgrTimerId, 50*portTICK_PERIOD_MS);  // TODO -check return code

          APP_DBG_MSG("HCI_LE_CONNECTION_COMPLETE_SUBEVT_CODE for connection handle 0x%x\n", connection_complete_event->Connection_Handle);
          if (BleApplicationContext.Device_Connection_Status == APP_BLE_LP_CONNECTING)
          {
            /* Connection as client */
            BleApplicationContext.Device_Connection_Status = APP_BLE_CONNECTED_CLIENT;
          }
          else
          {
            /* Connection as server */
            BleApplicationContext.Device_Connection_Status = APP_BLE_CONNECTED_SERVER;
          }
          BleApplicationContext.BleApplicationContext_legacy.connectionHandle = connection_complete_event->Connection_Handle;
          /* USER CODE BEGIN HCI_EVT_LE_CONN_COMPLETE */

          /* USER CODE END HCI_EVT_LE_CONN_COMPLETE */
        }
        break; /* HCI_LE_CONNECTION_COMPLETE_SUBEVT_CODE */

        /* USER CODE BEGIN META_EVT */

        /* USER CODE END META_EVT */

        default:
          /* USER CODE BEGIN SUBEVENT_DEFAULT */

          /* USER CODE END SUBEVENT_DEFAULT */
          break;
      }
    }
    break; /* HCI_LE_META_EVT_CODE */

    case HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE:
      blecore_evt = (evt_blecore_aci*) event_pckt->data;
      /* USER CODE BEGIN EVT_VENDOR */

      /* USER CODE END EVT_VENDOR */
      switch (blecore_evt->ecode)
      {
      /* USER CODE BEGIN ecode */

      /* USER CODE END ecode */
        case ACI_GAP_PROC_COMPLETE_VSEVT_CODE:
        APP_DBG_MSG("\r\n\r** ACI_GAP_PROC_COMPLETE_VSEVT_CODE \n");
        /* USER CODE BEGIN EVT_BLUE_GAP_PROCEDURE_COMPLETE */

        /* USER CODE END EVT_BLUE_GAP_PROCEDURE_COMPLETE */
          break; /* ACI_GAP_PROC_COMPLETE_VSEVT_CODE */

      /* USER CODE BEGIN BLUE_EVT */

      /* USER CODE END BLUE_EVT */
      }
      break; /* HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE */

      /* USER CODE BEGIN EVENT_PCKT */

      /* USER CODE END EVENT_PCKT */

      default:
      /* USER CODE BEGIN ECODE_DEFAULT*/

      /* USER CODE END ECODE_DEFAULT*/
      break;
  }

  return (SVCCTL_UserEvtFlowEnable);
}



static void HciUserEvtProcess(void *argument)
{
  (void)argument;
  HCI_TL_HciInitConf_t Hci_Tl_Init_Conf;

  MtxHciId = xSemaphoreCreateMutex();
  SemHciId = xSemaphoreCreateBinary();
  
  printf("Ble_Tl_Init: making hci\n");
  Hci_Tl_Init_Conf.p_cmdbuffer = (uint8_t*)&BleCmdBuffer;
  Hci_Tl_Init_Conf.StatusNotCallBack = BLE_StatusNot;
  hci_init(BLE_UserEvtRx, (void*) &Hci_Tl_Init_Conf);


  for(;;)
  {
	xTaskNotifyWait(1, 1, NULL, portMAX_DELAY);
	hci_user_evt_proc( );
  }
}


static void Ble_Hci_Gap_Gatt_Init(void){

  uint8_t role;
  uint16_t gap_service_handle, gap_dev_name_char_handle, gap_appearance_char_handle;
  const uint8_t *bd_addr;
  uint32_t srd_bd_addr[2];
  uint16_t appearance[1] = { BLE_CFG_GAP_APPEARANCE };

  /**
   * Initialize HCI layer
   */
  /*HCI Reset to synchronise BLE Stack*/
  hci_reset();

  /**
   * Write the BD Address
   */

  bd_addr = BleGetBdAddress();
  aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET,
                            CONFIG_DATA_PUBADDR_LEN,
                            (uint8_t*) bd_addr);

  /* BLE MAC in ADV Packet */
  manuf_data[ sizeof(manuf_data)-6] = bd_addr[5];
  manuf_data[ sizeof(manuf_data)-5] = bd_addr[4];
  manuf_data[ sizeof(manuf_data)-4] = bd_addr[3];
  manuf_data[ sizeof(manuf_data)-3] = bd_addr[2];
  manuf_data[ sizeof(manuf_data)-2] = bd_addr[1];
  manuf_data[ sizeof(manuf_data)-1] = bd_addr[0];

  /**
   * Write Identity root key used to derive LTK and CSRK
   */
    aci_hal_write_config_data(CONFIG_DATA_IR_OFFSET,
    CONFIG_DATA_IR_LEN,
                            (uint8_t*) BLE_CFG_IR_VALUE);

   /**
   * Write Encryption root key used to derive LTK and CSRK
   */
    aci_hal_write_config_data(CONFIG_DATA_ER_OFFSET,
    CONFIG_DATA_ER_LEN,
                            (uint8_t*) BLE_CFG_ER_VALUE);

   /**
   * Write random bd_address
   */
   /* random_bd_address = R_bd_address;
    aci_hal_write_config_data(CONFIG_DATA_RANDOM_ADDRESS_WR,
    CONFIG_DATA_RANDOM_ADDRESS_LEN,
                            (uint8_t*) random_bd_address);
  */

  /**
   * Static random Address
   * The two upper bits shall be set to 1
   * The lowest 32bits is read from the UDN to differentiate between devices
   * The RNG may be used to provide a random number on each power on
   */
  srd_bd_addr[1] =  0x0000ED6E;
  srd_bd_addr[0] =  STM32::Calibration::UID64_UDN;
  aci_hal_write_config_data( CONFIG_DATA_RANDOM_ADDRESS_OFFSET, CONFIG_DATA_RANDOM_ADDRESS_LEN, (uint8_t*)srd_bd_addr );

  /**
   * Write Identity root key used to derive LTK and CSRK
   */
    aci_hal_write_config_data( CONFIG_DATA_IR_OFFSET, CONFIG_DATA_IR_LEN, (uint8_t*)BLE_CFG_IR_VALUE );

   /**
   * Write Encryption root key used to derive LTK and CSRK
   */
    aci_hal_write_config_data( CONFIG_DATA_ER_OFFSET, CONFIG_DATA_ER_LEN, (uint8_t*)BLE_CFG_ER_VALUE );

  /**
   * Set TX Power to 0dBm.
   */
  aci_hal_set_tx_power_level(1, CFG_TX_POWER);

  /**
   * Initialize GATT interface
   */
  aci_gatt_init();

  /**
   * Initialize GAP interface
   */
  role = 0;

#if (BLE_CFG_PERIPHERAL == 1)
  role |= GAP_PERIPHERAL_ROLE;
#endif

#if (BLE_CFG_CENTRAL == 1)
  role |= GAP_CENTRAL_ROLE;
#endif

  if (role > 0)
  {
    const char *name = "HRSTM";
    aci_gap_init(role, 0,
                 APPBLE_GAP_DEVICE_NAME_LENGTH,
                 &gap_service_handle, &gap_dev_name_char_handle, &gap_appearance_char_handle);

    if (aci_gatt_update_char_value(gap_service_handle, gap_dev_name_char_handle, 0, strlen(name), (uint8_t *) name))
    {
      BLE_DBG_SVCCTL_MSG("Device Name aci_gatt_update_char_value failed.\n");
    }
  }

  if(aci_gatt_update_char_value(gap_service_handle,
                                gap_appearance_char_handle,
                                0,
                                2,
                                (uint8_t *)&appearance))
  {
    BLE_DBG_SVCCTL_MSG("Appearance aci_gatt_update_char_value failed.\n");
  }
  /**
   * Initialize Default PHY
   */
  hci_le_set_default_phy(ALL_PHYS_PREFERENCE,TX_2M_PREFERRED,RX_2M_PREFERRED);

  /**
   * Initialize IO capability
   */
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.ioCapability = CFG_IO_CAPABILITY;
  aci_gap_set_io_capability(BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.ioCapability);

  /**
   * Initialize authentication
   */
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.mitm_mode = CFG_MITM_PROTECTION;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMin = CFG_ENCRYPTION_KEY_SIZE_MIN;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMax = CFG_ENCRYPTION_KEY_SIZE_MAX;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Use_Fixed_Pin = CFG_USED_FIXED_PIN;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Fixed_Pin = CFG_FIXED_PIN;
  BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.bonding_mode = CFG_BONDING_MODE;

  aci_gap_set_authentication_requirement(BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.bonding_mode,
                                         BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.mitm_mode,
                                         CFG_SC_SUPPORT,
                                         CFG_KEYPRESS_NOTIFICATION_SUPPORT,
                                         BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMin,
                                         BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.encryptionKeySizeMax,
                                         BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Use_Fixed_Pin,
                                         BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.Fixed_Pin,
                                         PUBLIC_ADDR
                                         );

  /**
   * Initialize whitelist
   */
   if (BleApplicationContext.BleApplicationContext_legacy.bleSecurityParam.bonding_mode)
   {
     aci_gap_configure_whitelist();
   }
}


#if 1
static void Adv_Update( void )
{
	printf("K: LP_ADV\n");
  Adv_Request(APP_BLE_LP_ADV);

  return;
}

/** K Called on timer! */
static void Adv_Mgr( TimerHandle_t xTimer )
{
	(void)xTimer;
	printf("adv_mgr tick\n");
  /**
   * The code shall be executed in the background as an aci command may be sent
   * The background is the only place where the application can make sure a new aci command
   * is not sent if there is a pending one
   */
  xTaskNotifyGive(AdvUpdateProcessId);
  return;
}


static void AdvUpdateProcess(void *argument)
{
  (void)argument;

  for(;;)
  {
    xTaskNotifyWait(1, 1, NULL, portMAX_DELAY);
    Adv_Update( );
  }
}


#endif


void APP_BLE_Init( void )
{
/* USER CODE BEGIN APP_BLE_Init_1 */

/* USER CODE END APP_BLE_Init_1 */
  SHCI_C2_Ble_Init_Cmd_Packet_t ble_init_cmd_packet =
  {
    {{0,0,0}},                          /**< Header unused */
    {0,                                 /** pBleBufferAddress not used */
    0,                                  /** BleBufferSize not used */
    CFG_BLE_NUM_GATT_ATTRIBUTES,
    CFG_BLE_NUM_GATT_SERVICES,
    CFG_BLE_ATT_VALUE_ARRAY_SIZE,
    CFG_BLE_NUM_LINK,
    CFG_BLE_DATA_LENGTH_EXTENSION,
    CFG_BLE_PREPARE_WRITE_LIST_SIZE,
    CFG_BLE_MBLOCK_COUNT,
    CFG_BLE_MAX_ATT_MTU,
    CFG_BLE_SLAVE_SCA,
    CFG_BLE_MASTER_SCA,
    CFG_BLE_LSE_SOURCE,
    CFG_BLE_MAX_CONN_EVENT_LENGTH,
    CFG_BLE_HSE_STARTUP_TIME,
    CFG_BLE_VITERBI_MODE,
    CFG_BLE_OPTIONS,
    0,
    CFG_BLE_MAX_COC_INITIATOR_NBR,
    CFG_BLE_MIN_TX_POWER,
    CFG_BLE_MAX_TX_POWER}
  };

  /**
   * Do not allow standby in the application
   */
//  printf("KLPM:app:DISABLE\n");
// FIXME kkk  UTIL_LPM_SetOffMode(1 << CFG_LPM_APP_BLE, UTIL_LPM_DISABLE);

  /**
   * Register the hci transport layer to handle BLE User Asynchronous Events
   */
   xTaskCreate(HciUserEvtProcess, "hci_evt", configMINIMAL_STACK_SIZE*8, NULL, tskIDLE_PRIORITY + 1, &HciUserEvtProcessId);


  /**
   * Starts the BLE Stack on CPU2
   */
  if (SHCI_C2_BLE_Init( &ble_init_cmd_packet ) != SHCI_Success)
  {
    Error_Handler();
  }
   
   // Say here, we split and let apps take over again?
   xTaskNotify(th_ble, (1<<0), eSetBits);
  return;
}




/**
 * The type of the payload for a system user event is tSHCI_UserEvtRxParam
 * When the system event is both :
 *    - a ready event (subevtcode = SHCI_SUB_EVT_CODE_READY)
 *    - reported by the FUS (sysevt_ready_rsp == FUS_FW_RUNNING)
 * The buffer shall not be released
 * ( eg ((tSHCI_UserEvtRxParam*)pPayload)->status shall be set to SHCI_TL_UserEventFlow_Disable )
 * When the status is not filled, the buffer is released by default
 */
// KARL - called from schi-evt task
static void APPE_SysUserEvtRx( void * pPayload )
{
  (void)pPayload;
  /* Traces channel initialization */
  // TODO: this is only for getting the magic trace from the cpu2, which
  // ST support might ask for, we won't get any help with this stack anyway I suspect ;)
  // APPD_EnableCPU2( );

  APP_BLE_Init( );
  printf("Finished app user event (startup)\n");
  // FIXME kkk UTIL_LPM_SetOffMode(1U << CFG_LPM_APP, UTIL_LPM_ENABLE);
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
  
  for(;;)
  {
    /* USER CODE BEGIN SHCI_USER_EVT_PROCESS_1 */

    /* USER CODE END SHCI_USER_EVT_PROCESS_1 */
	xTaskNotifyWait(1, 1, NULL, portMAX_DELAY);
	printf("SCHI: event!\n");
     shci_user_evt_proc();
    /* USER CODE BEGIN SHCI_USER_EVT_PROCESS_2 */

    /* USER CODE END SHCI_USER_EVT_PROCESS_2 */
    }
}


static void task_ble_setup(void) {
	// MX_IPCC_Init()...
	RCC.enable(rcc::IPCC);
	NVIC.enable(interrupt::irq::IPCC_C1_RX);
	NVIC.enable(interrupt::irq::IPCC_C1_TX);
	// XXX: IPCC reset? yolo! I don't see what state we should bother with this in...
//	IPCC->C1CR = 0;
//	IPCC->C1MR = 0x3f << 16 | 0x3f;
//	IPCC->C1SCR = 0x3f;
	// Ok, but seriously,
	IPCC->C1CR |= (1<<16) | (1<<0); // TXFIE | RXOIE
	
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
	
	EXTI->IMR2 |= (1 << (36-32)) | (1<<(38-32)); // IPCC and HSEM wakeup EXTIs
	
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
}

void task_ble(void *pvParameters)
{
	struct ts_ble_t *ts = (struct ts_ble_t*)pvParameters;
	(void)ts; // FIXME - remove when you start using it!
	
	
	task_ble_setup(); // this is the hw, mostly....
	
	// Start the lowest level task, and pass our "user" startup task...
	// this will fire up C2, start hci task, and let us know when we can continue...
	xTaskCreate(ShciUserEvtProcess, "shci_evt", configMINIMAL_STACK_SIZE*7, (void*)APPE_SysUserEvtRx, tskIDLE_PRIORITY + 1, &ShciUserEvtProcessId);
	
	
	// Wait here til we get a notification from shci that we're up
	printf("t_ble: Waiting for SHCI\n");
	xTaskNotifyWait(ULONG_MAX, 0, NULL, portMAX_DELAY);
	
	// continue with "app" stuff
		
		
		
   
   
  /**
   * Initialization of HCI & GATT & GAP layer
   */
  Ble_Hci_Gap_Gatt_Init();

  /**
   * Initialization of the BLE Services
   */
  SVCCTL_Init();
  
  //// THEORY - everything below here is "private app" and above here is "required init...?

  /**
   * Initialization of the BLE App Context
   */
  BleApplicationContext.Device_Connection_Status = APP_BLE_IDLE;
  BleApplicationContext.BleApplicationContext_legacy.connectionHandle = 0xFFFF;
  /**
   * From here, all initialization are BLE application specific
   */
  xTaskCreate(AdvUpdateProcess, "adv", configMINIMAL_STACK_SIZE*6, NULL, tskIDLE_PRIORITY + 1, &AdvUpdateProcessId);


  /**
   * Initialization of ADV - Ad Manufacturer Element - Support OTA Bit Mask
   */
#if(BLE_CFG_OTA_REBOOT_CHAR != 0)
  manuf_data[sizeof(manuf_data)-8] = CFG_FEATURE_OTA_REBOOT;
#endif
  /**
   * Initialize DIS Application
   */
  DISAPP_Init();

  /**
   * Initialize HRS Application
   */
// FIXME kkk put it back later...  HRSAPP_Init();

  /**
   * Create timer to handle the connection state machine
   */
  AdvMgrTimerId = xTimerCreate("ble-adv-mgr", INITIAL_ADV_TIMEOUT, pdFALSE, NULL, &Adv_Mgr);

  /**
   * Make device discoverable
   */
  // yeah, but no, not like this thanks...
//  BleApplicationContext.BleApplicationContext_legacy.advtServUUID[0] = AD_TYPE_16_BIT_SERV_UUID;
//  BleApplicationContext.BleApplicationContext_legacy.advtServUUIDlen = 1;
  //Add_Advertisment_Service_UUID(HEART_RATE_SERVICE_UUID);
  BleApplicationContext.BleApplicationContext_legacy.advtServUUIDlen = 0;

  /* Initialize intervals for reconnexion without intervals update */
  AdvIntervalMin = CFG_FAST_CONN_ADV_INTERVAL_MIN;
  AdvIntervalMax = CFG_FAST_CONN_ADV_INTERVAL_MAX;

  /**
  * Start to Advertise to be connected by Collector
   */
   Adv_Request(APP_BLE_FAST_ADV);

/* USER CODE BEGIN APP_BLE_Init_2 */

/* USER CODE END APP_BLE_Init_2 */
		
	
	
	
	TickType_t xLastWakeTime = xTaskGetTickCount();
	while (1) {
		xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(250));
//		Adv_Request(APP_BLE_FAST_ADV);
	}
}



// Shitty ST code for dumping a buffer.
extern "C" {

void DbgTraceBuffer(const void *pBuffer, uint32_t u32Length, const char *strFormat, ...)
{
  va_list vaArgs;
  uint32_t u32Index;
  va_start(vaArgs, strFormat);
  vprintf(strFormat, vaArgs);
  va_end(vaArgs);
  for (u32Index = 0; u32Index < u32Length; u32Index ++)
  {
    printf(" %02X", ((const uint8_t *) pBuffer)[u32Index]);
  }
}

}