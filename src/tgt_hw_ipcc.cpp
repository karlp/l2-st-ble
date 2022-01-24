/*
 * The IPCC hardware layer, classically this is HAL/LL .c stuff...
 * we're going to do this completely with laks, ideally..
 * This is largly equivalent to STM32_WPAN/Target/hw_ipcc.c
 */

#include <exti/exti.h>
#include <interrupt/interrupt.h>
#include <pwr/pwr.h>
#include <rcc/rcc.h>
#include <wpan/ipcc.h>

#include "hw.h"
#include "mbox_def.h"

// WARNING! ST defines these shifted in LL!
#define LL_IPCC_CHANNEL_1 0
#define LL_IPCC_CHANNEL_2 1
#define LL_IPCC_CHANNEL_3 2
#define LL_IPCC_CHANNEL_4 3
#define LL_IPCC_CHANNEL_5 4
#define LL_IPCC_CHANNEL_6 5


// super rad C style function pointer, which I'm sure will blow up in my face...
static void (*FreeBufCb)( void );

/******************************************************************************
 * GENERAL
 ******************************************************************************/
void HW_IPCC_Enable( void )
{
  /**
  * Such as IPCC IP available to the CPU2, it is required to keep the IPCC clock running
    when FUS is running on CPU2 and CPU1 enters deep sleep mode
  */
	RCC.enable(rcc::C2IPCC);

   /**
   * When the device is out of standby, it is required to use the EXTI mechanism to wakeup CPU2
   */
	EXTI->C2EMR2 |= (1<<(41-32));
	EXTI->RTSR2 |= (1<<(41-32));

  /**
   * In case the SBSFU is implemented, it may have already set the C2BOOT bit to startup the CPU2.
   * In that case, to keep the mechanism transparent to the user application, it shall call the system command
   * SHCI_C2_Reinit( ) before jumping to the application.
   * When the CPU2 receives that command, it waits for its event input to be set to restart the CPU2 firmware.
   * This is required because once C2BOOT has been set once, a clear/set on C2BOOT has no effect.
   * When SHCI_C2_Reinit( ) is not called, generating an event to the CPU2 does not have any effect
   * So, by default, the application shall both set the event flag and set the C2BOOT bit.
   */
	asm volatile ("sev");
	asm volatile ("wfe");
	PWR->CR4 |= (1<<15); // C2BOOT

  return;
}

void HW_IPCC_Init( void )
{
	RCC.enable(rcc::IPCC);
	IPCC->C1CR |= (1<<16) | (1<<0); // TXFIE + RXOIE
	NVIC.enable(interrupt::irq::IPCC_C1_RX);
	NVIC.enable(interrupt::irq::IPCC_C1_TX);
	return;
}

/******************************************************************************
 * BLE
 ******************************************************************************/
void HW_IPCC_BLE_Init( void )
{
	IPCC.enable_rx(HW_IPCC_BLE_EVENT_CHANNEL);
	return;
}

void HW_IPCC_BLE_SendCmd( void )
{
	IPCC.c1_set_flag(HW_IPCC_BLE_CMD_CHANNEL);
	return;
}

static void HW_IPCC_BLE_EvtHandler( void )
{
	HW_IPCC_BLE_RxEvtNot();
	IPCC.c1_clear_flag(HW_IPCC_BLE_EVENT_CHANNEL);
	return;
}

void HW_IPCC_BLE_SendAclData( void )
{
	IPCC.c1_set_flag(HW_IPCC_HCI_ACL_DATA_CHANNEL);
	IPCC.enable_tx(HW_IPCC_HCI_ACL_DATA_CHANNEL);
	return;
}

static void HW_IPCC_BLE_AclDataEvtHandler( void )
{
	IPCC.disable_tx(HW_IPCC_HCI_ACL_DATA_CHANNEL);
	HW_IPCC_BLE_AclDataAckNot();
	return;
}

/******************************************************************************
 * SYSTEM
 ******************************************************************************/
void HW_IPCC_SYS_Init( void )
{
	IPCC.enable_rx(HW_IPCC_SYSTEM_EVENT_CHANNEL);
	return;
}

void HW_IPCC_SYS_SendCmd( void )
{
	IPCC.c1_set_flag(HW_IPCC_SYSTEM_CMD_RSP_CHANNEL);
	IPCC.enable_tx(HW_IPCC_SYSTEM_CMD_RSP_CHANNEL);
	return;
}

static void HW_IPCC_SYS_CmdEvtHandler( void )
{
	IPCC.disable_tx(HW_IPCC_SYSTEM_CMD_RSP_CHANNEL);
	HW_IPCC_SYS_CmdEvtNot();
	return;
}

static void HW_IPCC_SYS_EvtHandler( void )
{
	HW_IPCC_SYS_EvtNot();
	IPCC.c1_clear_flag(HW_IPCC_SYSTEM_EVENT_CHANNEL);
	return;
}


/******************************************************************************
 * MEMORY MANAGER
 ******************************************************************************/
void HW_IPCC_MM_SendFreeBuf( void (*cb)( void ) )
{
	if (IPCC->C1TOC2SR & (1<<HW_IPCC_MM_RELEASE_BUFFER_CHANNEL)) {
		FreeBufCb = cb;
		IPCC.enable_tx(HW_IPCC_MM_RELEASE_BUFFER_CHANNEL);
	} else {
		cb();
		IPCC.c1_set_flag(HW_IPCC_MM_RELEASE_BUFFER_CHANNEL);
	}
	return;
}

static void HW_IPCC_MM_FreeBufHandler( void )
{
	IPCC.disable_tx(HW_IPCC_MM_RELEASE_BUFFER_CHANNEL);
	FreeBufCb();
	IPCC.c1_set_flag(HW_IPCC_MM_RELEASE_BUFFER_CHANNEL);
	return;
}

/******************************************************************************
 * TRACES
 ******************************************************************************/
void HW_IPCC_TRACES_Init( void )
{
	IPCC.enable_rx(HW_IPCC_TRACES_CHANNEL);
	return;
}

static void HW_IPCC_TRACES_EvtHandler( void )
{
	HW_IPCC_TRACES_EvtNot();
	IPCC.c1_clear_flag(HW_IPCC_TRACES_CHANNEL);
	return;
}


template <>
void interrupt::handler<interrupt::irq::IPCC_C1_RX>() {
	if (IPCC.rx_pending(HW_IPCC_SYSTEM_EVENT_CHANNEL)) {
		HW_IPCC_SYS_EvtHandler();
	}
	
//#ifdef MAC_802_15_4_WB
//  else if (HW_IPCC_RX_PENDING( HW_IPCC_MAC_802_15_4_NOTIFICATION_ACK_CHANNEL ))
//  {
//    HW_IPCC_MAC_802_15_4_NotEvtHandler();
//  }
//#endif /* MAC_802_15_4_WB */
//#ifdef THREAD_WB
//  else if (HW_IPCC_RX_PENDING( HW_IPCC_THREAD_NOTIFICATION_ACK_CHANNEL ))
//  {
//    HW_IPCC_THREAD_NotEvtHandler();
//  }
//  else if (HW_IPCC_RX_PENDING( HW_IPCC_THREAD_CLI_NOTIFICATION_ACK_CHANNEL ))
//  {
//    HW_IPCC_THREAD_CliNotEvtHandler();
//  }
//#endif /* THREAD_WB */
//#ifdef LLD_TESTS_WB
//  else if (HW_IPCC_RX_PENDING( HW_IPCC_LLDTESTS_CLI_RSP_CHANNEL ))
//  {
//    HW_IPCC_LLDTESTS_ReceiveCliRspHandler();
//  }
//  else if (HW_IPCC_RX_PENDING( HW_IPCC_LLDTESTS_M0_CMD_CHANNEL ))
//  {
//    HW_IPCC_LLDTESTS_ReceiveM0CmdHandler();
//  }
//#endif /* LLD_TESTS_WB */
//#ifdef LLD_BLE_WB
//  else if (HW_IPCC_RX_PENDING( HW_IPCC_LLD_BLE_RSP_CHANNEL ))
//  {
//    HW_IPCC_LLD_BLE_ReceiveRspHandler();
//  }
//  else if (HW_IPCC_RX_PENDING( HW_IPCC_LLD_BLE_M0_CMD_CHANNEL ))
//  {
//    HW_IPCC_LLD_BLE_ReceiveM0CmdHandler();
//  }
//#endif /* LLD_TESTS_WB */
//#ifdef ZIGBEE_WB
//  else if (HW_IPCC_RX_PENDING( HW_IPCC_ZIGBEE_APPLI_NOTIF_ACK_CHANNEL ))
//  {
//    HW_IPCC_ZIGBEE_StackNotifEvtHandler();
//  }
//  else if (HW_IPCC_RX_PENDING( HW_IPCC_ZIGBEE_M0_REQUEST_CHANNEL ))
//  {
//    HW_IPCC_ZIGBEE_StackM0RequestHandler();
//  }
//#endif /* ZIGBEE_WB */

	else if (IPCC.rx_pending(HW_IPCC_BLE_EVENT_CHANNEL)) {
		HW_IPCC_BLE_EvtHandler();
	}
	else if (IPCC.rx_pending(HW_IPCC_TRACES_CHANNEL)) {
		HW_IPCC_TRACES_EvtHandler();
	}
}



template <>
void interrupt::handler<interrupt::irq::IPCC_C1_TX>() {

	if (IPCC.tx_pending(HW_IPCC_SYSTEM_CMD_RSP_CHANNEL)) {
		HW_IPCC_SYS_CmdEvtHandler();
	}
//#ifdef MAC_802_15_4_WB
//  else if (HW_IPCC_TX_PENDING( HW_IPCC_MAC_802_15_4_CMD_RSP_CHANNEL ))
//  {
//    HW_IPCC_MAC_802_15_4_CmdEvtHandler();
//  }
//#endif /* MAC_802_15_4_WB */
//#ifdef THREAD_WB
//  else if (HW_IPCC_TX_PENDING( HW_IPCC_THREAD_OT_CMD_RSP_CHANNEL ))
//  {
//    HW_IPCC_OT_CmdEvtHandler();
//  }
//#endif /* THREAD_WB */
//#ifdef LLD_TESTS_WB
//// No TX handler for LLD tests
//#endif /* LLD_TESTS_WB */
//#ifdef ZIGBEE_WB
//  if (HW_IPCC_TX_PENDING( HW_IPCC_ZIGBEE_CMD_APPLI_CHANNEL ))
//  {
//      HW_IPCC_ZIGBEE_CmdEvtHandler();
//  }
//#endif /* ZIGBEE_WB */
	else if (IPCC.tx_pending(HW_IPCC_SYSTEM_CMD_RSP_CHANNEL)) {
		HW_IPCC_SYS_CmdEvtHandler();
	}
	else if (IPCC.tx_pending(HW_IPCC_MM_RELEASE_BUFFER_CHANNEL)) {
		HW_IPCC_MM_FreeBufHandler();
	}
	else if (IPCC.tx_pending(HW_IPCC_HCI_ACL_DATA_CHANNEL)) {
		HW_IPCC_BLE_AclDataEvtHandler();
	}
	return;
}
