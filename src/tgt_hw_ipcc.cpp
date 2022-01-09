/*
 * The IPCC hardware layer, classically this is HAL/LL .c stuff...
 * we're going to do this completely with laks, ideally..
 */

#include <rcc/rcc.h>
#include <wpan/ipcc.h>

#include "hw.h"

//#define HW_IPCC_TX_PENDING( channel ) ( !(LL_C1_IPCC_IsActiveFlag_CHx( IPCC, channel )) ) &&  (((~(IPCC->C1MR)) & (channel << 16U)))
//#define HW_IPCC_RX_PENDING( channel )  (LL_C2_IPCC_IsActiveFlag_CHx( IPCC, channel )) && (((~(IPCC->C1MR)) & (channel << 0U)))
//__STATIC_INLINE uint32_t LL_C1_IPCC_IsActiveFlag_CHx(IPCC_TypeDef  const *const IPCCx, uint32_t Channel)
//{
//  return ((READ_BIT(IPCCx->C1TOC2SR, Channel) == (Channel)) ? 1UL : 0UL);
//}
//__STATIC_INLINE uint32_t LL_C2_IPCC_IsActiveFlag_CHx(IPCC_TypeDef  const *const IPCCx, uint32_t Channel)
//{
//  return ((READ_BIT(IPCCx->C2TOC1SR, Channel) == (Channel)) ? 1UL : 0UL);
//}



void myHW_IPCC_Rx_Handler(void) {
//	if ((IPCC->C2TOC1SR & HW_IPCC_SYSTEM_EVENT_CHANNEL)
//		&& (~())
	
//  if (HW_IPCC_RX_PENDING( HW_IPCC_SYSTEM_EVENT_CHANNEL ))
//  {
//      HW_IPCC_SYS_EvtHandler();
//  }
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
//  else if (HW_IPCC_RX_PENDING( HW_IPCC_BLE_EVENT_CHANNEL ))
//  {
//    HW_IPCC_BLE_EvtHandler();
//  }
//  else if (HW_IPCC_RX_PENDING( HW_IPCC_TRACES_CHANNEL ))
//  {
//    HW_IPCC_TRACES_EvtHandler();
//  }
}


extern "C" {
	
	
	
/******************************************************************************
 * INTERRUPT HANDLER
 ******************************************************************************/	
	
void HW_IPCC_Rx_Handler( void )
{
//  if (HW_IPCC_RX_PENDING( HW_IPCC_SYSTEM_EVENT_CHANNEL ))
//  {
//      HW_IPCC_SYS_EvtHandler();
//  }
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
//  else if (HW_IPCC_RX_PENDING( HW_IPCC_BLE_EVENT_CHANNEL ))
//  {
//    HW_IPCC_BLE_EvtHandler();
//  }
//  else if (HW_IPCC_RX_PENDING( HW_IPCC_TRACES_CHANNEL ))
//  {
//    HW_IPCC_TRACES_EvtHandler();
//  }

  return;
}

void HW_IPCC_Tx_Handler( void )
{
//  if (HW_IPCC_TX_PENDING( HW_IPCC_SYSTEM_CMD_RSP_CHANNEL ))
//  {
//    HW_IPCC_SYS_CmdEvtHandler();
//  }
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
//  else if (HW_IPCC_TX_PENDING( HW_IPCC_SYSTEM_CMD_RSP_CHANNEL ))
//  {
//    HW_IPCC_SYS_CmdEvtHandler();
//  }
//  else if (HW_IPCC_TX_PENDING( HW_IPCC_MM_RELEASE_BUFFER_CHANNEL ))
//  {
//    HW_IPCC_MM_FreeBufHandler();
//  }
//  else if (HW_IPCC_TX_PENDING( HW_IPCC_HCI_ACL_DATA_CHANNEL ))
//  {
//    HW_IPCC_BLE_AclDataEvtHandler();
//  }

  return;
}
/******************************************************************************
 * GENERAL
 ******************************************************************************/
void HW_IPCC_Enable( void )
{
  /**
  * Such as IPCC IP available to the CPU2, it is required to keep the IPCC clock running
    when FUS is running on CPU2 and CPU1 enters deep sleep mode
  */
//  LL_C2_AHB3_GRP1_EnableClock(LL_C2_AHB3_GRP1_PERIPH_IPCC);

   /**
   * When the device is out of standby, it is required to use the EXTI mechanism to wakeup CPU2
   */
//  LL_C2_EXTI_EnableEvent_32_63( LL_EXTI_LINE_41 );
//  LL_EXTI_EnableRisingTrig_32_63( LL_EXTI_LINE_41 );

  /**
   * In case the SBSFU is implemented, it may have already set the C2BOOT bit to startup the CPU2.
   * In that case, to keep the mechanism transparent to the user application, it shall call the system command
   * SHCI_C2_Reinit( ) before jumping to the application.
   * When the CPU2 receives that command, it waits for its event input to be set to restart the CPU2 firmware.
   * This is required because once C2BOOT has been set once, a clear/set on C2BOOT has no effect.
   * When SHCI_C2_Reinit( ) is not called, generating an event to the CPU2 does not have any effect
   * So, by default, the application shall both set the event flag and set the C2BOOT bit.
   */
//  __SEV( );       /* Set the internal event flag and send an event to the CPU2 */
//  __WFE( );       /* Clear the internal event flag */
//  LL_PWR_EnableBootC2( );

  return;
}

void HW_IPCC_Init( void )
{
//  LL_AHB3_GRP1_EnableClock( LL_AHB3_GRP1_PERIPH_IPCC );
//
//  LL_C1_IPCC_EnableIT_RXO( IPCC );
//  LL_C1_IPCC_EnableIT_TXF( IPCC );
//
//  HAL_NVIC_EnableIRQ(IPCC_C1_RX_IRQn);
//  HAL_NVIC_EnableIRQ(IPCC_C1_TX_IRQn);

  return;
}

/******************************************************************************
 * BLE
 ******************************************************************************/
void HW_IPCC_BLE_Init( void )
{
//  LL_C1_IPCC_EnableReceiveChannel( IPCC, HW_IPCC_BLE_EVENT_CHANNEL );

  return;
}

void HW_IPCC_BLE_SendCmd( void )
{
//  LL_C1_IPCC_SetFlag_CHx( IPCC, HW_IPCC_BLE_CMD_CHANNEL );

  return;
}

static void HW_IPCC_BLE_EvtHandler( void )
{
  HW_IPCC_BLE_RxEvtNot();

//  LL_C1_IPCC_ClearFlag_CHx( IPCC, HW_IPCC_BLE_EVENT_CHANNEL );

  return;
}

void HW_IPCC_BLE_SendAclData( void )
{
//  LL_C1_IPCC_SetFlag_CHx( IPCC, HW_IPCC_HCI_ACL_DATA_CHANNEL );
//  LL_C1_IPCC_EnableTransmitChannel( IPCC, HW_IPCC_HCI_ACL_DATA_CHANNEL );

  return;
}

static void HW_IPCC_BLE_AclDataEvtHandler( void )
{
//  LL_C1_IPCC_DisableTransmitChannel( IPCC, HW_IPCC_HCI_ACL_DATA_CHANNEL );

  HW_IPCC_BLE_AclDataAckNot();

  return;
}

//__weak void HW_IPCC_BLE_AclDataAckNot( void ){};
//__weak void HW_IPCC_BLE_RxEvtNot( void ){};

/******************************************************************************
 * SYSTEM
 ******************************************************************************/
void HW_IPCC_SYS_Init( void )
{
//  LL_C1_IPCC_EnableReceiveChannel( IPCC, HW_IPCC_SYSTEM_EVENT_CHANNEL );

  return;
}

void HW_IPCC_SYS_SendCmd( void )
{
//  LL_C1_IPCC_SetFlag_CHx( IPCC, HW_IPCC_SYSTEM_CMD_RSP_CHANNEL );
//  LL_C1_IPCC_EnableTransmitChannel( IPCC, HW_IPCC_SYSTEM_CMD_RSP_CHANNEL );

  return;
}

static void HW_IPCC_SYS_CmdEvtHandler( void )
{
//  LL_C1_IPCC_DisableTransmitChannel( IPCC, HW_IPCC_SYSTEM_CMD_RSP_CHANNEL );

  HW_IPCC_SYS_CmdEvtNot();

  return;
}

static void HW_IPCC_SYS_EvtHandler( void )
{
  HW_IPCC_SYS_EvtNot();

//  LL_C1_IPCC_ClearFlag_CHx( IPCC, HW_IPCC_SYSTEM_EVENT_CHANNEL );

  return;
}

//__weak void HW_IPCC_SYS_CmdEvtNot( void ){};
//__weak void HW_IPCC_SYS_EvtNot( void ){};
	


/******************************************************************************
 * MEMORY MANAGER
 ******************************************************************************/
void HW_IPCC_MM_SendFreeBuf( void (*cb)( void ) )
{
//  if ( LL_C1_IPCC_IsActiveFlag_CHx( IPCC, HW_IPCC_MM_RELEASE_BUFFER_CHANNEL ) )
//  {
//    FreeBufCb = cb;
//    LL_C1_IPCC_EnableTransmitChannel( IPCC, HW_IPCC_MM_RELEASE_BUFFER_CHANNEL );
//  }
//  else
//  {
//    cb();
//
//    LL_C1_IPCC_SetFlag_CHx( IPCC, HW_IPCC_MM_RELEASE_BUFFER_CHANNEL );
//  }

  return;
}

static void HW_IPCC_MM_FreeBufHandler( void )
{
//  LL_C1_IPCC_DisableTransmitChannel( IPCC, HW_IPCC_MM_RELEASE_BUFFER_CHANNEL );
//
//  FreeBufCb();
//
//  LL_C1_IPCC_SetFlag_CHx( IPCC, HW_IPCC_MM_RELEASE_BUFFER_CHANNEL );

  return;
}

/******************************************************************************
 * TRACES
 ******************************************************************************/
void HW_IPCC_TRACES_Init( void )
{
//  LL_C1_IPCC_EnableReceiveChannel( IPCC, HW_IPCC_TRACES_CHANNEL );

  return;
}

static void HW_IPCC_TRACES_EvtHandler( void )
{
  HW_IPCC_TRACES_EvtNot();

//  LL_C1_IPCC_ClearFlag_CHx( IPCC, HW_IPCC_TRACES_CHANNEL );

  return;
}

//__weak void HW_IPCC_TRACES_EvtNot( void ){};



} /* extern c shits... */