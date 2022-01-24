#pragma once
#include <stdint.h>
/**
 * TODO: see what you really need here... (turns out, a lot!)
 */

#define CFG_TX_POWER                      (0x18) /* -0.15dBm */

/**
 * Define Advertising parameters
 */
#define CFG_ADV_BD_ADDRESS                (0x7257acd87a6c)
#define CFG_FAST_CONN_ADV_INTERVAL_MIN    (0x80)   /**< 80ms */
#define CFG_FAST_CONN_ADV_INTERVAL_MAX    (0xa0)  /**< 100ms */
#define CFG_LP_CONN_ADV_INTERVAL_MIN      (0x640) /**< 1s */
#define CFG_LP_CONN_ADV_INTERVAL_MAX      (0xfa0) /**< 2.5s */

/**
 * Define IO Authentication
 */
#define CFG_BONDING_MODE                 (1)
#define CFG_FIXED_PIN                    (111111)
#define CFG_USED_FIXED_PIN               (0)
#define CFG_ENCRYPTION_KEY_SIZE_MAX      (16)
#define CFG_ENCRYPTION_KEY_SIZE_MIN      (8)

/**
 * Define IO capabilities
 */
#define CFG_IO_CAPABILITY_DISPLAY_ONLY       (0x00)
#define CFG_IO_CAPABILITY_DISPLAY_YES_NO     (0x01)
#define CFG_IO_CAPABILITY_KEYBOARD_ONLY      (0x02)
#define CFG_IO_CAPABILITY_NO_INPUT_NO_OUTPUT (0x03)
#define CFG_IO_CAPABILITY_KEYBOARD_DISPLAY   (0x04)

#define CFG_IO_CAPABILITY               CFG_IO_CAPABILITY_NO_INPUT_NO_OUTPUT

/**
 * Define MITM modes
 */
#define CFG_MITM_PROTECTION_NOT_REQUIRED      (0x00)
#define CFG_MITM_PROTECTION_REQUIRED          (0x01)

#define CFG_MITM_PROTECTION             CFG_MITM_PROTECTION_REQUIRED

/**
 * Define Secure Connections Support
 */
#define CFG_SECURE_NOT_SUPPORTED       (0x00)
#define CFG_SECURE_OPTIONAL            (0x01)
#define CFG_SECURE_MANDATORY           (0x02)

#define CFG_SC_SUPPORT                 CFG_SECURE_OPTIONAL

/**
 * Define Keypress Notification Support
 */
#define CFG_KEYPRESS_NOT_SUPPORTED      (0x00)
#define CFG_KEYPRESS_SUPPORTED          (0x01)

#define CFG_KEYPRESS_NOTIFICATION_SUPPORT             CFG_KEYPRESS_NOT_SUPPORTED



/******************************************************************************
 * Transport Layer
 ******************************************************************************/
/**
 * Queue length of BLE Event
 * This parameter defines the number of asynchronous events that can be stored in the HCI layer before
 * being reported to the application. When a command is sent to the BLE core coprocessor, the HCI layer
 * is waiting for the event with the Num_HCI_Command_Packets set to 1. The receive queue shall be large
 * enough to store all asynchronous events received in between.
 * When CFG_TLBLE_MOST_EVENT_PAYLOAD_SIZE is set to 27, this allow to store three 255 bytes long asynchronous events
 * between the HCI command and its event.
 * This parameter depends on the value given to CFG_TLBLE_MOST_EVENT_PAYLOAD_SIZE. When the queue size is to small,
 * the system may hang if the queue is full with asynchronous events and the HCI layer is still waiting
 * for a CC/CS event, In that case, the notification TL_BLE_HCI_ToNot() is called to indicate
 * to the application a HCI command did not receive its command event within 30s (Default HCI Timeout).
 */
#define CFG_TLBLE_EVT_QUEUE_LENGTH 5

/**
 * This parameter should be set to fit most events received by the HCI layer. It defines the buffer size of each element
 * allocated in the queue of received events and can be used to optimize the amount of RAM allocated by the Memory Manager.
 * It should not exceed 255 which is the maximum HCI packet payload size (a greater value is a lost of memory as it will
 * never be used)
 * With the current wireless firmware implementation, this parameter shall be kept to 255
 */
#define CFG_TLBLE_MOST_EVENT_PAYLOAD_SIZE 255   /**< Set to 255 with the memory manager and the mailbox */

#define TL_BLE_EVENT_FRAME_SIZE ( TL_EVT_HDR_SIZE + CFG_TLBLE_MOST_EVENT_PAYLOAD_SIZE )

/**
 * Define PHY
 */
#define ALL_PHYS_PREFERENCE                             0x00
#define RX_2M_PREFERRED                                 0x02
#define TX_2M_PREFERRED                                 0x02
#define TX_1M                                           0x01
#define TX_2M                                           0x02
#define RX_1M                                           0x01
#define RX_2M                                           0x02

/**
*   Identity root key used to derive LTK and CSRK
*/
#define CFG_BLE_IRK     {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0, 0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0}

/**
* Encryption root key used to derive LTK and CSRK
*/
#define CFG_BLE_ERK     {0xFE, 0xDC, 0xBA, 0x09, 0x87, 0x65, 0x43, 0x21, 0xFE, 0xDC, 0xBA, 0x09, 0x87, 0x65, 0x43, 0x21}


/**
* AD Element - Group B Feature
*/
/* LSB - Second Byte */
#define CFG_FEATURE_OTA_REBOOT                  (0x20)


/******************************************************************************
 * BLE Stack
 ******************************************************************************/
/**
 * Maximum number of simultaneous connections that the device will support.
 * Valid values are from 1 to 8
 */
#define CFG_BLE_NUM_LINK            4
/**
 * Maximum number of Services that can be stored in the GATT database.
 * Note that the GAP and GATT services are automatically added so this parameter should be 2 plus the number of user services
 */
#define CFG_BLE_NUM_GATT_SERVICES   8

/**
 * Maximum number of Attributes
 * (i.e. the number of characteristic + the number of characteristic values + the number of descriptors, excluding the services)
 * that can be stored in the GATT database.
 * Note that certain characteristics and relative descriptors are added automatically during device initialization
 * so this parameters should be 9 plus the number of user Attributes
 */
#define CFG_BLE_NUM_GATT_ATTRIBUTES 68

/**
 * Maximum supported ATT_MTU size
 * This parameter is ignored by the CPU2 when CFG_BLE_OPTIONS is set to 1"
 */
#define CFG_BLE_MAX_ATT_MTU             (156)

/**
 * Size of the storage area for Attribute values
 *  This value depends on the number of attributes used by application. In particular the sum of the following quantities (in octets) should be made for each attribute:
 *  - attribute value length
 *  - 5, if UUID is 16 bit; 19, if UUID is 128 bit
 *  - 2, if server configuration descriptor is used
 *  - 2*DTM_NUM_LINK, if client configuration descriptor is used
 *  - 2, if extended properties is used
 *  The total amount of memory needed is the sum of the above quantities for each attribute.
 * This parameter is ignored by the CPU2 when CFG_BLE_OPTIONS is set to 1"
 */
#define CFG_BLE_ATT_VALUE_ARRAY_SIZE    (1344)

/**
 * Prepare Write List size in terms of number of packet
 * This parameter is ignored by the CPU2 when CFG_BLE_OPTIONS is set to 1"
 */
#define CFG_BLE_PREPARE_WRITE_LIST_SIZE         BLE_PREP_WRITE_X_ATT(CFG_BLE_MAX_ATT_MTU)

/**
 * Number of allocated memory blocks
 * This parameter is overwritten by the CPU2 with an hardcoded optimal value when the parameter when CFG_BLE_OPTIONS is set to 1
 */
#define CFG_BLE_MBLOCK_COUNT            (BLE_MBLOCKS_CALC(CFG_BLE_PREPARE_WRITE_LIST_SIZE, CFG_BLE_MAX_ATT_MTU, CFG_BLE_NUM_LINK))

/**
 * Enable or disable the Extended Packet length feature. Valid values are 0 or 1.
 */
#define CFG_BLE_DATA_LENGTH_EXTENSION   1

/**
 * Sleep clock accuracy in Slave mode (ppm value)
 */
#define CFG_BLE_SLAVE_SCA   500

/**
 * Sleep clock accuracy in Master mode
 * 0 : 251 ppm to 500 ppm
 * 1 : 151 ppm to 250 ppm
 * 2 : 101 ppm to 150 ppm
 * 3 : 76 ppm to 100 ppm
 * 4 : 51 ppm to 75 ppm
 * 5 : 31 ppm to 50 ppm
 * 6 : 21 ppm to 30 ppm
 * 7 : 0 ppm to 20 ppm
 */
#define CFG_BLE_MASTER_SCA   0

/**
 *  Source for the low speed clock for RF wake-up
 *  1 : external high speed crystal HSE/32/32
 *  0 : external low speed crystal ( no calibration )
 */
#define CFG_BLE_LSE_SOURCE  0

/**
 * Start up time of the high speed (16 or 32 MHz) crystal oscillator in units of 625/256 us (~2.44 us)
 */
#define CFG_BLE_HSE_STARTUP_TIME  0x148

/**
 * Maximum duration of the connection event when the device is in Slave mode in units of 625/256 us (~2.44 us)
 */
#define CFG_BLE_MAX_CONN_EVENT_LENGTH  (0xFFFF)

/**
 * Viterbi Mode
 * 1 : enabled
 * 0 : disabled
 */
#define CFG_BLE_VITERBI_MODE  1

/**
 * BLE stack Options flags to be configured with:
 * - SHCI_C2_BLE_INIT_OPTIONS_LL_ONLY
 * - SHCI_C2_BLE_INIT_OPTIONS_LL_HOST
 * - SHCI_C2_BLE_INIT_OPTIONS_NO_SVC_CHANGE_DESC
 * - SHCI_C2_BLE_INIT_OPTIONS_WITH_SVC_CHANGE_DESC
 * - SHCI_C2_BLE_INIT_OPTIONS_DEVICE_NAME_RO
 * - SHCI_C2_BLE_INIT_OPTIONS_DEVICE_NAME_RW
 * - SHCI_C2_BLE_INIT_OPTIONS_POWER_CLASS_1
 * - SHCI_C2_BLE_INIT_OPTIONS_POWER_CLASS_2_3
 * which are used to set following configuration bits:
 * (bit 0): 1: LL only
 *          0: LL + host
 * (bit 1): 1: no service change desc.
 *          0: with service change desc.
 * (bit 2): 1: device name Read-Only
 *          0: device name R/W
 * (bit 7): 1: LE Power Class 1
 *          0: LE Power Class 2-3
 * other bits: reserved (shall be set to 0)
 */
#define CFG_BLE_OPTIONS  (SHCI_C2_BLE_INIT_OPTIONS_LL_HOST | SHCI_C2_BLE_INIT_OPTIONS_WITH_SVC_CHANGE_DESC | SHCI_C2_BLE_INIT_OPTIONS_DEVICE_NAME_RW | SHCI_C2_BLE_INIT_OPTIONS_POWER_CLASS_2_3)

#define CFG_BLE_MAX_COC_INITIATOR_NBR   (32)

#define CFG_BLE_MIN_TX_POWER            (0)

#define CFG_BLE_MAX_TX_POWER            (0)


#define APP_DBG_MSG(...)	printf(__VA_ARGS__)



/** tick timer value in us */
// FIXME - karl, this is important, you're going to have to get that "timer server" working one day!
//#define CFG_TS_TICK_VAL           DIVR( (CFG_RTCCLK_DIV * 1000000), LSE_VALUE )
#define CFG_TS_TICK_VAL           100



// Most of STM32_WPAN uses __WEAK, which is properly supported by
// cmsis_compiler.h -> cmsis_gcc.h.  However, a few files, eg: svc_ctl.c use
// __weak, and rely on some HAL defines to get back to the correct attributes.
// https://github.com/STMicroelectronics/STM32CubeWB/issues/14
#ifndef __weak
#if defined ( __GNUC__ )
#define __weak   __attribute__((weak))
#else
#error "This workaround for ST code has only been implemented for GCC/CLANG"
#endif
#endif


#define __DBG_TRACE_H  // sneaky hack to avoid upstream hal version

#define PRINT_NO_MESG(...)
#define PRINT_MESG_DBG(fmt, ...)  do { printf(fmt, ## __VA_ARGS__); } while (0)

#define CFG_DEBUG_BLE_TRACE 1
#define CFG_DEBUG_APP_TRACE 1
#if ( (CFG_DEBUG_BLE_TRACE != 0) || (CFG_DEBUG_APP_TRACE != 0) )
#define CFG_DEBUG_TRACE             1
#endif
#define CFG_DEBUG_TRACE_LIGHT     1
#define CFG_DEBUG_TRACE_FULL      1
