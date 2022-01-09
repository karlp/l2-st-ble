#pragma once
/**
 * What's really needed here?
 */

#define USE_NULL 1

#if USE_STDIO
/* Let stdio re-targetting handle this directly... ? */
#define TL_MM_DBG_MSG(...)	fprintf(stderr, __VA_ARGS__)
#define TL_HCI_CMD_DBG_MSG(...) fprintf(stderr, __VA_ARGS__)
#define TL_HCI_CMD_DBG_BUF(...) TODO()
#define TL_HCI_CMD_DBG_RAW(...) TODO()
#endif

#if USE_NULL
#define TL_MM_DBG_MSG(...)
#define TL_HCI_CMD_DBG_MSG(...)
#define TL_HCI_CMD_DBG_BUF(...)
#define TL_HCI_CMD_DBG_RAW(...)
#define TL_HCI_EVT_DBG_MSG(...)
#define TL_HCI_EVT_DBG_BUF(...)
#define TL_HCI_EVT_DBG_RAW(...)
#define TL_SHCI_CMD_DBG_MSG(...)
#define TL_SHCI_CMD_DBG_BUF(...)
#define TL_SHCI_CMD_DBG_RAW(...)
#define TL_SHCI_EVT_DBG_MSG(...)
#define TL_SHCI_EVT_DBG_BUF(...)
#define TL_SHCI_EVT_DBG_RAW(...)
#endif