#pragma once

/**
 * This is called by C code from ST middleware.
 */
#ifdef __cplusplus
extern "C" {
#endif
	void DbgTraceBuffer( const void *pBuffer , uint32_t u32Length , const char *strFormat , ... );
#ifdef __cplusplus	
}
#endif

#define USE_NULL 0
#define USE_STDIO 1

// Buffers can be pretty verbose, don't bother including them normally.
#define DEBUG_BUFS 0

#if USE_STDIO
/* Let stdio re-targetting handle this directly... ? */
#define TL_MM_DBG_MSG(...)	fprintf(stderr, __VA_ARGS__)
#define TL_HCI_CMD_DBG_MSG(...) fprintf(stderr, __VA_ARGS__)
#define TL_HCI_CMD_DBG_RAW(...)
#define TL_HCI_EVT_DBG_MSG(...) fprintf(stderr, __VA_ARGS__)
#define TL_HCI_EVT_DBG_RAW(...)
#define TL_SHCI_CMD_DBG_MSG(...) fprintf(stderr, __VA_ARGS__)
#define TL_SHCI_CMD_DBG_RAW(...)
#define TL_SHCI_EVT_DBG_MSG(...) fprintf(stderr, __VA_ARGS__)
#define TL_SHCI_EVT_DBG_RAW(...)
#if (DEBUG_BUFS == 1)
#define TL_HCI_CMD_DBG_BUF(...) DbgTraceBuffer(__VA_ARGS__)
#define TL_HCI_EVT_DBG_BUF(...) DbgTraceBuffer(__VA_ARGS__)
#define TL_SHCI_CMD_DBG_BUF(...) DbgTraceBuffer(__VA_ARGS__)
#define TL_SHCI_EVT_DBG_BUF(...) DbgTraceBuffer(__VA_ARGS__)
#else
#define TL_HCI_CMD_DBG_BUF(...)
#define TL_HCI_EVT_DBG_BUF(...)
#define TL_SHCI_CMD_DBG_BUF(...)
#define TL_SHCI_EVT_DBG_BUF(...)
#endif
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