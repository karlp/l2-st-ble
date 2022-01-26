/*
 * ~equivalent to ST's hrs_app.c, the "app" side corresponding
 * to the middleware's svc/hrs.c
 */

#include <cstdio>
#include <stdint.h>

#include "ble_types.h"
#include "hrs.h"
#include "hrs_app.h"

void HRSAPP_Init(void) {
	printf("HRS: App init\n");
}

extern "C" {
	void HRS_Notification(HRS_App_Notification_evt_t *pNotification) {
		printf("hrs not: lol, implementation wat?\n");
	}
}