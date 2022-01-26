/*
 * ~equivalent to ST's hrs_app.c, the "app" side corresponding
 * to the middleware's svc/hrs.c
 */

#include <climits>
#include <cstdio>
#include <stdint.h>

#include "ble.h"
#include "ble_types.h"
#include "uuid.h"
#include "hrs.h"
#include "hrs_app.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

static TaskHandle_t th_hrs;

struct task_state_hrs_t {
	HRS_BodySensorLocation_t char_location;
	HRS_MeasVal_t char_meas;
	bool reset_energy_expended;
	TimerHandle_t timer;
	TickType_t update_interval;
};
static struct task_state_hrs_t state;

/**
 * Called from WPAN middleware.
 * @param pNotification
 */
void hrs_handle_notification(HRS_App_Notification_evt_t *pNotification)
{
	BaseType_t rc;
	switch(pNotification->HRS_Evt_Opcode) {
	case HRS_RESET_ENERGY_EXPENDED_EVT:
		state.char_meas.EnergyExpended = 0;
		break;
	case HRS_NOTIFICATION_ENABLED:
		rc = xTimerReset(state.timer, state.update_interval);
		configASSERT(rc == pdPASS);
		break;
	case HRS_NOTIFICATION_DISABLED:
		rc = xTimerStop(state.timer, pdMS_TO_TICKS(3));
		configASSERT(rc == pdPASS);
		break;
	case HRS_STM_BOOT_REQUEST_EVT:
		// FIXME writes magic to that reserved word at the start of sram!
		// and then straight up reboots!
		printf("Unhandled: OTA reboot\n");
		break;
	default:
		printf("Ignoring unhandled HRS notification\n");
	}
}

/**
 * This just bounces out to the task again when we're being requested
 * to provide updates.
 * @param xTimer
 */
static void hrs_timer_handle(TimerHandle_t xTimer)
{
	xTaskNotifyGive(th_hrs);
}

static void hrs_update_measurements(struct task_state_hrs_t *st)
{
	static uint32_t measurement = 42;

	//measurement = ((HRSAPP_Read_RTC_SSR_SS()) & 0x07) + 65;
	measurement++;
	measurement = measurement > 80 ? 40 : measurement;

	st->char_meas.MeasurementValue = measurement;
#if (BLE_CFG_HRS_ENERGY_EXPENDED_INFO_FLAG != 0)
	if (st->char_meas.Flags & HRS_HRM_ENERGY_EXPENDED_PRESENT) {
		st->char_meas.EnergyExpended += 5;
	}
#endif

	HRS_UpdateChar(HEART_RATE_MEASURMENT_UUID, (uint8_t *) & st->char_meas);

	return;
}

void task_hrs(void *parameters)
{
	struct task_state_hrs_t *st = (struct task_state_hrs_t *)parameters;
	st->char_location = HRS_BODY_SENSOR_LOCATION_HAND;
	st->update_interval = pdMS_TO_TICKS(1000);
	HRS_UpdateChar(SENSOR_LOCATION_UUID, (uint8_t*) &st->char_location);

	/**
	 * Set Flags for measurement value
	 */
	st->char_meas.Flags = (HRS_HRM_VALUE_FORMAT_UINT16 |
		HRS_HRM_SENSOR_CONTACTS_PRESENT |
		HRS_HRM_SENSOR_CONTACTS_SUPPORTED |
		HRS_HRM_ENERGY_EXPENDED_PRESENT |
		HRS_HRM_RR_INTERVAL_PRESENT);

#if (BLE_CFG_HRS_ENERGY_EXPENDED_INFO_FLAG != 0)
	if (state.char_meas.Flags & HRS_HRM_ENERGY_EXPENDED_PRESENT)
		state.char_meas.EnergyExpended = 10;
#endif

#if (BLE_CFG_HRS_ENERGY_RR_INTERVAL_FLAG != 0)
	if (state.char_meas.Flags & HRS_HRM_RR_INTERVAL_PRESENT) {
		uint8_t i;

		state.char_meas.NbreOfValidRRIntervalValues = BLE_CFG_HRS_ENERGY_RR_INTERVAL_FLAG;
		for (i = 0; i < BLE_CFG_HRS_ENERGY_RR_INTERVAL_FLAG; i++)
			state.char_meas.aRRIntervalValues[i] = 1024;
	}
#endif 

	// Make a timer we'll use
	st->timer = xTimerCreate("hrs", st->update_interval, pdTRUE, NULL, &hrs_timer_handle);

	// then, just wait!
	while (1) {
		xTaskNotifyWait(ULONG_MAX, 0, NULL, portMAX_DELAY);
		hrs_update_measurements(st);
	}
}

/**
 * C style simple call to start things, keep things consistent with ST example
 */
void HRSAPP_Init(void)
{
	printf("HRS: App init\n");
	BaseType_t rc = xTaskCreate(task_hrs, "hrs", configMINIMAL_STACK_SIZE * 3, &state, tskIDLE_PRIORITY + 1, &th_hrs);
	configASSERT(rc == pdPASS);
}

extern "C" {

	void HRS_Notification(HRS_App_Notification_evt_t *pNotification)
	{
		hrs_handle_notification(pNotification);
	}
}