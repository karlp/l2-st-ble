#pragma once

#include "shci.h"
#include "tl_dbg_conf.h"

/** Lets the BLE stack do some presetup...  (this screams for a constructor? */
void ble_pre(void);

void APP_BLE_Init(SHCI_C2_Ble_Init_Cmd_Packet_t *ble_init_cmd_packet, void(*user_func)(void*));

void task_ble_setup(void(*user_func)(void*));
void task_ble(void *pvParameters);

/* opaque c style struct, cos c++ wat? */
struct ts_ble_t;
extern struct ts_ble_t task_state_ble;

extern TaskHandle_t th_ble;
