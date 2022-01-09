#pragma once

/** Lets the BLE stack do some presetup...  (this screams for a constructor? */
void ble_pre(void);
/** analogous to appe_Tl_Init(), sets up the mailboxes and shit.. */
void kble_tl_init();

void task_ble(void *pvParameters);

/* opaque c style struct, cos c++ wat? */
struct ts_ble_t;
extern struct ts_ble_t task_state_ble;

extern TaskHandle_t th_ble;