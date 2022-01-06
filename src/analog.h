#pragma once

void task_kadc(void *pvParameters);
void task_temperature(void *pvParameters);

/* opaque c style struct, cos c++ wat? */
struct ts_adc_t;
extern struct ts_adc_t task_state_adc;

extern TaskHandle_t th_kadc;