#pragma once

#define DETAILS_TAB_NAME "DETAILS"

TaskHandle_t DETAILS_handle;

void display_details_tab(lv_obj_t* tv);
void DETAILS_task(void *pvParameters);
void recal(void);