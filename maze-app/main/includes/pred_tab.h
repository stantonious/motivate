#pragma once

#define PRED_TAB_NAME "PRED"

TaskHandle_t Pred_handle;

void pred_task(void *pvParameters);
void display_pred_tab(lv_obj_t* tv);