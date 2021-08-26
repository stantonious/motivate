#pragma once

#define ETCH_TAB_NAME "ETCH"

TaskHandle_t Etch_handle;

void etch_task(void *pvParameters);
void display_etch_tab(lv_obj_t* tv);