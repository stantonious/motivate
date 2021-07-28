#pragma once
#define PLOT_TAB_NAME "PLOT"

TaskHandle_t PLOT_handle;

void plot_task(void *pvParameters);
void display_plot_tab(lv_obj_t* tv);