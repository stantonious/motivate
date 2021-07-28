#pragma once

#define TILT_MAZE_TAB_NAME "TILT_MAZE"

TaskHandle_t TILT_MAZE_handle;

void tilt_maze_task(void *pvParameters);
void display_tilt_maze_tab(lv_obj_t* tv);