#pragma once

#define MAZE_TAB_NAME "MAZE"

TaskHandle_t MAZE_handle;

void maze_task(void *pvParameters);
void display_maze_tab(lv_obj_t* tv);