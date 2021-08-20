#pragma once

#define MAZE_TAB_NAME "MAZE"

TaskHandle_t MAZE_handle;

void maze_task(void *pvParameters);
void display_maze_tab(lv_obj_t* tv);
void down_recal(void);
void reset_north(void);
void toggle_left_record(void);
void toggle_right_record(void);
void toggle_foward_record(void);