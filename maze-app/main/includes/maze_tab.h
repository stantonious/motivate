#pragma once

#define MAZE_TAB_NAME "MAZE"

#include "maze_utils.h"
TaskHandle_t MAZE_handle;

void maze_task(void *pvParameters);
void display_maze_tab(lv_obj_t* tv);

extern int MAZE[MAZE_HEIGHT][MAZE_LEN];
extern bool infer;