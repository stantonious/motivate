
#pragma once

#include "lvgl/lvgl.h"

#include <maze_utils.h>

void draw_status(lv_obj_t *canvas, int status, int x_pos, int y_pos, int status_width, int status_length);

void draw_cell(lv_obj_t *canvas, int status, int x_pos, int y_pos, int type, int dir, int status_width, int status_length, int wall_width, int wall_length);

void draw_maze(lv_obj_t *canvas, int maze[MAZE_HEIGHT][MAZE_LEN], int x_maze_len, int y_maze_len, int dir,int x_map_center,int y_map_center);