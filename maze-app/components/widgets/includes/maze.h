
#pragma once

#include "lvgl/lvgl.h"

void draw_status(lv_obj_t *canvas, int status, int x_pos, int y_pos,int status_width, int status_length);

void draw_cell(lv_obj_t *canvas, int status, int x_pos, int y_pos, int type,int status_width, int status_length, int wall_width, int wall_length);