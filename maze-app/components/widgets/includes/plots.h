
#pragma once

#include "lvgl/lvgl.h"


void draw_3_plot(lv_obj_t *canvas, float (*scale_f)(float), void *buf1, void *buf2, void *buf3, int canvas_height, int canvas_width, int plot_win_len);