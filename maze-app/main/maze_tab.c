
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"

#include "core2forAWS.h"

#include "maze_tab.h"

#define CANVAS_WIDTH 220
#define CANVAS_HEIGHT 220

#define NORTH_BIT 0x08
#define SOUTH_BIT 0x04
#define WEST_BIT 0x02
#define EAST_BIT 0x01

#define WALL_WIDTH 2
#define WALL_LENGTH 8

#define STATUS_WIDTH 4
#define STATUS_LENGTH 4

static lv_color_t *cbuf;
static const char *TAG = MAZE_TAB_NAME;

void draw_status(lv_obj_t *canvas, int status, int x_pos, int y_pos)
{
    lv_draw_rect_dsc_t rect_dsc;
    lv_draw_rect_dsc_init(&rect_dsc);

    if (status == 1)
    {
        rect_dsc.bg_color = LV_COLOR_YELLOW;
    }
    else if (status == 2)
    {
        rect_dsc.bg_color = LV_COLOR_MAROON;
    }
    else if (status == 3)
    {
        rect_dsc.bg_color = LV_COLOR_GREEN;
    }
    else if (status == 4)
    {
        rect_dsc.bg_color = LV_COLOR_BLUE;
    }
    else
    {
        return;
    }
    lv_canvas_draw_rect(
        canvas,
        x_pos,
        y_pos,
        STATUS_WIDTH,
        STATUS_LENGTH,
        &rect_dsc);
}

void draw_cell(lv_obj_t *canvas, int status, int x_pos, int y_pos, int type)
{

    lv_draw_rect_dsc_t rect_dsc;
    lv_draw_rect_dsc_init(&rect_dsc);
    rect_dsc.bg_color = LV_COLOR_RED;

    if (type & NORTH_BIT)
    {
        lv_canvas_draw_rect(
            canvas,
            x_pos,
            y_pos,
            WALL_LENGTH,
            WALL_WIDTH, &rect_dsc);
    }
    if (type & SOUTH_BIT)
    {
        lv_canvas_draw_rect(
            canvas,
            x_pos,
            y_pos + WALL_LENGTH - WALL_WIDTH,
            WALL_LENGTH,
            WALL_WIDTH, &rect_dsc);
    }
    if (type & WEST_BIT)
    {
        lv_canvas_draw_rect(
            canvas,
            x_pos,
            y_pos,
            WALL_WIDTH,
            WALL_LENGTH, &rect_dsc);
    }
    if (type & EAST_BIT)
    {
        lv_canvas_draw_rect(
            canvas,
            x_pos + WALL_LENGTH - WALL_WIDTH,
            y_pos,
            WALL_WIDTH,
            WALL_LENGTH, &rect_dsc);
    }
    draw_status(canvas,status,x_pos+WALL_WIDTH,y_pos+WALL_WIDTH);
}

void display_maze_tab(lv_obj_t *tv)
{
    xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);
    lv_obj_t *test_tab = lv_tabview_add_tab(tv, MAZE_TAB_NAME); // Create a tab

    cbuf = (lv_color_t *)heap_caps_malloc(LV_CANVAS_BUF_SIZE_TRUE_COLOR(CANVAS_WIDTH, CANVAS_HEIGHT), MALLOC_CAP_DEFAULT | MALLOC_CAP_SPIRAM);
    lv_obj_t *canvas = lv_canvas_create(test_tab, NULL);
    lv_canvas_set_buffer(canvas, cbuf, CANVAS_WIDTH, CANVAS_HEIGHT, LV_IMG_CF_TRUE_COLOR);

    lv_obj_align(canvas, NULL, LV_ALIGN_CENTER, 0, 0);
    lv_canvas_fill_bg(canvas, LV_COLOR_SILVER, LV_OPA_COVER);

    for (int i = 0; i < CANVAS_WIDTH / (WALL_LENGTH - WALL_WIDTH); i++)
    {
        for (int j = 0; j < CANVAS_HEIGHT / (WALL_LENGTH - WALL_WIDTH); j++)
        {
            uint t = (i * j) % 16; //All combinations
            uint s = (i * j) % 16; //All combinations
            int x_pos = (i * (WALL_LENGTH - WALL_WIDTH));
            int y_pos = (j * (WALL_LENGTH - WALL_WIDTH));
            //ESP_LOGI(TAG, "X-%.6i Y-%.6i", x_pos,y_pos);

            draw_cell(canvas, s, x_pos, y_pos, t);
        }
    }
    xSemaphoreGive(xGuiSemaphore);
}