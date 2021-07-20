
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"

#include "core2forAWS.h"

#include "motivate_math.h"
#include "maze_tab.h"

#define CANVAS_WIDTH 220
#define CANVAS_HEIGHT 220

#define NORTH_BIT 0x08
#define SOUTH_BIT 0x04
#define WEST_BIT 0x02
#define EAST_BIT 0x01

#define WALL_WIDTH 3
#define WALL_LENGTH 18

#define STATUS_WIDTH 8
#define STATUS_LENGTH 8

#define MOVE_THRESH 30
#define TURN_THRESH 60
#define HOP_THRESH -.2

static lv_color_t *cbuf;
static const char *TAG = MAZE_TAB_NAME;

static float calib_ax = 0.00;
static float calib_ay = 0.00;
static float calib_az = 0.00;
static float calib_a_uv[3];

static int x_current_cell = 0;
static int y_current_cell = 0;
static uint current_dir = 2;

static long last_move_ticks = 0;
static long last_turn_ticks = 0;

void get_pos_from_cell(int x_cell, int y_cell, int *x_pos, int *y_pos)
{
    *x_pos = x_cell * (WALL_LENGTH - WALL_WIDTH);
    *y_pos = y_cell * (WALL_LENGTH - WALL_WIDTH);
}

void get_status_pos_from_cell(int x_cell, int y_cell, int *x_pos, int *y_pos)
{
    *x_pos = x_cell * (WALL_LENGTH - WALL_WIDTH) + (WALL_LENGTH / 2) - (STATUS_WIDTH / 2);
    *y_pos = y_cell * (WALL_LENGTH - WALL_WIDTH) + (WALL_LENGTH / 2) - (STATUS_LENGTH / 2);
}

void draw_status(lv_obj_t *canvas, int status, int x_pos, int y_pos)
{
    lv_draw_rect_dsc_t rect_dsc;
    lv_draw_rect_dsc_init(&rect_dsc);
    if (status == 0)
    {
        rect_dsc.bg_color = LV_COLOR_SILVER;
    }
    else if (status == 1)
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
    else if (status == 5)
    {
        rect_dsc.bg_color = LV_COLOR_PURPLE;
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
    draw_status(
        canvas,
        status,
        x_pos + (WALL_LENGTH / 2) - (STATUS_WIDTH / 2),
        y_pos + (WALL_LENGTH / 2) - (STATUS_LENGTH / 2));
}

void display_maze_tab(lv_obj_t *tv)
{
    down_recal(); //initial cal
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
            uint s = (i * j) % 4;  //All combinations
            int x_pos = (i * (WALL_LENGTH - WALL_WIDTH));
            int y_pos = (j * (WALL_LENGTH - WALL_WIDTH));
            //ESP_LOGI(TAG, "X-%.6i Y-%.6i", x_pos,y_pos);

            if (i == 0)
                t |= WEST_BIT;
            if ((i + 2) * (WALL_LENGTH - WALL_WIDTH) >= CANVAS_WIDTH)
                t |= EAST_BIT;
            if (j == 0)
                t |= NORTH_BIT;
            if ((j + 2) * (WALL_LENGTH - WALL_WIDTH) >= CANVAS_HEIGHT)
                t |= SOUTH_BIT;
            draw_cell(canvas, s, x_pos, y_pos, t);
        }
    }

    static lv_color_t gauge_needle_colors[1];
    gauge_needle_colors[2] = LV_COLOR_BLUE;

    lv_obj_t *gauge_dir = lv_gauge_create(test_tab, NULL);
    lv_obj_set_click(gauge_dir, false);
    lv_obj_set_size(gauge_dir, 80, 80);
    lv_gauge_set_scale(gauge_dir, 330, 4, 0);
    lv_gauge_set_range(gauge_dir, 0, 4);
    lv_gauge_set_needle_count(gauge_dir, 1, gauge_needle_colors);
    lv_obj_set_style_local_image_recolor_opa(gauge_dir, LV_GAUGE_PART_NEEDLE, LV_STATE_DEFAULT, LV_OPA_COVER);

    lv_obj_align(gauge_dir, NULL, LV_ALIGN_OUT_RIGHT_TOP, -80, 20);
    xSemaphoreGive(xGuiSemaphore);

    static lv_obj_t *parms[2];
    parms[0] = canvas;
    parms[1] = gauge_dir;
    xTaskCreatePinnedToCore(maze_task, "MazeTask", 2048, parms, 1, &MAZE_handle, 1);
}

void maze_task(void *pvParameters)
{

    vTaskSuspend(NULL);

    for (;;)
    {
        float gx, gy, gz;
        float ax, ay, az;
        MPU6886_GetAccelData(&ax, &ay, &az);
        MPU6886_GetGyroData(&gx, &gy, &gz);

        float ax_raw[3] = {ax, ay, az};
        float ax_uv[3];
        float ax_cos;
        unit_vect(ax_raw, ax_uv, 3);

        dsps_dotprod_f32(ax_uv, calib_a_uv, &ax_cos, 3);
        long ticks = xTaskGetTickCount();
        //ESP_LOGI(TAG, "ticks-%ld, lmticks-%ld", ticks, last_move_ticks);
        //ESP_LOGI(TAG, "gyro info x-%f y-%f z-%f", gx, gy, gz);

        if (gz < -100. &&
            ticks - last_turn_ticks > TURN_THRESH)
        {
            ESP_LOGI(TAG, "right turn");
            current_dir = current_dir + 1 % 4;
            last_turn_ticks = ticks;

            lv_obj_t **parms = (lv_obj_t **)pvParameters;
            lv_obj_t *gauge = (lv_obj_t *)parms[1];
            lv_gauge_set_value(gauge, 0, current_dir);
        }
        else if (gz > 100. &&
                 ticks - last_turn_ticks > TURN_THRESH)
        {
            ESP_LOGI(TAG, "left turn");
            current_dir = current_dir - 1 % 4;
            last_turn_ticks = ticks;
            lv_obj_t **parms = (lv_obj_t **)pvParameters;
            lv_obj_t *gauge = (lv_obj_t *)parms[1];
            lv_gauge_set_value(gauge, 0, current_dir);
        }
        if (ax_cos < HOP_THRESH && //you hopped
            ticks - last_move_ticks > MOVE_THRESH)
        {
            last_move_ticks = ticks;
            ESP_LOGI(TAG, "Hopped dp-%.6f", ax_cos);

            int x_old_pos, y_old_pos;
            get_status_pos_from_cell(x_current_cell, y_current_cell, &x_old_pos, &y_old_pos);

            if (current_dir == 0)
            {
                y_current_cell -= 1;
            }
            else if (current_dir == 1)
            {
                x_current_cell += 1;
            }
            else if (current_dir == 2)
            {
                y_current_cell += 1;
            }
            else if (current_dir == 3)
            {
                x_current_cell -= 1;
            }

            int x_new_pos, y_new_pos;
            get_status_pos_from_cell(x_current_cell, y_current_cell, &x_new_pos, &y_new_pos);
            if ((x_new_pos < CANVAS_WIDTH) &&
                (y_new_pos < CANVAS_HEIGHT))
            {
                xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);
                lv_obj_t **parms = (lv_obj_t **)pvParameters;
                lv_obj_t *canvas = (lv_obj_t *)parms[0];
                draw_status(canvas, 0, x_old_pos, y_old_pos); //reset status
                draw_status(canvas, 5, x_new_pos, y_new_pos);
                ESP_LOGI(TAG, "time-%ld, old pos x-%i y-%i, new pos x-%i y-%i", ticks, x_old_pos, y_old_pos, x_new_pos, y_new_pos);
                xSemaphoreGive(xGuiSemaphore);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
    vTaskDelete(NULL); // Should never get to here...
}

void down_recal(void)
{
    MPU6886_GetGyroData(&calib_ax, &calib_ay, &calib_az);
    float ax_uv[3] = {calib_ax, calib_ay, calib_az};
    unit_vect(ax_uv, calib_a_uv, 3);
}
