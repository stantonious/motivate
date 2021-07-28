
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
#include "tilt_maze_tab.h"
#include "plots.h"
#include "maze.h"
#include "maze_utils.h"
#include "float_buffer.h"

#define CANVAS_WIDTH 220
#define CANVAS_HEIGHT 220

#define TILT_THRESH .4

#define MOVE_THRESH 40

#define MINI_PLOT_WIDTH 70
#define MINI_PLOT_HEIGHT 70
#define MINI_PLOT_NUM 20

#define WALL_WIDTH 3
#define WALL_LENGTH 18

#define STATUS_WIDTH 8
#define STATUS_LENGTH 8

static lv_color_t *cbuf;
static lv_color_t *miniplotbuf;
static lv_color_t *gyrominiplotbuf;
static const char *TAG = TILT_MAZE_TAB_NAME;

static int x_current_cell = 0;
static int y_current_cell = 13;


static long last_move_ticks = 0;

static void *ax_buf;
static void *ay_buf;
static void *az_buf;

static void *gx_buf;
static void *gy_buf;
static void *gz_buf;

static int TEST_MAZE[MAZE_HEIGHT][MAZE_LEN] = {
    {13, 11, 3, 1, 3, 3, 3, 3, 3, 20, 9, 3, 5, 13},
    {10, 3, 3, 6, 9, 7, 9, 3, 5, 10, 6, 13, 10, 4},
    {9, 1, 7, 9, 2, 3, 6, 13, 8, 3, 5, 8, 3, 6},
    {12, 10, 3, 6, 9, 5, 9, 4, 10, 5, 14, 10, 3, 5},
    {10, 3, 5, 11, 4, 10, 6, 12, 9, 6, 9, 1, 7, 12},
    {11, 5, 10, 5, 8, 5, 9, 6, 10, 5, 12, 10, 5, 12},
    {9, 6, 9, 6, 14, 12, 10, 5, 13, 10, 6, 13, 10, 6},
    {8, 3, 6, 9, 5, 10, 5, 10, 2, 3, 3, 4, 9, 5},
    {10, 3, 3, 6, 10, 5, 12, 9, 3, 3, 5, 10, 6, 12},
    {9, 5, 9, 1, 5, 12, 12, 12, 13, 9, 6, 11, 3, 4},
    {12, 10, 6, 12, 14, 12, 12, 8, 6, 10, 3, 3, 5, 12},
    {10, 3, 5, 12, 9, 6, 10, 6, 9, 3, 5, 9, 6, 12},
    {9, 7, 12, 12, 12, 9, 3, 5, 12, 9, 6, 10, 5, 12},
    {24, 3, 6, 10, 2, 6, 11, 2, 6, 10, 3, 3, 6, 14}

};

void display_tilt_maze_tab(lv_obj_t *tv)
{
    ax_buf = get_buffer();
    ay_buf = get_buffer();
    az_buf = get_buffer();

    gx_buf = get_buffer();
    gy_buf = get_buffer();
    gz_buf = get_buffer();

    xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);
    lv_obj_t *test_tab = lv_tabview_add_tab(tv, TILT_MAZE_TAB_NAME); // Create a tab

    cbuf = (lv_color_t *)heap_caps_malloc(LV_CANVAS_BUF_SIZE_TRUE_COLOR(CANVAS_WIDTH, CANVAS_HEIGHT), MALLOC_CAP_DEFAULT | MALLOC_CAP_SPIRAM);
    lv_obj_t *canvas = lv_canvas_create(test_tab, NULL);
    lv_canvas_set_buffer(canvas, cbuf, CANVAS_WIDTH, CANVAS_HEIGHT, LV_IMG_CF_TRUE_COLOR);

    lv_obj_align(canvas, NULL, LV_ALIGN_IN_LEFT_MID, 5, 0);
    lv_canvas_fill_bg(canvas, LV_COLOR_SILVER, LV_OPA_COVER);

    //mini plot
    miniplotbuf = (lv_color_t *)heap_caps_malloc(LV_CANVAS_BUF_SIZE_TRUE_COLOR(MINI_PLOT_WIDTH, MINI_PLOT_HEIGHT), MALLOC_CAP_DEFAULT | MALLOC_CAP_SPIRAM);
    lv_obj_t *miniplotcanvas = lv_canvas_create(test_tab, NULL);
    lv_canvas_set_buffer(miniplotcanvas, miniplotbuf, MINI_PLOT_WIDTH, MINI_PLOT_HEIGHT, LV_IMG_CF_TRUE_COLOR);
    lv_obj_align(miniplotcanvas, NULL, LV_ALIGN_IN_BOTTOM_RIGHT, -15, -15);
    lv_canvas_fill_bg(miniplotcanvas, LV_COLOR_SILVER, LV_OPA_COVER);

    //gyro mini plot
    gyrominiplotbuf = (lv_color_t *)heap_caps_malloc(LV_CANVAS_BUF_SIZE_TRUE_COLOR(MINI_PLOT_WIDTH, MINI_PLOT_HEIGHT), MALLOC_CAP_DEFAULT | MALLOC_CAP_SPIRAM);
    lv_obj_t *gyrominiplotcanvas = lv_canvas_create(test_tab, NULL);
    lv_canvas_set_buffer(gyrominiplotcanvas, gyrominiplotbuf, MINI_PLOT_WIDTH, MINI_PLOT_HEIGHT, LV_IMG_CF_TRUE_COLOR);
    lv_obj_align(gyrominiplotcanvas, NULL, LV_ALIGN_IN_BOTTOM_RIGHT, -15, -15 - MINI_PLOT_HEIGHT - 2);
    lv_canvas_fill_bg(gyrominiplotcanvas, LV_COLOR_SILVER, LV_OPA_COVER);

    int x_pos, y_pos;
    for (int i = 0; i < 14; i++)
    {
        for (int j = 0; j < 14; j++)
        {
            int s = 0;
            get_pos_from_cell(j, i, &x_pos, &y_pos);
            draw_cell(canvas, s, x_pos, y_pos, TEST_MAZE[i][j], STATUS_WIDTH, STATUS_LENGTH, WALL_WIDTH, WALL_LENGTH);
        }
    }

    int x_init_pos, y_init_pos;
    get_status_pos_from_cell(x_current_cell, y_current_cell, &x_init_pos, &y_init_pos);
    draw_status(canvas, 5, x_init_pos, y_init_pos, STATUS_WIDTH, STATUS_LENGTH);
    xSemaphoreGive(xGuiSemaphore);

    static lv_obj_t *parms[6];
    parms[0] = canvas;
    parms[1] = miniplotcanvas;
    parms[2] = gyrominiplotcanvas;
    xTaskCreatePinnedToCore(tilt_maze_task, "TiltMazeTask", 2048, parms, 1, &TILT_MAZE_handle, 1);
}

float dir_coeffs[] = {.1, .1, .1, .1, .1, .1, .1, .1, .1, .1};

void tilt_maze_task(void *pvParameters)
{

    vTaskSuspend(NULL);
    int x_new_cell, y_new_cell;

    for (;;)
    {
        long ticks = xTaskGetTickCount();
        float gx, gy, gz;
        float ax, ay, az;
        MPU6886_GetAccelData(&ax, &ay, &az);
        MPU6886_GetGyroData(&gx, &gy, &gz);

        //ESP_LOGI(TAG, "acc x:%.6f y:%.6f z:%.6f", ax,ay,az);
        push(ax_buf, ax);
        push(ay_buf, ay);
        push(az_buf, az);

        push(gx_buf, gx);
        push(gy_buf, gy);
        push(gz_buf, gz);

        float x_conv = conv(ax_buf, dir_coeffs, BUFSIZ);
        float y_conv = conv(ay_buf, dir_coeffs, BUFSIZ);


        ESP_LOGI(TAG,"convs x-%.2f y-%.2f",x_conv,y_conv);

        int dir = 0;
        if (x_conv < -TILT_THRESH)
        {
            dir = EAST_DIR;
        }
        else if (x_conv > TILT_THRESH)
        {
            dir = WEST_DIR;
        }
        else if (y_conv < -TILT_THRESH)
        {
            dir = NORTH_DIR;
        }
        else if (y_conv > TILT_THRESH)
        {
            dir = SOUTH_DIR;
        }

        //        ESP_LOGI(TAG, "step conv y:%.6f", y_conv);
        bool moved = can_move(TEST_MAZE, 14, 14, x_current_cell, y_current_cell, &x_new_cell, &y_new_cell, dir);
        if (!moved)
        {
            ESP_LOGI(TAG, "Can't move from [%i,%i] to [%i,%i] dir %i", x_current_cell, y_current_cell, x_new_cell, y_new_cell, dir);
        }
        else if (ticks - last_move_ticks > MOVE_THRESH)
        {
            last_move_ticks = ticks;
            ESP_LOGI(TAG, "moving from [%i,%i] to [%i,%i] dir %i", x_current_cell, y_current_cell, x_new_cell, y_new_cell, dir);
            int x_old_pos, y_old_pos;
            get_status_pos_from_cell(x_current_cell, y_current_cell, &x_old_pos, &y_old_pos);

            int x_new_pos, y_new_pos;
            get_status_pos_from_cell(x_new_cell, y_new_cell, &x_new_pos, &y_new_pos);
            x_current_cell = x_new_cell;
            y_current_cell = y_new_cell;

            if ((x_new_pos < CANVAS_WIDTH) &&
                (y_new_pos < CANVAS_HEIGHT))
            {
                xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);
                lv_obj_t **parms = (lv_obj_t **)pvParameters;
                lv_obj_t *canvas = (lv_obj_t *)parms[0];
                draw_status(canvas, 0, x_old_pos, y_old_pos, STATUS_WIDTH, STATUS_LENGTH); //reset status
                draw_status(canvas, 5, x_new_pos, y_new_pos, STATUS_WIDTH, STATUS_LENGTH);
                ESP_LOGI(TAG, "time-%ld, old pos x-%i y-%i, new pos x-%i y-%i", ticks, x_old_pos, y_old_pos, x_new_pos, y_new_pos);
                xSemaphoreGive(xGuiSemaphore);
            }
        }

        xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);
        lv_obj_t **parms = (lv_obj_t **)pvParameters;
        lv_obj_t *miniplotcanvas = (lv_obj_t *)parms[1];
        lv_obj_t *gyrominiplotcanvas = (lv_obj_t *)parms[2];
        draw_3_plot(miniplotcanvas, &scale_acc, ax_buf, ay_buf, az_buf, MINI_PLOT_HEIGHT, MINI_PLOT_WIDTH, MINI_PLOT_NUM);
        draw_3_plot(gyrominiplotcanvas, &scale_gyro, gx_buf, gy_buf, gz_buf, MINI_PLOT_HEIGHT, MINI_PLOT_WIDTH, MINI_PLOT_NUM);
        xSemaphoreGive(xGuiSemaphore);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelete(NULL); // Should never get to here...
}
