
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"

#include "core2forAWS.h"

#include "imu_task.h"
//#include "arrow-img-30x30.h"
#include "motivate_math.h"
#include "maze_tab.h"
#include "plots.h"
#include "maze_utils.h"
#include "maze.h"
#include "float_buffer.h"

#include "mot_mqtt_client.h"
#include "mot-imu-tf.h"
#include "game_tab.h"

#define MINI_PLOT_WIDTH 70
#define MINI_PLOT_HEIGHT 70
#define MINI_PLOT_NUM 20


static lv_color_t *cbuf;
static lv_color_t *minimapbuf;
static lv_color_t *miniplotbuf;
static lv_color_t *gyrominiplotbuf;
static const char *TAG = MAZE_TAB_NAME;

static float **abuf = NULL;
static float **gbuf = NULL;

static int x_current_cell = 0;
static int y_current_cell = 13;
static uint map_projection = NORTH_DIR;

static bool redraw_dir = true;

static long last_move_ticks = 0;

static int8_t last_test_x = -1;
static int8_t last_test_y = -1;

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

void display_maze_tab(lv_obj_t *tv)
{
    abuf = (float **)malloc(3 * sizeof(float *));
    for (int i = 0; i < 3; i++)
        abuf[i] = (float *)malloc(BUFSIZE * sizeof(float));
    gbuf = (float **)malloc(3 * sizeof(float *));
    for (int i = 0; i < 3; i++)
        gbuf[i] = (float *)malloc(BUFSIZE * sizeof(float));

    xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);
    lv_obj_t *test_tab = lv_tabview_add_tab(tv, MAZE_TAB_NAME); // Create a tab

    cbuf = (lv_color_t *)heap_caps_malloc(LV_CANVAS_BUF_SIZE_TRUE_COLOR(CANVAS_WIDTH, CANVAS_HEIGHT), MALLOC_CAP_DEFAULT | MALLOC_CAP_SPIRAM);
    lv_obj_t *canvas = lv_canvas_create(test_tab, NULL);
    lv_canvas_set_buffer(canvas, cbuf, CANVAS_WIDTH, CANVAS_HEIGHT, LV_IMG_CF_TRUE_COLOR);

    lv_obj_align(canvas, NULL, LV_ALIGN_IN_LEFT_MID, 5, 0);
    lv_canvas_fill_bg(canvas, LV_COLOR_SILVER, LV_OPA_COVER);

    //mini plot
    minimapbuf = (lv_color_t *)heap_caps_malloc(LV_CANVAS_BUF_SIZE_TRUE_COLOR(MINI_PLOT_WIDTH, MINI_PLOT_HEIGHT), MALLOC_CAP_DEFAULT | MALLOC_CAP_SPIRAM);
    lv_obj_t *minimapcanvas = lv_canvas_create(test_tab, NULL);
    lv_canvas_set_buffer(minimapcanvas, minimapbuf, MINI_PLOT_WIDTH, MINI_PLOT_HEIGHT, LV_IMG_CF_TRUE_COLOR);
    lv_obj_align(minimapcanvas, NULL, LV_ALIGN_IN_BOTTOM_RIGHT, -15, -15 - (2* MINI_PLOT_HEIGHT) - 4);
    lv_canvas_fill_bg(minimapcanvas, LV_COLOR_SILVER, LV_OPA_COVER);

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

    //inf label
    lv_obj_t *inf_lbl = lv_label_create(test_tab, NULL);
    lv_label_set_text(inf_lbl, "Unknown ");
    lv_obj_align(inf_lbl, NULL, LV_ALIGN_IN_TOP_RIGHT, 0, 0);

    draw_maze(canvas, TEST_MAZE, 14, 14, NORTH_DIR, x_current_cell, y_current_cell);

    int x_init_pos, y_init_pos;
    get_status_pos_from_cell(x_current_cell, y_current_cell, map_projection, &x_init_pos, &y_init_pos, x_current_cell, y_current_cell);
    draw_status(canvas, 5, x_init_pos, y_init_pos, STATUS_WIDTH, STATUS_LENGTH);
    xSemaphoreGive(xGuiSemaphore);

    static lv_obj_t *parms[6];
    parms[0] = canvas;
    parms[1] = inf_lbl;
    parms[2] = miniplotcanvas;
    parms[3] = gyrominiplotcanvas;
    parms[4] = minimapcanvas;
    xTaskCreatePinnedToCore(maze_task, "MazeTask", 2048 * 2, parms, 1, &MAZE_handle, 1);
}

//float step_coefs[] = {1., 1., 1., -1., -1};
float step_coefs[] = {.2, .2, .2, .2, .2};
void maze_task(void *pvParameters)
{

    lv_obj_t **parms = (lv_obj_t **)pvParameters;
    lv_obj_t *canvas = (lv_obj_t *)parms[0];
    lv_obj_t *inf_lbl = (lv_obj_t *)parms[1];
    lv_obj_t *miniplotcanvas = (lv_obj_t *)parms[2];
    lv_obj_t *gyrominiplotcanvas = (lv_obj_t *)parms[3];
    lv_obj_t *minimapcanvas = (lv_obj_t *)parms[4];

    vTaskSuspend(NULL);

    for (;;)
    {
        long ticks = xTaskGetTickCount();
        long update_delta = ticks - last_move_ticks;
        bool moved = false;
        int y_new_cell, x_new_cell;

        if (update_delta > get_sensitivity())
        {
            last_move_ticks = ticks;

            xSemaphoreTake(xImuSemaphore, portMAX_DELAY);

            int inf = buffer_infer(
                ax_buf,
                ay_buf,
                az_buf,
                gx_buf,
                gy_buf,
                gz_buf);
            xSemaphoreGive(xImuSemaphore);

            xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);
            switch (inf)
            {
            case REST_LABEL:
                lv_label_set_text(inf_lbl, "REST ");
                break;
            case FORWARD_LABEL:
                lv_label_set_text(inf_lbl, "Forward ");
                break;
            case BACKWARD_LABEL:
                lv_label_set_text(inf_lbl, "Backward ");
                break;
            case UP_LABEL:
                lv_label_set_text(inf_lbl, "Up ");
                break;
            case DOWN_LABEL:
                lv_label_set_text(inf_lbl, "Down ");
                break;
            case LEFT_LABEL:
                lv_label_set_text(inf_lbl, "Left ");
                break;
            case RIGHT_LABEL:
                lv_label_set_text(inf_lbl, "Right ");
                break;
            }
            xSemaphoreGive(xGuiSemaphore);

            switch (inf)
            {
            case FORWARD_LABEL:
                ESP_LOGI(TAG, "Moving %d", inf);
                int move_dir = map_projection;
                if (map_projection == EAST_DIR)
                    move_dir = WEST_DIR;
                else if (map_projection == WEST_DIR)
                    move_dir = EAST_DIR;
                moved = can_move(TEST_MAZE, 14, 14, x_current_cell, y_current_cell, &x_new_cell, &y_new_cell, move_dir);
                if (!moved)
                    ESP_LOGI(TAG, "Can't move from [%i,%i] to [%i,%i] dir %i", x_current_cell, y_current_cell, x_new_cell, y_new_cell, map_projection);
                break;
            case BACKWARD_LABEL:
                ESP_LOGI(TAG, "Moving %d", inf);
                int back_dir = (map_projection - 2) % 4;
                moved = can_move(TEST_MAZE, 14, 14, x_current_cell, y_current_cell, &x_new_cell, &y_new_cell, back_dir);
                if (!moved)
                    ESP_LOGI(TAG, "Can't move from [%i,%i] to [%i,%i] dir %i", x_current_cell, y_current_cell, x_new_cell, y_new_cell, map_projection);
                break;
            case LEFT_LABEL:
                ESP_LOGI(TAG, "Moving %d", inf);
                map_projection = (map_projection + 1) % 4;
                ESP_LOGI(TAG, "new current dir  %d", map_projection);
                redraw_dir = true;
                break;
            case RIGHT_LABEL:
                ESP_LOGI(TAG, "Moving %d", inf);
                map_projection = (map_projection - 1) % 4;
                ESP_LOGI(TAG, "new current dir  %d", map_projection);
                redraw_dir = true;
                break;
            case UP_LABEL:
                ESP_LOGI(TAG, "Moving %d", inf);
                break;
            case DOWN_LABEL:
                ESP_LOGI(TAG, "Moving %d", inf);
                break;
            }
        }

        xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);
        if (redraw_dir)
        {
            redraw_dir = false;
            draw_maze(canvas, TEST_MAZE, 14, 14, map_projection, x_current_cell, y_current_cell);
            int x_pos, y_pos;
            get_status_pos_from_cell(x_current_cell, y_current_cell, map_projection, &x_pos, &y_pos, x_current_cell, y_current_cell);
            draw_status(canvas, 5, x_pos, y_pos, STATUS_WIDTH, STATUS_LENGTH);
        }

        if (moved)
        {
            int x_new_pos = 0;
            int y_new_pos = 0;
            ESP_LOGI(TAG, "moving from [%i,%i] to [%i,%i] dir %i", x_current_cell, y_current_cell, x_new_cell, y_new_cell, map_projection);
            last_move_ticks = ticks;

            //Reset static status
            draw_static_maze(minimapcanvas,MINI_PLOT_WIDTH,MINI_PLOT_HEIGHT,TEST_MAZE,MAZE_LEN,MAZE_HEIGHT);
            get_static_status_pos_from_cell(x_current_cell, y_current_cell, MINI_PLOT_WIDTH/MAZE_LEN,1,3,3, &x_new_pos, &y_new_pos);
            draw_status(minimapcanvas,0,x_new_pos,y_new_pos,3,3);
            x_current_cell = x_new_cell;
            y_current_cell = y_new_cell;

            get_static_status_pos_from_cell(x_new_cell, y_new_cell, MINI_PLOT_WIDTH/MAZE_LEN,1,3,3, &x_new_pos, &y_new_pos);
            draw_status(minimapcanvas,5,x_new_pos,y_new_pos,3,3);

            draw_maze(canvas, TEST_MAZE, MAZE_LEN, MAZE_HEIGHT, map_projection, x_current_cell, y_current_cell);
            get_status_pos_from_cell(x_new_cell, y_new_cell, map_projection, &x_new_pos, &y_new_pos, x_new_cell, y_new_cell);
            draw_status(canvas, 5, x_new_pos, y_new_pos, STATUS_WIDTH, STATUS_LENGTH); //reset status
        }

        int8_t op_x = 0;
        int8_t op_y = 0;
        get_op_x_y(&op_x, &op_y);
        if (op_x >= 0 && op_y >= 0 && (last_test_x != op_x || last_test_y != op_y))
        {
            int x_pos, y_pos;
            get_status_pos_from_cell(last_test_x, last_test_y, map_projection, &x_pos, &y_pos, x_current_cell, y_current_cell);
            draw_status(canvas, 0, x_pos, y_pos, STATUS_WIDTH, STATUS_LENGTH);
            get_status_pos_from_cell(op_x, op_y, map_projection, &x_pos, &y_pos, x_current_cell, y_current_cell);
            draw_status(canvas, 6, x_pos, y_pos, STATUS_WIDTH, STATUS_LENGTH);
            last_test_x = op_x;
            last_test_y = op_y;
        }
        draw_3_plot(miniplotcanvas, &scale_acc, ax_buf, ay_buf, az_buf, MINI_PLOT_HEIGHT, MINI_PLOT_WIDTH, MINI_PLOT_NUM);
        draw_3_plot(gyrominiplotcanvas, &scale_gyro, gx_buf, gy_buf, gz_buf, MINI_PLOT_HEIGHT, MINI_PLOT_WIDTH, MINI_PLOT_NUM);

        xSemaphoreGive(xGuiSemaphore);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    vTaskDelete(NULL); // Should never get to here...
}