
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

#define MINI_PLOT_WIDTH 88
#define MINI_PLOT_HEIGHT 88 //multiple of MAZE_LEN

static lv_color_t *cbuf;
static lv_color_t *minimapbuf;
static const char *TAG = MAZE_TAB_NAME;

static int x_current_cell = 0;
static int y_current_cell = 0;
static uint map_projection = NORTH_DIR;

static bool redraw_dir = true;

static int8_t last_test_x = -1;
static int8_t last_test_y = -1;

bool infer = true;

int MAZE[MAZE_HEIGHT][MAZE_LEN] = {
    {13, 11, 3, 1, 3, 3, 3, 3, 3, 68, 9, 3, 5, 13},
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
    {9, 167, 12, 12, 12, 9, 3, 5, 12, 9, 6, 10, 5, 12},
    {104, 131, 214, 250, 2, 6, 11, 2, 6, 10, 3, 3, 6, 14}

};
/*
static int MAZE[MAZE_HEIGHT][MAZE_LEN] = {
    {155, 181, 9, 3, 3, 3, 3, 7, 9, 3, 3, 3, 1, 5}, 
    {13, 12, 8, 5, 9, 5, 9, 3, 6, 9, 1, 7, 12, 12}, 
    {12, 12, 174, 10, 6, 10, 2, 3, 3, 4, 14, 9, 6, 12}, 
    {12, 10, 131, 3, 3, 3, 5, 11, 3, 6, 9, 6, 13, 12}, 
    {8, 3, 1, 3, 147, 181, 12, 9, 3, 5, 12, 13, 8, 6}, 
    {12, 13, 10, 3, 5, 14, 10, 6, 13, 12, 12, 10, 2, 5}, 
    {12, 10, 1, 5, 10, 3, 145, 179, 4, 12, 10, 5, 9, 6}, 
    {10, 3, 6, 12, 11, 5, 10, 5, 14, 10, 3, 6, 12, 13}, 
    {9, 3, 5, 10, 3, 0, 7, 12, 153, 179, 3, 3, 6, 12}, 
    {10, 5, 10, 3, 3, 4, 9, 4, 12, 11, 1, 5, 9, 6}, 
    {13, 12, 9, 3, 5, 14, 12, 14, 12, 9, 166, 12, 8, 5}, 
    {8, 6, 10, 5, 10, 3, 6, 9, 6, 12, 137, 6, 14, 12}, 
    {2, 3, 5, 12, 9, 3, 5, 12, 11, 4, 12, 9, 147, 180}, 
    {9, 3, 6, 10, 2, 7, 10, 2, 3, 6, 10, 6, 11, 6}};
*/
static int step_cnt = 0;

void display_maze_tab(lv_obj_t *tv)
{
    xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);
    lv_obj_t *test_tab = lv_tabview_add_tab(tv, MAZE_TAB_NAME); // Create a tab

    cbuf = (lv_color_t *)heap_caps_malloc(LV_CANVAS_BUF_SIZE_TRUE_COLOR(CANVAS_WIDTH, CANVAS_HEIGHT), MALLOC_CAP_DEFAULT | MALLOC_CAP_SPIRAM);
    lv_obj_t *canvas = lv_canvas_create(test_tab, NULL);
    lv_canvas_set_buffer(canvas, cbuf, CANVAS_WIDTH, CANVAS_HEIGHT, LV_IMG_CF_TRUE_COLOR);

    lv_obj_align(canvas, NULL, LV_ALIGN_IN_TOP_LEFT, 5, 5);
    lv_canvas_fill_bg(canvas, LV_COLOR_SILVER, LV_OPA_COVER);

    //mini plot
    minimapbuf = (lv_color_t *)heap_caps_malloc(LV_CANVAS_BUF_SIZE_TRUE_COLOR(MINI_PLOT_WIDTH, MINI_PLOT_HEIGHT), MALLOC_CAP_DEFAULT | MALLOC_CAP_SPIRAM);
    lv_obj_t *minimapcanvas = lv_canvas_create(test_tab, NULL);
    lv_canvas_set_buffer(minimapcanvas, minimapbuf, MINI_PLOT_WIDTH, MINI_PLOT_HEIGHT, LV_IMG_CF_TRUE_COLOR);
    lv_obj_align(minimapcanvas, NULL, LV_ALIGN_IN_TOP_RIGHT, -5, 5);
    lv_canvas_fill_bg(minimapcanvas, LV_COLOR_SILVER, LV_OPA_COVER);


    //steps label
    lv_obj_t *steps_lbl = lv_label_create(test_tab, NULL);
    lv_label_set_text(steps_lbl, "Steps:0");
    lv_obj_align(steps_lbl, NULL, LV_ALIGN_IN_TOP_RIGHT, -40, MINI_PLOT_HEIGHT + 5);

    //inf label
    lv_obj_t *inf_lbl = lv_label_create(test_tab, NULL);
    lv_label_set_text(inf_lbl, "Unknown ");
    lv_obj_align(inf_lbl, NULL, LV_ALIGN_IN_TOP_RIGHT, -15, MINI_PLOT_HEIGHT + 25);

    draw_maze(canvas, MAZE, MAZE_LEN, MAZE_HEIGHT, NORTH_DIR, x_current_cell, y_current_cell);
    draw_static_maze(minimapcanvas, MINI_PLOT_WIDTH, MINI_PLOT_HEIGHT, MAZE, MAZE_LEN, MAZE_HEIGHT);

    int x_init_pos, y_init_pos;
    get_status_pos_from_cell(x_current_cell, y_current_cell, map_projection, &x_init_pos, &y_init_pos, x_current_cell, y_current_cell);
    draw_status(canvas, 5, x_init_pos, y_init_pos, STATUS_WIDTH, STATUS_LENGTH);
    xSemaphoreGive(xGuiSemaphore);

    //inf led
    lv_obj_t *inf_led = lv_led_create(test_tab, NULL);
    lv_obj_align(inf_led, NULL, LV_ALIGN_IN_BOTTOM_RIGHT, -5, -5);
    lv_obj_set_size(inf_led, 20, 20);
    if (infer)
        lv_led_on(inf_led);
    else
        lv_led_off(inf_led);

    static lv_obj_t *maze_parms[5];
    maze_parms[0] = canvas;
    maze_parms[1] = inf_lbl;
    maze_parms[2] = minimapcanvas;
    maze_parms[3] = steps_lbl;
    maze_parms[4] = inf_led;
    xTaskCreatePinnedToCore(maze_task, "MazeTask", 2048 * 2, maze_parms, 1, &MAZE_handle, 1);
}

//float step_coefs[] = {1., 1., 1., -1., -1};
float step_coefs[] = {.2, .2, .2, .2, .2};
void maze_task(void *pvParameters)
{

    long last_move_ticks = 0;
    long last_turn_ticks = 0;

    lv_obj_t **maze_parms = (lv_obj_t **)pvParameters;
    lv_obj_t *canvas = (lv_obj_t *)maze_parms[0];
    lv_obj_t *inf_lbl = (lv_obj_t *)maze_parms[1];
    lv_obj_t *minimapcanvas = (lv_obj_t *)maze_parms[2];
    lv_obj_t *steps_lbl = (lv_obj_t *)maze_parms[3];
    lv_obj_t *inf_led = (lv_obj_t *)maze_parms[4];

    vTaskSuspend(NULL);

    for (;;)
    {
        long ticks = xTaskGetTickCount();
        long update_delta = ticks - last_move_ticks;
        long turn_delta = ticks - last_turn_ticks;
        int y_new_cell, x_new_cell;
        bool moved = false;

        if (infer && turn_delta > get_turn_sensitivity())
        {
            last_turn_ticks = ticks;

            int inf = get_latest_inf(4);

            xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);
            switch (inf)
            {
            case LEFT_LABEL:
                lv_label_set_text(inf_lbl, "Left ");
                ESP_LOGI(TAG, "Moving %d", inf);
                map_projection = (map_projection + 1) % 4;
                ESP_LOGI(TAG, "new current dir  %d", map_projection);
                redraw_dir = true;
                break;
            case RIGHT_LABEL:
                lv_label_set_text(inf_lbl, "Right ");
                ESP_LOGI(TAG, "Moving %d", inf);
                map_projection = (map_projection + 4 - 1) % 4;
                ESP_LOGI(TAG, "new current dir  %d", map_projection);
                redraw_dir = true;
                break;
            }
            xSemaphoreGive(xGuiSemaphore);
        }

        if (infer && update_delta > get_move_sensitivity())
        {
            last_move_ticks = ticks;
            moved = false;

            int inf = get_latest_inf(4);

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
            case LEFTSIDE_LABEL:
                lv_label_set_text(inf_lbl, "Left Slide");
                break;
            case RIGHTSIDE_LABEL:
                lv_label_set_text(inf_lbl, "Right Slide");
                break;
            case UNCERTAIN_LABEL:
                lv_label_set_text(inf_lbl, "???");
                break;
            }
            xSemaphoreGive(xGuiSemaphore);

            int move_dir = map_projection;
            if (map_projection == EAST_DIR)
                move_dir = WEST_DIR;
            else if (map_projection == WEST_DIR)
                move_dir = EAST_DIR;
            switch (inf)
            {
            case FORWARD_LABEL:
                ESP_LOGI(TAG, "Moving %d", inf);
                moved = can_move(MAZE, MAZE_LEN, MAZE_HEIGHT, x_current_cell, y_current_cell, &x_new_cell, &y_new_cell, move_dir, false, false);
                if (!moved)
                    ESP_LOGI(TAG, "Can't move from [%i,%i] to [%i,%i] dir %i", x_current_cell, y_current_cell, x_new_cell, y_new_cell, map_projection);
                break;
            case BACKWARD_LABEL:
                if (!get_back())
                {
                    ESP_LOGI(TAG, "Back not enabled");
                    break;
                }
                ESP_LOGI(TAG, "Moving %d", inf);
                move_dir = (move_dir + 4 - 2) % 4;
                moved = can_move(MAZE, MAZE_LEN, MAZE_HEIGHT, x_current_cell, y_current_cell, &x_new_cell, &y_new_cell, move_dir, false, false);
                if (!moved)
                    ESP_LOGI(TAG, "Can't move from [%i,%i] to [%i,%i] dir %i", x_current_cell, y_current_cell, x_new_cell, y_new_cell, map_projection);
                break;
            case LEFTSIDE_LABEL:
                if (!get_side())
                {
                    ESP_LOGI(TAG, "Side not enabled");
                    break;
                }
                ESP_LOGI(TAG, "Moving %d", inf);
                move_dir = (move_dir + 4 - 1) % 4;
                moved = can_move(MAZE, MAZE_LEN, MAZE_HEIGHT, x_current_cell, y_current_cell, &x_new_cell, &y_new_cell, move_dir, false, false);
                if (!moved)
                    ESP_LOGI(TAG, "Can't move from [%i,%i] to [%i,%i] dir %i", x_current_cell, y_current_cell, x_new_cell, y_new_cell, map_projection);
                break;
            case RIGHTSIDE_LABEL:
                if (!get_side())
                {
                    ESP_LOGI(TAG, "Side not enabled");
                    break;
                }
                ESP_LOGI(TAG, "Moving %d", inf);
                move_dir = (move_dir + 1) % 4;
                moved = can_move(MAZE, MAZE_LEN, MAZE_HEIGHT, x_current_cell, y_current_cell, &x_new_cell, &y_new_cell, move_dir, false, false);
                if (!moved)
                    ESP_LOGI(TAG, "Can't move from [%i,%i] to [%i,%i] dir %i", x_current_cell, y_current_cell, x_new_cell, y_new_cell, map_projection);
                break;
            case UP_LABEL:
                ESP_LOGI(TAG, "Moving %d", inf);
                moved = can_move(MAZE, MAZE_LEN, MAZE_HEIGHT, x_current_cell, y_current_cell, &x_new_cell, &y_new_cell, move_dir, true, false);
                if (!moved)
                    ESP_LOGI(TAG, "Can't move from [%i,%i] to [%i,%i] dir %i", x_current_cell, y_current_cell, x_new_cell, y_new_cell, map_projection);
                break;
            case DOWN_LABEL:
                ESP_LOGI(TAG, "Moving %d", inf);
                moved = can_move(MAZE, MAZE_LEN, MAZE_HEIGHT, x_current_cell, y_current_cell, &x_new_cell, &y_new_cell, move_dir, false, true);
                if (!moved)
                    ESP_LOGI(TAG, "Can't move from [%i,%i] to [%i,%i] dir %i", x_current_cell, y_current_cell, x_new_cell, y_new_cell, map_projection);
                break;
            }
        }

        xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);
        if (redraw_dir)
        {
            redraw_dir = false;
            draw_maze(canvas, MAZE, MAZE_LEN, MAZE_HEIGHT, map_projection, x_current_cell, y_current_cell);
            int x_pos, y_pos;
            get_status_pos_from_cell(x_current_cell, y_current_cell, map_projection, &x_pos, &y_pos, x_current_cell, y_current_cell);
            draw_status(canvas, 5, x_pos, y_pos, STATUS_WIDTH, STATUS_LENGTH);
        }

        int8_t op_x = 0;
        int8_t op_y = 0;
        get_op_x_y(&op_x, &op_y);

        if (moved)
        {
            step_cnt += 1 ;
            int x_new_pos = 0;
            int y_new_pos = 0;
            ESP_LOGI(TAG, "moving from [%i,%i] to [%i,%i] dir %i", x_current_cell, y_current_cell, x_new_cell, y_new_cell, map_projection);
            last_move_ticks = ticks;

            //Reset static status
            draw_static_maze(minimapcanvas, MINI_PLOT_WIDTH, MINI_PLOT_HEIGHT, MAZE, MAZE_LEN, MAZE_HEIGHT);
            get_static_status_pos_from_cell(x_current_cell, y_current_cell, MINI_PLOT_WIDTH / MAZE_LEN, 1, 3, 3, &x_new_pos, &y_new_pos);
            draw_status(minimapcanvas, 0, x_new_pos, y_new_pos, 3, 3);
            x_current_cell = x_new_cell;
            y_current_cell = y_new_cell;

            get_static_status_pos_from_cell(x_new_cell, y_new_cell, MINI_PLOT_WIDTH / MAZE_LEN, 1, 3, 3, &x_new_pos, &y_new_pos);
            draw_status(minimapcanvas, 5, x_new_pos, y_new_pos, 3, 3);

            draw_maze(canvas, MAZE, MAZE_LEN, MAZE_HEIGHT, map_projection, x_current_cell, y_current_cell);
            get_status_pos_from_cell(x_new_cell, y_new_cell, map_projection, &x_new_pos, &y_new_pos, x_new_cell, y_new_cell);
            draw_status(canvas, 5, x_new_pos, y_new_pos, STATUS_WIDTH, STATUS_LENGTH); //reset status

            //Draw op
            int x_pos, y_pos;
            get_status_pos_from_cell(last_test_x, last_test_y, map_projection, &x_pos, &y_pos, x_current_cell, y_current_cell);
            draw_status(canvas, 0, x_pos, y_pos, STATUS_WIDTH, STATUS_LENGTH);
            get_status_pos_from_cell(op_x, op_y, map_projection, &x_pos, &y_pos, x_current_cell, y_current_cell);
            draw_status(canvas, 6, x_pos, y_pos, STATUS_WIDTH, STATUS_LENGTH);
            send_position(x_new_cell, y_new_cell, time(NULL));

            //Steps
            lv_label_set_text_fmt(steps_lbl, "Steps:%d", step_cnt);
        }

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

        if (infer)
            lv_led_on(inf_led);
        else
            lv_led_off(inf_led);
        xSemaphoreGive(xGuiSemaphore);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    vTaskDelete(NULL); // Should never get to here...
}