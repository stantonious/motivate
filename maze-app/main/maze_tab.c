
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
#include "motivate_math.h"
#include "maze_tab.h"
#include "plots.h"
#include "maze_utils.h"
#include "maze.h"
#include "float_buffer.h"

#include "globals.h"
#include "mot_mqtt_client.h"
#include "mot-imu-tf.h"
#include "imu_task.h"
#include "game_tab.h"

#define MINI_PLOT_WIDTH 88
#define MINI_PLOT_HEIGHT 88 //multiple of MAZE_LEN

#define STATS_THRESH 3000 //30 seconds
#define STA_BLINK_THRESH 40
#define LAST_N_INF 5
#define ACCEL_COS_THRES .95

static lv_color_t *cbuf;
static lv_color_t *minimapbuf;
static const char *TAG = MAZE_TAB_NAME;

static int x_current_cell = 0;
static int y_current_cell = 0;
static uint map_projection = NORTH_DIR;

static bool redraw_dir = true;

static int8_t last_test_x = -1;
static int8_t last_test_y = -1;

static int step_cnt = 0;
static int game_session;
static int capture_count = 0;
static int caught_count = 0;
static int last_game_id;
static long game_start_t = 0;

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

    //op label
    lv_obj_t *op_lbl = lv_label_create(test_tab, NULL);
    lv_label_set_text(op_lbl, "        ");
    lv_obj_align(op_lbl, NULL, LV_ALIGN_IN_TOP_RIGHT, -65, MINI_PLOT_HEIGHT + 45);

    //op label
    lv_obj_t *t_lbl = lv_label_create(test_tab, NULL);
    lv_label_set_text(t_lbl, "        ");
    lv_obj_align(t_lbl, NULL, LV_ALIGN_IN_TOP_RIGHT, -65, MINI_PLOT_HEIGHT + 65);

    //inf led
    lv_obj_t *inf_led = lv_led_create(test_tab, NULL);
    lv_obj_align(inf_led, NULL, LV_ALIGN_IN_BOTTOM_RIGHT, -5, -5);
    lv_obj_set_size(inf_led, 20, 20);

    if (infer)
        lv_led_on(inf_led);
    else
        lv_led_off(inf_led);

    //stable led
    lv_obj_t *sta_led = lv_led_create(test_tab, NULL);
    lv_obj_align(sta_led, NULL, LV_ALIGN_IN_BOTTOM_RIGHT, -45, -5);
    lv_obj_set_size(sta_led, 20, 20);
    lv_led_off(sta_led);
    xSemaphoreGive(xGuiSemaphore);

    static lv_obj_t *maze_parms[9];
    maze_parms[0] = canvas;
    maze_parms[1] = inf_lbl;
    maze_parms[2] = minimapcanvas;
    maze_parms[3] = steps_lbl;
    maze_parms[4] = inf_led;
    maze_parms[5] = op_lbl;
    maze_parms[6] = sta_led;
    maze_parms[7] = test_tab;
    maze_parms[8] = t_lbl;

    reset(canvas, minimapcanvas);
    xTaskCreatePinnedToCore(maze_task, "MazeTask", 2048 * 2, maze_parms, 1, &MAZE_handle, 1);
}

void reset(lv_obj_t *canvas, lv_obj_t *minimapcanvas)
{
    game_start_t = xTaskGetTickCount();
    x_current_cell = x_entry;
    y_current_cell = y_entry;
    step_cnt = 0;
    capture_count = 0;
    caught_count = 0;
    game_session = (int)time(NULL);

    last_game_id = game_id;
    xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);
    draw_maze(canvas, MAZE, MAZE_LEN, MAZE_HEIGHT, NORTH_DIR, x_current_cell, y_current_cell);
    draw_static_maze(minimapcanvas, MINI_PLOT_WIDTH, MINI_PLOT_HEIGHT, MAZE, MAZE_LEN, MAZE_HEIGHT);

    int x_init_pos, y_init_pos;
    get_status_pos_from_cell(x_current_cell, y_current_cell, map_projection, &x_init_pos, &y_init_pos, x_current_cell, y_current_cell);
    draw_status(canvas, player_type, x_init_pos, y_init_pos, STATUS_WIDTH, STATUS_LENGTH);
    xSemaphoreGive(xGuiSemaphore);
}
//float step_coefs[] = {1., 1., 1., -1., -1};
float step_coefs[] = {.2, .2, .2, .2, .2};
void maze_task(void *pvParameters)
{

    bool sta_blink = false;
    long last_sta_blink_ticks = 0;

    long last_move_ticks = 0;
    long last_turn_ticks = 0;
    long last_stats_ticks = 0;
    last_game_id = game_id;

    lv_obj_t **maze_parms = (lv_obj_t **)pvParameters;
    lv_obj_t *canvas = (lv_obj_t *)maze_parms[0];
    lv_obj_t *inf_lbl = (lv_obj_t *)maze_parms[1];
    lv_obj_t *minimapcanvas = (lv_obj_t *)maze_parms[2];
    lv_obj_t *steps_lbl = (lv_obj_t *)maze_parms[3];
    lv_obj_t *inf_led = (lv_obj_t *)maze_parms[4];
    lv_obj_t *op_lbl = (lv_obj_t *)maze_parms[5];
    lv_obj_t *sta_led = (lv_obj_t *)maze_parms[6];
    lv_obj_t *test_tab = (lv_obj_t *)maze_parms[7];
    lv_obj_t *t_lbl = (lv_obj_t *)maze_parms[8];

    vTaskSuspend(NULL);

    //Let the maze_client init with entry/exit info
    x_current_cell = x_entry;
    y_current_cell = y_entry;

    for (;;)
    {
        if (game_id != last_game_id)
            reset(canvas, minimapcanvas);
        long ticks = xTaskGetTickCount();
        long move_delta = ticks - last_move_ticks;
        long turn_delta = ticks - last_turn_ticks;
        long stats_delta = ticks - last_stats_ticks;
        long blink_delta = ticks - last_sta_blink_ticks;
        int y_new_cell, x_new_cell;
        bool moved = false;

        if (infer && turn_delta > turn_sensitivity)
        {
            last_turn_ticks = ticks;

            int inf = get_latest_inf(LAST_N_INF);

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

        if (infer && move_delta > move_sensitivity)
        {
            last_move_ticks = ticks;
            moved = false;

            int inf = get_latest_inf(LAST_N_INF);

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
                if (!use_back)
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
                if (!use_side)
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
                if (!use_side)
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
            draw_status(canvas, player_type, x_pos, y_pos, STATUS_WIDTH, STATUS_LENGTH);
        }

        if (op_x == x_current_cell && op_y == y_current_cell) //collision
        {
            if ((op_t == WIZARD && player_type == ROGUE) || (op_t == ROGUE && player_type == FIGHTER) || (op_t == FIGHTER && player_type == WIZARD))
            {
                //Back to start
                x_current_cell = x_entry;
                y_current_cell = y_entry;
                moved = true;

                lv_obj_t *end_mb = lv_msgbox_create(test_tab, NULL);
                static const char *end_mb_btns[] = {"Close", ""};
                lv_msgbox_set_text(end_mb, "Caught!");
                lv_msgbox_add_btns(end_mb, end_mb_btns);
                lv_obj_set_width(end_mb, 200);
                lv_obj_align(end_mb, NULL, LV_ALIGN_CENTER, 0, 0);
            }
            else if ((player_type == WIZARD && op_t == ROGUE) || (player_type == ROGUE && op_t == FIGHTER) || (player_type == FIGHTER && op_t == WIZARD))
            {
                //TODO Send Op to start
                lv_obj_t *end_mb = lv_msgbox_create(test_tab, NULL);
                static const char *end_mb_btns[] = {"Close", ""};
                lv_msgbox_set_text(end_mb, "Congrats!");
                lv_msgbox_add_btns(end_mb, end_mb_btns);
                lv_obj_set_width(end_mb, 200);
                lv_obj_align(end_mb, NULL, LV_ALIGN_CENTER, 0, 0);
            }
        }

        if (moved)
        {
            step_cnt += 1;
            int x_new_pos = 0;
            int y_new_pos = 0;
            ESP_LOGI(TAG, "moving from [%i,%i] to [%i,%i] dir %i", x_current_cell, y_current_cell, x_new_cell, y_new_cell, map_projection);
            last_move_ticks = ticks;

            //Reset static status
            draw_static_maze(minimapcanvas, MINI_PLOT_WIDTH, MINI_PLOT_HEIGHT, MAZE, MAZE_LEN, MAZE_HEIGHT);
            //get_static_status_pos_from_cell(x_current_cell, y_current_cell, (MINI_PLOT_WIDTH / MAZE_LEN) + 1, 1, 3, 3, &x_new_pos, &y_new_pos);
            //draw_status(minimapcanvas, 0, x_new_pos, y_new_pos, 3, 3);
            x_current_cell = x_new_cell;
            y_current_cell = y_new_cell;

            get_static_status_pos_from_cell(last_test_x, last_test_y, (MINI_PLOT_WIDTH / MAZE_LEN) + 1, 1, 3, 3, &x_new_pos, &y_new_pos);
            draw_status(minimapcanvas, op_t, x_new_pos, y_new_pos, 3, 3);

            get_static_status_pos_from_cell(x_new_cell, y_new_cell, (MINI_PLOT_WIDTH / MAZE_LEN) + 1, 1, 3, 3, &x_new_pos, &y_new_pos);
            draw_status(minimapcanvas, player_type, x_new_pos, y_new_pos, 3, 3);

            draw_maze(canvas, MAZE, MAZE_LEN, MAZE_HEIGHT, map_projection, x_current_cell, y_current_cell);
            get_status_pos_from_cell(x_new_cell, y_new_cell, map_projection, &x_new_pos, &y_new_pos, x_new_cell, y_new_cell);
            draw_status(canvas, player_type, x_new_pos, y_new_pos, STATUS_WIDTH, STATUS_LENGTH); //reset status

            //Draw op
            int x_pos, y_pos;
            //get_status_pos_from_cell(last_test_x, last_test_y, map_projection, &x_pos, &y_pos, x_current_cell, y_current_cell);
            //draw_status(canvas, 0, x_pos, y_pos, STATUS_WIDTH, STATUS_LENGTH);
            get_status_pos_from_cell(op_x, op_y, map_projection, &x_pos, &y_pos, x_current_cell, y_current_cell);
            draw_status(canvas, op_t, x_pos, y_pos, STATUS_WIDTH, STATUS_LENGTH);

            send_position(x_new_cell, y_new_cell, player_type, time(NULL));

            //Steps
            lv_label_set_text_fmt(steps_lbl, "Steps:%d", step_cnt);
        }

        if (op_x >= 0 && op_y >= 0 && (last_test_x != op_x || last_test_y != op_y))
        {
            int x_pos, y_pos;
            get_status_pos_from_cell(last_test_x, last_test_y, map_projection, &x_pos, &y_pos, x_current_cell, y_current_cell);
            draw_status(canvas, 0, x_pos, y_pos, STATUS_WIDTH, STATUS_LENGTH);
            get_status_pos_from_cell(op_x, op_y, map_projection, &x_pos, &y_pos, x_current_cell, y_current_cell);
            draw_status(canvas, op_t, x_pos, y_pos, STATUS_WIDTH, STATUS_LENGTH);

            get_static_status_pos_from_cell(last_test_x, last_test_y, (MINI_PLOT_WIDTH / MAZE_LEN) + 1, 1, 3, 3, &x_pos, &y_pos);
            draw_status(minimapcanvas, 0, x_pos, y_pos, 3, 3);
            last_test_x = op_x;
            last_test_y = op_y;

            get_static_status_pos_from_cell(last_test_x, last_test_y, (MINI_PLOT_WIDTH / MAZE_LEN) + 1, 1, 3, 3, &x_pos, &y_pos);
            draw_status(minimapcanvas, op_t, x_pos, y_pos, 3, 3);

            lv_label_set_text(op_lbl, op_id);
        }

        if (infer)
            lv_led_on(inf_led);
        else
            lv_led_off(inf_led);

        if (dot_avg < ACCEL_COS_THRES)
            sta_blink = true;
        else
        {
            sta_blink = false;
            lv_led_off(sta_led);
        }

        if (blink_delta > STA_BLINK_THRESH && sta_blink)
        {
            lv_led_toggle(sta_led);
            last_sta_blink_ticks = ticks;
        }

        lv_label_set_text_fmt(t_lbl,"%.*f",2,((xTaskGetTickCount()-game_start_t)/100.));
        xSemaphoreGive(xGuiSemaphore);

        if (stats_delta > STATS_THRESH)
        {
            last_stats_ticks = ticks;
            send_stats(game_id, game_session, step_cnt, capture_count, caught_count, (unsigned)time(NULL));
            ESP_LOGI(TAG, "sending stats");
        }

        if (x_current_cell == x_exit && y_current_cell == y_exit) //Exit!
        {

            xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);
            lv_obj_t *end_mb = lv_msgbox_create(test_tab, NULL);
            static const char *end_mb_btns[] = {"Close", ""};
            lv_msgbox_set_text_fmt(end_mb, "Congrats! Elapsed:%.*f",2,(xTaskGetTickCount()-game_start_t)/100.0);
            lv_msgbox_add_btns(end_mb, end_mb_btns);
            lv_obj_set_width(end_mb, 200);
            lv_obj_align(end_mb, NULL, LV_ALIGN_CENTER, 0, 0);
            xSemaphoreGive(xGuiSemaphore);
            reset(canvas,minimapcanvas);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    vTaskDelete(NULL); // Should never get to here...
}