
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

#include "arrow-img-30x30.h"
#include "motivate_math.h"
#include "maze_tab.h"
#include "plots.h"
#include "maze_utils.h"
#include "maze.h"
#include "float_buffer.h"

#include "mot_mqtt_client.h"
#include "mot-imu-tf.h"

#define CANVAS_WIDTH 220
#define CANVAS_HEIGHT 220

#define MINI_PLOT_WIDTH 70
#define MINI_PLOT_HEIGHT 70
#define MINI_PLOT_NUM 20

#define MOVE_THRESH 30
#define TURN_THRESH 60
#define STEP_THRESH .5
#define STEP_DELTA_Z .2
#define STEP_DELTA_Y .15

static lv_color_t *cbuf;
static lv_color_t *miniplotbuf;
static lv_color_t *gyrominiplotbuf;
static const char *TAG = MAZE_TAB_NAME;

static float calib_ax = 0.00;
static float calib_ay = 0.00;
static float calib_az = 0.00;
static float calib_a_uv[3];
static float **abuf = NULL;
static float **gbuf = NULL;

static int x_current_cell = 0;
static int y_current_cell = 13;
static uint current_dir = NORTH_DIR;

static bool redraw_dir = true;

static bool record_left = false;
static bool record_right = false;
static bool record_forward = false;

static long last_move_ticks = 0;
static long last_turn_ticks = 0;

static void *ax_buf;
static void *ay_buf;
static void *az_buf;

static void *gx_buf;
static void *gy_buf;
static void *gz_buf;

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
    ax_buf = get_buffer();
    ay_buf = get_buffer();
    az_buf = get_buffer();

    gx_buf = get_buffer();
    gy_buf = get_buffer();
    gz_buf = get_buffer();

    down_recal(); //initial cal

    xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);
    lv_obj_t *test_tab = lv_tabview_add_tab(tv, MAZE_TAB_NAME); // Create a tab

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

    //leds
    lv_obj_t *r_led = lv_led_create(test_tab, NULL);
    lv_obj_align(r_led, NULL, LV_ALIGN_IN_RIGHT_MID, -45, -50);
    lv_obj_set_size(r_led, 25, 25);
    lv_led_off(r_led);
    //ESP_LOGI(TAG, "after led");

    draw_maze(canvas, TEST_MAZE, 14, 14, NORTH_DIR, x_current_cell, y_current_cell);

    lv_obj_t *arrow_img = lv_img_create(test_tab, NULL);
    lv_img_set_src(arrow_img, &arrow_30x30);
    lv_obj_align(arrow_img, NULL, LV_ALIGN_IN_TOP_RIGHT, -15, 5);
    lv_img_set_angle(arrow_img, 900 * current_dir);
    int x_init_pos, y_init_pos;
    get_status_pos_from_cell(x_current_cell, y_current_cell, current_dir, &x_init_pos, &y_init_pos, x_current_cell, y_current_cell);
    draw_status(canvas, 5, x_init_pos, y_init_pos, STATUS_WIDTH, STATUS_LENGTH);
    xSemaphoreGive(xGuiSemaphore);

    static lv_obj_t *parms[6];
    parms[0] = canvas;
    parms[1] = arrow_img;
    parms[2] = miniplotcanvas;
    parms[3] = gyrominiplotcanvas;
    parms[4] = r_led;
    xTaskCreatePinnedToCore(maze_task, "MazeTask", 2048 * 2, parms, 1, &MAZE_handle, 1);
}

//float step_coefs[] = {1., 1., 1., -1., -1};
float step_coefs[] = {.2, .2, .2, .2, .2};
void maze_task(void *pvParameters)
{

    lv_obj_t **parms = (lv_obj_t **)pvParameters;
    lv_obj_t *canvas = (lv_obj_t *)parms[0];
    lv_obj_t *arrow = (lv_obj_t *)parms[1];
    lv_obj_t *miniplotcanvas = (lv_obj_t *)parms[2];
    lv_obj_t *gyrominiplotcanvas = (lv_obj_t *)parms[3];
    lv_obj_t *r_led = (lv_obj_t *)parms[4];

    vTaskSuspend(NULL);

    int cnt = 0;
    for (;;)
    {
        cnt++;
        long ticks = xTaskGetTickCount();
        float gx, gy, gz;
        float ax, ay, az;

        MPU6886_GetAccelData(&ax, &ay, &az);
        MPU6886_GetGyroData(&gx, &gy, &gz);

        // Substract out G
        az -= 1;

        //ESP_LOGI(TAG, "acc x:%.6f y:%.6f z:%.6f", ax,ay,az);
        push(ax_buf, ax);
        push(ay_buf, ay);
        push(az_buf, az);

        push(gx_buf, gx);
        push(gy_buf, gy);
        push(gz_buf, gz);

        float y_delta = get_delta(ay_buf);
        float z_delta = get_delta(az_buf);

        

        //TODO AVOID COPY!!!  Use Circ Buff
        //mk_copy(ax_buf, abuf[0], BUFSIZE);
        //mk_copy(ay_buf, abuf[1], BUFSIZE);
        //mk_copy(az_buf, abuf[2], BUFSIZE);
        //mk_copy(gx_buf, gbuf[0], BUFSIZE);
        //mk_copy(gy_buf, gbuf[1], BUFSIZE);
        //mk_copy(gz_buf, gbuf[2], BUFSIZE);
        //int inf = infer(abuf,gbuf,BUFSIZE,BUFSIZE); 
        int inf = buffer_infer(ax_buf,ay_buf,az_buf,gx_buf,gy_buf,gz_buf); 
        
        if (inf == 3&&
            ticks - last_turn_ticks > TURN_THRESH)
        {
            //ESP_LOGI(TAG, "right turn");
            current_dir = (current_dir + 1) % 4;
            last_turn_ticks = ticks;
            redraw_dir = true;
            //ESP_LOGI(TAG, "new dir-%i", current_dir);
        }
        else if (inf == 2 &&
                 ticks - last_turn_ticks > TURN_THRESH)
        {
            //ESP_LOGI(TAG, "left turn");
            current_dir = (current_dir - 1) % 4;
            last_turn_ticks = ticks;
            redraw_dir = true;
            //ESP_LOGI(TAG, "new dir-%i", current_dir);
        }

        xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);
        if (redraw_dir)
        {
            lv_img_set_angle(arrow, 900 * current_dir);
            redraw_dir = false;
            draw_maze(canvas, TEST_MAZE, 14, 14, current_dir, x_current_cell, y_current_cell);
            int x_pos, y_pos;
            get_status_pos_from_cell(x_current_cell, y_current_cell, current_dir, &x_pos, &y_pos, x_current_cell, y_current_cell);
            draw_status(canvas, 5, x_pos, y_pos, STATUS_WIDTH, STATUS_LENGTH);

            if (record_left)
            {
                ESP_LOGI(TAG, "Rec Left");
                ESP_LOGI(TAG, "Done Rec Left");
            }
            if (record_right)
            {
            }
        }

        bool moved = false;
        int y_new_cell, x_new_cell;
        if (inf == 0) //forward step
        {

            //dir is map projected dir
            int move_dir = current_dir;
            if (current_dir == EAST_DIR)
                move_dir = WEST_DIR;
            else if (current_dir == WEST_DIR)
                move_dir = EAST_DIR;
            moved = can_move(TEST_MAZE, 14, 14, x_current_cell, y_current_cell, &x_new_cell, &y_new_cell, move_dir);
            if (!moved)
                ESP_LOGI(TAG, "Can't move from [%i,%i] to [%i,%i] dir %i", x_current_cell, y_current_cell, x_new_cell, y_new_cell, current_dir);
            if (record_forward)
            {
            }
        }
        //TODO else if (step && y_conv < 0) //backward step
        else if (false) //backward step
        {
            int back_dir = (current_dir - 2) % 4;
            moved = can_move(TEST_MAZE, 14, 14, x_current_cell, y_current_cell, &x_new_cell, &y_new_cell, back_dir);
            if (!moved)
                ESP_LOGI(TAG, "Can't move from [%i,%i] to [%i,%i] dir %i", x_current_cell, y_current_cell, x_new_cell, y_new_cell, current_dir);
        }
        if (moved)
        {
            int x_new_pos = 0;
            int y_new_pos = 0;
            ESP_LOGI(TAG, "moving from [%i,%i] to [%i,%i] dir %i", x_current_cell, y_current_cell, x_new_cell, y_new_cell, current_dir);
            last_move_ticks = ticks;
            x_current_cell = x_new_cell;
            y_current_cell = y_new_cell;
            draw_maze(canvas, TEST_MAZE, 14, 14, current_dir, x_current_cell, y_current_cell);
            get_status_pos_from_cell(x_new_cell, y_new_cell, current_dir, &x_new_pos, &y_new_pos, x_new_cell, y_new_cell);
            draw_status(canvas, 5, x_new_pos, y_new_pos, STATUS_WIDTH, STATUS_LENGTH); //reset status
        }

        int8_t op_x = 0;
        int8_t op_y = 0;
        get_op_x_y(&op_x, &op_y);
        if (op_x >= 0 && op_y >= 0 && (last_test_x != op_x || last_test_y != op_y))
        {
            int x_pos, y_pos;
            get_status_pos_from_cell(last_test_x, last_test_y, current_dir, &x_pos, &y_pos, x_current_cell, y_current_cell);
            draw_status(canvas, 0, x_pos, y_pos, STATUS_WIDTH, STATUS_LENGTH);
            get_status_pos_from_cell(op_x, op_y, current_dir, &x_pos, &y_pos, x_current_cell, y_current_cell);
            draw_status(canvas, 6, x_pos, y_pos, STATUS_WIDTH, STATUS_LENGTH);
            last_test_x = op_x;
            last_test_y = op_y;
        }
        draw_3_plot(miniplotcanvas, &scale_acc, ax_buf, ay_buf, az_buf, MINI_PLOT_HEIGHT, MINI_PLOT_WIDTH, MINI_PLOT_NUM);
        draw_3_plot(gyrominiplotcanvas, &scale_gyro, gx_buf, gy_buf, gz_buf, MINI_PLOT_HEIGHT, MINI_PLOT_WIDTH, MINI_PLOT_NUM);

        if (record_forward || record_left || record_right)
            lv_led_on(r_led);
        else
            lv_led_off(r_led);
        xSemaphoreGive(xGuiSemaphore);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    vTaskDelete(NULL); // Should never get to here...
}

void down_recal(void)
{
    MPU6886_GetAccelData(&calib_ax, &calib_ay, &calib_az);
    float ax_uv[3] = {calib_ax, calib_ay, calib_az};
    unit_vect(ax_uv, calib_a_uv, 3);
    //ESP_LOGI(TAG, "recal raw x:%.6f y:%6.f z:%6.f  ", calib_ax, calib_ay, calib_az);
    //ESP_LOGI(TAG, "recal to x:%.6f y:%6.f z:%6.f  ", calib_a_uv[0], calib_a_uv[1], calib_a_uv[2]);
}

void reset_north(void)
{
    //ESP_LOGI(TAG, "Resetting North");
    current_dir = NORTH_DIR;
    redraw_dir = true;
}
void toggle_left_record(void)
{
    record_left = !record_left;
    record_right = false;
    record_forward = false;
}
void toggle_right_record(void)
{
    record_right = !record_right;
    record_left = false;
    record_forward = false;
}
void toggle_foward_record(void)
{
    record_forward = !record_forward;
    record_right = false;
    record_left = false;
}