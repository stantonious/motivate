
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"

#include "core2forAWS.h"

#include "arrow-img-30x30.h"
#include "motivate_math.h"
#include "maze_tab.h"
#include "float_buffer.h"

#define CANVAS_WIDTH 220
#define CANVAS_HEIGHT 220
#define ACCEL_MAX 1.

#define MINI_PLOT_WIDTH 80
#define MINI_PLOT_HEIGHT 80
#define MINI_PLOT_NUM 20

#define NORTH_BIT 0x0001
#define SOUTH_BIT 0x0002
#define EAST_BIT 0x0004
#define WEST_BIT 0x0008
#define INOUT_BIT 0x0010

#define NORTH_DIR 0
#define EAST_DIR 1
#define SOUTH_DIR 2
#define WEST_DIR 3

#define WALL_WIDTH 3
#define WALL_LENGTH 18

#define STATUS_WIDTH 8
#define STATUS_LENGTH 8

#define MOVE_THRESH 30
#define TURN_THRESH 60
#define STEP_THRESH .5
#define STEP_DELTA_Z .2
#define STEP_DELTA_Y .15

static lv_color_t *cbuf;
static lv_color_t *miniplotbuf;
static const char *TAG = MAZE_TAB_NAME;

static float calib_ax = 0.00;
static float calib_ay = 0.00;
static float calib_az = 0.00;
static float calib_a_uv[3];

static int x_current_cell = 0;
static int y_current_cell = 13;
static uint current_dir = 2;

static bool redraw_dir = true;

static long last_move_ticks = 0;
static long last_turn_ticks = 0;

#define MAZE_LEN 14
#define MAZE_HEIGHT 14

static void *ax_buf;
static void *ay_buf;
static void *az_buf;

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

void get_next_cell(int x_from, int y_from, int *x_to, int *y_to, int dir)
{
    if (dir == NORTH_DIR)
    {
        *y_to = y_from - 1;
        *x_to = x_from;
    }
    else if (dir == SOUTH_DIR)
    {
        *y_to = y_from + 1;
        *x_to = x_from;
    }
    else if (dir == WEST_DIR)
    {
        *x_to = x_from - 1;
        *y_to = y_from;
    }
    else if (dir == EAST_DIR)
    {
        *x_to = x_from + 1;
        *y_to = y_from;
    }
}
bool can_move(int maze[MAZE_HEIGHT][MAZE_LEN], int x_maze_len, int y_maze_len, int x_from, int y_from, int *x_to, int *y_to, int dir)
{
    get_next_cell(x_from, y_from, x_to, y_to, dir);

    if (*x_to >= x_maze_len ||
        *y_to >= y_maze_len ||
        *x_to < 0 ||
        *y_to < 0)
        return false;

    if (dir == NORTH_DIR && maze[y_from][x_from] & NORTH_BIT)
        return false;
    if (dir == SOUTH_DIR && maze[y_from][x_from] & SOUTH_BIT)
        return false;
    if (dir == EAST_DIR && maze[y_from][x_from] & EAST_BIT)
        return false;
    if (dir == WEST_DIR && maze[y_from][x_from] & WEST_BIT)
        return false;

    return true;
}
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

void draw_3_plot(lv_obj_t *canvas, void *buf1, void *buf2, void *buf3, int canvas_height, int canvas_width, int plot_win_len)
{
    lv_canvas_fill_bg(canvas, LV_COLOR_SILVER, LV_OPA_COVER);

    int plotSpace = canvas_width / plot_win_len;

    for (int i = 0; i < plot_win_len; i++)
    {
        int xy = get(buf1, i) * canvas_height;
        int yy = get(buf2, i) * canvas_height;
        int zy = get(buf3, i) * canvas_height;
        int x = plotSpace * i;

        if (buf1 != NULL)
        {
            lv_canvas_set_px(canvas, x, xy, LV_COLOR_RED);
            lv_canvas_set_px(canvas, x, xy + 1, LV_COLOR_RED);
            lv_canvas_set_px(canvas, x + 1, xy, LV_COLOR_RED);
            lv_canvas_set_px(canvas, x + 1, xy + 1, LV_COLOR_RED);
        }

        if (buf2 != NULL)
        {
            lv_canvas_set_px(canvas, x, yy, LV_COLOR_GREEN);
            lv_canvas_set_px(canvas, x, yy + 1, LV_COLOR_GREEN);
            lv_canvas_set_px(canvas, x + 1, yy, LV_COLOR_GREEN);
            lv_canvas_set_px(canvas, x + 1, yy + 1, LV_COLOR_GREEN);
        }

        if (buf2 != NULL)
        {
            lv_canvas_set_px(canvas, x, zy, LV_COLOR_BLUE);
            lv_canvas_set_px(canvas, x, zy + 1, LV_COLOR_BLUE);
            lv_canvas_set_px(canvas, x + 1, zy, LV_COLOR_BLUE);
            lv_canvas_set_px(canvas, x + 1, zy + 1, LV_COLOR_BLUE);
        }
    }
}
void display_maze_tab(lv_obj_t *tv)
{
    ax_buf = get_buffer();
    ay_buf = get_buffer();
    az_buf = get_buffer();
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
    lv_canvas_fill_bg(canvas, LV_COLOR_SILVER, LV_OPA_COVER);

    //leds
    lv_obj_t * y_led  = lv_led_create(test_tab, NULL);
    lv_obj_align(y_led, NULL, LV_ALIGN_IN_RIGHT_MID, -10, 0);
    lv_obj_set_size(y_led,25,25);
    lv_led_off(y_led);
    ESP_LOGI(TAG,"after led");

    lv_obj_t * z_led  = lv_led_create(test_tab, NULL);
    lv_obj_align(z_led, NULL, LV_ALIGN_IN_RIGHT_MID, -10, -35);
    lv_obj_set_size(z_led,25,25);
    lv_led_off(z_led);

    int x_pos, y_pos;
    for (int i = 0; i < 14; i++)
    {
        for (int j = 0; j < 14; j++)
        {
            int s = 0;
            get_pos_from_cell(j, i, &x_pos, &y_pos);
            draw_cell(canvas, s, x_pos, y_pos, TEST_MAZE[i][j]);
        }
    }

    lv_obj_t *arrow_img = lv_img_create(test_tab, NULL);
    lv_img_set_src(arrow_img, &arrow_30x30);
    lv_obj_align(arrow_img, NULL, LV_ALIGN_IN_TOP_RIGHT, -15, 5);
    lv_img_set_angle(arrow_img, 900 * current_dir);
    int x_init_pos, y_init_pos;
    get_status_pos_from_cell(x_current_cell, y_current_cell, &x_init_pos, &y_init_pos);
    draw_status(canvas, 5, x_init_pos, y_init_pos);
    xSemaphoreGive(xGuiSemaphore);

    static lv_obj_t *parms[5];
    parms[0] = canvas;
    parms[1] = arrow_img;
    parms[2] = miniplotcanvas;
    parms[3] = y_led;
    parms[4] = z_led;
    xTaskCreatePinnedToCore(maze_task, "MazeTask", 2048, parms, 1, &MAZE_handle, 1);
}

//float step_coefs[] = {1., 1., 1., -1., -1};
float step_coefs[] = {.2, .2, .2, .2, .2};
void maze_task(void *pvParameters)
{

    vTaskSuspend(NULL);

    for (;;)
    {
        long ticks = xTaskGetTickCount();
        float gx, gy, gz;
        float ax, ay, az;
        MPU6886_GetAccelData(&ax, &ay, &az);
        MPU6886_GetGyroData(&gx, &gy, &gz);

        float acc_raw[3] = {ax, ay, az};
        //float ax_uv[3];
        //float ax_cos;
        //unit_vect(acc_raw, ax_uv, 3);

        // Substract out G
        acc_raw[2] -= 1;

        acc_raw[0] = (acc_raw[0] + ACCEL_MAX) / (2 * ACCEL_MAX);
        acc_raw[1] = (acc_raw[1] + ACCEL_MAX) / (2 * ACCEL_MAX);
        acc_raw[2] = (acc_raw[2] + ACCEL_MAX) / (2 * ACCEL_MAX);
        push(ax_buf, acc_raw[0]);
        push(ay_buf, acc_raw[1]);
        push(az_buf, acc_raw[2]);

        float y_delta = get_delta(ay_buf);
        float z_delta = get_delta(az_buf);

        bool step = y_delta > STEP_DELTA_Y && z_delta > STEP_DELTA_Z && ticks - last_move_ticks > MOVE_THRESH;
        float y_conv = conv(ay_buf, step_coefs, 5);
        //        ESP_LOGI(TAG, "step conv y:%.6f", y_conv);
        //dsps_dotprod_f32(ax_uv, calib_a_uv, &ax_cos, 3);


        if (gz < -100. &&
            ticks - last_turn_ticks > TURN_THRESH)
        {
            ESP_LOGI(TAG, "right turn");
            current_dir = (current_dir + 1) % 4;
            last_turn_ticks = ticks;
            redraw_dir = true;
            ESP_LOGI(TAG, "new dir-%i", current_dir);
        }
        else if (gz > 100. &&
                 ticks - last_turn_ticks > TURN_THRESH)
        {
            ESP_LOGI(TAG, "left turn");
            current_dir = (current_dir - 1) % 4;
            last_turn_ticks = ticks;
            redraw_dir = true;
            ESP_LOGI(TAG, "new dir-%i", current_dir);
        }

        if (redraw_dir)
        {
            lv_obj_t **parms = (lv_obj_t **)pvParameters;
            lv_obj_t *arrow = (lv_obj_t *)parms[1];
            lv_img_set_angle(arrow, 900 * current_dir);
            redraw_dir = false;
        }

        bool moved = false;
        int y_new_cell, x_new_cell;
        if (step && y_conv >= 0) //forward step
        {

            moved = can_move(TEST_MAZE, 14, 14, x_current_cell, y_current_cell, &x_new_cell, &y_new_cell, current_dir);
            if (!moved)
                ESP_LOGI(TAG, "Can't move from [%i,%i] to [%i,%i] dir %i", x_current_cell, y_current_cell, x_new_cell, y_new_cell, current_dir);
        }
        else if (step && y_conv < 0) //backward step
        {
            int back_dir = (current_dir - 2) % 4;
            moved = can_move(TEST_MAZE, 14, 14, x_current_cell, y_current_cell, &x_new_cell, &y_new_cell, back_dir);
            if (!moved)
                ESP_LOGI(TAG, "Can't move from [%i,%i] to [%i,%i] dir %i", x_current_cell, y_current_cell, x_new_cell, y_new_cell, current_dir);
        }
        if (moved)
        {
            last_move_ticks = ticks;
            ESP_LOGI(TAG, "moving from [%i,%i] to [%i,%i] dir %i", x_current_cell, y_current_cell, x_new_cell, y_new_cell, current_dir);
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
                draw_status(canvas, 0, x_old_pos, y_old_pos); //reset status
                draw_status(canvas, 5, x_new_pos, y_new_pos);
                ESP_LOGI(TAG, "time-%ld, old pos x-%i y-%i, new pos x-%i y-%i", ticks, x_old_pos, y_old_pos, x_new_pos, y_new_pos);
                xSemaphoreGive(xGuiSemaphore);
            }
        }

        xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);
        lv_obj_t **parms = (lv_obj_t **)pvParameters;
        lv_obj_t *miniplotcanvas = (lv_obj_t *)parms[2];
        lv_obj_t *y_led = (lv_obj_t *)parms[3];
        lv_obj_t *z_led = (lv_obj_t *)parms[4];
        draw_3_plot(miniplotcanvas, ax_buf, ay_buf, az_buf, MINI_PLOT_HEIGHT, MINI_PLOT_WIDTH, MINI_PLOT_NUM);

        if (y_delta > STEP_DELTA_Y) lv_led_on(y_led);
        else lv_led_off(y_led);
        if (z_delta > STEP_DELTA_Z) lv_led_on(z_led);
        else lv_led_off(z_led);
        xSemaphoreGive(xGuiSemaphore);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelete(NULL); // Should never get to here...
}

void down_recal(void)
{
    MPU6886_GetAccelData(&calib_ax, &calib_ay, &calib_az);
    float ax_uv[3] = {calib_ax, calib_ay, calib_az};
    unit_vect(ax_uv, calib_a_uv, 3);
    ESP_LOGI(TAG, "recal raw x:%.6f y:%6.f z:%6.f  ", calib_ax, calib_ay, calib_az);
    ESP_LOGI(TAG, "recal to x:%.6f y:%6.f z:%6.f  ", calib_a_uv[0], calib_a_uv[1], calib_a_uv[2]);
}

void reset_north(void)
{
    ESP_LOGI(TAG, "Resetting North");
    current_dir = NORTH_DIR;
    redraw_dir = true;
}