
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
#include "plot_tab.h"
#include "float_buffer.h"

#define CANVAS_WIDTH 220
#define CANVAS_HEIGHT 220
#define PLOT_WINDOW_LEN 50
#define ACCEL_MAX 2.

static lv_color_t *cbuf;
static const char *TAG = PLOT_TAB_NAME;
static void *ax_buf;
static void *ay_buf;
static void *az_buf;

void display_plot_tab(lv_obj_t *tv)
{
    ax_buf = big_get_buffer();
    ay_buf = big_get_buffer();
    az_buf = big_get_buffer();
    ESP_LOGI(TAG, "after buffers");

    cbuf = (lv_color_t *)heap_caps_malloc(LV_CANVAS_BUF_SIZE_TRUE_COLOR(CANVAS_WIDTH, CANVAS_HEIGHT), MALLOC_CAP_DEFAULT | MALLOC_CAP_SPIRAM);
    xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);
    lv_obj_t *test_tab = lv_tabview_add_tab(tv, PLOT_TAB_NAME); // Create a tab

    lv_obj_t *canvas = lv_canvas_create(test_tab, NULL);
    lv_canvas_set_buffer(canvas, cbuf, CANVAS_WIDTH, CANVAS_HEIGHT, LV_IMG_CF_TRUE_COLOR);

    lv_obj_align(canvas, NULL, LV_ALIGN_IN_LEFT_MID, 5, 0);

    static lv_obj_t *parms[2];
    parms[0] = canvas;
    ESP_LOGI(TAG, "before task");
    xTaskCreatePinnedToCore(plot_task, "PlotTask", 2048, parms, 1, &PLOT_handle, 1);
    ESP_LOGI(TAG, "after task");

    xSemaphoreGive(xGuiSemaphore);
}

void draw_xyz_plot(lv_obj_t *canvas)
{
    lv_canvas_fill_bg(canvas, LV_COLOR_SILVER, LV_OPA_COVER);

    int plotSpace = CANVAS_WIDTH / PLOT_WINDOW_LEN;

    for (int i = 0; i < PLOT_WINDOW_LEN; i++)
    {
        int xy = big_get(ax_buf, i) * CANVAS_HEIGHT;
        int yy = big_get(ay_buf, i) * CANVAS_HEIGHT;
        int zy = big_get(az_buf, i) * CANVAS_HEIGHT;
        int x = plotSpace * i;

        /*
        lv_canvas_set_px(canvas, x, xy, LV_COLOR_RED);
        lv_canvas_set_px(canvas, x, xy+1, LV_COLOR_RED);
        lv_canvas_set_px(canvas, x+1, xy, LV_COLOR_RED);
        lv_canvas_set_px(canvas, x+1, xy+1, LV_COLOR_RED);
        */

        lv_canvas_set_px(canvas, x, yy, LV_COLOR_GREEN);
        lv_canvas_set_px(canvas, x, yy + 1, LV_COLOR_GREEN);
        lv_canvas_set_px(canvas, x + 1, yy, LV_COLOR_GREEN);
        lv_canvas_set_px(canvas, x + 1, yy + 1, LV_COLOR_GREEN);

        lv_canvas_set_px(canvas, x, zy, LV_COLOR_BLUE);
        lv_canvas_set_px(canvas, x, zy + 1, LV_COLOR_BLUE);
        lv_canvas_set_px(canvas, x + 1, zy, LV_COLOR_BLUE);
        lv_canvas_set_px(canvas, x + 1, zy + 1, LV_COLOR_BLUE);
    }
}

void plot_task(void *pvParameters)
{
    ESP_LOGI(TAG, "in task");
    float acc_raw[3];
    float acc_uv[3];

    vTaskSuspend(NULL);

    for (;;)
    {
        MPU6886_GetAccelData(&acc_raw[0], &acc_raw[1], &acc_raw[2]);

        //Subtract G
        acc_raw[2] -= 1.;

        //Scale
        //unit_vect(acc_raw, acc_uv, 3);
        acc_raw[0] = (acc_raw[0] + ACCEL_MAX) / (2 * ACCEL_MAX);
        acc_raw[1] = (acc_raw[1] + ACCEL_MAX) / (2 * ACCEL_MAX);
        acc_raw[2] = (acc_raw[2] + ACCEL_MAX) / (2 * ACCEL_MAX);

        big_push(ax_buf, acc_raw[0]);
        big_push(ay_buf, acc_raw[1]);
        big_push(az_buf, acc_raw[2]);

        xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);
        lv_obj_t **parms = (lv_obj_t **)pvParameters;
        lv_obj_t *canvas = (lv_obj_t *)parms[0];
        draw_xyz_plot(canvas);
        //ESP_LOGI(TAG, "time-%ld, old pos x-%i y-%i, new pos x-%i y-%i", ticks, x_old_pos, y_old_pos, x_new_pos, y_new_pos);
        xSemaphoreGive(xGuiSemaphore);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelete(NULL); // Should never get to here...
}
