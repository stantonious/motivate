#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"

#include "core2forAWS.h"

#include "details_tab.h"

static float calib_gx = 0.00;
static float calib_gy = 0.00;
static float calib_gz = 0.00;

static const char *TAG = DETAILS_TAB_NAME;

void display_details_tab(lv_obj_t *tv)
{

    xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);
    lv_obj_t *test_tab = lv_tabview_add_tab(tv, DETAILS_TAB_NAME); // Create a tab
    lv_obj_t *cont;

    cont = lv_cont_create(test_tab, NULL);
    lv_obj_set_auto_realign(cont, true);                   /*Auto realign when the size changes*/
    lv_obj_align_origo(cont, NULL, LV_ALIGN_CENTER, 0, 0); /*This parametrs will be sued when realigned*/
    lv_cont_set_fit(cont, LV_FIT_TIGHT);
    lv_cont_set_layout(cont, LV_LAYOUT_COLUMN_MID);

    lv_obj_t *label;
    label = lv_label_create(cont, NULL);
    lv_label_set_text(label, "Short text");

    /*Refresh and pause here for a while to see how `fit` works*/
    uint32_t t;
    lv_refr_now(NULL);
    t = lv_tick_get();
    while (lv_tick_elaps(t) < 500)
        ;

    label = lv_label_create(cont, NULL);
    lv_label_set_text(label, "It is a long text");

    /*Wait here too*/
    lv_refr_now(NULL);
    t = lv_tick_get();
    while (lv_tick_elaps(t) < 500)
        ;

    label = lv_label_create(cont, NULL);
    lv_label_set_text(label, "Here is an even longer text");

    static lv_color_t gauge_needle_colors[3];
    gauge_needle_colors[0] = LV_COLOR_RED;
    gauge_needle_colors[1] = LV_COLOR_GREEN;
    gauge_needle_colors[2] = LV_COLOR_BLUE;

    lv_obj_t *gauge_gyr = lv_gauge_create(cont, NULL);
    lv_obj_set_click(gauge_gyr, false);
    lv_obj_set_size(gauge_gyr, 106, 106);
    lv_gauge_set_scale(gauge_gyr, 300, 10, 0);
    lv_gauge_set_range(gauge_gyr, -400, 400);
    lv_gauge_set_critical_value(gauge_gyr, 2001);
    lv_gauge_set_needle_count(gauge_gyr, 3, gauge_needle_colors);
    //TODOlv_gauge_set_needle_img(gauge, &gauge_hand, 5, 4);
    lv_obj_set_style_local_image_recolor_opa(gauge_gyr, LV_GAUGE_PART_NEEDLE, LV_STATE_DEFAULT, LV_OPA_COVER);

    lv_obj_align(gauge_gyr, NULL, LV_ALIGN_IN_RIGHT_MID, -20, 0);

    xSemaphoreGive(xGuiSemaphore);

    static lv_obj_t *parms[2];
    parms[0] = gauge_gyr;
    xTaskCreatePinnedToCore(DETAILS_task, "DetailsTask", 2048, parms, 1, &DETAILS_handle, 1);
}

void DETAILS_task(void *pvParameters)
{

    vTaskSuspend(NULL);

    for (;;)
    {
        float gx, gy, gz;
        float ax, ay, az;
        MPU6886_GetAccelData(&ax, &ay, &az);
        MPU6886_GetGyroData(&gx, &gy, &gz);

        // float pitch, roll, yaw;
        // MahonyAHRSupdateIMU(gx * DEGREES_TO_RADIANS, gy * DEGREES_TO_RADIANS, gz * DEGREES_TO_RADIANS, ax, ay, az, &pitch, &roll, &yaw);
        // ESP_LOGI(TAG, "Pitch: %.6f Roll: %.6f Yaw: %.6f | Raw Accel: X-%.6f Y-%.6f Z-%.6f | Gyro: X-%.6f Y-%.6fZ- %.6f", pitch, yaw, roll, ax, ay, az, gx, gy, gz);

        ESP_LOGI(TAG, "Raw Accel: X-%.6f Y-%.6f Z-%.6f | Gyro: X-%.6f Y-%.6fZ- %.6f", ax, ay, az, gx, gy, gz);
        //ESP_LOGI(TAG, "Cal Accel: X-%.6f Y-%.6f Z-%.6f | Gyro: X-%.6f Y-%.6fZ- %.6f", calib_ax, calib_ay, calib_az, calib_gx, calib_gy, calib_gz);

        lv_obj_t **parms = (lv_obj_t **)pvParameters;
        lv_obj_t *gauge = (lv_obj_t *)parms[0];

        xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);
        lv_gauge_set_value(gauge, 0, (int)(gx - calib_gx));
        lv_gauge_set_value(gauge, 1, (int)(gy - calib_gy));
        lv_gauge_set_value(gauge, 2, (int)(gz - calib_gz));

        xSemaphoreGive(xGuiSemaphore);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
    vTaskDelete(NULL); // Should never get to here...
}
void recal(void)
{
    MPU6886_GetGyroData(&calib_gx, &calib_gy, &calib_gz);
}
