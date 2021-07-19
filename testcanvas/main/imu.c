#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"

#include "core2forAWS.h"

#include "imu.h"
//TODO#include "gauge_hand.c"

static const char* TAG = IMU_TAB_NAME;

static float calib_gx = 0.00;
static float calib_gy = 0.00;
static float calib_gz = 0.00;

static float calib_ax = 0.00;
static float calib_ay = 0.00;
static float calib_az = 0.00;

void display_imu_tab(lv_obj_t* tv){
    xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);   // Takes (blocks) the xGuiSemaphore mutex from being read/written by another task.

    lv_obj_t* mpu_tab = lv_tabview_add_tab(tv, IMU_TAB_NAME); // Create a tab
    /* Create the main body object and set background within the tab*/
    static lv_style_t bg_style;
    lv_obj_t* mpu_bg = lv_obj_create(mpu_tab, NULL);
    lv_obj_align(mpu_bg, NULL, LV_ALIGN_IN_TOP_LEFT, 16, 36);
    lv_obj_set_size(mpu_bg, 290, 190);
    lv_obj_set_click(mpu_bg, false);
    lv_style_init(&bg_style);
    lv_style_set_bg_color(&bg_style, LV_STATE_DEFAULT, lv_color_make(169, 0, 103));
    lv_obj_add_style(mpu_bg, LV_OBJ_PART_MAIN, &bg_style);

    /* Create the title within the main body object */
    static lv_style_t title_style;
    lv_style_init(&title_style);
    lv_style_set_text_font(&title_style, LV_STATE_DEFAULT, LV_THEME_DEFAULT_FONT_TITLE);
    lv_style_set_text_color(&title_style, LV_STATE_DEFAULT, LV_COLOR_WHITE);
    lv_obj_t* tab_title_label = lv_label_create(mpu_bg, NULL);
    lv_obj_add_style(tab_title_label, LV_OBJ_PART_MAIN, &title_style);
    lv_label_set_static_text(tab_title_label, "ACC   | GRYRO");
    lv_obj_align(tab_title_label, mpu_bg, LV_ALIGN_IN_TOP_MID, 0, 10);


    /* Create the sensor color legend */
    lv_obj_t* lgnd_bg = lv_obj_create(mpu_bg, NULL);
    lv_obj_set_size(lgnd_bg, 200, 24);
    lv_obj_align(lgnd_bg, mpu_bg, LV_ALIGN_IN_BOTTOM_MID, 0, -10);
    lv_obj_t* legend_label = lv_label_create(lgnd_bg, NULL);
    lv_label_set_recolor(legend_label, true); // Enable recoloring of the text within the label with color HEX
    lv_label_set_static_text(legend_label, "#ff0000 X#    #008000 Y#    #0000ff Z#");
    lv_label_set_align(legend_label, LV_LABEL_ALIGN_CENTER);
    lv_obj_align(legend_label, lgnd_bg, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_local_bg_color( lgnd_bg, LV_OBJ_PART_MAIN, LV_STATE_DEFAULT, LV_COLOR_WHITE );

    /* Create a gauge */
    static lv_color_t gauge_needle_colors[3];
    gauge_needle_colors[0] = LV_COLOR_RED;
    gauge_needle_colors[1] = LV_COLOR_GREEN;
    gauge_needle_colors[2] = LV_COLOR_BLUE;

    //TODOLV_IMG_DECLARE(gauge_hand);

    lv_obj_t* gauge_acc = lv_gauge_create(mpu_bg, NULL);
    lv_obj_set_click(gauge_acc, false);
    lv_obj_set_size(gauge_acc, 106, 106);
    lv_gauge_set_scale(gauge_acc, 300, 10, 0);
    lv_gauge_set_range(gauge_acc, -20,20);
    lv_gauge_set_critical_value(gauge_acc, 2001);
    lv_gauge_set_needle_count(gauge_acc, 3, gauge_needle_colors);
    //TODOlv_gauge_set_needle_img(gauge, &gauge_hand, 5, 4);
    lv_obj_set_style_local_image_recolor_opa(gauge_acc, LV_GAUGE_PART_NEEDLE, LV_STATE_DEFAULT, LV_OPA_COVER);

    lv_obj_align(gauge_acc, NULL, LV_ALIGN_IN_LEFT_MID, 20, 0);

    lv_obj_t* gauge_gyr= lv_gauge_create(mpu_bg, NULL);
    lv_obj_set_click(gauge_gyr, false);
    lv_obj_set_size(gauge_gyr, 106, 106);
    lv_gauge_set_scale(gauge_gyr, 300, 10, 0);
    lv_gauge_set_range(gauge_gyr, -400,400);
    lv_gauge_set_critical_value(gauge_gyr, 2001);
    lv_gauge_set_needle_count(gauge_gyr, 3, gauge_needle_colors);
    //TODOlv_gauge_set_needle_img(gauge, &gauge_hand, 5, 4);
    lv_obj_set_style_local_image_recolor_opa(gauge_gyr, LV_GAUGE_PART_NEEDLE, LV_STATE_DEFAULT, LV_OPA_COVER);

    lv_obj_align(gauge_gyr, NULL, LV_ALIGN_IN_RIGHT_MID, -20, 0);
    xSemaphoreGive(xGuiSemaphore);

    static lv_obj_t* parms[2];
    parms[0] = gauge_acc;
    parms[1] = gauge_gyr;
    xTaskCreatePinnedToCore(IMU_task, "IMUTask", 2048, parms, 1, &IMU_handle, 1);

    
}

void recal(void){

    MPU6886_GetAccelData(&calib_ax, &calib_ay, &calib_az);
    MPU6886_GetGyroData(&calib_gx, &calib_gy, &calib_gz);
}

void IMU_task(void* pvParameters){

    vTaskSuspend(NULL);

    for (;;) {
        float gx, gy, gz;
        float ax, ay, az;
        MPU6886_GetAccelData(&ax, &ay, &az);
        MPU6886_GetGyroData(&gx, &gy, &gz);


        // float pitch, roll, yaw;
        // MahonyAHRSupdateIMU(gx * DEGREES_TO_RADIANS, gy * DEGREES_TO_RADIANS, gz * DEGREES_TO_RADIANS, ax, ay, az, &pitch, &roll, &yaw);
        // ESP_LOGI(TAG, "Pitch: %.6f Roll: %.6f Yaw: %.6f | Raw Accel: X-%.6f Y-%.6f Z-%.6f | Gyro: X-%.6f Y-%.6fZ- %.6f", pitch, yaw, roll, ax, ay, az, gx, gy, gz);

        ESP_LOGI(TAG, "Raw Accel: X-%.6f Y-%.6f Z-%.6f | Gyro: X-%.6f Y-%.6fZ- %.6f", ax, ay, az, gx, gy, gz);
        //ESP_LOGI(TAG, "Cal Accel: X-%.6f Y-%.6f Z-%.6f | Gyro: X-%.6f Y-%.6fZ- %.6f", calib_ax, calib_ay, calib_az, calib_gx, calib_gy, calib_gz);

        lv_obj_t** parms = (lv_obj_t**)pvParameters;
        lv_obj_t* gauge_acc = (lv_obj_t*)parms[0];
        lv_obj_t* gauge_gyr = (lv_obj_t*)parms[1];

        xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);
        lv_gauge_set_value(gauge_acc, 0, (int) (ax-calib_ax)*10);
        lv_gauge_set_value(gauge_acc, 1, (int) (ay-calib_ay)*10);
        lv_gauge_set_value(gauge_acc, 2, (int) (az-calib_az)*10);

        lv_gauge_set_value(gauge_gyr, 0, (int) (gx-calib_gx));
        lv_gauge_set_value(gauge_gyr, 1, (int) (gy-calib_gy));
        lv_gauge_set_value(gauge_gyr, 2, (int) (gz-calib_gz));
        xSemaphoreGive(xGuiSemaphore);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
    vTaskDelete(NULL); // Should never get to here...
}

