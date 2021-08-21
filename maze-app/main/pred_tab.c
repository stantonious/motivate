
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include "motivate_math.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "imu_task.h"
#include "esp_log.h"

#include "core2forAWS.h"

#include "pred_tab.h"
#include "float_buffer.h"

#include "mot-imu-tf.h"

#define UPDATE_THRESH 70

// -1 to convert to 0 based out of the model
#define REST_LABEL 0 - 1
#define FORWARD_LABEL 1 - 1
#define BACKWARD_LABEL 2 - 1
#define LEFT_LABEL 3 - 1
#define RIGHT_LABEL 4 - 1
#define UP_LABEL 5 - 1
#define DOWN_LABEL 6 - 1

static const char *TAG = PRED_TAB_NAME;

static float a_std_thresh = .20;
static float g_std_thresh = 80;

bool is_at_rest(void *ax, void *ay, void *az, void *gx, void *gy, void *gz)
{
    if (stdev(ax) > a_std_thresh)
        return false;
    if (stdev(ay) > a_std_thresh)
        return false;
    if (stdev(az) > a_std_thresh)
        return false;
    if (stdev(gx) > g_std_thresh)
        return false;
    if (stdev(gy) > g_std_thresh)
        return false;
    if (stdev(gz) > g_std_thresh)
        return false;
    return true;
}

void display_pred_tab(lv_obj_t *tv)
{

    xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);
    lv_obj_t *test_tab = lv_tabview_add_tab(tv, PRED_TAB_NAME); // Create a tab

    //leds
    lv_obj_t *l_lbl = lv_label_create(test_tab, NULL);
    lv_label_set_text(l_lbl, "Unknown ");
    lv_obj_align(l_lbl, NULL, LV_ALIGN_CENTER, -70, -75);

    xSemaphoreGive(xGuiSemaphore);
    static lv_obj_t *parms[6];
    parms[0] = l_lbl;

    xTaskCreatePinnedToCore(pred_task, "PredTask", 2048, parms, 1, &Pred_handle, 1);
}

void pred_task(void *pvParameters)
{
    lv_obj_t **parms = (lv_obj_t **)pvParameters;
    lv_obj_t *l_lbl = (lv_obj_t *)parms[0];

    vTaskSuspend(NULL);

    unsigned now = xTaskGetTickCount();
    unsigned update_delta = 0;
    unsigned last_update = 0; //xTaskGetTickCount();
    for (;;)
    {
        now = xTaskGetTickCount();
        update_delta = now - last_update;

        //ESP_LOGI(TAG, "ud:%ld now: %ld last_update :%ld is_at_rest:%d", update_delta,now,last_update,is_at_rest());
        xSemaphoreTake(xImuSemaphore, portMAX_DELAY);
        bool resting = is_at_rest(ax_buf, ay_buf, az_buf, gx_buf, gy_buf, gz_buf);
        xSemaphoreGive(xImuSemaphore);
        if (update_delta > UPDATE_THRESH && !resting)
        {

            ESP_LOGI(TAG, "infer ");
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
                lv_label_set_text(l_lbl, "REST ");
                break;
            case FORWARD_LABEL:
                lv_label_set_text(l_lbl, "Forward ");
                break;
            case BACKWARD_LABEL:
                lv_label_set_text(l_lbl, "Backward ");
                break;
            case UP_LABEL:
                lv_label_set_text(l_lbl, "Up ");
                break;
            case DOWN_LABEL:
                lv_label_set_text(l_lbl, "Down ");
                break;
            case LEFT_LABEL:
                lv_label_set_text(l_lbl, "Left ");
                break;
            case RIGHT_LABEL:
                lv_label_set_text(l_lbl, "Right ");
                break;
            }
            xSemaphoreGive(xGuiSemaphore);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    vTaskDelete(NULL); // Should never get to here...
}