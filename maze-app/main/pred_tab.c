
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

    xSemaphoreGive(xGuiSemaphore );
    static lv_obj_t *parms[6];
    parms[0] = l_lbl;

    xTaskCreatePinnedToCore(pred_task, "PredTask", 2048, parms, 1, &Pred_handle, 1);
}

void pred_task(void *pvParameters)
{
    init_mot_imu();

    float **abuf = (float **)malloc(3 * sizeof(float *));
    for (int i = 0; i < 3; i++)
        abuf[i] = (float *)malloc(BUFSIZE * sizeof(float));
    float **gbuf = (float **)malloc(3 * sizeof(float *));
    for (int i = 0; i < 3; i++)
        gbuf[i] = (float *)malloc(BUFSIZE * sizeof(float));

    void *ax_buf_train = get_buffer();
    void *ay_buf_train = get_buffer();
    void *az_buf_train = get_buffer();

    void *gz_buf_train = get_buffer();
    void *gx_buf_train = get_buffer();
    void *gy_buf_train = get_buffer();

    lv_obj_t **parms = (lv_obj_t **)pvParameters;
    lv_obj_t *l_lbl = (lv_obj_t *)parms[0];

    vTaskSuspend(NULL);

    float gx, gy, gz;
    float ax, ay, az;
    unsigned now = xTaskGetTickCount();
    unsigned update_delta = 0;
    unsigned last_update = 0; //xTaskGetTickCount();
    for (;;)
    {
        now = xTaskGetTickCount();
        update_delta = now - last_update;

        MPU6886_GetAccelData(&ax, &ay, &az);
        MPU6886_GetGyroData(&gx, &gy, &gz);

        push(ax_buf_train, ax);
        push(ay_buf_train, ay);
        push(az_buf_train, az);

        push(gx_buf_train, gx);
        push(gy_buf_train, gy);
        push(gz_buf_train, gz);

        //ESP_LOGI(TAG, "acc x:%.6f y:%.6f z:%.6f", ax,ay,az);
        //ESP_LOGI(TAG, "gyr x:%.6f y:%.6f z:%.6f", gx,gy,gz);
        //ESP_LOGI(TAG, "acc abs x:%.6f y:%.6f z:%.6f", stdev(ax_buf),stdev(ay_buf),stdev(az_buf_train));
        //ESP_LOGI(TAG, "gyr abs x:%.6f y:%.6f z:%.6f", stdev(gx_buf),stdev(gy_buf),stdev(gz_buf));

        //ESP_LOGI(TAG, "ud:%ld now: %ld last_update :%ld is_at_rest:%d", update_delta,now,last_update,is_at_rest());
        if (update_delta > UPDATE_THRESH && !is_at_rest(ax_buf_train, ay_buf_train, az_buf_train, gx_buf_train, gy_buf_train, gz_buf_train))
        {
            last_update = now;
            ESP_LOGI(TAG, "pred!");
            //last_update = xTaskGetTickCount();

            //mk_copy(ax_buf_train, abuf[0], BUFSIZE);
            //mk_copy(ay_buf_train, abuf[1], BUFSIZE);
            //mk_copy(az_buf_train, abuf[2], BUFSIZE);
            //mk_copy(gx_buf_train, gbuf[0], BUFSIZE);
            //mk_copy(gy_buf_train, gbuf[1], BUFSIZE);
            //mk_copy(gz_buf_train, gbuf[2], BUFSIZE);

            ESP_LOGI(TAG, "infer ");
            int inf = buffer_infer(
                ax_buf_train,
                ay_buf_train,
                az_buf_train,
                gx_buf_train,
                gy_buf_train,
                gz_buf_train);


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