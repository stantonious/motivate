
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

#include "train_tab.h"
#include "float_buffer.h"
#include "imu_task.h"

#include "mot_mqtt_client.h"
#include "mot-imu-tf.h"

#define UPDATE_THRESH 30

static const char *TAG = TRAIN_TAB_NAME;

static int current_label = 0;
static bool train_on_off = false;

void record_sample(char *topic, int type)
{
}

static float a_std_thresh = .20;
static float g_std_thresh = 80;

bool is_at_rest_pred(void *ax, void *ay, void *az, void *gx, void *gy, void *gz)
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

void display_train_tab(lv_obj_t *tv)
{

    xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);
    lv_obj_t *test_tab = lv_tabview_add_tab(tv, TRAIN_TAB_NAME); // Create a tab

    //leds
    lv_obj_t *l_lbl = lv_label_create(test_tab, NULL);
    lv_label_set_text(l_lbl, "Left ");
    lv_obj_align(l_lbl, NULL, LV_ALIGN_CENTER, -70, -75);
    lv_obj_t *l_led = lv_led_create(test_tab, NULL);
    lv_obj_align(l_led, NULL, LV_ALIGN_CENTER, 30, -75);
    lv_obj_set_size(l_led, 25, 25);
    lv_led_off(l_led);

    //leds
    lv_obj_t *r_lbl = lv_label_create(test_tab, NULL);
    lv_label_set_text(r_lbl, "Right ");
    lv_obj_align(r_lbl, NULL, LV_ALIGN_CENTER, -70, -45);
    lv_obj_t *r_led = lv_led_create(test_tab, NULL);
    lv_obj_align(r_led, NULL, LV_ALIGN_CENTER, 30, -45);
    lv_obj_set_size(r_led, 25, 25);
    lv_led_off(r_led);

    lv_obj_t *u_lbl = lv_label_create(test_tab, NULL);
    lv_label_set_text(u_lbl, "Up");
    lv_obj_align(u_lbl, NULL, LV_ALIGN_CENTER, -70, -15);

    lv_obj_t *u_led = lv_led_create(test_tab, NULL);
    lv_obj_align(u_led, NULL, LV_ALIGN_CENTER, 30, -15);
    lv_obj_set_size(u_led, 25, 25);
    lv_led_off(u_led);

    lv_obj_t *d_lbl = lv_label_create(test_tab, NULL);
    lv_label_set_text(d_lbl, "Down");
    lv_obj_align(d_lbl, NULL, LV_ALIGN_CENTER, -70, 15);
    lv_obj_t *d_led = lv_led_create(test_tab, NULL);
    lv_obj_align(d_led, NULL, LV_ALIGN_CENTER, 30, 15);
    lv_obj_set_size(d_led, 25, 25);
    lv_led_off(d_led);

    lv_obj_t *f_lbl = lv_label_create(test_tab, NULL);
    lv_label_set_text(f_lbl, "Backward");
    lv_obj_align(f_lbl, NULL, LV_ALIGN_CENTER, -70, 45);

    lv_obj_t *b_led = lv_led_create(test_tab, NULL);
    lv_obj_align(b_led, NULL, LV_ALIGN_CENTER, 30, 45);
    lv_obj_set_size(b_led, 25, 25);
    lv_led_off(b_led);

    lv_obj_t *b_lbl = lv_label_create(test_tab, NULL);
    lv_label_set_text(b_lbl, "Foward");
    lv_obj_align(b_lbl, NULL, LV_ALIGN_CENTER, -70, 75);

    lv_obj_t *f_led = lv_led_create(test_tab, NULL);
    lv_obj_align(f_led, NULL, LV_ALIGN_CENTER, 30, 75);
    lv_obj_set_size(f_led, 25, 25);
    lv_led_off(f_led);

    lv_obj_t *o_lbl = lv_label_create(test_tab, NULL);
    lv_label_set_text(o_lbl, "On/Off");
    lv_obj_align(o_lbl, NULL, LV_ALIGN_CENTER, -70, 100);

    lv_obj_t *o_led = lv_led_create(test_tab, NULL);
    lv_obj_align(o_led, NULL, LV_ALIGN_CENTER, 30, 100);
    lv_obj_set_size(o_led, 25, 25);
    lv_led_off(o_led);
    xSemaphoreGive(xGuiSemaphore);

    static lv_obj_t *parms[6];
    parms[0] = l_led;
    parms[1] = r_led;
    parms[2] = u_led;
    parms[3] = d_led;
    parms[4] = f_led;
    parms[5] = b_led;
    parms[6] = o_led;

    xTaskCreatePinnedToCore(train_task, "TrainTask", 4096, parms, 1, &Train_handle, 1);
}

void train_task(void *pvParameters)
{

    float **abuf = (float **)malloc(3 * sizeof(float *));
    for (int i = 0; i < 3; i++)
        abuf[i] = (float *)malloc(BUFSIZE * sizeof(float));
    float **gbuf = (float **)malloc(3 * sizeof(float *));
    for (int i = 0; i < 3; i++)
        gbuf[i] = (float *)malloc(BUFSIZE * sizeof(float));

    lv_obj_t **parms = (lv_obj_t **)pvParameters;
    lv_obj_t *l_led = (lv_obj_t *)parms[0];
    lv_obj_t *r_led = (lv_obj_t *)parms[1];
    lv_obj_t *u_led = (lv_obj_t *)parms[2];
    lv_obj_t *d_led = (lv_obj_t *)parms[3];
    lv_obj_t *f_led = (lv_obj_t *)parms[4];
    lv_obj_t *b_led = (lv_obj_t *)parms[5];
    lv_obj_t *o_led = (lv_obj_t *)parms[6];

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
        bool resting = is_at_rest_pred(ax_buf, ay_buf, az_buf, gx_buf, gy_buf, gz_buf);
        xSemaphoreGive(xImuSemaphore);
        if (current_label == REST_LABEL && update_delta > UPDATE_THRESH)
        {
            last_update = xTaskGetTickCount();
            if (train_on_off == true)
            {
                ESP_LOGI(TAG, "rec rest samp");
                unsigned long t = time(NULL);

                xSemaphoreTake(xImuSemaphore, portMAX_DELAY);
                mk_copy(ax_buf, abuf[0], BUFSIZE);
                mk_copy(ay_buf, abuf[1], BUFSIZE);
                mk_copy(az_buf, abuf[2], BUFSIZE);
                mk_copy(gx_buf, gbuf[0], BUFSIZE);
                mk_copy(gy_buf, gbuf[1], BUFSIZE);
                mk_copy(gz_buf, gbuf[2], BUFSIZE);
                xSemaphoreGive(xImuSemaphore);
                ESP_LOGI(TAG, "send sample");
                send_sample("topic_2", abuf, gbuf, BUFSIZE, BUFSIZE, current_label, t);
                ESP_LOGI(TAG, "send sample done");
            }
        }
        if (update_delta > UPDATE_THRESH && !resting)
        {
            ESP_LOGI(TAG, "record!");
            last_update = xTaskGetTickCount();
            if (train_on_off == true)
            {

                ESP_LOGI(TAG, "rec samp");
                unsigned long t = time(NULL);

                xSemaphoreTake(xImuSemaphore, portMAX_DELAY);
                mk_copy(ax_buf, abuf[0], BUFSIZE);
                mk_copy(ay_buf, abuf[1], BUFSIZE);
                mk_copy(az_buf, abuf[2], BUFSIZE);
                mk_copy(gx_buf, gbuf[0], BUFSIZE);
                mk_copy(gy_buf, gbuf[1], BUFSIZE);
                mk_copy(gz_buf, gbuf[2], BUFSIZE);
                xSemaphoreGive(xImuSemaphore);
                ESP_LOGI(TAG, "send sample");
                send_sample("topic_2", abuf, gbuf, BUFSIZE, BUFSIZE, current_label, t);
                ESP_LOGI(TAG, "send sample done");
            }
        }
        xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);
        if (current_label == LEFT_LABEL)
            lv_led_on(l_led);
        else
            lv_led_off(l_led);
        if (current_label == RIGHT_LABEL)
            lv_led_on(r_led);
        else
            lv_led_off(r_led);
        if (current_label == UP_LABEL)
            lv_led_on(u_led);
        else
            lv_led_off(u_led);
        if (current_label == DOWN_LABEL)
            lv_led_on(d_led);
        else
            lv_led_off(d_led);
        if (current_label == FORWARD_LABEL)
            lv_led_on(f_led);
        else
            lv_led_off(f_led);
        if (current_label == BACKWARD_LABEL)
            lv_led_on(b_led);
        else
            lv_led_off(b_led);

        if (train_on_off == true)
            lv_led_on(o_led);
        else
            lv_led_off(o_led);
        xSemaphoreGive(xGuiSemaphore);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    vTaskDelete(NULL); // Should never get to here...
}

void toggle_train_class(void)
{
    current_label = (current_label + 1) % 7;
    ESP_LOGI(TAG, "toggling class :%d", current_label);
}
void toggle_train(void)
{
    if (train_on_off == true)
        train_on_off = false;
    else
        train_on_off = true;
    ESP_LOGI(TAG, "toggling on off :%d", train_on_off);
}