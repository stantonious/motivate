
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

int current_label;
bool train_on_off;

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

    lv_obj_t *b_lbl = lv_label_create(test_tab, NULL);
    lv_label_set_text(b_lbl, "Foward");
    lv_obj_align(b_lbl, NULL, LV_ALIGN_CENTER, -70, -70);

    lv_obj_t *f_led = lv_led_create(test_tab, NULL);
    lv_obj_align(f_led, NULL, LV_ALIGN_CENTER, 30, -70);
    lv_obj_set_size(f_led, 20, 20);
    lv_led_off(f_led);

    lv_obj_t *f_lbl = lv_label_create(test_tab, NULL);
    lv_label_set_text(f_lbl, "Backward");
    lv_obj_align(f_lbl, NULL, LV_ALIGN_CENTER, -70, -50);

    lv_obj_t *b_led = lv_led_create(test_tab, NULL);
    lv_obj_align(b_led, NULL, LV_ALIGN_CENTER, 30, -50);
    lv_obj_set_size(b_led, 20, 20);
    lv_led_off(b_led);

    //leds
    lv_obj_t *l_lbl = lv_label_create(test_tab, NULL);
    lv_label_set_text(l_lbl, "Left ");
    lv_obj_align(l_lbl, NULL, LV_ALIGN_CENTER, -70, -30);
    lv_obj_t *l_led = lv_led_create(test_tab, NULL);
    lv_obj_align(l_led, NULL, LV_ALIGN_CENTER, 30, -30);
    lv_obj_set_size(l_led, 20, 20);
    lv_led_off(l_led);

    //leds
    lv_obj_t *r_lbl = lv_label_create(test_tab, NULL);
    lv_label_set_text(r_lbl, "Right ");
    lv_obj_align(r_lbl, NULL, LV_ALIGN_CENTER, -70, -10);
    lv_obj_t *r_led = lv_led_create(test_tab, NULL);
    lv_obj_align(r_led, NULL, LV_ALIGN_CENTER, 30, -10);
    lv_obj_set_size(r_led, 20, 20);
    lv_led_off(r_led);

    lv_obj_t *u_lbl = lv_label_create(test_tab, NULL);
    lv_label_set_text(u_lbl, "Up");
    lv_obj_align(u_lbl, NULL, LV_ALIGN_CENTER, -70, 10);

    lv_obj_t *u_led = lv_led_create(test_tab, NULL);
    lv_obj_align(u_led, NULL, LV_ALIGN_CENTER, 30, 10);
    lv_obj_set_size(u_led, 20, 20);
    lv_led_off(u_led);

    lv_obj_t *d_lbl = lv_label_create(test_tab, NULL);
    lv_label_set_text(d_lbl, "Down");
    lv_obj_align(d_lbl, NULL, LV_ALIGN_CENTER, -70, 30);
    lv_obj_t *d_led = lv_led_create(test_tab, NULL);
    lv_obj_align(d_led, NULL, LV_ALIGN_CENTER, 30, 30);
    lv_obj_set_size(d_led, 20, 20);
    lv_led_off(d_led);

    lv_obj_t *ls_lbl = lv_label_create(test_tab, NULL);
    lv_label_set_text(ls_lbl, "Left Side");
    lv_obj_align(ls_lbl, NULL, LV_ALIGN_CENTER, -70, 50);

    lv_obj_t *ls_led = lv_led_create(test_tab, NULL);
    lv_obj_align(ls_led, NULL, LV_ALIGN_CENTER, 30, 50);
    lv_obj_set_size(ls_led, 20, 20);
    lv_led_off(ls_led);

    lv_obj_t *rs_lbl = lv_label_create(test_tab, NULL);
    lv_label_set_text(rs_lbl, "Right Side");
    lv_obj_align(rs_lbl, NULL, LV_ALIGN_CENTER, -70, 70);

    lv_obj_t *rs_led = lv_led_create(test_tab, NULL);
    lv_obj_align(rs_led, NULL, LV_ALIGN_CENTER, 30, 70);
    lv_obj_set_size(rs_led, 20, 20);
    lv_led_off(rs_led);

    lv_obj_t *o_lbl = lv_label_create(test_tab, NULL);
    lv_label_set_text(o_lbl, "On/Off");
    lv_obj_align(o_lbl, NULL, LV_ALIGN_CENTER, -70, 90);

    lv_obj_t *o_led = lv_led_create(test_tab, NULL);
    lv_obj_align(o_led, NULL, LV_ALIGN_CENTER, 30, 90);
    lv_obj_set_size(o_led, 20, 20);
    lv_led_off(o_led);
    xSemaphoreGive(xGuiSemaphore);

    static lv_obj_t *train_parms[9];
    train_parms[0] = l_led;
    train_parms[1] = r_led;
    train_parms[2] = u_led;
    train_parms[3] = d_led;
    train_parms[4] = f_led;
    train_parms[5] = b_led;
    train_parms[6] = o_led;
    train_parms[7] = ls_led;
    train_parms[8] = rs_led;

    xTaskCreatePinnedToCore(train_task, "TrainTask", 4096, train_parms, 1, &Train_handle, 1);
}

void train_task(void *pvParameters)
{
    current_label = 0;
    train_on_off = false;

    float **abuf = (float **)malloc(3 * sizeof(float *));
    for (int i = 0; i < 3; i++)
        abuf[i] = (float *)malloc(BUFSIZE * sizeof(float));
    float **gbuf = (float **)malloc(3 * sizeof(float *));
    for (int i = 0; i < 3; i++)
        gbuf[i] = (float *)malloc(BUFSIZE * sizeof(float));

    lv_obj_t **train_parms = (lv_obj_t **)pvParameters;
    lv_obj_t *l_led = (lv_obj_t *)train_parms[0];
    lv_obj_t *r_led = (lv_obj_t *)train_parms[1];
    lv_obj_t *u_led = (lv_obj_t *)train_parms[2];
    lv_obj_t *d_led = (lv_obj_t *)train_parms[3];
    lv_obj_t *f_led = (lv_obj_t *)train_parms[4];
    lv_obj_t *b_led = (lv_obj_t *)train_parms[5];
    lv_obj_t *o_led = (lv_obj_t *)train_parms[6];
    lv_obj_t *ls_led = (lv_obj_t *)train_parms[7];
    lv_obj_t *rs_led = (lv_obj_t *)train_parms[8];

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
        //bool resting = is_at_rest_pred(ax_buf, ay_buf, az_buf, gx_buf, gy_buf, gz_buf);
        bool resting = false;
        xSemaphoreGive(xImuSemaphore);

        if (train_on_off == true && update_delta > UPDATE_THRESH)
        {
            last_update = xTaskGetTickCount();
            unsigned long t = time(NULL);

            xSemaphoreTake(xImuSemaphore, portMAX_DELAY);
            mk_copy(ax_buf, abuf[0], BUFSIZE);
            mk_copy(ay_buf, abuf[1], BUFSIZE);
            mk_copy(az_buf, abuf[2], BUFSIZE);
            mk_copy(gx_buf, gbuf[0], BUFSIZE);
            mk_copy(gy_buf, gbuf[1], BUFSIZE);
            mk_copy(gz_buf, gbuf[2], BUFSIZE);
            xSemaphoreGive(xImuSemaphore);
            if (current_label == REST_LABEL)
            {
                send_sample(abuf, gbuf, BUFSIZE, BUFSIZE, current_label, t);
            }
            else if (resting != true)
            {
                send_sample(abuf, gbuf, BUFSIZE, BUFSIZE, current_label, t);
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
        if (current_label == LEFTSIDE_LABEL)
            lv_led_on(ls_led);
        else
            lv_led_off(ls_led);
        if (current_label == RIGHTSIDE_LABEL)
            lv_led_on(rs_led);
        else
            lv_led_off(rs_led);

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
    current_label = (current_label + 1) % 9;
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