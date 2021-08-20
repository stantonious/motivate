
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

#include "mot_mqtt_client.h"
#include "mot-imu-tf.h"

#define UPDATE_THRESH 70

static const char *TAG = TRAIN_TAB_NAME;

static float **abuf = NULL;
static float **gbuf = NULL;

static bool record_left = false;
static bool record_right = false;
static bool record_forward = false;
static bool record_up = false;
static bool record_down = false;
static bool record_back = false;

static void *ax_buf;
static void *ay_buf;
static void *az_buf;

static void *gx_buf;
static void *gy_buf;
static void *gz_buf;

static long last_update = 0;

void record_sample(char *topic, int type)
{
    ESP_LOGI(TAG, "rec samp");
    unsigned long t = time(NULL);
    ESP_LOGI(TAG, "got time %lu", t);

    mk_copy(ax_buf, abuf[0], BUFSIZE);
    mk_copy(ay_buf, abuf[1], BUFSIZE);
    mk_copy(az_buf, abuf[2], BUFSIZE);
    mk_copy(gx_buf, gbuf[0], BUFSIZE);
    mk_copy(gy_buf, gbuf[1], BUFSIZE);
    mk_copy(gz_buf, gbuf[2], BUFSIZE);

    ESP_LOGI(TAG, "send sample");
    send_sample(topic, abuf, gbuf, BUFSIZE, BUFSIZE, type, t);
    ESP_LOGI(TAG, "send sample done");
}


static float gyr_coefs[] = {};
static float a_std_thresh = .20;
static float g_std_thresh = 80;

bool is_at_rest(){
    if (stdev(gx_buf) > g_std_thresh) return false;
    if (stdev(gy_buf) > g_std_thresh) return false;
    if (stdev(gz_buf) > g_std_thresh) return false;
    if (stdev(ax_buf) > a_std_thresh) return false;
    if (stdev(ay_buf) > a_std_thresh) return false;
    if (stdev(az_buf) > a_std_thresh) return false;
    return true;

}

void display_train_tab(lv_obj_t *tv)
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

    xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);
    lv_obj_t *test_tab = lv_tabview_add_tab(tv, TRAIN_TAB_NAME); // Create a tab

    //leds
    lv_obj_t *l_led = lv_led_create(test_tab, NULL);
    lv_obj_align(l_led, NULL, LV_ALIGN_CENTER, -30, -45);
    lv_obj_set_size(l_led, 25, 25);
    lv_led_off(l_led);

    lv_obj_t *r_led = lv_led_create(test_tab, NULL);
    lv_obj_align(r_led, NULL, LV_ALIGN_CENTER, 5, -45);
    lv_obj_set_size(r_led, 25, 25);
    lv_led_off(r_led);

    lv_obj_t *u_led = lv_led_create(test_tab, NULL);
    lv_obj_align(u_led, NULL, LV_ALIGN_CENTER, -30, -15);
    lv_obj_set_size(u_led, 25, 25);
    lv_led_off(u_led);

    lv_obj_t *d_led = lv_led_create(test_tab, NULL);
    lv_obj_align(d_led, NULL, LV_ALIGN_CENTER, 5, -15);
    lv_obj_set_size(d_led, 25, 25);
    lv_led_off(d_led);

    lv_obj_t *b_led = lv_led_create(test_tab, NULL);
    lv_obj_align(b_led, NULL, LV_ALIGN_CENTER, -30, 45);
    lv_obj_set_size(b_led, 25, 25);
    lv_led_off(b_led);

    lv_obj_t *f_led = lv_led_create(test_tab, NULL);
    lv_obj_align(f_led, NULL, LV_ALIGN_CENTER, 5, 45);
    lv_obj_set_size(f_led, 25, 25);
    lv_led_off(f_led);
    xSemaphoreGive(xGuiSemaphore);

    static lv_obj_t *parms[6];
    parms[0] = l_led;
    parms[1] = r_led;
    parms[2] = u_led;
    parms[3] = d_led;
    parms[4] = f_led;
    parms[5] = b_led;

    xTaskCreatePinnedToCore(train_task, "TrainTask", 2048, parms, 1, &Train_handle, 1);
}

void train_task(void *pvParameters)
{

    lv_obj_t **parms = (lv_obj_t **)pvParameters;
    lv_obj_t *l_led = (lv_obj_t *)parms[0];
    lv_obj_t *r_led = (lv_obj_t *)parms[1];
    lv_obj_t *u_led = (lv_obj_t *)parms[2];
    lv_obj_t *d_led = (lv_obj_t *)parms[3];
    lv_obj_t *f_led = (lv_obj_t *)parms[4];
    lv_obj_t *b_led = (lv_obj_t *)parms[5];

    vTaskSuspend(NULL);

    for (;;)
    {
        float gx, gy, gz;
        float ax, ay, az;

        long now = xTaskGetTickCount();
        long update_delta = now - last_update;

        MPU6886_GetAccelData(&ax, &ay, &az);
        MPU6886_GetGyroData(&gx, &gy, &gz);

        //ESP_LOGI(TAG, "acc x:%.6f y:%.6f z:%.6f", ax,ay,az);
        //ESP_LOGI(TAG, "gyr x:%.6f y:%.6f z:%.6f", gx,gy,gz);
        //ESP_LOGI(TAG, "acc abs x:%.6f y:%.6f z:%.6f", stdev(ax_buf),stdev(ay_buf),stdev(az_buf));
        //ESP_LOGI(TAG, "gyr abs x:%.6f y:%.6f z:%.6f", stdev(gx_buf),stdev(gy_buf),stdev(gz_buf));
        push(ax_buf, ax);
        push(ay_buf, ay);
        push(az_buf, az);

        push(gx_buf, gx);
        push(gy_buf, gy);
        push(gz_buf, gz);

        if (update_delta > UPDATE_THRESH && !is_at_rest())
        {
            ESP_LOGI(TAG, "record!");
            last_update = now;

        }
        xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);
        if (record_left)lv_led_on(l_led);
        else lv_led_off(l_led);
        if (record_right)lv_led_on(r_led);
        else lv_led_off(r_led);
        if (record_up)lv_led_on(u_led);
        else lv_led_off(u_led);
        if (record_down)lv_led_on(d_led);
        else lv_led_off(d_led);
        if (record_forward)lv_led_on(f_led);
        else lv_led_off(f_led);
        if (record_back)lv_led_on(b_led);
        else lv_led_off(b_led);
        xSemaphoreGive(xGuiSemaphore);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    vTaskDelete(NULL); // Should never get to here...
}

void toggle_train_left_right(void)
{
    ESP_LOGI(TAG, "toggling lr");
    if (record_left){
        record_left=false;
        record_right=true;
    }else if (record_right){
        record_left=false;
        record_right=false;
    }else{
        record_left=true;
        record_right=false;
    }
}
void toggle_train_up_down(void)
{
    ESP_LOGI(TAG, "toggling ud");
    if (record_up){
        record_up=false;
        record_down=true;
    }else if (record_down){
        record_up=false;
        record_down=false;
    }else{
        record_up=true;
        record_down=false;
    }
}
void toggle_train_forward_back(void)
{
    ESP_LOGI(TAG, "toggling fb");
    if (record_forward){
        record_forward=false;
        record_back=true;
    }else if (record_back){
        record_forward=false;
        record_back=false;
    }else{
        record_forward=true;
        record_back=false;
    }
}