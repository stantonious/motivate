#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define REST_LABEL 0
#define FORWARD_LABEL 1
#define BACKWARD_LABEL 2
#define LEFT_LABEL 3
#define RIGHT_LABEL 4
#define UP_LABEL 5
#define DOWN_LABEL 6
#define LEFTSIDE_LABEL 7
#define RIGHTSIDE_LABEL 8
#define UNCERTAIN_LABEL 9

#define NUM_CLASSES 10

    TaskHandle_t mot_imu_handle;

    void init_mot_imu(void);
    void mot_imu_task(void *pvParameters);
    int buffer_infer(void *ax,
                     void *ay,
                     void *az,
                     void *gx,
                     void *gy,
                     void *gz);

    int buffer_confs(void *ax,
                     void *ay,
                     void *az,
                     void *gx,
                     void *gy,
                     void *gz,
                     float *buf);
    int get_latest_inf(int n_last);
#ifdef __cplusplus
}
#endif
