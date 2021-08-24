#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


#ifdef __cplusplus
extern "C" {
#endif


#define UNCERTAIN_LABEL -1
#define REST_LABEL 0
#define FORWARD_LABEL 1
#define BACKWARD_LABEL 2
#define LEFT_LABEL 3
#define RIGHT_LABEL 4
#define UP_LABEL 5
#define DOWN_LABEL 6
#define LEFTSIDE_LABEL 7
#define RIGHTSIDE_LABEL 8

TaskHandle_t mot_imu_handle;

void init_mot_imu(void);
void mot_imu_task(void* pvParameters);
int infer(float **a_samples, float **g_samples, int a_size, int g_size);
int buffer_infer(void* ax,
                 void* ay,
                 void* az,
                 void* gx,
                 void* gy,
                 void* gz);
#ifdef __cplusplus
}
#endif

