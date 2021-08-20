#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

TaskHandle_t mot_imu_handle;

#ifdef __cplusplus
extern "C" {
#endif
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

