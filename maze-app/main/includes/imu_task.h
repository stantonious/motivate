#pragma once

#include "float_buffer.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

TaskHandle_t IMU_handle;

void init_imu(void);
void imu_handler_task(void *pvParameters);

extern SemaphoreHandle_t xImuSemaphore;

extern void *ax_buf;
extern void *ay_buf;
extern void *az_buf;

extern void *gz_buf;
extern void *gx_buf;
extern void *gy_buf;
