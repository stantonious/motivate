#pragma once

#include "float_buffer.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

TaskHandle_t IMU_handle;

void init_imu(void);
void imu_handler_task(void *pvParameters);

SemaphoreHandle_t xImuSemaphore;

void *ax_buf;
void *ay_buf;
void *az_buf;

void *gz_buf;
void *gx_buf;
void *gy_buf;
