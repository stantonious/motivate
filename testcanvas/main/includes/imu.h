#pragma once

#define IMU_TAB_NAME "IMU"

#include "math.h"

TaskHandle_t IMU_handle;

void display_imu_tab(lv_obj_t* tv);
void IMU_task(void* pvParameters);
void recal(void);
