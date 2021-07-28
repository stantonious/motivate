#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

TaskHandle_t tf_handle;

#ifdef __cplusplus
extern "C" {
#endif
void init_tf(void);
void tf_task(void* pvParameters);
#ifdef __cplusplus
}
#endif

