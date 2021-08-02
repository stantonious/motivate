
#pragma once
#include "freertos/task.h"

TaskHandle_t MotClientHandle;

void mot_client_init(void);
void mot_client_task(void *pvParameters);
