
#pragma once
#include "freertos/task.h"

TaskHandle_t MotClientHandle;

void mot_mqtt_aws_client_init(void);
