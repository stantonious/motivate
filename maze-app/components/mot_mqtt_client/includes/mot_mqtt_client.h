
#ifndef _MOT_CLIENT2_H
#define _MOT_CLIENT2_H

#include "freertos/task.h"

TaskHandle_t MotMqttClientHandle;

void mot_mqtt_client_init(void);
void get_op_x_y(int8_t* x,int8_t* y);

#endif