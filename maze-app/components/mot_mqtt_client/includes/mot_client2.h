
#ifndef _MOT_CLIENT2_H
#define _MOT_CLIENT2_H

#include "freertos/task.h"

TaskHandle_t MotClient2Handle;

void mot_client2_init(void);
void mot_client2_task(void *pvParameters);
void get_op_x_y(int8_t* x,int8_t* y);

#endif