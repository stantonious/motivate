
#ifndef _MOT_CLIENT2_H
#define _MOT_CLIENT2_H

#include "freertos/task.h"

TaskHandle_t MotMqttClientHandle;

void mot_mqtt_client_init(int game_id);
void get_op_x_y_t(int8_t* x,int8_t* y,int8_t* t);
void send_position(int x, int y, int t,unsigned time);
void send_sample(float **a_samples,float **g_samples,int a_size,int g_size,int type,unsigned time);

#endif