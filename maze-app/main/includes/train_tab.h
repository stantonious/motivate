#pragma once

#define TRAIN_TAB_NAME "TRAIN"

TaskHandle_t Train_handle;

void train_task(void *pvParameters);
void display_train_tab(lv_obj_t* tv);
void toggle_train_class(void);
void toggle_train(void);
void record_sample(char* topic,int type);