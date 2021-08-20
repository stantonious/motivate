#pragma once

#define TRAIN_TAB_NAME "TRAIN"

TaskHandle_t Train_handle;

void train_task(void *pvParameters);
void display_train_tab(lv_obj_t* tv);
void toggle_train_left_right(void);
void toggle_train_up_down(void);
void toggle_train_forward_back(void);
void record_sample(char* topic,int type);