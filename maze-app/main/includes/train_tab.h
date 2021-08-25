#pragma once

#define TRAIN_TAB_NAME "TRAIN"

TaskHandle_t Train_handle;

extern  int current_label;
extern  bool train_on_off;

void train_task(void *pvParameters);
void display_train_tab(lv_obj_t* tv);
void toggle_train_class(void);
void toggle_train(void);