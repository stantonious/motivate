#pragma once

TaskHandle_t button_handle;

void init_button_handler(void);
void button_handler_task(void* pvParameters);
