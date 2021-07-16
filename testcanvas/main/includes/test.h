#pragma once

#define TEST_TAB_NAME "TEST"

TaskHandle_t test_handle;

void display_test_tab(lv_obj_t* tv);
void Test_task(void* pvParameters);
