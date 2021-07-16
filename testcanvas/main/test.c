
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"

#include "core2forAWS.h"

#include "test.h"

#define CANVAS_WIDTH  200
#define CANVAS_HEIGHT  100

static lv_color_t* cbuf;

void display_test_tab(lv_obj_t* tv)
{
  xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);
  lv_obj_t* test_tab = lv_tabview_add_tab(tv, TEST_TAB_NAME); // Create a tab

  cbuf =(lv_color_t*)heap_caps_malloc(LV_CANVAS_BUF_SIZE_TRUE_COLOR(CANVAS_WIDTH, CANVAS_HEIGHT), MALLOC_CAP_DEFAULT | MALLOC_CAP_SPIRAM);
  lv_obj_t * canvas = lv_canvas_create(test_tab, NULL);
  lv_canvas_set_buffer(canvas, cbuf, CANVAS_WIDTH, CANVAS_HEIGHT, LV_IMG_CF_TRUE_COLOR);

  lv_obj_align(canvas, NULL, LV_ALIGN_CENTER, 0, 0);
  lv_canvas_fill_bg(canvas, LV_COLOR_SILVER, LV_OPA_COVER);

  lv_draw_rect_dsc_t rect_dsc;
  lv_draw_rect_dsc_init(&rect_dsc);
  rect_dsc.bg_color = LV_COLOR_RED;

  for (int i=0;i<10;i++){
    lv_canvas_draw_rect(canvas,i*10,(i+1)*10,20,30,&rect_dsc);
  }
  xSemaphoreGive(xGuiSemaphore);

}

void Test_task(void* pvParameters)
{

}