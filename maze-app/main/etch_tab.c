
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"

#include "core2forAWS.h"

#include "imu_task.h"
//#include "arrow-img-30x30.h"
#include "etch_tab.h"

#include "maze_utils.h"
#include "mot-imu-tf.h"

#define ETCH_HEIGHT 160
#define ETCH_WIDTH 180
static lv_color_t *etchbuf;
static const char *TAG = ETCH_TAB_NAME;

static uint map_projection = NORTH_DIR;

static long last_move_ticks = 0;

void display_etch_tab(lv_obj_t *tv)
{
    xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);
    lv_obj_t *test_tab = lv_tabview_add_tab(tv, ETCH_TAB_NAME); // Create a tab

    etchbuf = (lv_color_t *)heap_caps_malloc(LV_CANVAS_BUF_SIZE_TRUE_COLOR(ETCH_WIDTH, ETCH_HEIGHT), MALLOC_CAP_DEFAULT | MALLOC_CAP_SPIRAM);
    lv_obj_t *canvas = lv_canvas_create(test_tab, NULL);
    lv_canvas_set_buffer(canvas, etchbuf, ETCH_WIDTH, ETCH_HEIGHT, LV_IMG_CF_TRUE_COLOR);

    lv_obj_align(canvas, NULL, LV_ALIGN_IN_LEFT_MID, 5, 0);
    lv_canvas_fill_bg(canvas, LV_COLOR_SILVER, LV_OPA_COVER);
//inf label
    lv_obj_t *inf_lbl = lv_label_create(test_tab, NULL);
    lv_label_set_text(inf_lbl, "Unknown ");
    lv_obj_align(inf_lbl, NULL, LV_ALIGN_IN_TOP_RIGHT, 0, 0);


    xSemaphoreGive(xGuiSemaphore);

    static lv_obj_t *etch_parms[1];
    etch_parms[0] = canvas;
    etch_parms[1] = inf_lbl;
    xTaskCreatePinnedToCore(etch_task, "EtchTask", 2048 * 2, etch_parms, 1, &Etch_handle, 1);
}

void etch_task(void *pvParameters)
{

    lv_obj_t **etch_parms = (lv_obj_t **)pvParameters;
    lv_obj_t *canvas = (lv_obj_t *)etch_parms[0];
    lv_obj_t *inf_lbl = (lv_obj_t *)etch_parms[1];

    int x_coord = ETCH_HEIGHT / 2;
    int y_coord = ETCH_WIDTH / 2;
    int x_step = 3;
    int y_step = 3;
    lv_draw_rect_dsc_t rect_dsc;
    lv_draw_rect_dsc_init(&rect_dsc);
            rect_dsc.bg_color=LV_COLOR_ORANGE;

    vTaskSuspend(NULL);

    for (;;)
    {
        long ticks = xTaskGetTickCount();
        long update_delta = ticks - last_move_ticks;
        bool moved = false;
        int dir = NORTH_DIR;

        int inf = get_latest_inf(4);

        xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);
        switch (inf)
        {
        case REST_LABEL:
        lv_label_set_text(inf_lbl, "Rest");
            break;
        case FORWARD_LABEL:
            lv_label_set_text(inf_lbl, "Forward");
            lv_canvas_draw_rect(canvas,x_coord,y_coord,x_step,y_step,&rect_dsc);
            if (x_coord + x_step <= ETCH_WIDTH) x_coord += x_step;
            break;
        case BACKWARD_LABEL:
            lv_label_set_text(inf_lbl, "Backward");
            lv_canvas_draw_rect(canvas,x_coord,y_coord,x_step,y_step,&rect_dsc);
            if (x_coord - x_step >= 0)x_coord -= x_step;
            break;
        case UP_LABEL:
            lv_label_set_text(inf_lbl, "Up");
            break;
        case DOWN_LABEL:
            lv_label_set_text(inf_lbl, "Down");
            break;
        case LEFT_LABEL:
            lv_label_set_text(inf_lbl, "Left");
            lv_canvas_transform(canvas,lv_canvas_get_img(canvas),-5,256,0,0,ETCH_WIDTH/2,ETCH_HEIGHT/2,false);
            break;
        case RIGHT_LABEL:
            lv_label_set_text(inf_lbl, "Right");
            lv_canvas_transform(canvas,lv_canvas_get_img(canvas),5,256,0,0,ETCH_WIDTH/2,ETCH_HEIGHT/2,false);
            break;
        case LEFTSIDE_LABEL:
            lv_label_set_text(inf_lbl, "Left Slide");
            lv_canvas_draw_rect(canvas,x_coord,y_coord,x_step,y_step,&rect_dsc);
            if (y_coord - y_step >= 0)y_coord -= y_step;
            break;
        case RIGHTSIDE_LABEL:
            lv_label_set_text(inf_lbl, "Right Slide");
            lv_canvas_draw_rect(canvas,x_coord,y_coord,x_step,y_step,&rect_dsc);
            if (y_coord + y_step <= ETCH_HEIGHT)y_coord += y_step;
            break;
        }
        xSemaphoreGive(xGuiSemaphore);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
    vTaskDelete(NULL); // Should never get to here...
}