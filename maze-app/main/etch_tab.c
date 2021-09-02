
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
#define CBOX_HEIGHT 10
#define CBOX_WIDTH 10
static lv_color_t *etchbuf;
static lv_color_t *cbuf;
static const char *TAG = ETCH_TAB_NAME;

#define COLOR_SIZE 5
static lv_color_t colors[COLOR_SIZE] = {LV_COLOR_RED,
                                        LV_COLOR_ORANGE,
                                        LV_COLOR_YELLOW,
                                        LV_COLOR_GREEN,
                                        LV_COLOR_BLUE,
                                        LV_COLOR_PURPLE,
                                        LV_COLOR_TEAL};

static int current_color_idx = 0;

void display_etch_tab(lv_obj_t *tv)
{
    xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);
    lv_obj_t *test_tab = lv_tabview_add_tab(tv, ETCH_TAB_NAME); // Create a tab

    etchbuf = (lv_color_t *)heap_caps_malloc(LV_CANVAS_BUF_SIZE_TRUE_COLOR(ETCH_WIDTH, ETCH_HEIGHT), MALLOC_CAP_DEFAULT | MALLOC_CAP_SPIRAM);
    lv_obj_t *canvas = lv_canvas_create(test_tab, NULL);
    lv_canvas_set_buffer(canvas, etchbuf, ETCH_WIDTH, ETCH_HEIGHT, LV_IMG_CF_TRUE_COLOR);
    lv_obj_align(canvas, NULL, LV_ALIGN_IN_LEFT_MID, 5, 0);
    lv_canvas_fill_bg(canvas, LV_COLOR_SILVER, LV_OPA_COVER);

    cbuf = (lv_color_t *)heap_caps_malloc(LV_CANVAS_BUF_SIZE_TRUE_COLOR(CBOX_WIDTH, CBOX_HEIGHT), MALLOC_CAP_DEFAULT | MALLOC_CAP_SPIRAM);
    lv_obj_t *cbox_canvas = lv_canvas_create(test_tab, NULL);
    lv_canvas_set_buffer(cbox_canvas, cbuf, CBOX_WIDTH, CBOX_HEIGHT, LV_IMG_CF_TRUE_COLOR);
    lv_obj_align(cbox_canvas, NULL, LV_ALIGN_IN_BOTTOM_RIGHT, 5, 0);
    lv_canvas_fill_bg(cbox_canvas, colors[current_color_idx], LV_OPA_COVER);

    //inf label
    lv_obj_t *inf_lbl = lv_label_create(test_tab, NULL);
    lv_label_set_text(inf_lbl, "Unknown ");
    lv_obj_align(inf_lbl, NULL, LV_ALIGN_IN_TOP_RIGHT, 0, 0);

    lv_obj_t *ang_lbl = lv_label_create(test_tab, NULL);
    lv_label_set_text(ang_lbl, "0 ");
    lv_obj_align(ang_lbl, NULL, LV_ALIGN_IN_TOP_LEFT, 0, 0);

    xSemaphoreGive(xGuiSemaphore);

    static lv_obj_t *etch_parms[4];
    etch_parms[0] = canvas;
    etch_parms[1] = cbox_canvas;
    etch_parms[2] = inf_lbl;
    etch_parms[3] = ang_lbl;
    xTaskCreatePinnedToCore(etch_task, "EtchTask", 2048 * 2, etch_parms, 1, &Etch_handle, 1);
}

#define MOVE_ANG 15
#define X_STEP 3
#define Y_STEP 3
int current_ang = 0;

void get_step(int x, int y, int ang, int *x_new, int *y_new)
{
    if (ang > 338 || ang <= 23) //N
    {
        *x_new = x;
        *y_new = y - Y_STEP;
    }
    else if (23 < ang && ang <= 68) //NE
    {
        *x_new = x + X_STEP;
        *y_new = y - Y_STEP;
    }
    else if (68 < ang && ang <= 113) //E
    {
        *x_new = x + X_STEP;
        *y_new = y;
    }
    else if (113 < ang && ang <= 158) //SE
    {
        *x_new = x + X_STEP;
        *y_new = y + Y_STEP;
    }
    else if (158 < ang && ang <= 203) //S
    {
        *x_new = x;
        *y_new = y + Y_STEP;
    }
    else if (203 < ang && ang <= 248) //SW
    {
        *x_new = x - X_STEP;
        *y_new = y + Y_STEP;
    }
    else if (248 < ang && ang <= 293) //W
    {
        *x_new = x - X_STEP;
        *y_new = y;
    }
    else //NW
    {
        *x_new = x - X_STEP;
        *y_new = y - Y_STEP;
    }
}

void etch_task(void *pvParameters)
{

    lv_obj_t **etch_parms = (lv_obj_t **)pvParameters;
    lv_obj_t *canvas = (lv_obj_t *)etch_parms[0];
    lv_obj_t *cbox_canvas = (lv_obj_t *)etch_parms[1];
    lv_obj_t *inf_lbl = (lv_obj_t *)etch_parms[2];
    lv_obj_t *ang_lbl = (lv_obj_t *)etch_parms[3];

    int x_coord = ETCH_HEIGHT / 2;
    int y_coord = ETCH_WIDTH / 2;
    int x_new = 0;
    int y_new = 0;
    lv_draw_rect_dsc_t rect_dsc;
    lv_draw_rect_dsc_init(&rect_dsc);
    rect_dsc.bg_color = colors[current_color_idx];
    vTaskSuspend(NULL);

    for (;;)
    {
        long ticks = xTaskGetTickCount();

        int inf = get_latest_inf(4);

        xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);
        switch (inf)
        {
        case REST_LABEL:
            lv_label_set_text(inf_lbl, "Rest");
            break;
        case FORWARD_LABEL:
            lv_label_set_text(inf_lbl, "Forward");
            get_step(x_coord, y_coord, current_ang, &x_new, &y_new);
            printf("x:%d y:%d x:%d y:%d", x_coord, y_coord, x_new, y_new);
            x_coord = x_new;
            y_coord = y_new;
            lv_canvas_draw_rect(canvas, x_coord, y_coord, X_STEP, Y_STEP, &rect_dsc);
            break;
        case BACKWARD_LABEL:
            lv_label_set_text(inf_lbl, "Backward");
            get_step(x_coord, y_coord, (current_ang + 180) % 360, &x_new, &y_new);
            x_coord = x_new;
            y_coord = y_new;
            lv_canvas_draw_rect(canvas, x_coord, y_coord, X_STEP, Y_STEP, &rect_dsc);
            break;
        case UP_LABEL:
            lv_label_set_text(inf_lbl, "Up");
            current_color_idx = (current_color_idx + 1) % COLOR_SIZE;
            rect_dsc.bg_color = colors[current_color_idx];
            lv_canvas_fill_bg(cbox_canvas, colors[current_color_idx], LV_OPA_COVER);
            break;
        case DOWN_LABEL:
            lv_label_set_text(inf_lbl, "Down");
            current_color_idx = (current_color_idx - 1) % COLOR_SIZE;
            rect_dsc.bg_color = colors[current_color_idx];
            lv_canvas_fill_bg(cbox_canvas, colors[current_color_idx], LV_OPA_COVER);
            break;
        case LEFT_LABEL:
            lv_label_set_text(inf_lbl, "Left");
            current_ang += (360 - MOVE_ANG);
            current_ang = current_ang % 360;
            break;
        case RIGHT_LABEL:
            current_ang += MOVE_ANG;
            current_ang = current_ang % 360;
            lv_label_set_text(inf_lbl, "Right");
            break;
        case LEFTSIDE_LABEL:
            lv_label_set_text(inf_lbl, "Left Slide");
            get_step(x_coord, y_coord, (current_ang + 270) % 360, &x_new, &y_new);
            x_coord = x_new;
            y_coord = y_new;
            lv_canvas_draw_rect(canvas, x_coord, y_coord, X_STEP, Y_STEP, &rect_dsc);
            break;
        case RIGHTSIDE_LABEL:
            lv_label_set_text(inf_lbl, "Right Slide");
            get_step(x_coord, y_coord, (current_ang + 90) % 360, &x_new, &y_new);
            x_coord = x_new;
            y_coord = y_new;
            lv_canvas_draw_rect(canvas, x_coord, y_coord, X_STEP, Y_STEP, &rect_dsc);
            break;
        }
        lv_label_set_text_fmt(ang_lbl, "Ang:%d", current_ang);
        xSemaphoreGive(xGuiSemaphore);

        vTaskDelay(pdMS_TO_TICKS(400));
    }
    vTaskDelete(NULL); // Should never get to here...
}