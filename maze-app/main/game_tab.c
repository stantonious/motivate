
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"

#include "core2forAWS.h"

#include "game_tab.h"

static int16_t game_move_sensitivity = 0;

static void event_handler(lv_obj_t * obj, lv_event_t event)
{
    printf(" changing sense  %d",event);
    if(event == LV_EVENT_VALUE_CHANGED) {
        game_move_sensitivity = 100 - lv_slider_get_value(obj);
        printf(" changing sense to %d",game_move_sensitivity);
    }
}

int16_t get_sensitivity()
{
    return game_move_sensitivity;
}

void display_game_tab(lv_obj_t *tv)
{

    xSemaphoreTake(xGuiSemaphore, portMAX_DELAY);
    lv_obj_t *test_tab = lv_tabview_add_tab(tv, GAME_TAB_NAME); // Create a tab
    lv_obj_t *cont;

    cont = lv_cont_create(test_tab, NULL);
    lv_obj_set_auto_realign(cont, true);                   /*Auto realign when the size changes*/
    lv_obj_align_origo(cont, NULL, LV_ALIGN_CENTER, 0, 0); /*This parametrs will be sued when realigned*/
    lv_cont_set_fit(cont, LV_FIT_TIGHT);
    lv_cont_set_layout(cont, LV_LAYOUT_COLUMN_MID);

    lv_obj_t *label = lv_label_create(cont, NULL);
    lv_label_set_text_fmt(label, "MOTIVE - MAZE");
    lv_obj_align(label, NULL, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_size(label, LV_VER_RES, LV_HOR_RES - 10);


     /*Create a slider*/
    lv_obj_t *s_lbl = lv_label_create(cont, NULL);
    lv_label_set_text_fmt(s_lbl, "Sensitivity");
    lv_obj_align(s_lbl, NULL, LV_ALIGN_IN_LEFT_MID, 0, 0);
    lv_obj_t * move_sensitivity_slider = lv_slider_create(cont, NULL);
    lv_obj_align(move_sensitivity_slider, NULL, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_event_cb(move_sensitivity_slider, event_handler);
    lv_slider_set_range(move_sensitivity_slider, 20 , 90);

    lv_obj_t *table = lv_table_create(cont, NULL);
    lv_table_set_col_cnt(table, 2);
    lv_table_set_row_cnt(table, 3);
    lv_obj_align(table, NULL, LV_ALIGN_CENTER, 0, 0);

    lv_table_set_cell_value(table, 0, 0, "Maze");
    lv_table_set_cell_value(table, 1, 0, "Level");
    lv_table_set_cell_value(table, 2, 0, "Name");

    lv_table_set_cell_value_fmt(table, 0, 1, "CrazeMaze");
    lv_table_set_cell_value_fmt(table, 1, 1, "7");
    lv_table_set_cell_value_fmt(table, 2, 1, "Stantonious");

    xSemaphoreGive(xGuiSemaphore);
}