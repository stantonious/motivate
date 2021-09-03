
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"

#include "core2forAWS.h"

#include "globals.h"
#include "maze_client.h"
#include "game_tab.h"


static void back_ev(lv_obj_t * obj, lv_event_t event)
{
    printf(" changing sense  %d",event);
    if(event == LV_EVENT_VALUE_CHANGED) {
        use_back = lv_checkbox_get_state(obj);
    }
}

static void side_ev(lv_obj_t * obj, lv_event_t event)
{
    printf(" changing sense  %d",event);
    if(event == LV_EVENT_VALUE_CHANGED) {
        use_side = lv_checkbox_get_state(obj);
    }
}
static void event_handler(lv_obj_t * obj, lv_event_t event)
{
    printf(" changing sense  %d",event);
    if(event == LV_EVENT_VALUE_CHANGED) {
        move_sensitivity = 100 - lv_slider_get_value(obj);
        printf(" changing sense to %d",move_sensitivity);
    }
}

static void event_handler_turn(lv_obj_t * obj, lv_event_t event)
{
    printf(" changing sense  %d",event);
    if(event == LV_EVENT_VALUE_CHANGED) {
        turn_sensitivity = 100 - lv_slider_get_value(obj);
        printf(" changing sense to %d",turn_sensitivity);
    }
}

static void game_id_event_handler(lv_obj_t * obj, lv_event_t event)
{
    if(event == LV_EVENT_VALUE_CHANGED) {
        char buf[32];
        lv_dropdown_get_selected_str(obj, buf, sizeof(buf));
        printf("Option: %s\n", buf);
        if (strcmp(buf,"new")==0)game_id = 0;
        else game_id = atoi(buf);
        get_maze(game_id,MAZE_HEIGHT, MAZE_LEN, MAZE,&x_entry,&y_entry,&x_exit,&y_exit);
    }
}

static void player_type_event_handler(lv_obj_t * obj, lv_event_t event)
{
    if(event == LV_EVENT_VALUE_CHANGED) {
        char buf[32];
        lv_dropdown_get_selected_str(obj, buf, sizeof(buf));
        printf("Option: %s\n", buf);
        if (strcmp(buf,"wizard")==0)player_type = WIZARD;
        else if (strcmp(buf,"rogue")==0)player_type = ROGUE;
        else if (strcmp(buf,"fighter")==0)player_type = FIGHTER;
        printf("platyer type is %d",player_type);
    }
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

    lv_obj_t * gamelist = lv_dropdown_create(cont, NULL);
    lv_dropdown_set_options(gamelist, 
            "1630704403\n"
            "1630704410\n"
            "1630704411\n"
            "new\n"
            );

    lv_obj_align(gamelist, NULL, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_event_cb(gamelist, game_id_event_handler);

    lv_obj_t * playerlist = lv_dropdown_create(cont, NULL);
    lv_dropdown_set_options(playerlist, 
            "wizard\n"
            "rogue\n"
            "fighter\n"
            );

    lv_obj_align(playerlist, NULL, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_event_cb(playerlist, player_type_event_handler);

     /*Create a slider*/
    lv_obj_t *s_lbl = lv_label_create(cont, NULL);
    lv_label_set_text_fmt(s_lbl, "Turn/Move Sens");
    lv_obj_align(s_lbl, NULL, LV_ALIGN_IN_LEFT_MID, 0, 0);
    lv_obj_t * move_sensitivity_slider = lv_slider_create(cont, NULL);
    lv_obj_align(move_sensitivity_slider, NULL, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_event_cb(move_sensitivity_slider, event_handler);
    lv_slider_set_range(move_sensitivity_slider, -20 , 90);
    lv_slider_set_value(move_sensitivity_slider,100-move_sensitivity,LV_ANIM_OFF);

    lv_obj_t * turn_sensitivity_slider = lv_slider_create(cont, NULL);
    lv_obj_align(turn_sensitivity_slider, NULL, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_event_cb(turn_sensitivity_slider, event_handler_turn);
    lv_slider_set_range(turn_sensitivity_slider, -20 , 90);
    lv_slider_set_value(turn_sensitivity_slider,100-turn_sensitivity,LV_ANIM_OFF);

     lv_obj_t * b_cb = lv_checkbox_create(cont, NULL);
    lv_checkbox_set_text(b_cb, "Back");
    lv_obj_align(b_cb, NULL, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_event_cb(b_cb, back_ev);

     lv_obj_t * s_cb = lv_checkbox_create(cont,NULL);
    lv_checkbox_set_text(s_cb, "Side");
    lv_obj_align(s_cb, NULL, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_event_cb(s_cb, side_ev);

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