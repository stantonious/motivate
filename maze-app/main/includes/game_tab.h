#pragma once

#define GAME_TAB_NAME "GAME"

TaskHandle_t IMU_handle;

void display_game_tab(lv_obj_t* tv);

int16_t get_move_sensitivity();
int16_t get_turn_sensitivity();
bool get_side();
bool get_back();