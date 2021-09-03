#pragma once

#include <stdbool.h>
#include "maze_utils.h"


extern int game_id;

extern int move_sensitivity;
extern int turn_sensitivity;
extern bool infer;
extern bool use_back;
extern bool use_side;
extern int MAZE[MAZE_HEIGHT][MAZE_LEN];
extern int x_entry;
extern int y_entry;
extern int x_exit;
extern int y_exit;
