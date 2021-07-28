#pragma once

#include <stdbool.h>

#define MAZE_LEN 14
#define MAZE_HEIGHT 14

#define NORTH_BIT 0x0001
#define SOUTH_BIT 0x0002
#define EAST_BIT 0x0004
#define WEST_BIT 0x0008
#define INOUT_BIT 0x0010

#define NORTH_DIR 0
#define EAST_DIR 1
#define SOUTH_DIR 2
#define WEST_DIR 3


#define WALL_WIDTH 3
#define WALL_LENGTH 18

#define STATUS_WIDTH 8
#define STATUS_LENGTH 8

#define ACCEL_MAX 1.
#define GYRO_MAX 300.

void get_next_cell(int x_from, int y_from, int *x_to, int *y_to, int dir);
bool can_move(int maze[MAZE_HEIGHT][MAZE_LEN], int x_maze_len, int y_maze_len, int x_from, int y_from, int *x_to, int *y_to, int dir);
void get_pos_from_cell(int x_cell, int y_cell, int *x_pos, int *y_pos);
void get_status_pos_from_cell(int x_cell, int y_cell, int *x_pos, int *y_pos);

float scale_gyro(float g);
float scale_acc(float g);