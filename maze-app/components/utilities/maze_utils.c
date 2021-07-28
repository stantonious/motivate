
#include "maze_utils.h"

void get_next_cell(int x_from, int y_from, int *x_to, int *y_to, int dir)
{
    if (dir == NORTH_DIR)
    {
        *y_to = y_from - 1;
        *x_to = x_from;
    }
    else if (dir == SOUTH_DIR)
    {
        *y_to = y_from + 1;
        *x_to = x_from;
    }
    else if (dir == WEST_DIR)
    {
        *x_to = x_from - 1;
        *y_to = y_from;
    }
    else if (dir == EAST_DIR)
    {
        *x_to = x_from + 1;
        *y_to = y_from;
    }
}

bool can_move(int maze[MAZE_HEIGHT][MAZE_LEN], int x_maze_len, int y_maze_len, int x_from, int y_from, int *x_to, int *y_to, int dir)
{
    get_next_cell(x_from, y_from, x_to, y_to, dir);

    if (*x_to >= x_maze_len ||
        *y_to >= y_maze_len ||
        *x_to < 0 ||
        *y_to < 0)
        return false;

    if (dir == NORTH_DIR && maze[y_from][x_from] & NORTH_BIT)
        return false;
    if (dir == SOUTH_DIR && maze[y_from][x_from] & SOUTH_BIT)
        return false;
    if (dir == EAST_DIR && maze[y_from][x_from] & EAST_BIT)
        return false;
    if (dir == WEST_DIR && maze[y_from][x_from] & WEST_BIT)
        return false;

    return true;
}
void get_pos_from_cell(int x_cell, int y_cell, int *x_pos, int *y_pos)
{
    *x_pos = x_cell * (WALL_LENGTH - WALL_WIDTH);
    *y_pos = y_cell * (WALL_LENGTH - WALL_WIDTH);
}

void get_status_pos_from_cell(int x_cell, int y_cell, int *x_pos, int *y_pos)
{
    *x_pos = x_cell * (WALL_LENGTH - WALL_WIDTH) + (WALL_LENGTH / 2) - (STATUS_WIDTH / 2);
    *y_pos = y_cell * (WALL_LENGTH - WALL_WIDTH) + (WALL_LENGTH / 2) - (STATUS_LENGTH / 2);
}

float scale_gyro(float g)
{
    return (g + GYRO_MAX) / (2 * GYRO_MAX);
}

float scale_acc(float g)
{
    return (g + ACCEL_MAX) / (2 * ACCEL_MAX);
}