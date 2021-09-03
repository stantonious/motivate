
#include "maze_utils.h"
#include "esp_log.h"

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

bool can_move(int maze[MAZE_HEIGHT][MAZE_LEN], int x_maze_len, int y_maze_len, int x_from, int y_from, int *x_to, int *y_to, int dir, bool is_jump, bool is_duck)
{

    get_next_cell(x_from, y_from, x_to, y_to, dir);
    int wall = maze[y_from][x_from];
    int status_wall = ((wall & WALL_02_BIT) | (wall & WALL_01_BIT)) >> 4;
    int status_wall_status = ((wall & WALL_STAT_02_BIT) | (wall & WALL_STAT_01_BIT)) >> 6;

    if (*x_to >= x_maze_len ||
        *y_to >= y_maze_len ||
        *x_to < 0 ||
        *y_to < 0)
        return false;

    if (dir == NORTH_DIR && wall & NORTH_BIT)
    {
        if (status_wall != WALL_N)
            return false;
        else if (status_wall_status == WALL_STAT_LOW && is_duck)
            return true;
        else if (status_wall_status == WALL_STAT_HIGH && is_jump)
            return true;
        return false;
    }

    if (dir == SOUTH_DIR && wall & SOUTH_BIT)
    {
        if (status_wall != WALL_S)
            return false;
        else if (status_wall_status == WALL_STAT_LOW && is_duck)
            return true;
        else if (status_wall_status == WALL_STAT_HIGH && is_jump)
            return true;
        return false;
    }
    if (dir == EAST_DIR && wall & EAST_BIT)
    {
        if (status_wall != WALL_E)
            return false;
        else if (status_wall_status == WALL_STAT_LOW && is_duck)
            return true;
        else if (status_wall_status == WALL_STAT_HIGH && is_jump)
            return true;
        return false;
    }
    if (dir == WEST_DIR && wall & WEST_BIT)
    {
        if (status_wall != WALL_W)
            return false;
        else if (status_wall_status == WALL_STAT_LOW && is_duck)
            return true;
        else if (status_wall_status == WALL_STAT_HIGH && is_jump)
            return true;
        return false;
    }

    return true;
}
void get_pos_from_cell(int x_cell, int y_cell, int dir, int wall_length, int wall_width, int *x_pos, int *y_pos)
{
    int t_x, t_y;
    translate(x_cell, y_cell, &t_x, &t_y, dir, MAZE_LEN, MAZE_HEIGHT);

    *x_pos = t_x * (wall_length - wall_width);
    *y_pos = t_y * (wall_length - wall_width);
}

void get_status_pos_from_cell(int x_cell, int y_cell, int dir, int *x_pos, int *y_pos, int x_map_center, int y_map_center)
{
    translate_pos(x_map_center, y_map_center, &x_map_center, &y_map_center, dir, MAZE_LEN, MAZE_HEIGHT);
    // Translate map to screen
    int t_x = 0;
    int t_y = 0;
    translate_pos(x_cell, y_cell, &t_x, &t_y, dir, MAZE_LEN, MAZE_HEIGHT);
    t_x = t_x + (.5 * MAZE_SCALE) - x_map_center;
    t_y = t_y + (.5 * MAZE_SCALE) - y_map_center;

    ESP_LOGI("UTIL", "status trans x:%d y:%d x:%d y:%d", x_cell, y_cell, t_x, t_y);
    *x_pos = t_x * (WALL_LENGTH - WALL_WIDTH) + (WALL_LENGTH / 2) - (STATUS_WIDTH / 2);
    *y_pos = t_y * (WALL_LENGTH - WALL_WIDTH) + (WALL_LENGTH / 2) - (STATUS_LENGTH / 2);
}

void get_static_status_pos_from_cell(int x_cell, int y_cell, int wall_length, int wall_width, int status_length, int status_width, int *x_pos, int *y_pos)
{
    *x_pos = x_cell * (wall_length - wall_width) + (wall_length / 2) - (status_width / 2);
    *y_pos = y_cell * (wall_length - wall_width) + (wall_length / 2) - (status_length / 2);
}

float scale_gyro(float g)
{
    return (g + GYRO_MAX) / (2 * GYRO_MAX);
}

float scale_acc(float g)
{
    return (g + ACCEL_MAX) / (2 * ACCEL_MAX);
}

void translate_pos(int from_x, int from_y, int *to_x, int *to_y, int dir, int max_x, int max_y)
{
    if (dir == NORTH_DIR)
    {
        *to_x = from_x;
        *to_y = from_y;
    }
    else if (dir == SOUTH_DIR)
    {
        *to_x = max_x - from_x - 1;
        *to_y = max_y - from_y - 1;
    }
    else if (dir == EAST_DIR)
    {
        *to_x = max_y - from_y - 1;
        *to_y = from_x;
    }
    else if (dir == WEST_DIR)
    {
        *to_x = from_y;
        *to_y = max_x - from_x - 1;
    }
}

void translate(int from_x, int from_y, int *to_x, int *to_y, int dir, int max_x, int max_y)
{
    if (dir == NORTH_DIR)
    {
        *to_x = from_x;
        *to_y = from_y;
    }
    else if (dir == SOUTH_DIR)
    {
        *to_x = max_x - from_x - 1;
        *to_y = max_y - from_y - 1;
    }
    else if (dir == EAST_DIR)
    {
        *to_x = from_y;
        *to_y = max_x - from_x - 1;
    }
    else if (dir == WEST_DIR)
    {
        *to_x = max_y - from_y - 1;
        *to_y = from_x;
    }
}
int translate_wall(int wall, int dir)
{
    if (dir == SOUTH_DIR)
        return (wall + 2) % 4;
    if (dir == WEST_DIR)
        return (wall + 3) % 4;
    if (dir == EAST_DIR)
        return (wall + 1) % 4;
    return wall;
}
int translate_walls(int wall, int dir)
{
    // WESN  <= bit layout North

    if (dir == SOUTH_DIR)
    {
        // WESN => EWNS
        return ((wall & 0x04) << 1) |
               ((wall & 0x08) >> 1) |
               ((wall & 0x01) << 1) |
               ((wall & 0x02) >> 1);
    }
    else if (dir == WEST_DIR)
    {
        // WESN => NSWE
        return ((wall & 0x01) << 3) |
               ((wall & 0x02) << 1) |
               ((wall & 0x08) >> 2) |
               ((wall & 0x04) >> 2);
    }
    else if (dir == EAST_DIR)
    {
        return ((wall & 0x02) << 2) |
               ((wall & 0x01) << 2) |
               ((wall & 0x04) >> 1) |
               ((wall & 0x08) >> 3);
    }

    return wall;
}
