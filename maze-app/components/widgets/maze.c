
#include "esp_log.h"
#include "maze.h"
#include "maze_utils.h"

void draw_status(lv_obj_t *canvas, int status, int x_pos, int y_pos, int status_width, int status_length)
{
    lv_draw_rect_dsc_t rect_dsc;
    lv_draw_rect_dsc_init(&rect_dsc);
    if (status == 0)
    {
        rect_dsc.bg_color = LV_COLOR_SILVER;
    }
    else if (status == 1)
    {
        rect_dsc.bg_color = LV_COLOR_YELLOW;
    }
    else if (status == 2)
    {
        rect_dsc.bg_color = LV_COLOR_MAROON;
    }
    else if (status == 3)
    {
        rect_dsc.bg_color = LV_COLOR_GREEN;
    }
    else if (status == 4)
    {
        rect_dsc.bg_color = LV_COLOR_BLUE;
    }
    else if (status == 5)
    {
        rect_dsc.bg_color = LV_COLOR_PURPLE;
    }
    else if (status == 6)
    {
        rect_dsc.bg_color = LV_COLOR_ORANGE;
    }
    else
    {
        return;
    }
    lv_canvas_draw_rect(
        canvas,
        x_pos,
        y_pos,
        status_width,
        status_length,
        &rect_dsc);
}

void draw_cell(lv_obj_t *canvas, int status, int x_pos, int y_pos, int type, int dir, int status_width, int status_length, int wall_width, int wall_length)
{

    // Translate type

    lv_draw_rect_dsc_t rect_dsc;
    lv_draw_rect_dsc_init(&rect_dsc);
    rect_dsc.bg_color = LV_COLOR_RED;

    int t_type = translate_walls(type, dir);

    if (t_type & NORTH_BIT)
    {
        lv_canvas_draw_rect(
            canvas,
            x_pos,
            y_pos,
            wall_length,
            wall_width, &rect_dsc);
    }
    if (t_type & SOUTH_BIT)
    {
        lv_canvas_draw_rect(
            canvas,
            x_pos,
            y_pos + wall_length - wall_width,
            wall_length,
            wall_width, &rect_dsc);
    }
    if (t_type & WEST_BIT)
    {
        lv_canvas_draw_rect(
            canvas,
            x_pos,
            y_pos,
            wall_width,
            wall_length, &rect_dsc);
    }
    if (t_type & EAST_BIT)
    {
        lv_canvas_draw_rect(
            canvas,
            x_pos + wall_length - wall_width,
            y_pos,
            wall_width,
            wall_length, &rect_dsc);
    }
    draw_status(
        canvas,
        status,
        x_pos + (wall_length / 2) - (status_width / 2),
        y_pos + (wall_length / 2) - (status_length / 2),
        status_width,
        status_length);
}

void draw_maze(lv_obj_t *canvas, int maze[MAZE_HEIGHT][MAZE_LEN], int x_maze_len, int y_maze_len, int dir, int x_map_center, int y_map_center)
{
    int x_pos, y_pos;
    int x_cell, y_cell;

    lv_canvas_fill_bg(canvas, LV_COLOR_SILVER, LV_OPA_COVER);

    translate_pos(x_map_center, y_map_center, &x_map_center, &y_map_center, dir, x_maze_len, y_maze_len);
    ESP_LOGI("UTILS", "maze dir :%d", dir);
    for (int y = 0; y < y_maze_len; y++)
    {
        for (int x = 0; x < x_maze_len; x++)
        {
            int t_x = x - (.5 * x_maze_len) + x_map_center;
            int t_y = y - (.5 * y_maze_len) + y_map_center;
            if (t_y < 0 || t_x < 0 || t_x >= x_maze_len || t_y >= y_maze_len)
                continue;
            int s = 0;
            //to screen coords
            translate(t_x, t_y, &x_cell, &y_cell, dir, x_maze_len, y_maze_len);
            //Don't use translated coords for screen
            get_pos_from_cell(x, y, NORTH_DIR, &x_pos, &y_pos);
            draw_cell(canvas, s, x_pos, y_pos, maze[y_cell][x_cell], dir, STATUS_WIDTH, STATUS_LENGTH, WALL_WIDTH, WALL_LENGTH);
        }
    }
}