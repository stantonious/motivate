
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

void draw_cell(lv_obj_t *canvas, int status, int x_pos, int y_pos, int type, int status_width, int status_length, int wall_width, int wall_length)
{

    lv_draw_rect_dsc_t rect_dsc;
    lv_draw_rect_dsc_init(&rect_dsc);
    rect_dsc.bg_color = LV_COLOR_RED;

    if (type & NORTH_BIT)
    {
        lv_canvas_draw_rect(
            canvas,
            x_pos,
            y_pos,
            wall_length,
            wall_width, &rect_dsc);
    }
    if (type & SOUTH_BIT)
    {
        lv_canvas_draw_rect(
            canvas,
            x_pos,
            y_pos + wall_length - wall_width,
            wall_length,
            wall_width, &rect_dsc);
    }
    if (type & WEST_BIT)
    {
        lv_canvas_draw_rect(
            canvas,
            x_pos,
            y_pos,
            wall_width,
            wall_length, &rect_dsc);
    }
    if (type & EAST_BIT)
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
