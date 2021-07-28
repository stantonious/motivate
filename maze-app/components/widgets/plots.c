
#include "plots.h"
#include "float_buffer.h"


void draw_3_plot(lv_obj_t *canvas, float (*scale_f)(float), void *buf1, void *buf2, void *buf3, int canvas_height, int canvas_width, int plot_win_len)
{
    lv_canvas_fill_bg(canvas, LV_COLOR_SILVER, LV_OPA_COVER);

    int plotSpace = canvas_width / plot_win_len;

    for (int i = 0; i < plot_win_len; i++)
    {
        int xy = scale_f(get(buf1, i)) * canvas_height;
        int yy = scale_f(get(buf2, i)) * canvas_height;
        int zy = scale_f(get(buf3, i)) * canvas_height;
        int x = plotSpace * i;

        if (buf1 != NULL)
        {
            lv_canvas_set_px(canvas, x, xy, LV_COLOR_RED);
            lv_canvas_set_px(canvas, x, xy + 1, LV_COLOR_RED);
            lv_canvas_set_px(canvas, x + 1, xy, LV_COLOR_RED);
            lv_canvas_set_px(canvas, x + 1, xy + 1, LV_COLOR_RED);
        }

        if (buf2 != NULL)
        {
            lv_canvas_set_px(canvas, x, yy, LV_COLOR_GREEN);
            lv_canvas_set_px(canvas, x, yy + 1, LV_COLOR_GREEN);
            lv_canvas_set_px(canvas, x + 1, yy, LV_COLOR_GREEN);
            lv_canvas_set_px(canvas, x + 1, yy + 1, LV_COLOR_GREEN);
        }

        if (buf2 != NULL)
        {
            lv_canvas_set_px(canvas, x, zy, LV_COLOR_BLUE);
            lv_canvas_set_px(canvas, x, zy + 1, LV_COLOR_BLUE);
            lv_canvas_set_px(canvas, x + 1, zy, LV_COLOR_BLUE);
            lv_canvas_set_px(canvas, x + 1, zy + 1, LV_COLOR_BLUE);
        }
    }
}