#include "screen.h"
#include "zf_device_mt9v03x.h"

#include <stdio.h>
#include <string.h>

// ========================= 内部类型与状态 =========================

typedef struct
{
    uint8 max_count;
    uint8 name_width;
    uint8 value_width;
    uint8 row_height;
    uint8 font_width;
} screen_data_layout_t;

static uint8 screen_initialized = 0;
static uint8 screen_data_table_first_draw = 1;
static ips200_font_size_enum screen_data_table_font = IPS200_8X16_FONT;

// ========================= 内部辅助函数 =========================

static screen_data_layout_t screen_get_data_layout(void)
{
    screen_data_layout_t layout;

    if(IPS200_8X16_FONT == screen_data_table_font)
    {
        layout.max_count   = SCREEN_DATA_MAX_COUNT_8x16;
        layout.name_width  = SCREEN_NAME_WIDTH_8x16;
        layout.value_width = SCREEN_VALUE_WIDTH_8x16;
        layout.row_height  = SCREEN_ROW_HEIGHT_8x16;
        layout.font_width  = SCREEN_FONT_WIDTH_8x16;
    }
    else
    {
        layout.max_count   = SCREEN_DATA_MAX_COUNT_6x8;
        layout.name_width  = SCREEN_NAME_WIDTH_6x8;
        layout.value_width = SCREEN_VALUE_WIDTH_6x8;
        layout.row_height  = SCREEN_ROW_HEIGHT_6x8;
        layout.font_width  = SCREEN_FONT_WIDTH_6x8;
    }

    return layout;
}

static void screen_format_text_field(char *out, const char *text, uint8 width)
{
    uint8 index;
    uint8 copy_len;
    uint16 text_len;

    if(0 == out)
    {
        return;
    }

    if(0 == text)
    {
        text = "";
    }

    text_len = (uint16)strlen(text);

    for(index = 0; index < width; index++)
    {
        out[index] = ' ';
    }
    out[width] = '\0';

    if(text_len <= width)
    {
        copy_len = (uint8)text_len;
    }
    else if(width > 3)
    {
        copy_len = (uint8)(width - 3);
    }
    else
    {
        copy_len = width;
    }

    for(index = 0; index < copy_len; index++)
    {
        out[index] = text[index];
    }

    if((text_len > width) && (width > 3))
    {
        out[width - 3] = '.';
        out[width - 2] = '.';
        out[width - 1] = '.';
    }
}

static void screen_format_value(char *out, const screen_data_item_t *item, uint8 width)
{
    char temp[48];
    uint8 decimal;

    if((0 == out) || (0 == item))
    {
        return;
    }

    temp[0] = '\0';

    switch(item->type)
    {
        case SCREEN_DATA_STRING:
        {
            if(0 != item->value.str_value)
            {
                strncpy(temp, item->value.str_value, sizeof(temp) - 1);
                temp[sizeof(temp) - 1] = '\0';
            }
        } break;

        case SCREEN_DATA_INT:
        {
            sprintf(temp, "%d", item->value.int_value);
        } break;

        case SCREEN_DATA_UINT:
        {
            sprintf(temp, "%u", item->value.uint_value);
        } break;

        case SCREEN_DATA_FLOAT:
        {
            decimal = item->float_decimal;
            if(decimal > 6)
            {
                decimal = 6;
            }
            sprintf(temp, "%.*f", decimal, item->value.float_value);
        } break;

        default:
        {
            strncpy(temp, "ERR", sizeof(temp) - 1);
            temp[sizeof(temp) - 1] = '\0';
        } break;
    }

    screen_format_text_field(out, temp, width);
}

// ========================= 屏幕基础封装函数 =========================

void screen_init(void)
{
    if(screen_initialized)
    {
        return;
    }

    ips200_set_dir(IPS200_CROSSWISE);
    ips200_set_font(IPS200_8X16_FONT);
    ips200_set_color(RGB565_WHITE, RGB565_BLACK);
    ips200_init(SCREEN_IPS200_TYPE);

    screen_initialized = 1;
}

void screen_clear(void)
{
    screen_init();
    ips200_clear();
}

void screen_set_color(uint16 pen_color, uint16 bg_color)
{
    screen_init();
    ips200_set_color(pen_color, bg_color);
}

void screen_show_string(uint16 x, uint16 y, const char *text)
{
    screen_init();

    if(0 == text)
    {
        return;
    }

    ips200_show_string(x, y, text);
}

void screen_show_camera_image(uint16 x, uint16 y, const uint8 *image, uint16 display_width, uint16 display_height)
{
    screen_init();

    if((0 == image) || (0 == display_width) || (0 == display_height))
    {
        return;
    }

    if((x >= ips200_width_max) || (y >= ips200_height_max))
    {
        return;
    }

    if(display_width > (ips200_width_max - x))
    {
        display_width = ips200_width_max - x;
    }

    if(display_height > (ips200_height_max - y))
    {
        display_height = ips200_height_max - y;
    }

    ips200_show_gray_image(x, y, image, MT9V03X_W, MT9V03X_H, display_width, display_height, 0);
}

void screen_show_image_threshold_bar(uint16 y, uint16 length, uint8 width)
{
    for (uint8 i = 0; i < width; i++)
    {
        ips200_draw_line(0, y + i, length, y + i, RGB565_GREEN);
    }
}

void show_string_demo(void)
{
    screen_init();
    ips200_set_font(IPS200_6X8_FONT);
    ips200_clear();

    ips200_show_string(0, 0,  "hello world!");
    ips200_show_string(0, 12, "abcdefghijklmnopqrstuvwxyz");
    ips200_show_string(0, 24, "0123456789");
}

// ========================= 通用数据表显示函数 =========================

void screen_data_table_reset(void)
{
    screen_data_table_first_draw = 1;
}

void screen_data_table_set_font(ips200_font_size_enum font)
{
    screen_init();

    if(IPS200_8X16_FONT == font)
    {
        screen_data_table_font = IPS200_8X16_FONT;
    }
    else
    {
        screen_data_table_font = IPS200_6X8_FONT;
    }

    ips200_set_font(screen_data_table_font);
    screen_data_table_reset();
}

void screen_show_data_table(const screen_data_item_t *items, uint8 count)
{
    screen_data_layout_t layout;
    uint8 index;
    uint8 draw_count;
    uint16 y;
    uint16 value_x;
    char name_buf[SCREEN_NAME_WIDTH_MAX + 1];
    char value_buf[SCREEN_VALUE_WIDTH_MAX + 1];

    if((0 == items) || (0 == count))
    {
        return;
    }

    screen_init();
    ips200_set_font(screen_data_table_font);

    layout = screen_get_data_layout();
    draw_count = count;
    if(draw_count > layout.max_count)
    {
        draw_count = layout.max_count;
    }

    value_x = (uint16)((layout.name_width + SCREEN_NAME_VALUE_SPACE_WIDTH) * layout.font_width);

    if(screen_data_table_first_draw)
    {
        ips200_clear();

        for(index = 0; index < draw_count; index++)
        {
            y = (uint16)(index * layout.row_height);
            screen_format_text_field(name_buf, items[index].name, layout.name_width);
            ips200_show_string(0, y, name_buf);
        }

        screen_data_table_first_draw = 0;
    }

    for(index = 0; index < draw_count; index++)
    {
        y = (uint16)(index * layout.row_height);
        screen_format_value(value_buf, &items[index], layout.value_width);
        ips200_show_string(value_x, y, value_buf);
    }
}
