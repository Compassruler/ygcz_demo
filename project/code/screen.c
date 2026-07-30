#include "screen.h"
#include "flag_value.h"
#include "imu.h"
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
static tft180_font_size_enum screen_data_table_font = TFT180_8X16_FONT;

// ========================= 固定显示内容列表 1 =========================
static screen_data_item_t screen_table_1[] =
{
    {"Pitch",     SCREEN_DATA_FLOAT,   {.float_value  = 0.0f}, 2},  // 1
    {"Roll",      SCREEN_DATA_FLOAT,   {.float_value  = 0.0f}, 2},  // 2
    {"Yaw",       SCREEN_DATA_FLOAT,   {.float_value  = 0.0f}, 2},  // 3
    {"TrueSpeed", SCREEN_DATA_FLOAT,   {.float_value  = 0.0f}, 2},  // 4
    {"-",         SCREEN_DATA_STRING,  {.str_value    = "/"},  0},  // 5
    {"-",         SCREEN_DATA_STRING,  {.str_value    = "/"},  0},  // 6
    {"-",         SCREEN_DATA_STRING,  {.str_value    = "/"},  0},  // 7
    {"-",         SCREEN_DATA_STRING,  {.str_value    = "/"},  0},  // 8
    {"-",         SCREEN_DATA_STRING,  {.str_value    = "/"},  0},  // 9
    {"-",         SCREEN_DATA_STRING,  {.str_value    = "/"},  0},  // 10
    {"-",         SCREEN_DATA_STRING,  {.str_value    = "/"},  0},  // 11
    {"-",         SCREEN_DATA_STRING,  {.str_value    = "/"},  0},  // 12
};

// ========================= 固定显示内容列表 2 =========================
screen_data_item_t screen_table_2[] =
{
    {"Jump",     SCREEN_DATA_STRING,   {.str_value  = ""}, 0},
    {"FPS",      SCREEN_DATA_UINT,     {.uint_value = 0 }, 0},
    {"ROI",      SCREEN_DATA_STRING,   {.str_value  = ""}, 0},
    {"Area",     SCREEN_DATA_STRING,   {.str_value  = ""}, 0},
    {"Limits",   SCREEN_DATA_STRING,   {.str_value  = ""}, 0},
    {"DotCount", SCREEN_DATA_STRING,   {.str_value  = ""}, 0},
}; 

// ========================= 固定显示内容列表 3 =========================
screen_data_item_t screen_table_3[] =
{
    {"FnOpt",      SCREEN_DATA_UINT,    {.uint_value = 0},  0},
    {"Frames",     SCREEN_DATA_UINT,    {.uint_value = 0 }, 0},
    {"DotCount",   SCREEN_DATA_UINT,    {.uint_value = 0},  0},
    {"Area",       SCREEN_DATA_STRING,  {.str_value  = ""}, 0},
    {"FPS",        SCREEN_DATA_UINT,    {.uint_value = 0},  0},
}; 

// ========================= 内部辅助函数 =========================

static screen_data_layout_t screen_get_data_layout(void)
{
    screen_data_layout_t layout;

    if(TFT180_8X16_FONT == screen_data_table_font)
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

// 将 MT9V03X 原始坐标映射到 TFT180 中的等比例图像区域
static uint16 screen_camera_map_x(uint16 source_x)
{
    if(source_x >= MT9V03X_W)
    {
        source_x = MT9V03X_W - 1u;
    }

    return (uint16)(
        IMAGE_X +
        (uint32)source_x * (IMAGE_DISPLAY_WIDTH - 1u) / (MT9V03X_W - 1u)
    );
}

static uint16 screen_camera_map_y(uint16 source_y)
{
    if(source_y >= MT9V03X_H)
    {
        source_y = MT9V03X_H - 1u;
    }

    return (uint16)(
        IMAGE_Y +
        (uint32)source_y * (IMAGE_DISPLAY_HEIGHT - 1u) / (MT9V03X_H - 1u)
    );
}

// 在缩放后的摄像头画面上绘制带宽度的线段
static void screen_draw_camera_line(
    uint16 source_x1,
    uint16 source_y1,
    uint16 source_x2,
    uint16 source_y2,
    uint8 width,
    rgb565_color_enum color)
{
    uint16 x1 = screen_camera_map_x(source_x1);
    uint16 y1 = screen_camera_map_y(source_y1);
    uint16 x2 = screen_camera_map_x(source_x2);
    uint16 y2 = screen_camera_map_y(source_y2);
    int32 delta_x = (int32)x2 - (int32)x1;
    int32 delta_y = (int32)y2 - (int32)y1;
    uint8 offset = 0;

    screen_init();

    if(0u == width)
    {
        width = 1u;
    }

    if(delta_x < 0)
    {
        delta_x = -delta_x;
    }
    if(delta_y < 0)
    {
        delta_y = -delta_y;
    }

    for(offset = 0; offset < width; offset++)
    {
        // 接近水平的线沿 Y 加粗，其余线沿 X 加粗
        if(delta_x >= delta_y)
        {
            if(((uint32)y1 + offset < tft180_height_max) &&
               ((uint32)y2 + offset < tft180_height_max))
            {
                tft180_draw_line(x1, y1 + offset, x2, y2 + offset, color);
            }
        }
        else if(((uint32)x1 + offset < tft180_width_max) &&
                ((uint32)x2 + offset < tft180_width_max))
        {
            tft180_draw_line(x1 + offset, y1, x2 + offset, y2, color);
        }
    }
}

// ========================= 屏幕基础封装函数 =========================

void screen_init(void)
{
    if(screen_initialized)
    {
        return;
    }

    tft180_set_dir(TFT180_CROSSWISE);
    tft180_set_color(RGB565_WHITE, RGB565_BLACK);
    tft180_init();
    tft180_set_font(TFT180_8X16_FONT);

    screen_initialized = 1;
}

void screen_clear(void)
{
    screen_init();
    tft180_clear();
}

void screen_set_color(uint16 pen_color, uint16 bg_color)
{
    screen_init();
    tft180_set_color(pen_color, bg_color);
}

void screen_show_string(uint16 x, uint16 y, const char *text)
{
    screen_init();

    if(0 == text)
    {
        return;
    }

    if((x < tft180_width_max) && (y < tft180_height_max))
    {
        tft180_show_string(x, y, text);
    }
}

void screen_show_camera_image(uint16 x, uint16 y, const uint8 *image, uint16 display_width, uint16 display_height)
{
    screen_init();

    if((0 == image) || (0 == display_width) || (0 == display_height))
    {
        return;
    }

    if((x >= tft180_width_max) || (y >= tft180_height_max))
    {
        return;
    }

    if(display_width > (tft180_width_max - x))
    {
        display_width = tft180_width_max - x;
    }

    if(display_height > (tft180_height_max - y))
    {
        display_height = tft180_height_max - y;
    }

    tft180_show_gray_image(x, y, image, MT9V03X_W, MT9V03X_H, display_width, display_height, 0);
}

void screen_show_threshold_horizontal_bar(uint16 y, uint16 length, uint8 width, rgb565_color_enum color)
{
    uint8 i = 0;

    screen_init();

    if((y >= tft180_height_max) || (0u == width))
    {
        return;
    }

    if(length >= tft180_width_max)
    {
        length = (uint16)(tft180_width_max - 1u);
    }

    for(i = 0; (i < width) && ((uint32)y + i < tft180_height_max); i++)
    {
        tft180_draw_line(0, y + i, length, y + i, color);
    }
}

void screen_show_threshold_vertical_bar(uint16 x, uint16 y, uint16 length, uint8 width, rgb565_color_enum color)
{
    uint8 i = 0;
    uint16 end_y = 0;

    screen_init();

    if((x >= tft180_width_max) || (y >= tft180_height_max) || (0u == width))
    {
        return;
    }

    end_y = (uint16)((uint32)y + length);
    if(end_y >= tft180_height_max)
    {
        end_y = (uint16)(tft180_height_max - 1u);
    }

    for(i = 0; (i < width) && ((uint32)x + i < tft180_width_max); i++)
    {
        tft180_draw_line(x + i, y, x + i, end_y, color);
    }
}

void show_string_demo(void)
{
    screen_init();
    tft180_set_font(TFT180_6X8_FONT);
    tft180_clear();

    tft180_show_string(0, 0,  "hello world!");
    tft180_show_string(0, 12, "abcdefghijklmnopqrstuvwxyz");
    tft180_show_string(0, 24, "0123456789");
}

// ========================= 通用数据表显示函数 =========================

void screen_data_table_reset(void)
{
    screen_data_table_first_draw = 1;
}

void screen_data_table_set_font(tft180_font_size_enum font)
{
    screen_init();

    if(TFT180_8X16_FONT == font)
    {
        screen_data_table_font = TFT180_8X16_FONT;
    }
    else
    {
        screen_data_table_font = TFT180_6X8_FONT;
    }

    tft180_set_font(screen_data_table_font);
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
    tft180_set_font(screen_data_table_font);

    layout = screen_get_data_layout();
    draw_count = count;
    if(draw_count > layout.max_count)
    {
        draw_count = layout.max_count;
    }

    value_x = (uint16)((layout.name_width + SCREEN_NAME_VALUE_SPACE_WIDTH) * layout.font_width);

    if(screen_data_table_first_draw)
    {
        tft180_clear();

        for(index = 0; index < draw_count; index++)
        {
            y = (uint16)(index * layout.row_height);
            screen_format_text_field(name_buf, items[index].name, layout.name_width);
            tft180_show_string(0, y, name_buf);
        }

        screen_data_table_first_draw = 0;
    }

    for(index = 0; index < draw_count; index++)
    {
        y = (uint16)(index * layout.row_height);
        screen_format_value(value_buf, &items[index], layout.value_width);
        tft180_show_string(value_x, y, value_buf);
    }
}

void screen_show_detect_threshold_bar(JumpDetectParams_t jump_params)
{
    if((jump_params.check_row >= MT9V03X_H) ||
       (0u == jump_params.check_row_count) ||
       (jump_params.check_row_count > jump_params.check_row + 1u) ||
       (jump_params.check_column >= MT9V03X_W) ||
       (0u == jump_params.check_column_count) ||
       (jump_params.check_column_count > MT9V03X_W - jump_params.check_column))
    {
        return;
    }

    // 四条参考线贯穿缩放后的摄像头显示区域
    screen_show_threshold_horizontal_bar(
        screen_camera_map_y((uint16)(jump_params.check_row - jump_params.check_row_count + 1u)),
        IMAGE_X + IMAGE_DISPLAY_WIDTH - 1,
        2,
        RGB565_GREEN
    );

    screen_show_threshold_horizontal_bar(
        screen_camera_map_y(jump_params.check_row),
        IMAGE_X + IMAGE_DISPLAY_WIDTH - 1,
        2,
        RGB565_GREEN
    );

    screen_show_threshold_vertical_bar(
        screen_camera_map_x(jump_params.check_column),
        IMAGE_Y,
        IMAGE_DISPLAY_HEIGHT - 1,
        2,
        RGB565_GREEN
    );

    screen_show_threshold_vertical_bar(
        screen_camera_map_x((uint16)(jump_params.check_column + jump_params.check_column_count - 1u)),
        IMAGE_Y,
        IMAGE_DISPLAY_HEIGHT - 1,
        2,
        RGB565_GREEN
    );
}

void screen_show_bridge_align_box(
    const CameraBridgeResult_t *bridge_result,
    const CameraBridgeAlignParams_t *align_params)
{
    uint16 far_left = 0;
    uint16 far_right = 0;
    uint16 near_left = 0;
    uint16 near_right = 0;
    uint16 fallback_far_left = 0;
    uint16 fallback_far_right = 0;
    uint16 fallback_near_left = 0;
    uint16 fallback_near_right = 0;
    uint16 far_y = 0;
    uint16 near_y = MT9V03X_H - 1u;

    if((0 == align_params) ||
       (align_params->target_center_x >= MT9V03X_W) ||
       (0u == align_params->far_tolerance_px) ||
       (0u == align_params->near_tolerance_px) ||
       (0u == align_params->fallback_far_tolerance_px) ||
       (0u == align_params->fallback_near_tolerance_px))
    {
        return;
    }

    // 识别有效时跟随实际边线段，无结果时覆盖整幅图像便于观察目标范围
    if((0 != bridge_result) &&
       bridge_result->valid &&
       (bridge_result->center_y1 < bridge_result->center_y2) &&
       (bridge_result->center_y2 < MT9V03X_H))
    {
        far_y = bridge_result->center_y1;
        near_y = bridge_result->center_y2;
    }

    far_left = (align_params->target_center_x > align_params->far_tolerance_px) ?
        (uint16)(align_params->target_center_x - align_params->far_tolerance_px) : 0u;
    far_right = (align_params->far_tolerance_px >=
                 (MT9V03X_W - align_params->target_center_x)) ?
        (MT9V03X_W - 1u) :
        (uint16)(align_params->target_center_x + align_params->far_tolerance_px);
    near_left = (align_params->target_center_x > align_params->near_tolerance_px) ?
        (uint16)(align_params->target_center_x - align_params->near_tolerance_px) : 0u;
    near_right = (align_params->near_tolerance_px >=
                  (MT9V03X_W - align_params->target_center_x)) ?
        (MT9V03X_W - 1u) :
        (uint16)(align_params->target_center_x + align_params->near_tolerance_px);

    fallback_far_left =
        (align_params->target_center_x > align_params->fallback_far_tolerance_px) ?
        (uint16)(align_params->target_center_x -
                 align_params->fallback_far_tolerance_px) : 0u;
    fallback_far_right =
        (align_params->fallback_far_tolerance_px >=
         (MT9V03X_W - align_params->target_center_x)) ?
        (MT9V03X_W - 1u) :
        (uint16)(align_params->target_center_x +
                 align_params->fallback_far_tolerance_px);
    fallback_near_left =
        (align_params->target_center_x > align_params->fallback_near_tolerance_px) ?
        (uint16)(align_params->target_center_x -
                 align_params->fallback_near_tolerance_px) : 0u;
    fallback_near_right =
        (align_params->fallback_near_tolerance_px >=
         (MT9V03X_W - align_params->target_center_x)) ?
        (MT9V03X_W - 1u) :
        (uint16)(align_params->target_center_x +
                 align_params->fallback_near_tolerance_px);

    // 紫色外框表示保底冲刺范围，红色内框表示正常连续帧对准范围
    screen_draw_camera_line(
        fallback_far_left, far_y, fallback_far_right, far_y, 1u, RGB565_PURPLE);
    screen_draw_camera_line(
        fallback_near_left, near_y, fallback_near_right, near_y, 1u, RGB565_PURPLE);
    screen_draw_camera_line(
        fallback_far_left, far_y, fallback_near_left, near_y, 1u, RGB565_PURPLE);
    screen_draw_camera_line(
        fallback_far_right, far_y, fallback_near_right, near_y, 1u, RGB565_PURPLE);

    screen_draw_camera_line(far_left, far_y, far_right, far_y, 2u, RGB565_RED);
    screen_draw_camera_line(near_left, near_y, near_right, near_y, 2u, RGB565_RED);
    screen_draw_camera_line(far_left, far_y, near_left, near_y, 2u, RGB565_RED);
    screen_draw_camera_line(far_right, far_y, near_right, near_y, 2u, RGB565_RED);
}

void screen_show_bridge_roi(const CameraBridgeParams_t *bridge_params)
{
    if((0 == bridge_params) ||
       (bridge_params->roi_top >= bridge_params->roi_bottom) ||
       (bridge_params->roi_bottom >= MT9V03X_H) ||
       (bridge_params->roi_left >= bridge_params->roi_right) ||
       (bridge_params->roi_right >= MT9V03X_W))
    {
        return;
    }

    // 绿色矩形表示当前双边线扫描的有效区域
    screen_draw_camera_line(
        bridge_params->roi_left, bridge_params->roi_top,
        bridge_params->roi_right, bridge_params->roi_top,
        1u, RGB565_GREEN
    );
    screen_draw_camera_line(
        bridge_params->roi_left, bridge_params->roi_bottom,
        bridge_params->roi_right, bridge_params->roi_bottom,
        1u, RGB565_GREEN
    );
    screen_draw_camera_line(
        bridge_params->roi_left, bridge_params->roi_top,
        bridge_params->roi_left, bridge_params->roi_bottom,
        1u, RGB565_GREEN
    );
    screen_draw_camera_line(
        bridge_params->roi_right, bridge_params->roi_top,
        bridge_params->roi_right, bridge_params->roi_bottom,
        1u, RGB565_GREEN
    );
}

void screen_show_bridge_fitted_line(const CameraBridgeResult_t *bridge_result)
{
    if((0 == bridge_result) || !bridge_result->valid)
    {
        return;
    }

    if((bridge_result->left_x1 >= MT9V03X_W) ||
       (bridge_result->left_x2 >= MT9V03X_W) ||
       (bridge_result->right_x1 >= MT9V03X_W) ||
       (bridge_result->right_x2 >= MT9V03X_W) ||
       (bridge_result->center_x1 >= MT9V03X_W) ||
       (bridge_result->center_x2 >= MT9V03X_W) ||
       (bridge_result->left_y1 >= MT9V03X_H) ||
       (bridge_result->left_y2 >= MT9V03X_H) ||
       (bridge_result->right_y1 >= MT9V03X_H) ||
       (bridge_result->right_y2 >= MT9V03X_H) ||
       (bridge_result->center_y1 >= MT9V03X_H) ||
       (bridge_result->center_y2 >= MT9V03X_H))
    {
        return;
    }

    // 左右边线使用 2 像素红线，控制中线使用 3 像素绿线
    screen_draw_camera_line(
        bridge_result->left_x1, bridge_result->left_y1,
        bridge_result->left_x2, bridge_result->left_y2,
        2u, RGB565_RED
    );
    screen_draw_camera_line(
        bridge_result->right_x1, bridge_result->right_y1,
        bridge_result->right_x2, bridge_result->right_y2,
        2u, RGB565_RED
    );
    screen_draw_camera_line(
        bridge_result->center_x1, bridge_result->center_y1,
        bridge_result->center_x2, bridge_result->center_y2,
        3u, RGB565_GREEN
    );
}


void screen_show_roi_threshold_bar(JumpDetectParams_t jump_params)
{
    if((jump_params.otsu_roi_row >= MT9V03X_H) ||
       (0u == jump_params.otsu_roi_row_count) ||
       (jump_params.otsu_roi_row_count > jump_params.otsu_roi_row + 1u) ||
       (jump_params.otsu_roi_column >= MT9V03X_W) ||
       (0u == jump_params.otsu_roi_column_count) ||
       (jump_params.otsu_roi_column_count > MT9V03X_W - jump_params.otsu_roi_column))
    {
        return;
    }

    // 四条粉色参考线使用缩放后的摄像头坐标
    screen_show_threshold_horizontal_bar(
        screen_camera_map_y((uint16)(jump_params.otsu_roi_row - jump_params.otsu_roi_row_count + 1u)),
        IMAGE_X + IMAGE_DISPLAY_WIDTH - 1,
        2,
        RGB565_PINK
    );

    screen_show_threshold_horizontal_bar(
        screen_camera_map_y(jump_params.otsu_roi_row),
        IMAGE_X + IMAGE_DISPLAY_WIDTH - 1,
        2,
        RGB565_PINK
    );

    screen_show_threshold_vertical_bar(
        screen_camera_map_x(jump_params.otsu_roi_column),
        IMAGE_Y,
        IMAGE_DISPLAY_HEIGHT - 1,
        2,
        RGB565_PINK
    );

    screen_show_threshold_vertical_bar(
        screen_camera_map_x((uint16)(jump_params.otsu_roi_column + jump_params.otsu_roi_column_count - 1u)),
        IMAGE_Y,
        IMAGE_DISPLAY_HEIGHT - 1,
        2,
        RGB565_PINK
    );
}

void screen_show_table_t1(void)
{
    screen_table_1[0].value.float_value = pitch_acc2angle;
    screen_table_1[1].value.float_value = roll_acc2angle;
    screen_table_1[2].value.float_value = yaw_angle;
    screen_table_1[3].value.float_value = true_speed;
    screen_table_1[4].value.str_value   = "/";
    screen_table_1[5].value.str_value   = "/";
    screen_table_1[6].value.str_value   = "/";
    screen_table_1[7].value.str_value   = "/";
    screen_table_1[8].value.str_value   = "/";
    screen_table_1[9].value.str_value   = "/";
    screen_table_1[10].value.str_value  = "/";
    screen_table_1[11].value.str_value  = "/";

    screen_show_data_table(screen_table_1, (uint8)(sizeof(screen_table_1) / sizeof(screen_table_1[0])));
}

void screen_show_table_t2(JumpDetectParams_t jump_params, uint32 fps, uint32 is_jump, uint16 carspd)
{
    char str_roi_info[32];            // ROI范围显示用字符串
    char str_area_info[32];           // 识别矩形框信息显示用字符串
    char str_limit_info[32];          // 视觉限制信息显示用字符串
    char str_dot_info[32];            // 检测点信息显示用字符串
    
    sprintf(str_roi_info,     "%d | %d | %d | %d", jump_params.otsu_roi_row, jump_params.otsu_roi_column, jump_params.otsu_roi_row_count, jump_params.otsu_roi_column_count);
    sprintf(str_area_info,    "%d | %d | %d | %d", jump_params.check_row,    jump_params.check_column,    jump_params.check_row_count,    jump_params.check_column_count);
    sprintf(str_limit_info,   "Spd %d | CD %d",          carspd,  jump_params.cooldown_time_ms);
    sprintf(str_dot_info,     "%d | (%d)%s",               jump_params.dot_count,    jump_params.steps,           (jump_params.dot_type) ? "White" : "Black");
    screen_table_2[0].value.str_value   = (is_jump) ? "JUMP" : "Waiting...";
    screen_table_2[1].value.uint_value  = (uint16)fps;
    screen_table_2[2].value.str_value   = str_roi_info;  // data_table[2].value.str_value   = (ipc_result == APPIPC_OK) ? "OK" : "Failed";  // 显示 IPC 状态
    screen_table_2[3].value.str_value   = str_area_info;
    screen_table_2[4].value.str_value   = str_limit_info;
    screen_table_2[5].value.str_value   = str_dot_info;

    screen_show_data_table(screen_table_2, (uint8)(sizeof(screen_table_2) / sizeof(screen_table_2[0])));
}

void screen_show_table_t3(BridgeExitParams_t bridge_params, uint8 func_opt, uint32 fps)
{
    char str_area_info[32];  // 识别矩形框信息显示用字符串
    
    sprintf(str_area_info,    "%d | %d | %d | %d", bridge_params.check_row, bridge_params.check_column, bridge_params.check_row_count, bridge_params.check_column_count);
    screen_table_3[0].value.uint_value   = func_opt;
    screen_table_3[1].value.uint_value   = bridge_params.white_frame_count;
    screen_table_3[2].value.uint_value   = (uint16)bridge_params.white_dot_count;
    screen_table_3[3].value.str_value    = str_area_info;
    screen_table_3[4].value.uint_value   = (uint16)fps;

    screen_show_data_table(screen_table_3, (uint8)(sizeof(screen_table_3) / sizeof(screen_table_3[0])));
}
