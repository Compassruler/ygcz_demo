#ifndef _SCREEN_H_
#define _SCREEN_H_

#include "zf_common_typedef.h"
#include "zf_common_font.h"
#include "zf_device_ips200.h"

// 屏幕基础信息
// IPS200 横屏分辨率为 320x240
// 6x8 字体：单个 ASCII 字符宽 6 像素、高 8 像素，一行最多显示 53 个 ASCII 字符
// 8x16 字体：单个 ASCII 字符宽 8 像素、高 16 像素，一行最多显示 40 个 ASCII 字符

// IPS200 屏幕接口类型配置。
// 默认使用 SPI 两寸屏；如果硬件使用八位并口屏，可改为 IPS200_TYPE_PARALLEL8。
#define SCREEN_IPS200_TYPE              IPS200_TYPE_SPI

// ========================= 通用纯文本数据显示函数配置 =========================
// 6x8 字体时：10 字符名称 + 5 字符间隔 + 38 字符数据 = 53 字符，刚好适配 320 像素横屏宽度
#define SCREEN_LINE_CHAR_MAX_6x8        53          // 一行最大 ASCII 字符数
#define SCREEN_DATA_MAX_COUNT_6x8       24          // 最大数据种类显示数量
#define SCREEN_NAME_WIDTH_6x8           10          // 参数名称部分最大字符数
#define SCREEN_VALUE_WIDTH_6x8          38          // 参数数据部分最大字符数
#define SCREEN_ROW_HEIGHT_6x8           10          // 行高

// 8x16 字体时：10 字符名称 + 5 字符间隔 + 25 字符数据 = 40 字符，刚好适配 320 像素横屏宽度
#define SCREEN_LINE_CHAR_MAX_8x16       40          // 一行最大 ASCII 字符数
#define SCREEN_DATA_MAX_COUNT_8x16      12          // 最大数据种类显示数量
#define SCREEN_NAME_WIDTH_8x16          10          // 参数名称部分最大字符数
#define SCREEN_VALUE_WIDTH_8x16         25          // 参数数据部分最大字符数
#define SCREEN_ROW_HEIGHT_8x16          20          // 行高

// 名称与数据本体之间固定保留 5 个空格
#define SCREEN_NAME_VALUE_SPACE_WIDTH    5

// 字体宽度，用于计算数据区域刷新位置
#define SCREEN_FONT_WIDTH_6x8            6
#define SCREEN_FONT_WIDTH_8x16           8

// 内部缓冲区使用的最大宽度
#define SCREEN_NAME_WIDTH_MAX           10
#define SCREEN_VALUE_WIDTH_MAX          38

// 数据类型枚举
// screen_show_data_table() 会把不同类型统一转换成字符串后显示。
typedef enum
{
    SCREEN_DATA_STRING = 0,                         // 字符串数据
    SCREEN_DATA_INT,                                // 有符号整数数据
    SCREEN_DATA_UINT,                               // 无符号整数数据
    SCREEN_DATA_FLOAT                               // 浮点数据
} screen_data_type_t;

// 单条数据显示项
// name           : 数据名称，显示宽度由当前字体配置决定，过长会自动截断为 xxx...。
// type           : 数据类型。
// value          : 数据值，按照 type 选择对应成员填写。
// float_decimal  : 浮点数据显示的小数位数，仅 type 为 SCREEN_DATA_FLOAT 时使用。
//
// 示例：
// screen_data_item_t items[] =
// {
//     {"gyro_x", SCREEN_DATA_INT,   {.int_value = imu660rb_gyro_x}, 0},
//     {"acc_z",  SCREEN_DATA_INT,   {.int_value = imu660rb_acc_z},  0},
//     {"pitch",  SCREEN_DATA_FLOAT, {.float_value = pitch},         2},
// };
typedef struct
{
    const char *name;
    screen_data_type_t type;

    union
    {
        const char *str_value;
        int32       int_value;
        uint16      uint_value;
        float       float_value;
    } value;

    uint8 float_decimal;
} screen_data_item_t;




// ========================= 屏幕基础封装函数 =========================

// 屏幕初始化。
// 函数内部带有初始化标志，多次调用不会重复初始化屏幕硬件。
// 默认设置为横屏、8x16 字体、白色前景、黑色背景。
void screen_init(void);


// 清空屏幕。
// 若屏幕尚未初始化，会先自动调用 screen_init()。
void screen_clear(void);


// 设置显示颜色。
// pen_color 为画笔颜色，bg_color 为背景颜色，颜色值使用 RGB565 格式。
void screen_set_color(uint16 pen_color, uint16 bg_color);


// 在指定坐标显示字符串。
// x、y 为像素坐标；text 为待显示字符串。
// 当前字体由 ips200_set_font() 或 screen_data_table_set_font() 决定。
void screen_show_string(uint16 x, uint16 y, const char *text);


/**
 * @brief 在 IPS200 屏幕上显示处理后的 MT9V03X 图像。
 *
 * 该函数用于显示已经完成二值化、滤噪等处理后的摄像头图像。
 * 图像源尺寸固定按照 MT9V03X 当前配置，即 `MT9V03X_W * MT9V03X_H`；
 * 显示时可以通过 `display_width` 和 `display_height` 设置缩放后的显示大小。
 *
 * @param x              图像显示区域左上角 x 坐标。
 * @param y              图像显示区域左上角 y 坐标。
 * @param image          待显示图像首地址，通常传入 `image[0]`。
 * @param display_width  图像显示宽度。
 * @param display_height 图像显示高度。
 *
 * @return void
 *
 * @note 本函数使用 IPS200 灰度图显示接口，适合显示 0/255 二值图或 8bit 灰度图。
 * @note 若显示区域超过屏幕边界，函数会自动裁剪显示宽高，避免越界。
 * @note 函数内部会自动调用 screen_init()，无需重复初始化屏幕。
 */
void screen_show_camera_image(uint16 x, uint16 y, const uint8 *image, uint16 display_width, uint16 display_height);


/**
 * @brief 在 IPS200 屏幕上绘制绿色横向阈值标记条。
 *
 * 该函数从屏幕最左侧 x=0 开始，在指定 y 坐标处绘制一条绿色横线；
 * 可通过 `width` 设置线条厚度，用于在摄像头图像显示区域上叠加横向检测行、
 * 阈值行或调试参考线。
 *
 * @param y      横向标记条起始 y 坐标。
 * @param length 横向标记条终点 x 坐标，实际绘制范围为 x=0 到 x=length。
 * @param width  横向标记条厚度，单位为像素；例如 1 表示单像素横线，3 表示三像素厚。
 *
 * @return void
 *
 * @note 当前颜色固定为 RGB565_GREEN。
 * @note 调用时需确保 length 小于 ips200_width_max，且 y + width - 1 小于 ips200_height_max。
 * @note 如果该线用于叠加在图像上，应在显示图像之后调用，否则可能被图像刷新覆盖。
 */
void screen_show_threshold_horizontal_bar(uint16 y, uint16 length, uint8 width);


/**
 * @brief 在 IPS200 屏幕上绘制绿色纵向阈值标记条。
 *
 * 该函数从指定坐标 `(x, y)` 开始，向下绘制一条绿色竖线；
 * 可通过 `width` 设置线条厚度，用于在摄像头图像显示区域上叠加纵向检测列、
 * 列范围边界或调试参考线。
 *
 * @param x      纵向标记条起始 x 坐标。
 * @param y      纵向标记条起始 y 坐标。
 * @param length 纵向标记条终点相对长度，实际绘制范围为 y 到 y + length。
 * @param width  纵向标记条厚度，单位为像素；例如 1 表示单像素竖线，3 表示三像素厚。
 *
 * @return void
 *
 * @note 当前颜色固定为 RGB565_GREEN。
 * @note 调用时需确保 x + width - 1 小于 ips200_width_max，且 y + length 小于 ips200_height_max。
 * @note 如果该线用于叠加在图像上，应在显示图像之后调用，否则可能被图像刷新覆盖。
 */
void screen_show_threshold_vertical_bar(uint16 x, uint16 y, uint16 length, uint8 width);


// 字符串显示示例。
// 屏幕会显示 hello world!、a-z、0-9，便于快速验证屏幕是否正常。
void show_string_demo(void);





// ========================= 通用数据表显示函数 =========================

// 重置数据表绘制状态
// 调用后，下一次 screen_show_data_table() 会重新清屏并绘制左侧名称
// 当切换页面、切换数据种类、修改名称或修改字体时，建议调用该函数
void screen_data_table_reset(void);

// 设置通用数据表使用的字体
// 当前支持 IPS200_6X8_FONT 与 IPS200_8X16_FONT
// 6x8 字体最多显示 24 行，名称 10 字符，间隔 5 字符，数据 38 字符
// 8x16 字体最多显示 12 行，名称 10 字符，间隔 5 字符，数据 25 字符
// 设置字体后会自动重置数据表绘制状态
void screen_data_table_set_font(ips200_font_size_enum font);

/**
 * @brief 显示并刷新通用纯文本数据表。
 *
 * 本函数用于把一组“名称 + 数值”数据按行显示在 IPS200 屏幕上，适合显示
 * IMU 原始值、PID 参数、电机速度、姿态角等周期性变化的数据。
 *
 * 显示区域固定从屏幕左上角 (0, 0) 开始：
 * - 左侧显示 name，即参数名称；
 * - 右侧显示 value，即参数数据；
 * - 第一次调用会清屏并绘制所有名称；
 * - 后续调用只刷新右侧数据部分，减少闪烁，也避免重复刷新固定文字。
 *
 * 模拟显示效果：
 * Data01     1.123456
 * Data02     123456
 * Data03     Hello World.
 * 
 * @param items 数据项数组首地址，类型为 screen_data_item_t *
 *              每个数组元素代表屏幕上的一行数据，填写方法如下：
 * 
 *              - name：数据名称，例如 "acc_x"、"gyro_y"、"pitch"
 *                名称会按当前字体配置自动对齐；超出宽度时会截断并显示为 xxx...
 * 
 *              - type：数据类型，决定 value 联合体中应该使用哪个成员。
 *                SCREEN_DATA_STRING 使用 value.str_value；
 *                SCREEN_DATA_INT    使用 value.int_value；
 *                SCREEN_DATA_UINT   使用 value.uint_value；
 *                SCREEN_DATA_FLOAT  使用 value.float_value
 * 
 *              - value：实际显示的数据。调用本函数前，应先把最新数据写入对应成员
 * 
 *              - float_decimal：设置浮点数小数位数，仅 type 为 SCREEN_DATA_FLOAT 时有效
 *                               例如设置为 2 时，12.345 会显示为 12.35。
 *
 * @param count 数据项数量，即 items 数组中希望显示的元素个数。
 *              本函数会根据当前字体自动限制最大显示数量：
 * 
 *              - IPS200_6X8_FONT  ：最多显示 SCREEN_DATA_MAX_COUNT_6x8 行，即 24 行。
 *                每行名称 10 字符，中间间隔 5 字符，数据最多 38 字符。
 * 
 *              - IPS200_8X16_FONT ：最多显示 SCREEN_DATA_MAX_COUNT_8x16 行，即 12 行。
 *                每行名称 10 字符，中间间隔 5 字符，数据最多 25 字符。
 * 
 *              - 当前默认是 8x16 字体
 * 
 *              如果 count 超过当前字体允许的最大行数，超出的数据项会被忽略。
 *              如果 items 为 0 或 count 为 0，函数会直接返回，不进行显示。
 *
 * @note 若只是数据值变化，可以在循环中直接重复调用 screen_show_data_table()
 * 
 * @note 若数据名称、数据数量、显示字体或页面内容发生变化，应先调用
 *       screen_data_table_reset()，让下一次刷新重新绘制名称区域
 * 
 * @note 可通过 screen_data_table_set_font(IPS200_6X8_FONT) 或
 *       screen_data_table_set_font(IPS200_8X16_FONT) 切换数据表字体
 * 
 * 例程
 * @code
 *
 * int main(void)
 * {
 *     ...
 *     screen_init();                               // 初始化屏幕
 *
 *     screen_data_item_t imu_table[] =
 *     {
 *         {"acc_x",  SCREEN_DATA_INT,   {.int_value = 0},   0},
 *         {"acc_y",  SCREEN_DATA_INT,   {.int_value = 0},   0},
 *         {"acc_z",  SCREEN_DATA_INT,   {.int_value = 0},   0},
 *         {"gyro_x", SCREEN_DATA_INT,   {.int_value = 0},   0},
 *         {"gyro_y", SCREEN_DATA_INT,   {.int_value = 0},   0},
 *         {"gyro_z", SCREEN_DATA_INT,   {.int_value = 0},   0},
 *         {"state",  SCREEN_DATA_STRING,{.str_value = "OK"},0},
 *     }; // 设定要显示的数据表列表 imu_table
 *     
 *     // 在循环函数中执行的代码，具体可根据需求调整
 *     while(true)
 *     {
 *         imu660rb_get_acc();                     // 更新 imu660rb_acc_x/y/z 原始值
 *         imu660rb_get_gyro();                    // 更新 imu660rb_gyro_x/y/z 原始值
 *
 *         // 设定 实际值
 *         imu_table[0].value.int_value = imu660rb_acc_x;
 *         imu_table[1].value.int_value = imu660rb_acc_y;
 *         imu_table[2].value.int_value = imu660rb_acc_z;
 *         imu_table[3].value.int_value = imu660rb_gyro_x;
 *         imu_table[4].value.int_value = imu660rb_gyro_y;
 *         imu_table[5].value.int_value = imu660rb_gyro_z;
 *          
 *         screen_show_data_table(imu_table, 7);
 * 
 *         system_delay_ms(100);
 *     }
 * }
 * @endcode
 */
void screen_show_data_table(const screen_data_item_t *items, uint8 count);

#endif
