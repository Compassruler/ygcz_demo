#include "zf_common_headfile.h"
#include "camera.h"

/*
核0 使用示例，将收到的 is_jump_from_core1 标志位输出到无线串口

#include "zf_common_headfile.h"
char txt[32];

volatile uint8 is_jump_from_core1 = 0;  // 来自 核1 的跳跃标志位值
volatile uint8 is_jump_updated = 0;     // 数据更新标志位

// IPC 接收回调函数：当接收到另一核心发送的数据时被调用
static void ipc_callback(uint32 data)
{
    is_jump_from_core1 = (uint8)(data & 0x01);
    is_jump_updated = 1;
}

int main(void)
{
    ...

    ipc_communicate_init(IPC_PORT_1, ipc_callback);  // IPC 初始化
    SCB_DisableDCache();  // 关闭 CM7 DCache 放在所有初始化的最后

    while(true)
    {   
        if(is_jump_updated)
        {
            is_jump_updated = 0;

            sprintf(txt, "%d\r\n", is_jump_from_core1);  // 使用了 is_jump_from_core1 
            wireless_uart_send_string(txt);
        }
        ...
    }
}

*/

//=========================== WiFi SPI 调参配置 ===========================
#define ENABLE_ASSISTANT_PARAM  (1)                             // 1 开启 | 0 关闭

//=========================== 显示模式选择 ===========================
#define IMAGE_DEBUG_TYPE        (3)                             // 0 无显示 | 1 仅屏幕 | 2 仅 WiFi 图传 | 3 同时显示

//=========================== WiFi SPI 配置 ===========================
#define WIFI_SSID               "WiFi名称"                      // WiFi SSID
#define WIFI_PWD                "12345678"                      // WiFi 密码
#define TARGET_IP               "192.168.137.1"                 // 上位机IP地址
#define TARGET_PORT             "8086"                          // 上位机端口
#define LOCAL_PORT              "6666"                          // 本机端口

//=========================== 跳跃判断条件 ===========================
#define JUMP_ALGO_TYPE          (1)                             // 跳跃检测算法选择

#define JUMP_ROW                (82)                            // 行起始位置
#define JUMP_ROW_TOTAL          (10)                            // 行向上检查行数

#define JUMP_COLUMN             (40)                            // 列起始位置
#define JUMP_COLUMN_TOTAL       (80)                            // 列向右检查行数

#define JUMP_DOT_TYPE           (CAMERA_IMAGE_DOT_BLACK)        // 检测点类型
#define JUMP_DOT_COUNT          (400)                           // 矩形内点阈值

#define JUMP_COOLDOWN_MS        (500)                           // 跳跃触发一次后的禁止重复触发时间

#define JUMP_MULTI_FRAME        (3)                             // 有效帧阈值
//====================================================================

volatile uint32 sys_ms = 0;      // 毫秒计时器

// IPC 空接收回调：当前核心只发送 is_jump，不处理接收数据
static void ipc_callback(uint32 data)
{
    (void)data;
}

// 屏幕显示函数
void debug_image_screen_display(JumpDetectParams_t jump_params)
{
    #if IMAGE_DEBUG_TYPE == 1 || IMAGE_DEBUG_TYPE == 3
    //在显示屏上显示摄像头图像
    camera_debug_on_screen(
        IMAGE_X,
        IMAGE_Y,
        IMAGE_DISPLAY_WIDTH,
        IMAGE_DISPLAY_HEIGHT
    );

    // 四个绘制绿色标识线屏幕函数
    screen_show_threshold_horizontal_bar(
        IMAGE_Y + jump_params.check_row - jump_params.check_row_count + 1,
        IMAGE_X + IMAGE_DISPLAY_WIDTH - 1,
        2
    );

    screen_show_threshold_horizontal_bar(
        IMAGE_Y + jump_params.check_row,
        IMAGE_X + IMAGE_DISPLAY_WIDTH - 1,
        2
    );

    screen_show_threshold_vertical_bar(
        IMAGE_X + jump_params.check_column,
        IMAGE_Y,
        IMAGE_DISPLAY_HEIGHT - 1,
        2
    );

    screen_show_threshold_vertical_bar(
        IMAGE_X + jump_params.check_column + jump_params.check_column_count - 1,
        IMAGE_Y,
        IMAGE_DISPLAY_HEIGHT - 1,
        2
    );
    #endif
}

// WiFi SPI 函数
void debug_image_wifispi_display()
{
    #if IMAGE_DEBUG_TYPE == 2 || IMAGE_DEBUG_TYPE == 3
    camera_debug_on_wifi_spi(CAMERA_WIFI_IMAGE_SEND_DIV_DEFAULT);
    #endif
}

// 调参设定与更新函数
static void jump_param_update_from_assistant(JumpDetectParams_t *jump_params)
{
    camera_assistant_parameter_update();
    camera_assistant_parameter_read_uint16(1, &jump_params->check_row,          0, MT9V03X_H - 1);
    camera_assistant_parameter_read_uint16(2, &jump_params->check_row_count,    1, MT9V03X_H);
    camera_assistant_parameter_read_uint16(3, &jump_params->check_column,       0, MT9V03X_W - 1);
    camera_assistant_parameter_read_uint16(4, &jump_params->check_column_count, 1, MT9V03X_W);
    camera_assistant_parameter_read_uint32(5, &jump_params->dot_count,          0, MT9V03X_IMAGE_SIZE);
    camera_assistant_parameter_read_uint32(6, &jump_params->multi_frame,        1, 30);
    camera_assistant_parameter_read_uint32(7, &jump_params->cooldown_time_ms,   0, 5000);
}

int main(void)
{
    JumpDetectParams_t jump_params = 
    {
    .algo_type           = JUMP_ALGO_TYPE,
    .check_row           = JUMP_ROW,
    .check_row_count     = JUMP_ROW_TOTAL,
    .check_column        = JUMP_COLUMN,
    .check_column_count  = JUMP_COLUMN_TOTAL,
    .dot_type            = JUMP_DOT_TYPE,
    .dot_count           = JUMP_DOT_COUNT,
    .cooldown_time_ms    = JUMP_COOLDOWN_MS,
    .multi_frame         = JUMP_MULTI_FRAME
    };  // 跳跃检测参数结构体

    screen_data_item_t data_table[] =
    {
        {"Jump",     SCREEN_DATA_STRING,   {.str_value  = ""}, 0},
        {"FPS",      SCREEN_DATA_UINT,     {.uint_value = 0 }, 0},
        {"IPCstate", SCREEN_DATA_STRING,   {.str_value  = ""}, 0},
        {"Row",      SCREEN_DATA_STRING,   {.str_value  = ""}, 0},
        {"Column",   SCREEN_DATA_STRING,   {.str_value  = ""}, 0},
        {"DotCount", SCREEN_DATA_STRING,   {.str_value  = ""}, 0},
    };  // 屏幕数据列表

    uint8 is_jump = 0;                // 跳跃触发标志位，触发后受冷却时间限制
    uint8 ipc_result = 0;             // IPC发送结果：0成功，1失败或超时
    uint32 frame_count = 0;           // 帧计数
    uint32 fps = 0;                   // FPS
    char str_row_range[32];           // 行识别信息显示用字符串
    char str_column_range[32];        // 列识别信息显示用字符串
    char str_dot_info[32];            // 检测点信息显示用字符串

    clock_init(SYSTEM_CLOCK_250M);
    debug_info_init();

    camera_init();                                      // MT9V03X 摄像头初始化
    screen_init();                                      // 屏幕 初始化

    #if ENABLE_ASSISTANT_PARAM
    camera_assistant_wifi_spi_init(
        WIFI_SSID, 
        WIFI_PWD, 
        TARGET_IP, 
        TARGET_PORT, 
        LOCAL_PORT

    );                                                  // WiFi SPI + 调参 初始化

    #else
    camera_wifi_spi_init(
        WIFI_SSID, 
        WIFI_PWD, 
        TARGET_IP, 
        TARGET_PORT, 
        LOCAL_PORT
    );                                                  // 仅限 WiFi SPI 初始化
    #endif

    pit_ms_init(PIT_CH1, 1);                            // PIT_CH1 1ms周期中断，用于 sys_ms 计时
    ipc_communicate_init(IPC_PORT_2, ipc_callback);     // IPC 初始化
    system_delay_ms(500);                               // 等待 核0 完成 IPC 初始化
    SCB_DisableDCache();                                // 关闭 CM7 DCache 放在所有初始化的最后
    
    while(true)
    {
        // 如果启用 WiFi SPI 调参，则更新跳跃参数
        #if ENABLE_ASSISTANT_PARAM
        jump_param_update_from_assistant(&jump_params);
        #endif

        // 当检测到有帧时
        if(camera_has_frame())
        {
            frame_count++;
            is_jump = camera_processing(sys_ms, &jump_params);  // 检测跳跃
            ipc_result = ipc_send_data((uint32)is_jump);  // 发送 跳跃标志位值

            // 使用屏幕显示图像
            debug_image_screen_display(jump_params);   
            
            // 使用 WiFi SPI 发送图像
            debug_image_wifispi_display();

            // 屏幕显示参数
            sprintf(str_row_range,    "%d | %d", jump_params.check_row,    jump_params.check_row_count);
            sprintf(str_column_range, "%d | %d", jump_params.check_column, jump_params.check_column_count);
            sprintf(str_dot_info,     "%d | %s", jump_params.dot_count,    (jump_params.dot_type) ? "White" : "Black");
            data_table[0].value.str_value   = (is_jump) ? "JUMP" : "Waiting...";
            data_table[1].value.uint_value  = calc_fps(sys_ms, &frame_count, &fps);
            data_table[2].value.str_value   = (ipc_result == 0) ? "OK" : "Failed";
            data_table[3].value.str_value   = str_row_range;
            data_table[4].value.str_value   = str_column_range;
            data_table[5].value.str_value   = str_dot_info;
            screen_show_data_table(data_table, 6);

        }
    }
}

