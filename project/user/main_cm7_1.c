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

// 屏幕显示坐标
#define IMAGE_X                 (0)
#define IMAGE_Y                 (120)
#define IMAGE_DISPLAY_WIDTH     (188)
#define IMAGE_DISPLAY_HEIGHT    (120)

//=========================== 跳跃判断条件 ===========================
#define JUMP_ROW                (82)            // 行起始位置
#define JUMP_ROW_TOTAL          (10)            // 行向上检查行数

#define JUMP_COLUMN             (40)            // 列起始位置
#define JUMP_COLUMN_TOTAL       (80)            // 列向右检查行数

#define BLACK_PIX_COUNT         (400)           // 矩形内黑色像素阈值

#define JUMP_COOLDOWN_MS        (500)           // 跳跃触发一次后的禁止重复触发时间
//====================================================================

volatile uint32 sys_ms = 0;      // 毫秒计时器

// IPC 空接收回调：当前核心只发送 is_jump，不处理接收数据
static void ipc_callback(uint32 data)
{
    (void)data;
}

// 计算 FPS
uint32 calc_fps(uint32 *frame_count, uint32 *fps)
{
    static uint32 last_1s_time = 0;
    if (sys_ms - last_1s_time >= 1000)
    {
        last_1s_time = sys_ms;
        *fps = *frame_count;
        *frame_count = 0;
        return *fps;
    }
    
    return *fps;
}

// 跳跃触发冷却时间检查
uint8 jump_trigger_filter(uint8 jump_detected)
{
    static uint32 last_jump_time = 0;

    if(sys_ms - last_jump_time < JUMP_COOLDOWN_MS)
    {
        return 0;
    }

    if(jump_detected)
    {
        last_jump_time = sys_ms;
        return 1;
    }

    return 0;
}


int main(void)
{
    uint8 is_jump = 0;                // 跳跃触发标志位，触发后受冷却时间限制
    uint8 ipc_result = 0;             // IPC发送结果：0成功，1失败或超时
    char *str_jump_state = "";        // 跳跃状态 显示信息
    char *str_ipc_state = "";         // IPC状态 显示信息
    uint32 frame_count = 0;           // 帧计数
    uint32 fps = 0;                   // FPS

    char str_row_range[32];           // 行识别信息显示用字符串
    char str_column_range[32];        // 列识别信息显示用字符串

    clock_init(SYSTEM_CLOCK_250M);
    debug_info_init();
    camera_init();                                      // MT9V03X 摄像头初始化
    screen_init();                                      // 屏幕 初始化
    pit_ms_init(PIT_CH1, 1);                            // PIT_CH1 1ms周期中断，用于 sys_ms 计时

    ipc_communicate_init(IPC_PORT_2, ipc_callback);     // IPC 初始化
    system_delay_ms(500);                               // 等待 核0 完成 IPC 初始化
    SCB_DisableDCache();                                // 关闭 CM7 DCache 放在所有初始化的最后
    
    
    screen_data_item_t data_table[] =
    {
        {"Jump",     SCREEN_DATA_STRING,   {.str_value = ""}, 0},
        {"FPS",      SCREEN_DATA_UINT,   {.uint_value = 0}, 0},
        {"IPCstate", SCREEN_DATA_STRING, {.str_value  = ""}, 0},
        {"Row",    SCREEN_DATA_STRING,   {.str_value = ""}, 0},
        {"Column",    SCREEN_DATA_STRING,   {.str_value = ""}, 0},
        {"BlackPix", SCREEN_DATA_UINT,   {.uint_value = 0}, 0},
    };  // 屏幕数据列表

        
    while(true)
    {
        // 当检测到有帧时
        if(camera_has_frame())
        {
            frame_count++;
            fps = calc_fps(&frame_count, &fps);  // 计算帧率

            is_jump = camera_processing(
                JUMP_ROW,
                JUMP_ROW_TOTAL,
                JUMP_COLUMN,
                JUMP_COLUMN_TOTAL,
                BLACK_PIX_COUNT
            );  // 检测跳跃

            is_jump = jump_trigger_filter(is_jump); // 检测冷却判断
            ipc_result = ipc_send_data((uint32)is_jump);  // 发送 跳跃标志位值

            // 屏幕数据处理
            str_jump_state = (is_jump) ? "JUMP" : "Waiting";
            str_ipc_state = (ipc_result == 0) ? "OK" : "Failed";
            sprintf(str_row_range, "%d | %d", JUMP_ROW, JUMP_ROW_TOTAL);
            sprintf(str_column_range, "%d | %d", JUMP_COLUMN, JUMP_COLUMN_TOTAL);
            data_table[0].value.str_value = str_jump_state;
            data_table[1].value.uint_value = fps;
            data_table[2].value.str_value = str_ipc_state;
            data_table[3].value.str_value = str_row_range;
            data_table[4].value.str_value = str_column_range;
            data_table[5].value.uint_value = BLACK_PIX_COUNT;
            screen_show_data_table(data_table, 6);
            
            //在显示屏上显示摄像头图像
            camera_debug_on_screen(
                IMAGE_X,
                IMAGE_Y,
                IMAGE_DISPLAY_WIDTH,
                IMAGE_DISPLAY_HEIGHT
            );

            // 四个绘制绿色标识线屏幕函数
            screen_show_threshold_horizontal_bar(
                IMAGE_Y + JUMP_ROW - JUMP_ROW_TOTAL + 1,
                IMAGE_X + IMAGE_DISPLAY_WIDTH - 1,
                2
            );

            screen_show_threshold_horizontal_bar(
                IMAGE_Y + JUMP_ROW,
                IMAGE_X + IMAGE_DISPLAY_WIDTH - 1,
                2
            );

            screen_show_threshold_vertical_bar(
                IMAGE_X + JUMP_COLUMN,
                IMAGE_Y,
                IMAGE_DISPLAY_HEIGHT - 1,
                2
            );

            screen_show_threshold_vertical_bar(
                IMAGE_X + JUMP_COLUMN + JUMP_COLUMN_TOTAL - 1,
                IMAGE_Y,
                IMAGE_DISPLAY_HEIGHT - 1,
                2
            );
        }
    }
}
