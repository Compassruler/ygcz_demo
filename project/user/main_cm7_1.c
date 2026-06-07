#include "zf_common_headfile.h"
#include "camera.h"
#include "button.h"
#include "appipc.h"

/*
核0 最简使用示例

#include "zf_common_headfile.h"
#include "appipc.h"

volatile uint8 is_jump_from_core1 = 0;  // 来自 核1 的跳跃标志位值
volatile uint8 is_jump_updated = 0;     // 数据更新标志位

// IPC 接收回调函数：当接收到另一核心发送的数据时被调用
static void appipc_callback(uint32 data)
{
    is_jump_from_core1 = (uint8)(data & 0x01);
    is_jump_updated = 1;
}

int main(void)
{
    appipc_rx_init(appipc_callback);  // 初始化 IPC

    while(true)
    {   
        if(is_jump_updated)
        {
            is_jump_updated = 0;  // 需要手动复位

            // 在这里使用 is_jump_from_core1 标志位, 1 为跳跃 0 为不跳跃，自动更新，无需手动复位
        }
    }
}

*/

//=========================== 启动选择 ===========================
#define HOLD_MS                 (1500)                          // 启动时暂停跳跃时间

//=========================== WiFi SPI 调参配置 ===========================
#define ENABLE_ASSISTANT_PARAM  (0)                             // 2 启动 WiFi SPI 但不使用调参 | 1 开启 WiFi SPI 并启动调参 | 0 彻底关闭 WiFi SPI

//=========================== 显示模式选择 ===========================
#define IMAGE_DEBUG_TYPE        (1)                             // 0 无显示 | 1 仅屏幕 | 2 仅 WiFi 图传 | 3 同时显示
                                                                // 注意 当使用 WiFi 图传时 上面的 ENABLE_ASSISTANT_PARAM 是否设置为 1 或 2

//=========================== WiFi SPI 配置 ===========================
#define WIFI_SSID               "WiFi名称"                      // WiFi SSID
#define WIFI_PWD                "12345678"                      // WiFi 密码
#define TARGET_IP               "192.168.137.1"                 // 上位机IP地址
#define TARGET_PORT             "8086"                          // 上位机端口
#define LOCAL_PORT              "6666"                          // 本机端口

//=========================== 跳跃判断条件 ===========================
#define JUMP_ALGO_TYPE          (1)                             // 跳跃检测算法选择

#define JUMP_ROW                (98)                            // 行起始位置
#define JUMP_ROW_TOTAL          (25)                            // 行向上检查行数

#define JUMP_COLUMN             (55)                            // 列起始位置
#define JUMP_COLUMN_TOTAL       (73)                            // 列向右检查行数

#define OTSU_ROI_ROW            (119)                           // ROI 区域行起始
#define OTSU_ROI_ROW_TOTAL      (80)                            // ROI 区域向上行计数
#define OTSU_ROI_COLUMN         (20)                            // ROI 区域列起始
#define OTSU_ROI_COLUMN_TOTAL   (148)                           // ROI 区域向上列计数

#define JUMP_DOT_TYPE           (CAMERA_IMAGE_DOT_BLACK)        // 检测点类型
#define JUMP_DOT_COUNT          (837)                           // 矩形内点阈值

#define JUMP_COOLDOWN_MS        (450)                           // 跳跃触发一次后的禁止重复触发时间

#define JUMP_MULTI_FRAME        (1)                             // 有效帧阈值
//====================================================================

volatile uint32 sys_ms = 0;  // 毫秒计时器

// 屏幕显示函数
void debug_image_screen_display(JumpDetectParams_t jump_params)
{
    #if IMAGE_DEBUG_TYPE == 1 || IMAGE_DEBUG_TYPE == 3
    camera_debug_on_screen();                            // 在显示屏上显示摄像头图像
    screen_show_detect_threshold_bar(jump_params);       // 识别矩形框 绘制四个绿色标识线
    screen_show_roi_threshold_bar(jump_params);          // ROI矩形框 绘制四个粉色标识线
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
    #if ENABLE_ASSISTANT_PARAM == 1
    camera_assistant_parameter_update();
    camera_assistant_parameter_read_uint16(1, &jump_params->check_row,          0, MT9V03X_H - 1);
    camera_assistant_parameter_read_uint16(2, &jump_params->check_row_count,    1, MT9V03X_H);
    camera_assistant_parameter_read_uint16(3, &jump_params->check_column,       0, MT9V03X_W - 1);
    camera_assistant_parameter_read_uint16(4, &jump_params->check_column_count, 1, MT9V03X_W);
    camera_assistant_parameter_read_uint32(5, &jump_params->dot_count,          0, MT9V03X_IMAGE_SIZE);
    camera_assistant_parameter_read_uint32(6, &jump_params->multi_frame,        1, 30);
    camera_assistant_parameter_read_uint32(7, &jump_params->cooldown_time_ms,   0, 5000);
    #endif
}

// 统一更新函数
void updates(uint32 sys_ms, JumpDetectParams_t *jump_params, uint32 fps, uint32 is_jump, uint8 ipc_result, uint32 *frame_count)
{
    button_update();                                   // 按钮状态更新
    screen_show_table_t2(*jump_params, fps, is_jump);  // 屏幕显示参数更新

    // 图像信息更新时
    if (camera_has_frame())
    {
        (*frame_count)++;                              // 更新帧计数
        debug_image_screen_display(*jump_params);      // 使用屏幕显示图像
        debug_image_wifispi_display();                 // 使用 WiFi SPI 发送图像
    }
    
    // 更新按钮状态，当检测到 按钮1 被按下后，重置检测序列
    if (button_flag[BTN_1])
    {
        jump_params->dot_type = camera_dot_type_reset();
        jump_params->steps    = camera_dot_type_get_steps();
    }

    // 如果启用 WiFi SPI 调参，则更新跳跃参数
    #if ENABLE_ASSISTANT_PARAM
    jump_param_update_from_assistant(jump_params);
    #endif
}

int main(void)
{
    JumpDetectParams_t jump_params = 
    {
    .algo_type                = JUMP_ALGO_TYPE,
    .check_row                = JUMP_ROW,
    .check_row_count          = JUMP_ROW_TOTAL,
    .check_column             = JUMP_COLUMN,
    .check_column_count       = JUMP_COLUMN_TOTAL,
    .otsu_roi_row             = OTSU_ROI_ROW,
    .otsu_roi_row_count       = OTSU_ROI_ROW_TOTAL,
    .otsu_roi_column          = OTSU_ROI_COLUMN,
    .otsu_roi_column_count    = OTSU_ROI_COLUMN_TOTAL,
    .dot_type                 = JUMP_DOT_TYPE,
    .dot_count                = JUMP_DOT_COUNT,
    .cooldown_time_ms         = JUMP_COOLDOWN_MS,
    .multi_frame              = JUMP_MULTI_FRAME,
    .steps                    = 0
    };                                        // 跳跃检测参数结构体
    uint8  is_jump      = 0;                  // 跳跃触发标志位
    uint8  ipc_result   = APPIPC_BUSY;        // IPC发送结果：APPIPC_OK 成功，APPIPC_BUSY 失败或超时
    uint32 frame_count  = 0;                  // 帧计数
    uint32 fps          = 0;                  // FPS

    clock_init(SYSTEM_CLOCK_250M);            // 系统 初始化
    screen_init();                            // 屏幕 初始化
    button_init();                            // 主板按钮 初始化
    camera_init();                            // MT9V03X 摄像头初始化
    pit_ms_init(PIT_CH1, 1);                  // PIT_CH1 1ms周期中断，用于 sys_ms 计时
    
    #if ENABLE_ASSISTANT_PARAM == 1
    camera_assistant_wifi_spi_init(WIFI_SSID, WIFI_PWD, TARGET_IP, TARGET_PORT, LOCAL_PORT);  // WiFi SPI + 调参 初始化

    #elif ENABLE_ASSISTANT_PARAM == 2
    camera_wifi_spi_init(WIFI_SSID, WIFI_PWD, TARGET_IP, TARGET_PORT, LOCAL_PORT);            // 仅限 WiFi SPI 初始化
    #endif

    while(true)
    {
        updates(sys_ms, &jump_params, calc_fps(sys_ms, &frame_count, &fps), is_jump, ipc_result, &frame_count);  // 统一更新函数

        // 当检测到有帧时 且 当计时器时间大于启动时暂停跳跃时间执行
        if (camera_has_frame() && sys_ms > HOLD_MS)
        {
            is_jump    = camera_processing_roi(sys_ms, &jump_params);  // 检测跳跃（使用ROI）
            ipc_result = appipc_send_u32((uint32)is_jump);             // 发送 跳跃标志位值
        }
        
        // 不同跳跃下的识别参数细调
        switch (jump_params.steps)
        {
            // 控制第一次跳跃时的检测参数
            case 0:
                jump_params.check_row = 100;
                jump_params.check_row_count= 25;
                break;

            // 控制第二次跳跃时的检测参数
            case 1:
                jump_params.check_row = 110;
                jump_params.check_row_count= 25;
                break;

            // 控制第三次跳跃时的检测参数
            case 2:
                jump_params.check_row = 110;
                jump_params.check_row_count= 25;
                break;
        }
    }
}
