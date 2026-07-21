#include "zf_common_headfile.h"

/*
跳跃时：
    1. 调整视觉重心
    2. 核0 的中断
*/

//=========================== 启动选择 ===========================
#define HOLD_MS                 (1500)                          // 启动时暂停跳跃时间

//=========================== 显示模式选择 ===========================
#define IMAGE_DEBUG_TYPE        (1)                             // 0 无显示 | <1 仅屏幕>当前不可用

//=========================== 跳跃判断条件 ===========================
#define JUMP_ROW_TOTAL          (25)                            // 行向上检查行数
#define JUMP_COLUMN             (55)                            // 列起始位置
#define JUMP_COLUMN_TOTAL       (73)                            // 列向右检查行数
#define JUMP_DOT_COUNT          (1600)                          // 矩形内点阈值
#define JUMP_COOLDOWN_MS        (0)                             // 跳跃触发一次后的禁止重复触发时间
#define JUMP_MULTI_FRAME        (1)                             // 有效帧阈值
#define ADAPTIVE_ROW_COEFF      (-5)                            // 自适应 Row 系数，数字为正，更晚切换到低 Row，更偏高 Row，跳跃时间更早；数字为负，更早切换到低 Row，更偏低 Row，跳跃时间越晚

//=========================== 单边桥识别参数 ===========================
#define BRIDGE_BINARY_THRESHOLD     (100)                       // 单边桥识别二值化阈值

#define BRIDGE_SEARCH_LEFT          (0)                         // 搜索区间：左起点
#define BRIDGE_SEARCH_RIGHT         (MT9V03X_W - 1)             // 搜索区间：右终点
#define BRIDGE_SEARCH_TOP           (0)                         // 搜索区间：上起点
#define BRIDGE_SEARCH_BOTTOM        (MT9V03X_H - 1)             // 搜索区间：下终点

#define BRIDGE_MIN_WIDTH            (5)                         // 连续黑色线最短长度
#define BRIDGE_MIN_HEIGHT           (5)                         // 连续黑色线上下最低宽度
#define BRIDGE_MIN_EDGE_LENGTH      (10)                        // 拟合右边线最小实际长度，单位像素
#define BRIDGE_MAX_EDGE_LENGTH      (100)                       // 拟合右边线最大实际长度，单位像素
#define BRIDGE_MIN_EDGE_X           (30)                        // 拟合右边线端点允许的最小横坐标
#define BRIDGE_MAX_EDGE_X           (140)                       // 拟合右边线端点允许的最大横坐标
#define BRIDGE_MIN_AREA             (400)                        // 黑色块最小面积
#define BRIDGE_CONNECT_GAP          (5)                         // 行间黑色矩形错开长度

#define BRIDGE_TARGET_EDGE_X        (MT9V03X_W / 2)             // 目标中线

// 小车正确对准时，视觉输出的 angle，单位 0.1 度
// 例如对准时 angle 始终为 2.3 度，则设置为 23
#define BRIDGE_ALIGNED_ANGLE_D10         (5)

// 小车正确对准时，视觉输出的 distance，单位像素
// 例如对准时 distance 始终为 -4，则设置为 -4
#define BRIDGE_ALIGNED_DISTANCE_PX       (-6)

// 角度误差增益：输入和输出单位都是 0.1 度
#define BRIDGE_ANGLE_GAIN                (1.0f)

// 距离误差增益：每偏差 1 像素产生多少个 0.1 度的修正
#define BRIDGE_DISTANCE_GAIN             (30.0f)

// 方向修正系数。如果小车修正方向相反，将对应值从 1 改成 -1
#define BRIDGE_ANGLE_DIRECTION           (-1.0f)
#define BRIDGE_DISTANCE_DIRECTION        (1.0f)

// 校准后的误差死区
#define BRIDGE_ANGLE_DEADBAND_D10        (10)    // 1.0 度
#define BRIDGE_DISTANCE_DEADBAND_PX      (3)     // 3 像素

// 最大输出航向角修正量，单位 0.1 度
#define BRIDGE_YAW_OFFSET_LIMIT_D10      (60)    // 最大正负 6.0 度

#define BRIDGE_CONTROL_GAIN_PER_DEG    (5.0f)  // 每1度视觉航向偏差转换成多少angle控制量
#define BRIDGE_CONTROL_DIRECTION       (1.0f)  // 底盘控制方向，控制方向相反时修改正负号
#define BRIDGE_CONTROL_LIMIT           (60)     // angle控制量的最大绝对值

//=========================== 单边桥离开检测参数 ===========================
#define BRIDGE_EXIT_BINARY_THRESHOLD   (100)    // 离桥检测固定二值化阈值
#define BRIDGE_EXIT_CHECK_ROW          (100)    // 离桥检测矩形起始行
#define BRIDGE_EXIT_CHECK_ROW_COUNT    (25)     // 从起始行向上检查的行数
#define BRIDGE_EXIT_CHECK_COLUMN       (55)     // 离桥检测矩形起始列
#define BRIDGE_EXIT_CHECK_COLUMN_COUNT (73)     // 从起始列向右检查的列数
#define BRIDGE_EXIT_WHITE_DOT_COUNT    (1400)   // 判断离桥所需的白色像素数量
#define BRIDGE_EXIT_CONFIRM_FRAMES     (5)      // 连续满足要求的帧数
#define BRIDGE_EXIT_CHECK_DELAY_MS     (3000)    // 冲桥后延迟开始离桥检测的时间
//================================================================

volatile uint8 function_option                  = APPIPC_VISION_MODE_IDLE;      // 核心0同步的视觉工作模式
volatile uint8 bridge_phase                     = APPIPC_BRIDGE_PHASE_ALIGN;    // 核心0同步的单边桥工作子状态
volatile uint32 sys_ms                          = 0;                            // 毫秒计时器
static volatile uint16 core0_car_speed          = 0;                            // 实际车速
static volatile uint8  core0_speed_updated      = 0;                            // 车速更新标志位

// IPC 接收核0的车速和视觉工作模式
static void appipc_speed_callback(uint32 data)
{
    appipc_core0_data_t core0_data;

    if(appipc_decode_core0_data(data, &core0_data))
    {
        core0_car_speed = core0_data.car_speed;
        function_option = core0_data.vision_detect_mode;
        bridge_phase = core0_data.bridge_phase;
        core0_speed_updated = 1;
    }
}

// 屏幕显示函数
void debug_image_screen_display(
    JumpDetectParams_t jump_params, 
    const CameraBridgeResult_t *bridge_result, 
    BridgeExitParams_t bridge_exit_params,
    uint32 fps, 
    uint16 carspd
)
{
    if (function_option == APPIPC_VISION_MODE_JUMP)  // 跳跃时
    {
        #if IMAGE_DEBUG_TYPE == 1
        camera_debug_on_screen();                                   // 在显示屏上显示摄像头图像
        screen_show_detect_threshold_bar(jump_params);              // 识别矩形框 绘制四个绿色标识线
        screen_show_table_t2(jump_params, fps, 0, carspd);          // 显示信息 - 跳跃
        #endif
    }
    else if (function_option == APPIPC_VISION_MODE_BRIDGE)  // 单边桥
    {
        #if IMAGE_DEBUG_TYPE == 1 
        camera_debug_on_screen();
        if(bridge_phase == APPIPC_BRIDGE_PHASE_ALIGN)
        {
            screen_show_bridge_fitted_line(bridge_result);          // 对齐阶段绘制绿色拟合曲线
        }
        screen_show_table_t3(bridge_exit_params, function_option, fps); // 显示信息 - 单边桥
        #endif
    }
}

int main(void)
{
    fps_counter_t camera_fps;                         // FPS 结构体初始化
    JumpDetectParams_t jump_params = 
    {
        .check_row                = 100,
        .check_row_count          = JUMP_ROW_TOTAL,
        .check_column             = JUMP_COLUMN,
        .check_column_count       = JUMP_COLUMN_TOTAL,
        .otsu_roi_row             = 0,
        .otsu_roi_row_count       = 0,
        .otsu_roi_column          = 0,
        .otsu_roi_column_count    = 0,
        .dot_type                 = 1,
        .dot_count                = JUMP_DOT_COUNT,
        .cooldown_time_ms         = JUMP_COOLDOWN_MS,
        .multi_frame              = JUMP_MULTI_FRAME,
        .steps                    = 0
    };                                                // 跳跃检测参数结构体
    BridgeExitParams_t bridge_exit_params =
    {
        .binary_threshold         = BRIDGE_EXIT_BINARY_THRESHOLD,
        .check_row                = BRIDGE_EXIT_CHECK_ROW,
        .check_row_count          = BRIDGE_EXIT_CHECK_ROW_COUNT,
        .check_column             = BRIDGE_EXIT_CHECK_COLUMN,
        .check_column_count       = BRIDGE_EXIT_CHECK_COLUMN_COUNT,
        .white_dot_count          = BRIDGE_EXIT_WHITE_DOT_COUNT,
        .confirm_frame_count      = BRIDGE_EXIT_CONFIRM_FRAMES,
        .continuous_frame_count   = 0,
        .exited                   = 0
    };
    CameraBridgeParams_t bridge_params =
    {
        .binary_threshold         = BRIDGE_BINARY_THRESHOLD,
        .search_left              = BRIDGE_SEARCH_LEFT,
        .search_right             = BRIDGE_SEARCH_RIGHT,
        .search_top               = BRIDGE_SEARCH_TOP,
        .search_bottom            = BRIDGE_SEARCH_BOTTOM,
        .min_width                = BRIDGE_MIN_WIDTH,
        .min_height               = BRIDGE_MIN_HEIGHT,
        .min_edge_length          = BRIDGE_MIN_EDGE_LENGTH,
        .max_edge_length          = BRIDGE_MAX_EDGE_LENGTH,
        .min_edge_x               = BRIDGE_MIN_EDGE_X,
        .max_edge_x               = BRIDGE_MAX_EDGE_X,
        .min_area                 = BRIDGE_MIN_AREA,
        .connect_gap              = BRIDGE_CONNECT_GAP,
        .target_edge_x            = BRIDGE_TARGET_EDGE_X
    };                                                // 单边桥识别参数结构体
    CameraBridgeControlParams_t bridge_control_params =
    {
        .aligned_angle_d10        = BRIDGE_ALIGNED_ANGLE_D10,
        .aligned_distance_px      = BRIDGE_ALIGNED_DISTANCE_PX,
        .angle_gain               = BRIDGE_ANGLE_GAIN,
        .distance_gain            = BRIDGE_DISTANCE_GAIN,
        .angle_direction          = BRIDGE_ANGLE_DIRECTION,
        .distance_direction       = BRIDGE_DISTANCE_DIRECTION,
        .angle_deadband_d10       = BRIDGE_ANGLE_DEADBAND_D10,
        .distance_deadband_px     = BRIDGE_DISTANCE_DEADBAND_PX,
        .yaw_offset_limit_d10     = BRIDGE_YAW_OFFSET_LIMIT_D10,
        .control_gain_per_deg     = BRIDGE_CONTROL_GAIN_PER_DEG,
        .control_direction        = BRIDGE_CONTROL_DIRECTION,
        .control_limit            = BRIDGE_CONTROL_LIMIT
    };                                                // 单边桥控制换算参数结构体
    CameraBridgeResult_t bridge_result = {0};         // 单边桥识别结果结构体
    CameraBridgeControlResult_t bridge_control_result = {0}; // 单边桥控制换算结果结构体
    uint8  is_jump              = 0;                  // 跳跃触发标志位
    uint8  ipc_result           = APPIPC_BUSY;        // IPC发送结果：APPIPC_OK 成功，APPIPC_BUSY 失败或超时
    uint32 independent_fps      = 0;                  // 独立 FPS
    uint8  jump_uart_latched    = 0;                  // 串口发送的跳跃标志位
    uint32 uart_last_ms         = 0;                  // 串口更新计时
    uint8  actual_jump_count    = 0;                  // 实际跳跃次数计数
    uint8  bridge_aligned       = 0;                  // 单边桥是否已经对齐
    uint8  last_bridge_phase    = APPIPC_BRIDGE_PHASE_ALIGN; // 上一次执行的单边桥子状态
    uint32 bridge_exit_start_ms = 0;                  // 进入离桥检测阶段的时间
    uint8  bridge_exit_ipc_sent = 0;
    char txt[128];                                    // 串口发送文本

    clock_init(SYSTEM_CLOCK_250M);                    // 系统 初始化
    // screen_init();                                    // 屏幕 初始化 核0 已
    camera_init();                                    // MT9V03X 摄像头初始化
    pit_ms_init(PIT_CH1, 1);                          // PIT_CH1 1ms周期中断，用于 sys_ms 计时
    appipc_speed_rx_init(appipc_speed_callback);      // IPC接收 初始化
    camera_fps_counter_init(&camera_fps, sys_ms);     // 独立 FPS 计算初始化


    while(true)
    {   
        //=========================== 执行单边桥检测程序 ===========================
        if (function_option == APPIPC_VISION_MODE_BRIDGE)
        {
            if(last_bridge_phase != bridge_phase)
            {
                last_bridge_phase = bridge_phase;

                if(bridge_phase == APPIPC_BRIDGE_PHASE_EXIT_CHECK)
                {
                    bridge_exit_start_ms = sys_ms;
                }
            }

            if((bridge_phase == APPIPC_BRIDGE_PHASE_ALIGN) &&
               camera_bridge_processing(&bridge_params, &bridge_result))
            {
                independent_fps      = camera_fps_counter_update(&camera_fps, sys_ms);
                camera_bridge_calculate_control(&bridge_result, &bridge_control_params, &bridge_control_result);
                bridge_aligned = (uint8)(
                    bridge_result.valid &&
                    (-30 <= bridge_result.distance_px) &&
                    (bridge_result.distance_px <= 8) &&
                    (-100 <= bridge_result.angle_d10) &&
                    (bridge_result.angle_d10 <= 50)
                );

                appipc_send_bridge_data(bridge_result.valid, bridge_aligned, 0,
                                        (uint8)bridge_result.bottom, bridge_control_result.control_value);
                sprintf(txt, "Valid %d |Aligned %d |Bottom %d |Distance %d |Angle %d |CtrlAng %d |FPS %d\r\n",
                        bridge_result.valid, bridge_aligned, bridge_result.bottom,
                        bridge_result.distance_px, bridge_result.angle_d10,
                        bridge_control_result.control_value, independent_fps);
                wireless_uart_send_string(txt);
            }
            else if((bridge_phase == APPIPC_BRIDGE_PHASE_EXIT_CHECK) &&
                    ((sys_ms - bridge_exit_start_ms) >= BRIDGE_EXIT_CHECK_DELAY_MS))
            {
                if(!bridge_exit_params.exited && camera_has_frame())
                {
                    independent_fps = camera_fps_counter_update(&camera_fps, sys_ms);
                    camera_bridge_exit_processing(&bridge_exit_params);
                }

                // 离桥状态会保持为 1，IPC 忙时可在后续循环继续发送
                if(bridge_exit_params.exited && !bridge_exit_ipc_sent)
                {
                    if(APPIPC_OK == appipc_send_bridge_data(0, 0, 1, 0, 0))
                    {
                        bridge_exit_ipc_sent = 1;
                    }
                }
            }
        }

        //=========================== 执行跳跃检测程序 ===========================
        if (function_option == APPIPC_VISION_MODE_JUMP)
        {
            // 来自核0 的速度更新后 设置自适应识别距离
            if(core0_speed_updated)
            {
                core0_speed_updated = 0;
                jump_params.check_row = camera_jump_check_row_from_speed(core0_car_speed, ADAPTIVE_ROW_COEFF);
            }

            // 当检测到有帧时 且 当计时器时间大于启动时暂停跳跃时间执行
            if (camera_has_frame() && sys_ms > HOLD_MS)
            {
                independent_fps = camera_fps_counter_update(&camera_fps, sys_ms);            // 独立 FPS 计算
                is_jump         = camera_jump_processing(sys_ms, &jump_params);              // 检测跳跃

                // 发送更新到核0
                if(is_jump)
                {
                    actual_jump_count++;                    // 实际跳跃次数++
                    ipc_result   = appipc_send_u32(1);      // IPC 发送
                    uart_last_ms = sys_ms;                  // 串口间隔发送时间
                    sprintf(txt, "Spd %04d |J %d |Row %03u |Fps %d | %d ms\r\n",
                            core0_car_speed, is_jump, jump_params.check_row, independent_fps, sys_ms);
                    wireless_uart_send_string(txt);
                }
            }
        }
        // debug_image_screen_display(jump_params, &bridge_result, bridge_exit_params, independent_fps, core0_car_speed);
    }
}

/* 
// 核0代码 - 视觉测试用
#include "zf_common_headfile.h"

#define LED1                    (P19_0)
#define BUZZER_PIN              (P19_4)

char txt[128];

volatile uint8 is_jump_from_core1 = 0;              // 核1发送是否跳跃标志位
volatile uint8 is_jump_updated = 0;                 // 跳跃更新标志位

volatile uint8 bridge_valid_from_core1 = 0;         // 单边桥是否识别有效
volatile uint8 bridge_aligned_from_core1 = 0;       // 单边桥是否已经对齐
volatile int16 bridge_control_from_core1 = 0;       // 单边桥航向控制量
volatile uint8 bridge_control_updated = 0;          // 单边桥控制更新标志位

uint32 jump_count = 0;                              // 跳跃计数

// 接收核心1发送的数据
static void appipc_callback(uint32 data)
{
    if(vision_detect_mode == APPIPC_VISION_MODE_JUMP)
    {
        is_jump_from_core1 = (uint8)(data & 0x01);
        is_jump_updated = 1;
    }
    else if(vision_detect_mode == APPIPC_VISION_MODE_BRIDGE)
    {
        appipc_bridge_data_t bridge_data;

        if(appipc_decode_bridge_data(data, &bridge_data))
        {
            bridge_valid_from_core1 = bridge_data.valid;
            bridge_aligned_from_core1 = bridge_data.aligned;
            bridge_control_from_core1 = bridge_data.control_value;
        }
        else
        {
            bridge_valid_from_core1 = 0;
            bridge_aligned_from_core1 = 0;
            bridge_control_from_core1 = 0;
        }

        bridge_control_updated = 1;
    }
}

int main(void)
{
    clock_init(SYSTEM_CLOCK_250M);
    debug_init();
    wireless_uart_init();                             // 无线串口初始化
    servo_init();                                     // 舵机初始化
    filter_init();                                    // 滤波初始化
    banlance_init();                                  // PID参数初始化
    imu660rb_init();                                  // IMU初始化
    small_driver_uart_init();                         // 电机驱动初始化
    pit_ms_init(PIT_CH0, 1);                          // PIT中断初始化
    // screen_init();                                 // 屏幕初始化
    flash_init();
    remote_control_init();                            // 遥控器初始化
    button_init();                                    // 按键初始化
    buzzer_init();                                    // 蜂鸣器初始化
    appipc_rx_init(appipc_callback);                  // IPC初始化

    while(true)
    {
        //=========================== 视觉模式同步 ===========================

        vision_detect_mode = APPIPC_VISION_MODE_BRIDGE;

        appipc_send_core0_data(
            (uint16)fabsf(car_speed),
            (uint8)vision_detect_mode,
            APPIPC_BRIDGE_PHASE_ALIGN
        );

        //=========================== 跳跃模式 ===========================

        if(vision_detect_mode == APPIPC_VISION_MODE_JUMP)
        {
            switch(jump_count)
            {
                case 0:
                    vision_target_speed = 160;
                    break;

                case 1:
                    vision_target_speed = 130;
                    break;

                case 2:
                    vision_target_speed = 130;
                    break;
            }

            if(is_jump_updated)
            {
                is_jump_updated = 0;

                if(jump_count >= 2)
                {
                    continue;
                }

                if(is_jump_from_core1)
                {
                    jump_flag = 1;
                    jump_count++;
                }
            }
        }

        //=========================== 单边桥模式 ===========================

        else if(vision_detect_mode == APPIPC_VISION_MODE_BRIDGE)
        {
            if(bridge_control_updated)
            {
                bridge_control_updated = 0;

                // 识别有效但尚未对齐：低速前进并进行视觉转向
                if(bridge_valid_from_core1 &&
                   !bridge_aligned_from_core1)
                {
                    vision_target_speed = 50;
                    vision_target_yaw = bridge_control_from_core1;
                }
                // 识别丢失或者已经对齐：停车
                else
                {
                    vision_target_speed = 0;
                    vision_target_yaw = 0;
                }
            }
        }
    }
}
*/
