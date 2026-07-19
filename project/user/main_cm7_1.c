#include "zf_common_headfile.h"
#include "camera.h"
#include "button.h"
#include "appipc.h"

/*
跳跃时：
    1. 调整视觉重心
    2. 核0 的中断
*/

//=========================== 工作模式 ===========================
#define CURRENT_WORK_MODE              (1)                      // 当前工作模式| 1 单边桥 | 0 跳跃 |

//=========================== 启动选择 ===========================
#define HOLD_MS                 (1500)                          // 启动时暂停跳跃时间

//=========================== 显示模式选择 ===========================
#define IMAGE_DEBUG_TYPE        (0)                             // 0 无显示 | <1 仅屏幕>当前不可用

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
#define BRIDGE_MIN_AREA             (50)                        // 黑色块最小面积
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
#define BRIDGE_ANGLE_DIRECTION           (1.0f)
#define BRIDGE_DISTANCE_DIRECTION        (1.0f)

// 校准后的误差死区
#define BRIDGE_ANGLE_DEADBAND_D10        (10)    // 1.0 度
#define BRIDGE_DISTANCE_DEADBAND_PX      (3)     // 3 像素

// 最大输出航向角修正量，单位 0.1 度
#define BRIDGE_YAW_OFFSET_LIMIT_D10      (60)    // 最大正负 6.0 度

#define BRIDGE_CONTROL_GAIN_PER_DEG    (10.0f)  // 每1度视觉航向偏差转换成多少angle控制量
#define BRIDGE_CONTROL_LIMIT           (60)     // angle控制量的最大绝对值

//================================================================

uint8 function_option                           = CURRENT_WORK_MODE;            // 程序功能
volatile uint32 sys_ms                          = 0;                            // 毫秒计时器
static volatile uint16 core0_car_speed          = 0;                            // 实际车速
static volatile uint8  core0_speed_updated      = 0;                            // 车速更新标志位

// IPC 接收 核0 的车速
static void appipc_speed_callback(uint32 data)
{
    core0_car_speed = (uint16)(data & 0xFFFFu);
    core0_speed_updated = 1;
}

// 屏幕显示函数-跳跃
void debug_image_screen_display(JumpDetectParams_t jump_params)
{
    #if IMAGE_DEBUG_TYPE == 1 || IMAGE_DEBUG_TYPE == 3
    camera_debug_on_screen();                            // 在显示屏上显示摄像头图像
    screen_show_detect_threshold_bar(jump_params);       // 识别矩形框 绘制四个绿色标识线
    #endif
}

// 屏幕显示函数-单边桥
void debug_bridge_image_screen_display(const CameraBridgeResult_t *bridge_result)
{
    #if IMAGE_DEBUG_TYPE == 1 
    camera_debug_on_screen();
    screen_show_bridge_fitted_line(bridge_result);
    #endif
}

// 统一更新函数
void jump_updates(uint32 sys_ms, JumpDetectParams_t *jump_params, uint32 fps, uint32 is_jump, uint8 ipc_result, uint16 carspd)
{
    #if IMAGE_DEBUG_TYPE == 1 
    screen_show_table_t2(*jump_params, 0, is_jump, carspd);   // 屏幕显示参数更新
    #endif
}

// 单边桥 angle + distance -> CtrlAngle
static int16 bridge_calculate_yaw_offset_d10(const CameraBridgeResult_t *bridge_result)
{
    int16 angle_error_d10;
    int16 distance_error_px;
    float yaw_offset;
    int16 yaw_offset_d10;

    if((0 == bridge_result) || !bridge_result->valid)
    {
        return 0;
    }

    // 以“实际正确对准时的视觉输出”为零点
    angle_error_d10 = bridge_result->angle_d10 - BRIDGE_ALIGNED_ANGLE_D10;
    distance_error_px = bridge_result->distance_px - BRIDGE_ALIGNED_DISTANCE_PX;

    // 消除对准位置附近的小幅抖动
    if((angle_error_d10 >= -BRIDGE_ANGLE_DEADBAND_D10) && (angle_error_d10 <=  BRIDGE_ANGLE_DEADBAND_D10))
    {
        angle_error_d10 = 0;
    }

    if((distance_error_px >= -BRIDGE_DISTANCE_DEADBAND_PX) && (distance_error_px <=  BRIDGE_DISTANCE_DEADBAND_PX))
    {
        distance_error_px = 0;
    }

    // 角度误差负责使小车与桥平行
    // 距离误差负责使小车逐渐靠近目标中线
    yaw_offset = BRIDGE_ANGLE_DIRECTION * BRIDGE_ANGLE_GAIN * (float)angle_error_d10 
                + BRIDGE_DISTANCE_DIRECTION * BRIDGE_DISTANCE_GAIN * (float)distance_error_px;

    // 四舍五入转换成 int16
    if(yaw_offset >= 0.0f)
    {
        yaw_offset_d10 = (int16)(yaw_offset + 0.5f);
    }
    else
    {
        yaw_offset_d10 = (int16)(yaw_offset - 0.5f);
    }

    // 限制最大航向修正，防止识别异常时突然大幅转向
    if(yaw_offset_d10 > BRIDGE_YAW_OFFSET_LIMIT_D10)
    {
        yaw_offset_d10 = BRIDGE_YAW_OFFSET_LIMIT_D10;
    }
    else if(yaw_offset_d10 < -BRIDGE_YAW_OFFSET_LIMIT_D10)
    {
        yaw_offset_d10 = -BRIDGE_YAW_OFFSET_LIMIT_D10;
    }

    return yaw_offset_d10;
}


static int32 bridge_yaw_offset_to_control(int16 yaw_offset_d10)
{
    float control_output;
    int32 control_value;

    control_output =
          (float)yaw_offset_d10
        * 0.1f
        * BRIDGE_CONTROL_GAIN_PER_DEG
        * -1.0f;

    if(control_output >= 0.0f)
    {
        control_value = (int32)(control_output + 0.5f);
    }
    else
    {
        control_value = (int32)(control_output - 0.5f);
    }

    if(control_value > BRIDGE_CONTROL_LIMIT)
    {
        control_value = BRIDGE_CONTROL_LIMIT;
    }
    else if(control_value < -BRIDGE_CONTROL_LIMIT)
    {
        control_value = -BRIDGE_CONTROL_LIMIT;
    }

    return control_value;
}

int main(void)
{
    fps_counter_t camera_fps;                         // FPS 结构体初始化
    JumpDetectParams_t jump_params = 
    {
        .algo_type                = 1,
        .check_row                = 100,
        .check_row_count          = JUMP_ROW_TOTAL,
        .check_column             = JUMP_COLUMN,
        .check_column_count       = JUMP_COLUMN_TOTAL,
        .otsu_roi_row             = 0,
        .otsu_roi_row_count       = 0,
        .otsu_roi_column          = 0,
        .otsu_roi_column_count    = 0,
        .dot_type                 = dot_type_list[0],
        .dot_count                = JUMP_DOT_COUNT,
        .cooldown_time_ms         = JUMP_COOLDOWN_MS,
        .multi_frame              = JUMP_MULTI_FRAME,
        .steps                    = 0
    };                                                // 跳跃检测参数结构体
    CameraBridgeParams_t bridge_params =
    {
        .binary_threshold         = BRIDGE_BINARY_THRESHOLD,
        .search_left              = BRIDGE_SEARCH_LEFT,
        .search_right             = BRIDGE_SEARCH_RIGHT,
        .search_top               = BRIDGE_SEARCH_TOP,
        .search_bottom            = BRIDGE_SEARCH_BOTTOM,
        .min_width                = BRIDGE_MIN_WIDTH,
        .min_height               = BRIDGE_MIN_HEIGHT,
        .min_area                 = BRIDGE_MIN_AREA,
        .connect_gap              = BRIDGE_CONNECT_GAP,
        .target_edge_x            = BRIDGE_TARGET_EDGE_X
    };                                                // 单边桥识别参数结构体
    CameraBridgeResult_t bridge_result = {0};         // 单边桥识别结构结构体
    uint8  is_jump              = 0;                  // 跳跃触发标志位
    uint8  ipc_result           = APPIPC_BUSY;        // IPC发送结果：APPIPC_OK 成功，APPIPC_BUSY 失败或超时
    uint32 independent_fps      = 0;                  // 独立 FPS
    uint8  jump_uart_latched    = 0;                  // 串口发送的跳跃标志位
    uint32 uart_last_ms         = 0;                  // 串口更新计时
    uint8  actual_jump_count    = 0;                  // 实际跳跃次数计数
    int32 bridge_control_value  = 0;                  // 单边桥控制角度
    char txt[64];                                     // 串口发送文本

    clock_init(SYSTEM_CLOCK_250M);                    // 系统 初始化
    // screen_init();                                    // 屏幕 初始化 核0 已
    camera_init();                                    // MT9V03X 摄像头初始化
    pit_ms_init(PIT_CH1, 1);                          // PIT_CH1 1ms周期中断，用于 sys_ms 计时
    appipc_speed_rx_init(appipc_speed_callback);      // IPC接收 初始化
    camera_fps_counter_init(&camera_fps, sys_ms);     // 独立 FPS 计算初始化


    while(true)
    {   
        //=========================== 执行单边桥检测程序 ===========================
        if (function_option == 1)
        {
            if(camera_bridge_processing(&bridge_params, &bridge_result))
            {
                bridge_control_value = bridge_yaw_offset_to_control(bridge_calculate_yaw_offset_d10(&bridge_result));
                appipc_send_bridge_data(bridge_result.valid, 0, (int16)bridge_control_value);

//                sprintf(txt, "Valid %d |Distance %d |Angle %d |CtrlAng %d\r\n",
//                        bridge_result.valid, bridge_result.distance_px, bridge_result.angle_d10, bridge_control_value);
                wireless_uart_send_string(txt);
            }
            debug_bridge_image_screen_display(&bridge_result);
        }

        //=========================== 执行跳跃检测程序 ===========================
        if (function_option == 0)
        {
            // 来自核0 的速度更新后 设置自适应识别距离
            if(core0_speed_updated)
            {
                core0_speed_updated = 0;
                jump_params.check_row = camera_check_row_from_speed(core0_car_speed, ADAPTIVE_ROW_COEFF); 
            }

            // 当检测到有帧时 且 当计时器时间大于启动时暂停跳跃时间执行
            if (camera_has_frame() && sys_ms > HOLD_MS)
            {
                independent_fps = camera_fps_counter_update(&camera_fps, sys_ms);       // 独立 FPS 计算
                is_jump         = camera_processing(sys_ms, &jump_params);              // 检测跳跃
                debug_image_screen_display(jump_params);                                // 使用屏幕显示图像

                // 发送更新到核0
                if(is_jump)
                {
                    jump_uart_latched   = 1;                                            // 串口发送的数据
                    ipc_result          = appipc_send_u32(1);                           // IPC 发送
                    actual_jump_count++;                                                // 实际跳跃次数++
                }
            }
            
            // 串口数据发送
            if((jump_uart_latched)) // (sys_ms - uart_last_ms) >= 50u && 
            {
                uart_last_ms = sys_ms;
                sprintf(txt,
                        "Spd %04d |J %d |Row %03u |Fps %d | %d ms\r\n",
                        core0_car_speed,
                        jump_uart_latched,
                        jump_params.check_row,
                        independent_fps,
                        sys_ms
                        );
                wireless_uart_send_string(txt);
                jump_uart_latched = 0;
            }

            //jump_updates(sys_ms, &jump_params, 0, is_jump, ipc_result, core0_car_speed);  // 统一更新函数，只保留了屏幕更新 禁用防止出现问题
        }
    }
}

/* 
// 配套核0代码，未融合惯导
#include "zf_common_headfile.h"
#include "appipc.h"

//=========================== 单边桥控制参数 =====================
#define BRIDGE_FORWARD_SPEED           (60)     // 单边桥模式固定前进速度
#define BRIDGE_CONTROL_GAIN_PER_DEG    (10.0f)  // 每1度视觉航向偏差转换成多少angle控制量
#define BRIDGE_CONTROL_LIMIT           (60)     // angle控制量的最大绝对值

//================================================================

uint8 function_option = 0;                  // 程序功能 | 1 单边桥 | 0 跳跃 |

volatile uint8 is_jump_from_core1 = 0;
volatile uint8 is_jump_updated = 0;
volatile uint8 bridge_valid_from_core1 = 0;
volatile int16 bridge_yaw_offset_d10 = 0;
volatile uint8 bridge_control_updated = 0;

volatile int32 spd = 130; 
volatile int32 angle = 0;

uint32 jump_count = 0;
uint32 jump_complete = 0;


// 将核心1的视觉角度修正量转换为angle控制量
static int32 bridge_yaw_offset_to_control(int16 yaw_offset_d10)
{
    float angle_error_deg;
    float control_output;
    int32 control_value;

    // 核心1的单位为0.1度，转换为度
    angle_error_deg = (float)yaw_offset_d10 * 0.1f;

    // 将视觉角度误差转换成航向偏转控制量
    control_output =
          angle_error_deg
        * BRIDGE_CONTROL_GAIN_PER_DEG
        * -1.0f;

    // 四舍五入转换为整数
    if(control_output >= 0.0f)
    {
        control_value = (int32)(control_output + 0.5f);
    }
    else
    {
        control_value = (int32)(control_output - 0.5f);
    }

    // 限制最大偏转，防止小车突然大幅转向
    if(control_value > BRIDGE_CONTROL_LIMIT)
    {
        control_value = BRIDGE_CONTROL_LIMIT;
    }
    else if(control_value < -BRIDGE_CONTROL_LIMIT)
    {
        control_value = -BRIDGE_CONTROL_LIMIT;
    }

    return control_value;
}

// 接收核心1发送的数据
static void appipc_callback(uint32 data)
{
    if(function_option == 0)
    {
        is_jump_from_core1 = (uint8)(data & 0x01);
        is_jump_updated = 1;
    }
    else if(function_option == 1)
    {
        appipc_bridge_data_t bridge_data;

        if(appipc_decode_bridge_data(data, &bridge_data))
        {
            bridge_valid_from_core1 = bridge_data.valid;
            bridge_yaw_offset_d10 = bridge_data.angle_d10;
        }
        else
        {
            bridge_valid_from_core1 = 0;
            bridge_yaw_offset_d10 = 0;
        }

        bridge_control_updated = 1;
    }
}


int main(void)
{
    clock_init(SYSTEM_CLOCK_250M);                      
    debug_init();
    servo_init();                                         // 舵机初始化
    filter_init();                                        // 滤波初始化
    banlance_init();                                      // PID参数初始化
    imu660rb_init();                                      // IMU初始化
    small_driver_uart_init();                             // 电机驱动初始化
    pit_ms_init(PIT_CH0,1);                               // PIT中断初始化
    flash_init();
    appipc_rx_init(appipc_callback);                      // IPC 初始化

    while(true)
    {
        uint16 speed_for_row;
        if(car_speed >= 0)
        {
            speed_for_row = (uint16)car_speed;
        }
        else
        {
            speed_for_row = (uint16)(-car_speed);
        }

        (void)appipc_send_speed_u32((uint32)speed_for_row);


        //=========================== 跳跃模式 ===========================
        if(function_option == 0)
        {   
            // 速度控制
            switch (jump_count)
            {
                case 0:
                    spd = 160;  // 第1次跳跃前的速度
                    break;

                case 1:
                    spd = 130;  // 第2次跳跃前的速度
                    break;

                case 2:
                    spd = 130;  // 第3次跳跃前的速度
                    break;
            }

            // 跳跃控制
            if(is_jump_updated)
            {
                is_jump_updated = 0;
                if(jump_count >= 2) continue;  // 限定跳跃次数
                
                // 执行跳跃
                if(is_jump_from_core1)
                {
                    jump_flag = 1;
                    jump_count++;
                }
            } 
        }


        //=========================== 单边桥模式 =========================
        else if(function_option == 1)
        {
            if(bridge_control_updated)
            {
                bridge_control_updated = 0;

                if(bridge_valid_from_core1)
                {
                    spd = BRIDGE_FORWARD_SPEED;                                    // 识别有效：固定低速前进
                    angle = bridge_yaw_offset_to_control(bridge_yaw_offset_d10);   // 将核心1计算的视觉偏差转换成航向偏转加权量
                }
                else
                {
                    spd = 0;  // 识别丢失或者进入单边桥
                    angle = 0;
                }
            }
        }

    }
}

*/