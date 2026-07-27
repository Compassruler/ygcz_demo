#include "zf_common_headfile.h"



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
#define BRIDGE_BINARY_THRESHOLD           (115)                  // 单边桥固定二值化阈值

#define BRIDGE_ROI_TOP                     (3)                    // 单边桥 ROI 最上行
#define BRIDGE_ROI_BOTTOM                  (117)                  // 单边桥 ROI 最下行，降低该值可排除更多底部暗区
#define BRIDGE_ROI_LEFT                    (3)                    // 单边桥 ROI 最左列
#define BRIDGE_ROI_RIGHT                   (MT9V03X_W - 3)        // 单边桥 ROI 最右列

#define BRIDGE_MIN_LANE_WIDTH             (20)                   // 左右边线最小间距
#define BRIDGE_MAX_LANE_WIDTH             (180)                  // 左右边线最大间距

#define BRIDGE_MAX_EDGE_JUMP              (2)                   // 相邻采样行边线最大横向变化
#define BRIDGE_MIN_POINT_COUNT            (50)                    // 最少连续有效采样行数量
#define BRIDGE_MIN_Y_SPAN                 (20)                   // 有效边线段最小纵向跨度
#define BRIDGE_ROW_STEP                   (1)                    // 纵向采样行间隔
#define BRIDGE_STABLE_PIXEL_COUNT         (4)                    // 边缘两侧连续黑白像素数量
#define BRIDGE_MAX_MISSING_ROWS           (5)                    // 同一边线段允许连续缺失的采样行数量

#define BRIDGE_TARGET_CENTER_X            (MT9V03X_W / 2)       // 赛道中线期望对齐横坐标

#define BRIDGE_ALIGN_FAR_TOLERANCE_PX     (3)                    // 远处中线允许误差
#define BRIDGE_ALIGN_NEAR_TOLERANCE_PX    (3)                    // 近处中线允许误差

#define BRIDGE_CONTROL_DEADBAND_PX        (1)                    // 当前倾角或中点控制误差死区
#define BRIDGE_TILT_REFERENCE_SPAN         (64)                   // 倾斜误差归一化参考纵向跨度
#define BRIDGE_TILT_ENTER_THRESHOLD_PX    (2)                    // 进入倾角优先控制的倾斜误差
#define BRIDGE_TILT_EXIT_THRESHOLD_PX     (1)                    // 退出倾角优先控制的倾斜误差
#define BRIDGE_TILT_GAIN_D10_PER_PX       (8.0f)                 // 每像素归一化倾斜误差产生的航向修正量
#define BRIDGE_ALIGN_COMPLETE_CONFIRM_FRAMES (1)                 // 中线完成对准的连续确认帧数
#define BRIDGE_ALIGN_LOST_RESET_FRAMES    (1)                    // 连续丢失目标后的复位帧数
#define BRIDGE_ALIGN_POINT_FILTER_ALPHA   (0.6f)                 // 中线中点横坐标低通滤波旧值权重
#define BRIDGE_POINT_GAIN_D10_PER_PX      (8.0f)                 // 每像素中点误差产生的航向修正量
#define BRIDGE_POINT_DIRECTION            (1.0f)                 // 倾角和中点误差共用的修正方向
#define BRIDGE_YAW_OFFSET_LIMIT_D10       (30)                  // 最大航向修正量，单位 0.1 度
#define BRIDGE_ALIGN_YAW_SLEW_LIMIT_D10   (10)                   // 每帧航向修正量最大变化
#define BRIDGE_CONTROL_GAIN_PER_DEG       (10.0f)                 // 每 1 度对应的底盘 angle 控制量
#define BRIDGE_CONTROL_DIRECTION          (1.0f)                 // 底盘控制方向
#define BRIDGE_CONTROL_LIMIT              (30)                  // 底盘 angle 控制量限制

//=========================== 单边桥离开检测参数 ===========================
#define BRIDGE_EXIT_BINARY_THRESHOLD   (85)    // 离桥检测固定二值化阈值
#define BRIDGE_EXIT_CHECK_ROW          (100)    // 离桥检测矩形起始行
#define BRIDGE_EXIT_CHECK_ROW_COUNT    (25)     // 从起始行向上检查的行数
#define BRIDGE_EXIT_CHECK_COLUMN       (55)     // 离桥检测矩形起始列
#define BRIDGE_EXIT_CHECK_COLUMN_COUNT (73)     // 从起始列向右检查的列数
#define BRIDGE_EXIT_WHITE_DOT_COUNT    (1400)   // 判断离桥所需的白色像素数量
#define BRIDGE_EXIT_CONFIRM_FRAMES     (5)      // 连续满足要求的帧数
#define BRIDGE_EXIT_CHECK_DELAY_MS     (4000)    // 冲桥后延迟开始离桥检测的时间

//=========================== 颠簸路段离开检测参数 ===========================
#define BUMP_EXIT_BINARY_THRESHOLD       (85)    // 颠簸路段检测固定二值化阈值
#define BUMP_EXIT_CHECK_ROW              (100)    // 检测矩形起始行
#define BUMP_EXIT_CHECK_ROW_COUNT        (25)     // 从起始行向上检查的行数
#define BUMP_EXIT_CHECK_COLUMN           (55)     // 检测矩形起始列
#define BUMP_EXIT_CHECK_COLUMN_COUNT     (73)     // 从起始列向右检查的列数
#define BUMP_SEEN_BLACK_DOT_COUNT        (300)    // 确认看到黑色凸起所需的黑色像素数量
#define BUMP_SEEN_CONFIRM_FRAMES         (2)      // 连续看到黑色凸起的帧数
#define BUMP_EXIT_WHITE_DOT_COUNT        (1600)   // 判断驶出所需的白色像素数量
#define BUMP_EXIT_CONFIRM_FRAMES         (5)      // 连续满足白色出口要求的帧数
#define BUMP_EXIT_CHECK_DELAY_MS         (2000)   // 进入颠簸阶段后允许判断出口的最短时间

//================================================================

volatile uint8 function_option                  = VISION_IDLE;                      // 核心0同步的视觉工作模式
volatile uint8 vision_phase_bab                 = VISION_PHASE_BAB_BRIDGE_ALIGN;    // 核心0同步的 BAB 工作子状态
volatile uint32 sys_ms                          = 0;                                // 毫秒计时器
static volatile uint16 core0_car_speed          = 0;                                // 实际车速
static volatile uint8  core0_speed_updated      = 0;                                // 车速更新标志位
static volatile uint8  core0_remote_ch9_value   = BRIDGE_BINARY_THRESHOLD;          // 核心0发送的通道9映射值

// IPC 接收核0的车速和视觉工作模式
static void appipc_speed_callback(uint32 data)
{
    appipc_core0_data_t core0_data;

    if(appipc_decode_core0_data(data, &core0_data))
    {
        core0_car_speed = core0_data.car_speed;
        function_option = core0_data.vision_detect_mode;
        vision_phase_bab = core0_data.vision_phase_bab;
        core0_remote_ch9_value = core0_data.remote_ch9_value;
        core0_speed_updated = 1;
    }
}

// 屏幕显示函数
void debug_image_screen_display(
    JumpDetectParams_t jump_params, 
    const CameraBridgeParams_t *bridge_params,
    const CameraBridgeResult_t *bridge_result, 
    const CameraBridgeAlignParams_t *bridge_align_params,
    BridgeExitParams_t bridge_exit_params,
    uint32 fps, 
    uint16 carspd
)
{
    if (function_option == VISION_JUMP)  // 跳跃时
    {
        #if IMAGE_DEBUG_TYPE == 1
        camera_debug_on_screen();                                   // 在显示屏上显示摄像头图像
        screen_show_detect_threshold_bar(jump_params);              // 识别矩形框 绘制四个绿色标识线
        screen_show_table_t2(jump_params, fps, 0, carspd);          // 显示信息 - 跳跃
        #endif
    }
    else if (function_option == VISION_BRIDGE_BUMP)  // 单边桥
    {
        #if IMAGE_DEBUG_TYPE == 1 
        camera_debug_on_screen();
        if(vision_phase_bab == VISION_PHASE_BAB_BRIDGE_ALIGN)
        {
            screen_show_bridge_roi(bridge_params);                              // 绘制绿色 ROI 边框
            screen_show_bridge_align_box(bridge_result, bridge_align_params);       // 绘制红色中线对准范围
            screen_show_bridge_fitted_line(bridge_result);           // 绘制左右边线和绿色中线
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
    BumpExitParams_t bump_exit_params =
    {
        .binary_threshold             = BUMP_EXIT_BINARY_THRESHOLD,
        .check_row                    = BUMP_EXIT_CHECK_ROW,
        .check_row_count              = BUMP_EXIT_CHECK_ROW_COUNT,
        .check_column                 = BUMP_EXIT_CHECK_COLUMN,
        .check_column_count           = BUMP_EXIT_CHECK_COLUMN_COUNT,
        .black_dot_count              = BUMP_SEEN_BLACK_DOT_COUNT,
        .white_dot_count              = BUMP_EXIT_WHITE_DOT_COUNT,
        .black_confirm_frame_count    = BUMP_SEEN_CONFIRM_FRAMES,
        .white_confirm_frame_count    = BUMP_EXIT_CONFIRM_FRAMES,
        .black_continuous_frame_count = 0,
        .white_continuous_frame_count = 0,
        .bump_seen                    = 0,
        .exited                       = 0
    };
    CameraBridgeParams_t bridge_params =
    {
        .binary_threshold         = BRIDGE_BINARY_THRESHOLD,
        .roi_top                  = BRIDGE_ROI_TOP,
        .roi_bottom               = BRIDGE_ROI_BOTTOM,
        .roi_left                 = BRIDGE_ROI_LEFT,
        .roi_right                = BRIDGE_ROI_RIGHT,
        .min_lane_width           = BRIDGE_MIN_LANE_WIDTH,
        .max_lane_width           = BRIDGE_MAX_LANE_WIDTH,
        .max_edge_jump            = BRIDGE_MAX_EDGE_JUMP,
        .min_point_count          = BRIDGE_MIN_POINT_COUNT,
        .min_y_span               = BRIDGE_MIN_Y_SPAN,
        .row_step                 = BRIDGE_ROW_STEP,
        .stable_pixel_count       = BRIDGE_STABLE_PIXEL_COUNT,
        .max_missing_rows         = BRIDGE_MAX_MISSING_ROWS
    };                                                // 单边桥识别参数结构体
    CameraBridgeAlignParams_t bridge_align_params =
    {
        .target_center_x         = BRIDGE_TARGET_CENTER_X,
        .far_tolerance_px        = BRIDGE_ALIGN_FAR_TOLERANCE_PX,
        .near_tolerance_px       = BRIDGE_ALIGN_NEAR_TOLERANCE_PX,
        .control_deadband_px     = BRIDGE_CONTROL_DEADBAND_PX,
        .tilt_reference_span     = BRIDGE_TILT_REFERENCE_SPAN,
        .tilt_enter_threshold_px = BRIDGE_TILT_ENTER_THRESHOLD_PX,
        .tilt_exit_threshold_px  = BRIDGE_TILT_EXIT_THRESHOLD_PX,
        .complete_confirm_frames = BRIDGE_ALIGN_COMPLETE_CONFIRM_FRAMES,
        .lost_reset_frames       = BRIDGE_ALIGN_LOST_RESET_FRAMES,
        .point_filter_alpha      = BRIDGE_ALIGN_POINT_FILTER_ALPHA,
        .point_gain_d10_per_px   = BRIDGE_POINT_GAIN_D10_PER_PX,
        .tilt_gain_d10_per_px    = BRIDGE_TILT_GAIN_D10_PER_PX,
        .point_direction         = BRIDGE_POINT_DIRECTION,
        .yaw_offset_limit_d10    = BRIDGE_YAW_OFFSET_LIMIT_D10,
        .yaw_slew_limit_d10      = BRIDGE_ALIGN_YAW_SLEW_LIMIT_D10,
        .control_gain_per_deg    = BRIDGE_CONTROL_GAIN_PER_DEG,
        .control_direction       = BRIDGE_CONTROL_DIRECTION,
        .control_limit           = BRIDGE_CONTROL_LIMIT
    };                                                // 单边桥对准控制参数结构体
    CameraBridgeAlignState_t bridge_align_state = {0}; // 单边桥对准控制运行状态
    CameraBridgeResult_t bridge_result = {0};          // 单边桥识别结果结构体
    CameraBridgeAlignResult_t bridge_align_result = {0}; // 单边桥对准控制结果结构体
    uint8  is_jump              = 0;                  // 跳跃触发标志位
    uint8  ipc_result           = APPIPC_BUSY;        // IPC发送结果：APPIPC_OK 成功，APPIPC_BUSY 失败或超时
    uint32 independent_fps      = 0;                  // 独立 FPS
    uint8  jump_uart_latched    = 0;                  // 串口发送的跳跃标志位
    uint32 uart_last_ms         = 0;                  // 串口更新计时
    uint8  actual_jump_count    = 0;                  // 实际跳跃次数计数
    uint8  last_vision_phase_bab = VISION_PHASE_BAB_BRIDGE_ALIGN; // 上一次执行的 BAB 子状态
    uint32 bridge_exit_start_ms  = 0;                 // 进入单边桥离开检测阶段的时间
    uint32 bump_exit_start_ms    = 0;                 // 进入颠簸路段离开检测阶段的时间
    uint8  bridge_exit_ipc_sent  = 0;                 // 单边桥离开信号是否发送成功
    uint8  bump_exit_ipc_sent    = 0;                 // 颠簸路段离开信号是否发送成功
    char txt[128];                                    // 串口发送文本

    clock_init(SYSTEM_CLOCK_250M);                    // 系统 初始化
    
    #if IMAGE_DEBUG_TYPE == 1
    screen_init();                                    // 屏幕 初始化
    #endif

    camera_init();                                    // MT9V03X 摄像头初始化
    pit_ms_init(PIT_CH1, 1);                          // PIT_CH1 1ms周期中断，用于 sys_ms 计时
    appipc_speed_rx_init(appipc_speed_callback);      // IPC接收 初始化
    camera_fps_counter_init(&camera_fps, sys_ms);     // 独立 FPS 计算初始化
    camera_bridge_align_reset(&bridge_align_state);  // 单边桥对准控制状态初始化


    while(true)
    {
        bridge_params.binary_threshold = core0_remote_ch9_value;  // 通道9实时调整单边桥二值化阈值

        //=========================== 执行单边桥与颠簸路段检测程序 ===========================
        // 核心视觉步骤判断条件，目前有| 0 空闲 | 1 单边桥-颠簸路段 | 2 跳跃 | 后续可能增加的步骤：返回三级台阶
        if (function_option == VISION_BRIDGE_BUMP)
        {
            // 更新单边桥内部状态，并且确定运行时变量状态，主要是计时相关变量
            if(last_vision_phase_bab != vision_phase_bab)
            {
                last_vision_phase_bab = vision_phase_bab;

                // 状态恢复
                if(vision_phase_bab == VISION_PHASE_BAB_BRIDGE_ALIGN)
                {
                    camera_bridge_align_reset(&bridge_align_state);
                }
                else if(vision_phase_bab == VISION_PHASE_BAB_BRIDGE_EXIT_CHECK)
                {
                    bridge_exit_start_ms = sys_ms;  // 时间更新
                    bridge_exit_params.continuous_frame_count = 0;
                    bridge_exit_params.exited = 0;
                    bridge_exit_ipc_sent = 0;
                }
                else if(vision_phase_bab == VISION_PHASE_BAB_BUMP_EXIT_CHECK)
                {
                    bump_exit_start_ms = sys_ms;  // 时间更新
                    bump_exit_params.black_continuous_frame_count = 0;
                    bump_exit_params.white_continuous_frame_count = 0;
                    bump_exit_params.bump_seen = 0;
                    bump_exit_params.exited = 0;
                    bump_exit_ipc_sent = 0;
                }
            }
            
            // 进入单边桥对齐状态 并且 单边桥对齐处理函数工作正常
            if((vision_phase_bab == VISION_PHASE_BAB_BRIDGE_ALIGN) && camera_bridge_processing(&bridge_params, &bridge_result))
            {
                // 根据中线倾斜和中点位置计算转向控制量
                independent_fps = camera_fps_counter_update(&camera_fps, sys_ms);  // 独立 FPS 计算
                camera_bridge_align_update(&bridge_result, &bridge_align_params, &bridge_align_state, &bridge_align_result);

                // 入桥 IPC 发送部分
                appipc_send_bridge_data(bridge_align_result.valid, bridge_align_result.aligned, 0,
                                        (uint8)bridge_result.bottom, bridge_align_result.control_value);

                // 串口部分
                sprintf(txt, "Valid %d |Aligned %d |Phase %d |Bottom %d |Point %d,%d |Tilt %d,%d |Err %d |Ctrl %d |FPS %d\r\n",
                        bridge_align_result.valid, bridge_align_result.aligned, bridge_align_result.phase,
                        bridge_result.bottom, bridge_align_result.active_x, bridge_align_result.active_y,
                        bridge_align_result.tilt_control_active, bridge_align_result.tilt_error_px,
                        bridge_align_result.point_error_px, bridge_align_result.control_value, independent_fps);
                wireless_uart_send_string(txt);
            }

            // 如果不是对齐状态，那进入 离开单边桥检测阶段 
            // 需要满足的条件是 子状态进入 单边桥离开检测 状态 并且 状态更新时间到实际使能时间 大于 最短检测延时，防止过早检测而提前离开单边桥
            else if((vision_phase_bab == VISION_PHASE_BAB_BRIDGE_EXIT_CHECK) &&      // 子状态为 单边桥离开检测
                    ((sys_ms - bridge_exit_start_ms) >= BRIDGE_EXIT_CHECK_DELAY_MS)) // 满足单边桥离开检测使能延时
            {
                // 离桥检测部分
                // 单边桥离开检测标志位为 非离开状态，即尚未离开 并且 摄像头有新帧
                if(!bridge_exit_params.exited && camera_has_frame())
                {
                    independent_fps = camera_fps_counter_update(&camera_fps, sys_ms);  // 独立 FPS 计算
                    camera_bridge_exit_processing(&bridge_exit_params);  // 单边桥离开检测，通过修改 bridge_exit_params 返回检测状态
                }
                
                // 离桥 IPC 发送部分
                // 已经离桥 并且 IPC 尚未成功发送时，尝试发送，一旦发送成功则立即停止，防止死循环
                if(bridge_exit_params.exited && !bridge_exit_ipc_sent)
                {
                    bridge_exit_ipc_sent = (APPIPC_OK == appipc_send_bridge_data(0, 0, 1, 0, 0)) ? 1 : 0;  // 发送一次后就不再发送
                }
            }

            // 如果不是前两者，那么就应该判断是否进入 离开颠簸路段检测状态
            else if(vision_phase_bab == VISION_PHASE_BAB_BUMP_EXIT_CHECK)
            {
                // 离开颠簸路段检测部分
                // 颠簸路段离开检测标志位为 非离开状态，即尚未离开 并且 摄像头有新帧
                if(!bump_exit_params.exited && camera_has_frame())
                {
                    independent_fps = camera_fps_counter_update(&camera_fps, sys_ms); 
                    // 检测离开 颠簸路段的返回值修改在结构体中，目前支持延时检测和进入增强检测等多个检测方法
                    camera_bump_exit_processing(&bump_exit_params, (uint8)((sys_ms - bump_exit_start_ms) >= BUMP_EXIT_CHECK_DELAY_MS));
                }
                
                // 离开颠簸路段 IPC 发送部分
                if(bump_exit_params.exited && !bump_exit_ipc_sent)
                {
                    bump_exit_ipc_sent = (APPIPC_OK == appipc_send_bridge_data(0, 0, 1, 0, 0)) ? 1 : 0;
                }
            }
        }

        //=========================== 执行跳跃检测程序 ===========================
        if (function_option == VISION_JUMP)
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
        // 统一视觉更新
        debug_image_screen_display(jump_params, &bridge_params, &bridge_result, &bridge_align_params, bridge_exit_params, independent_fps, core0_car_speed);
    }
}

