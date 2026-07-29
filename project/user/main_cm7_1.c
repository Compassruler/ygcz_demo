#include "zf_common_headfile.h"



//=========================== 启动选择 ===========================
#define HOLD_MS                 (1500)                          // 启动时暂停跳跃时间

//=========================== 显示模式选择 ===========================
#define IMAGE_DEBUG_TYPE        (1)                             // 0 无显示 | <1 仅屏幕>当前不可用

//=========================== WiFi SPI 图传参数 ===========================
#define WIFI_SSID               "/"                          // WiFi SSID
#define WIFI_PWD                "/"                          // WiFi 密码
#define TARGET_IP               "192.168.137.1"              // 上位机 IP 地址
#define TARGET_PORT             "8086"                       // 上位机 TCP 端口
#define LOCAL_PORT              "6666"                       // WiFi 模块本地端口

//=========================== 跳跃判断条件 ===========================
#define JUMP_ROW_TOTAL          (25)                            // 行向上检查行数
#define JUMP_COLUMN             (55)                            // 列起始位置
#define JUMP_COLUMN_TOTAL       (73)                            // 列向右检查行数
#define JUMP_DOT_COUNT          (1600)                          // 矩形内点阈值
#define JUMP_COOLDOWN_MS        (0)                             // 跳跃触发一次后的禁止重复触发时间
#define JUMP_MULTI_FRAME        (1)                             // 有效帧阈值
#define ADAPTIVE_ROW_COEFF      (-5)                            // 自适应 Row 系数，数字为正，更晚切换到低 Row，更偏高 Row，跳跃时间更早；数字为负，更早切换到低 Row，更偏低 Row，跳跃时间越晚

//=========================== 单边桥识别参数 ===========================
#define BRIDGE_BINARY_THRESHOLD              (115)             // 固定阈值，也是大津法失效时的备用阈值
#define BRIDGE_USE_OTSU_THRESHOLD             (0)               // 1 使用 ROI 大津法 | 0 使用固定阈值
#define BRIDGE_THRESHOLD_FILTER_ALPHA         (0.70f)           // 自动阈值低通滤波旧值权重

#define BRIDGE_ROI_TOP                        (15)               // 单边桥 ROI 最上行
#define BRIDGE_ROI_BOTTOM                     (110)             // 单边桥 ROI 最下行
#define BRIDGE_ROI_LEFT                       (3)               // 单边桥 ROI 最左列
#define BRIDGE_ROI_RIGHT                      (MT9V03X_W - 3)   // 单边桥 ROI 最右列
#define BRIDGE_MIN_LANE_WIDTH                 (20)              // 左右边线最小间距
#define BRIDGE_MAX_LANE_WIDTH                 (BRIDGE_ROI_RIGHT - BRIDGE_ROI_LEFT) // 左右边线最大间距


#define BRIDGE_RANSAC_ITERATIONS              (250)             // 单条边线的 RANSAC 采样次数
#define BRIDGE_RANSAC_DISTANCE_PX             (3)               // RANSAC 内点距离阈值

#define BRIDGE_MIN_LINE_POINT_COUNT           (30)              // 单条拟合线最少内点数量
#define BRIDGE_MIN_Y_SPAN                     (25)              // 单条拟合线最小纵向跨度

#define BRIDGE_ROW_STEP                       (1)               // 纵向采样行间隔

#define BRIDGE_STABLE_PIXEL_COUNT             (2)               // 跳变两侧连续黑白像素数量
#define BRIDGE_SINGLE_EDGE_HOLD_FRAMES        (6)               // 单边线暂时出画时允许补线的帧数
#define BRIDGE_LOST_HOLD_FRAMES               (2)               // 双边线丢失后保持上一结果的帧数
#define BRIDGE_LANE_WIDTH_FILTER_ALPHA        (0.75f)           // 赛道宽度模型低通滤波旧值权重

#define BRIDGE_TARGET_CENTER_X                (MT9V03X_W / 2)   // 车辆视觉中心对应的目标横坐标
#define BRIDGE_ALIGN_FAR_TOLERANCE_PX         (2)               // 中线上端允许误差
#define BRIDGE_ALIGN_NEAR_TOLERANCE_PX        (2)               // 中线下端允许误差

#define BRIDGE_FALLBACK_FAR_TOLERANCE_PX      (6)              // 保底冲刺允许的中线上端误差
#define BRIDGE_FALLBACK_NEAR_TOLERANCE_PX     (8)              // 保底冲刺允许的中线下端误差

#define BRIDGE_FALLBACK_TIMEOUT_MS            (500)             // 连续处于保底范围后直接确认对准
#define BRIDGE_FALLBACK_ESTIMATED_GRACE_FRAMES (2)              // 保底计时允许连续出现的估算结果帧数
#define BRIDGE_CONTROL_DEADBAND_D10           (5)               // 预瞄航向控制死区，单位 0.1 度
#define BRIDGE_LOOKAHEAD_MIN_PX               (15)              // 大角度时最小预瞄距离
#define BRIDGE_LOOKAHEAD_MAX_PX               (40)              // 小角度时最大预瞄距离
#define BRIDGE_LOOKAHEAD_FULL_ANGLE_D10       (400)             // 使用最小预瞄距离的角度，单位 0.1 度
#define BRIDGE_ALIGN_COMPLETE_CONFIRM_FRAMES  (2)               // 完成对准所需的连续新鲜入框帧数
#define BRIDGE_ALIGN_LOST_RESET_FRAMES        (1)               // 连续丢失目标后的复位帧数

#define BRIDGE_NEAR_ENTER_ANGLE_D10           (80)              // 中线夹角小于该值时进入近对准模式，单位 0.1 度
#define BRIDGE_NEAR_EXIT_ANGLE_D10            (120)             // 中线夹角大于该值时退出近对准模式，单位 0.1 度
#define BRIDGE_NEAR_LINE_FILTER_ALPHA         (0.85f)           // 近对准模式中线滤波旧值权重
#define BRIDGE_NEAR_YAW_GAIN                  (0.45f)           // 近对准模式预瞄航向角增益
#define BRIDGE_NEAR_CONTROL_DEADBAND_D10      (12)              // 近对准模式航向控制死区，单位 0.1 度
#define BRIDGE_NEAR_YAW_SLEW_LIMIT_D10        (10)              // 近对准模式每帧航向修正量最大变化
#define BRIDGE_NEAR_CONTROL_GAIN_PER_DEG      (5.0f)            // 近对准模式每 1 度对应的底盘 angle 控制量
#define BRIDGE_NEAR_REVERSE_CONFIRM_FRAMES    (2)               // 近对准模式反向修正所需的连续确认帧数

#define BRIDGE_LINE_FILTER_ALPHA              (0.50f)           // 中线模型低通滤波旧值权重
#define BRIDGE_YAW_GAIN                       (1.0f)            // 预瞄航向角增益
#define BRIDGE_YAW_DIRECTION                  (1.0f)            // 预瞄航向角修正方向

#define BRIDGE_YAW_OFFSET_LIMIT_D10           (40)             // 最大航向修正量，单位 0.1 度
#define BRIDGE_ALIGN_YAW_SLEW_LIMIT_D10       (40)              // 每帧航向修正量最大变化
#define BRIDGE_CONTROL_GAIN_PER_DEG           (7.0f)           // 每 1 度对应的底盘 angle 控制量
#define BRIDGE_CONTROL_DIRECTION              (1.0f)            // 底盘控制方向
#define BRIDGE_CONTROL_LIMIT                  (60)              // 底盘 angle 控制量限制

#define BRIDGE_BLIND_TRIGGER_ANGLE_D10        (350)             // 允许准备盲转的最小中线角度，单位 0.1 度
#define BRIDGE_FORCE_BLIND_TRIGGER_ANGLE_D10  (350)             // 强制 85 度盲转所需的最小可靠中线角度，单位 0.1 度
#define BRIDGE_BLIND_RELEASE_ANGLE_D10        (150)             // 强制盲转提前结束允许的中线夹角，单位 0.1 度

#define BRIDGE_BLIND_RELEASE_RESET_D10        (300)             // 超过该夹角后清除提前结束连续帧计数
#define BRIDGE_BLIND_RELEASE_CONFIRM_FRAMES   (1)               // 提前结束盲转所需的连续可靠帧数
#define BRIDGE_BLIND_RELEASE_MIN_TIME_MS      (250)             // 强制盲转开始后允许视觉提前接管的最短时间
#define BRIDGE_FRESH_TARGET_CONFIRM_FRAMES    (2)               // 确认重新找到新鲜赛道目标所需的连续帧数

#define BRIDGE_BLIND_TURN_TIME_MS             (100)             // 单次盲转持续时间

#define BRIDGE_BLIND_EDGE_MARGIN_PX           (8)               // 边线接近画幅边界的触发距离
#define BRIDGE_BLIND_RELIABLE_FRAMES          (2)               // 保存盲转方向所需的连续可靠帧数
#define BRIDGE_BLIND_ESTIMATED_FRAMES         (2)               // 连续使用补线结果后允许盲转的帧数
#define BRIDGE_BLIND_CONTROL_PERCENT          (90)              // 盲转控制量占可靠视觉控制量的百分比
//=========================== 单边桥离开检测参数 ===========================
#define BRIDGE_EXIT_BINARY_THRESHOLD   (85)    // 离桥检测固定二值化阈值（无效）
#define BRIDGE_EXIT_CHECK_ROW          (100)    // 离桥检测矩形起始行
#define BRIDGE_EXIT_CHECK_ROW_COUNT    (25)     // 从起始行向上检查的行数
#define BRIDGE_EXIT_CHECK_COLUMN       (55)     // 离桥检测矩形起始列
#define BRIDGE_EXIT_CHECK_COLUMN_COUNT (73)     // 从起始列向右检查的列数
#define BRIDGE_EXIT_WHITE_DOT_COUNT    (1400)   // 判断离桥所需的白色像素数量
#define BRIDGE_EXIT_CONFIRM_FRAMES     (5)      // 连续满足要求的帧数
#define BRIDGE_EXIT_CHECK_DELAY_MS     (4000)    // 冲桥后延迟开始离桥检测的时间

//=========================== 颠簸路段离开检测参数 ===========================
#define BUMP_EXIT_BINARY_THRESHOLD       (85)    // 颠簸路段检测固定二值化阈值（无效）
#define BUMP_EXIT_CHECK_ROW              (100)    // 检测矩形起始行
#define BUMP_EXIT_CHECK_ROW_COUNT        (25)     // 从起始行向上检查的行数
#define BUMP_EXIT_CHECK_COLUMN           (55)     // 检测矩形起始列
#define BUMP_EXIT_CHECK_COLUMN_COUNT     (73)     // 从起始列向右检查的列数
#define BUMP_SEEN_BLACK_DOT_COUNT        (300)    // 确认看到黑色凸起所需的黑色像素数量
#define BUMP_SEEN_CONFIRM_FRAMES         (0)      // 连续看到黑色凸起的帧数
#define BUMP_EXIT_WHITE_DOT_COUNT        (1600)   // 判断驶出所需的白色像素数量
#define BUMP_EXIT_CONFIRM_FRAMES         (5)      // 连续满足白色出口要求的帧数
#define BUMP_EXIT_CHECK_DELAY_MS         (1500)   // 进入颠簸阶段后允许判断出口的最短时间

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
    (void)bridge_exit_params;
    (void)fps;
    (void)carspd;

    if (function_option == VISION_JUMP)  // 跳跃时
    {
        #if IMAGE_DEBUG_TYPE == 1
        if(camera_debug_on_screen())
        {
            screen_show_detect_threshold_bar(jump_params);          // 绘制跳跃检测参考线
        }
        // screen_show_table_t2(jump_params, fps, 0, carspd);        // TFT180 暂不显示通用数据表
        #endif
    }
    else if (function_option == VISION_BRIDGE_BUMP)  // 单边桥
    {
        #if IMAGE_DEBUG_TYPE == 1
        if(camera_debug_on_screen() &&
           (vision_phase_bab == VISION_PHASE_BAB_BRIDGE_ALIGN))
        {
            screen_show_bridge_roi(bridge_params);                   // 绘制绿色 ROI 边框
            screen_show_bridge_align_box(bridge_result, bridge_align_params); // 绘制红色中线对准范围
            screen_show_bridge_fitted_line(bridge_result);           // 绘制左右边线和绿色中线
        }
        // screen_show_table_t3(bridge_exit_params, function_option, fps); // TFT180 暂不显示通用数据表
        #endif
    }
}

// 将单边桥内部对准阶段转换为串口可读文本
static const char *bridge_align_phase_text(CameraBridgeAlignPhase_t phase)
{
    switch(phase)
    {
        case CAMERA_BRIDGE_ALIGN_TRACK:
            return "TRACK";

        case CAMERA_BRIDGE_ALIGN_BLIND_TURN:
            return "BLIND_TURN";

        case CAMERA_BRIDGE_ALIGN_COMPLETE:
            return "COMPLETE";

        default:
            return "UNKNOWN";
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
        .binary_threshold          = BRIDGE_BINARY_THRESHOLD,
        .use_otsu_threshold        = BRIDGE_USE_OTSU_THRESHOLD,
        .threshold_filter_alpha    = BRIDGE_THRESHOLD_FILTER_ALPHA,
        .roi_top                   = BRIDGE_ROI_TOP,
        .roi_bottom                = BRIDGE_ROI_BOTTOM,
        .roi_left                  = BRIDGE_ROI_LEFT,
        .roi_right                 = BRIDGE_ROI_RIGHT,
        .min_lane_width            = BRIDGE_MIN_LANE_WIDTH,
        .max_lane_width            = BRIDGE_MAX_LANE_WIDTH,
        .ransac_iterations         = BRIDGE_RANSAC_ITERATIONS,
        .min_line_point_count      = BRIDGE_MIN_LINE_POINT_COUNT,
        .min_y_span                = BRIDGE_MIN_Y_SPAN,
        .ransac_distance_px        = BRIDGE_RANSAC_DISTANCE_PX,
        .row_step                  = BRIDGE_ROW_STEP,
        .stable_pixel_count        = BRIDGE_STABLE_PIXEL_COUNT,
        .single_edge_hold_frames   = BRIDGE_SINGLE_EDGE_HOLD_FRAMES,
        .lost_hold_frames          = BRIDGE_LOST_HOLD_FRAMES,
        .lane_width_filter_alpha   = BRIDGE_LANE_WIDTH_FILTER_ALPHA
    };                                                // 单边桥识别参数结构体
    CameraBridgeAlignParams_t bridge_align_params =
    {
        .target_center_x          = BRIDGE_TARGET_CENTER_X,
        .far_tolerance_px         = BRIDGE_ALIGN_FAR_TOLERANCE_PX,
        .near_tolerance_px        = BRIDGE_ALIGN_NEAR_TOLERANCE_PX,
        .fallback_far_tolerance_px   = BRIDGE_FALLBACK_FAR_TOLERANCE_PX,
        .fallback_near_tolerance_px  = BRIDGE_FALLBACK_NEAR_TOLERANCE_PX,
        .fallback_timeout_ms          = BRIDGE_FALLBACK_TIMEOUT_MS,
        .fallback_estimated_grace_frames = BRIDGE_FALLBACK_ESTIMATED_GRACE_FRAMES,
        .control_deadband_d10     = BRIDGE_CONTROL_DEADBAND_D10,
        .lookahead_min_px         = BRIDGE_LOOKAHEAD_MIN_PX,
        .lookahead_max_px         = BRIDGE_LOOKAHEAD_MAX_PX,
        .lookahead_full_angle_d10 = BRIDGE_LOOKAHEAD_FULL_ANGLE_D10,
        .complete_confirm_frames  = BRIDGE_ALIGN_COMPLETE_CONFIRM_FRAMES,
        .lost_reset_frames        = BRIDGE_ALIGN_LOST_RESET_FRAMES,
        .near_enter_angle_d10     = BRIDGE_NEAR_ENTER_ANGLE_D10,
        .near_exit_angle_d10      = BRIDGE_NEAR_EXIT_ANGLE_D10,
        .near_line_filter_alpha   = BRIDGE_NEAR_LINE_FILTER_ALPHA,
        .near_yaw_gain            = BRIDGE_NEAR_YAW_GAIN,
        .near_control_deadband_d10 = BRIDGE_NEAR_CONTROL_DEADBAND_D10,
        .near_yaw_slew_limit_d10 = BRIDGE_NEAR_YAW_SLEW_LIMIT_D10,
        .near_control_gain_per_deg = BRIDGE_NEAR_CONTROL_GAIN_PER_DEG,
        .near_reverse_confirm_frames = BRIDGE_NEAR_REVERSE_CONFIRM_FRAMES,
        .line_filter_alpha        = BRIDGE_LINE_FILTER_ALPHA,
        .yaw_gain                 = BRIDGE_YAW_GAIN,
        .yaw_direction            = BRIDGE_YAW_DIRECTION,
        .yaw_offset_limit_d10     = BRIDGE_YAW_OFFSET_LIMIT_D10,
        .yaw_slew_limit_d10       = BRIDGE_ALIGN_YAW_SLEW_LIMIT_D10,
        .control_gain_per_deg     = BRIDGE_CONTROL_GAIN_PER_DEG,
        .control_direction        = BRIDGE_CONTROL_DIRECTION,
        .control_limit            = BRIDGE_CONTROL_LIMIT,
        .blind_trigger_angle_d10  = BRIDGE_BLIND_TRIGGER_ANGLE_D10,
        .blind_turn_time_ms       = BRIDGE_BLIND_TURN_TIME_MS,
        .blind_edge_margin_px     = BRIDGE_BLIND_EDGE_MARGIN_PX,
        .blind_reliable_frames    = BRIDGE_BLIND_RELIABLE_FRAMES,
        .blind_estimated_frames   = BRIDGE_BLIND_ESTIMATED_FRAMES,
        .blind_control_percent    = BRIDGE_BLIND_CONTROL_PERCENT
    };                                                // 单边桥对准控制参数结构体
    CameraBridgeAlignState_t bridge_align_state = {0}; // 单边桥对准控制运行状态
    CameraBridgeResult_t bridge_result = {0};          // 单边桥识别结果结构体
    CameraBridgeAlignResult_t bridge_align_result = {0}; // 单边桥对准控制结果结构体
    CameraWifiOverlay_t wifi_overlay =
    {
        .type                = CAMERA_WIFI_OVERLAY_NONE,
        .jump_params         = &jump_params,
        .bridge_params       = &bridge_params,
        .bridge_result       = &bridge_result,
        .bridge_align_params = &bridge_align_params
    };                                                // WiFi SPI 图像叠加参数
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
    uint8  bridge_align_updated  = 0;                 // 当前帧对准控制计算是否有效
    uint8  force_blind_request   = 0;                 // 请求核心0使用 IMU 强制完成大角度盲转
    uint8  blind_release_request = 0;                 // 请求核心0提前结束强制盲转
    uint8  force_blind_monitoring = 0;                // 是否正在监测强制盲转后的可靠中线
    uint8  blind_release_count   = 0;                 // 中线夹角连续满足提前结束条件的帧数
    uint8  fresh_target_count    = 0;                 // 连续获得新鲜赛道目标的帧数
    uint8  fresh_target          = 0;                 // 当前是否已经确认重新找到赛道目标
    uint8  blind_phase_seen      = 0;                 // 当前盲转阶段是否已经判断过强制盲转条件
    uint32 force_blind_start_ms  = 0;                 // 强制盲转请求开始时间
    int16  last_reliable_heading_d10 = 0;             // 最近一次新鲜双边线的中线角度
    int16  blind_heading_abs_d10 = 0;                 // 进入盲转时中线角度的绝对值
    int16  release_heading_abs_d10 = 0;               // 当前可靠中线夹角的绝对值
    char txt[192];                                    // 串口发送文本

    clock_init(SYSTEM_CLOCK_250M);                    // 系统 初始化
    
    #if IMAGE_DEBUG_TYPE == 1
    // screen_init();                                 // TFT180 在首个处理帧显示时延迟初始化
    #endif

    camera_init();                                    // MT9V03X 摄像头初始化
    // camera_wifi_spi_init(WIFI_SSID, WIFI_PWD, TARGET_IP, TARGET_PORT, LOCAL_PORT); // WiFi SPI 图传初始化
    pit_ms_init(PIT_CH1, 1);                          // PIT_CH1 1ms周期中断，用于 sys_ms 计时
    appipc_speed_rx_init(appipc_speed_callback);      // IPC接收 初始化
    camera_fps_counter_init(&camera_fps, sys_ms);     // 独立 FPS 计算初始化
    camera_bridge_align_reset(&bridge_align_state);  // 单边桥对准控制状态初始化


    while(true)
    {
        bridge_params.binary_threshold = core0_remote_ch9_value;  // 固定阈值模式使用，自动阈值模式下作为备用值
        bridge_exit_params.binary_threshold = core0_remote_ch9_value;
        bump_exit_params.binary_threshold = core0_remote_ch9_value;

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
                    force_blind_request = 0;
                    blind_release_request = 0;
                    force_blind_monitoring = 0;
                    blind_release_count = 0;
                    fresh_target_count = 0;
                    fresh_target = 0;
                    blind_phase_seen = 0;
                    force_blind_start_ms = 0;
                    last_reliable_heading_d10 = 0;
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
                // 根据中线预瞄目标计算转向控制量
                independent_fps = camera_fps_counter_update(&camera_fps, sys_ms);  // 独立 FPS 计算
                bridge_align_updated = camera_bridge_align_update(
                    sys_ms, &bridge_result, &bridge_align_params, &bridge_align_state, &bridge_align_result);

                // 只把连续获得的新鲜双边线结果作为倒车恢复结束条件
                if(bridge_result.valid && !bridge_result.estimated)
                {
                    if(fresh_target_count < BRIDGE_FRESH_TARGET_CONFIRM_FRAMES)
                    {
                        fresh_target_count++;
                    }
                }
                else
                {
                    fresh_target_count = 0;
                }
                fresh_target = (uint8)(
                    fresh_target_count >= BRIDGE_FRESH_TARGET_CONFIRM_FRAMES);

                if(bridge_result.valid && !bridge_result.estimated &&
                   (CAMERA_BRIDGE_ALIGN_BLIND_TURN != bridge_align_result.phase))
                {
                    last_reliable_heading_d10 = bridge_align_result.heading_error_d10;
                }

                if(CAMERA_BRIDGE_ALIGN_BLIND_TURN == bridge_align_result.phase)
                {
                    if(!blind_phase_seen)
                    {
                        blind_phase_seen = 1;
                        blind_heading_abs_d10 = last_reliable_heading_d10;
                        if(blind_heading_abs_d10 < 0)
                        {
                            blind_heading_abs_d10 = -blind_heading_abs_d10;
                        }

                        force_blind_request = (uint8)(
                            blind_heading_abs_d10 >= BRIDGE_FORCE_BLIND_TRIGGER_ANGLE_D10);

                        if(force_blind_request)
                        {
                            force_blind_monitoring = 1;
                            blind_release_count = 0;
                            force_blind_start_ms = sys_ms;
                        }
                    }
                }
                else
                {
                    blind_phase_seen = 0;
                    force_blind_request = 0;
                }

                // 强制盲转期间持续观察新鲜中线，夹角稳定进入范围后允许视觉提前接管
                if(force_blind_monitoring &&
                   ((sys_ms - force_blind_start_ms) >= BRIDGE_BLIND_RELEASE_MIN_TIME_MS))
                {
                    if(bridge_align_updated &&
                       bridge_result.valid &&
                       !bridge_result.estimated &&
                       (CAMERA_BRIDGE_ALIGN_BLIND_TURN != bridge_align_result.phase))
                    {
                        release_heading_abs_d10 = bridge_align_result.heading_error_d10;
                        if(release_heading_abs_d10 < 0)
                        {
                            release_heading_abs_d10 = -release_heading_abs_d10;
                        }

                        if(release_heading_abs_d10 <= BRIDGE_BLIND_RELEASE_ANGLE_D10)
                        {
                            if(blind_release_count < BRIDGE_BLIND_RELEASE_CONFIRM_FRAMES)
                            {
                                blind_release_count++;
                            }

                            if(blind_release_count >= BRIDGE_BLIND_RELEASE_CONFIRM_FRAMES)
                            {
                                blind_release_request = 1;
                                force_blind_monitoring = 0;
                                blind_release_count = 0;
                            }
                        }
                        else if(release_heading_abs_d10 >= BRIDGE_BLIND_RELEASE_RESET_D10)
                        {
                            blind_release_count = 0;
                        }
                    }
                    else
                    {
                        // 丢失、补线或内部盲转结果不能计入连续可靠帧
                        blind_release_count = 0;
                    }
                }

                // 入桥 IPC 发送部分
                ipc_result = appipc_send_bridge_data(
                    bridge_align_result.valid, bridge_align_result.aligned, 0,
                    (uint8)bridge_align_result.bottom_y, bridge_align_result.control_value,
                    force_blind_request, blind_release_request, fresh_target);

                // 同时输出识别层和控制层状态，便于定位数据在哪一层失效
                sprintf(txt, "Det %d |Est %d |Aret %d |Valid %d |Align %d |B %d |C %d,%d-%d,%d |E %d,%d |U %d |Type %s |Near %d |Blind %d |Force %d |Rel %d |Fresh %d |F %d\r\n",
                        bridge_result.valid, bridge_result.estimated, bridge_align_updated,
                        bridge_align_result.valid, bridge_align_result.aligned,
                        bridge_align_result.bottom_y,
                        bridge_result.center_x1, bridge_result.center_y1,
                        bridge_result.center_x2, bridge_result.center_y2,
                        bridge_align_result.heading_error_d10, bridge_align_result.lateral_error_px,
                        bridge_align_result.control_value,
                        bridge_align_phase_text(bridge_align_result.phase),
                        bridge_align_result.near_mode,
                        (CAMERA_BRIDGE_ALIGN_BLIND_TURN == bridge_align_result.phase),
                        force_blind_request,
                        blind_release_request,
                        fresh_target,
                        independent_fps);
                // wireless_uart_send_string(txt);

                if((APPIPC_OK == ipc_result) && blind_release_request)
                {
                    blind_release_request = 0;
                }
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
                    bridge_exit_ipc_sent = (APPIPC_OK == appipc_send_bridge_data(0, 0, 1, 0, 0, 0, 0, 0)) ? 1 : 0;  // 发送一次后就不再发送
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
                    bump_exit_ipc_sent = (APPIPC_OK == appipc_send_bridge_data(0, 0, 1, 0, 0, 0, 0, 0)) ? 1 : 0;
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
//                jump_params.check_row = camera_jump_check_row_from_speed(core0_car_speed, ADAPTIVE_ROW_COEFF);
                 jump_params.check_row = 80;
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
                    // wireless_uart_send_string(txt);
                }
            }
        }
        // 统一视觉更新
        debug_image_screen_display(jump_params, &bridge_params, &bridge_result, &bridge_align_params, bridge_exit_params, independent_fps, core0_car_speed);
        // wifi_overlay.type = CAMERA_WIFI_OVERLAY_NONE;
        // if(function_option == VISION_JUMP)
        // {
        //     wifi_overlay.type = CAMERA_WIFI_OVERLAY_JUMP;
        // }
        // else if((function_option == VISION_BRIDGE_BUMP) &&
        //         (vision_phase_bab == VISION_PHASE_BAB_BRIDGE_ALIGN))
        // {
        //     wifi_overlay.type = CAMERA_WIFI_OVERLAY_BRIDGE;
        // }
        // camera_debug_on_wifi_spi(CAMERA_WIFI_IMAGE_SEND_DIV_DEFAULT, &wifi_overlay);
    }
}

