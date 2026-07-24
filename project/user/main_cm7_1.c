#include "zf_common_headfile.h"



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
#define BRIDGE_EDGE_TYPE          /*主要修改项*/   (CAMERA_BRIDGE_EDGE_RIGHT)       // 边线选择：CAMERA_BRIDGE_EDGE_LEFT | CAMERA_BRIDGE_EDGE_RIGHT
#define BRIDGE_BINARY_THRESHOLD   /*主要修改项*/   (55)                            // 单边桥识别二值化阈值

#define BRIDGE_SEARCH_LEFT          (0)                               // 搜索区间：左起点
#define BRIDGE_SEARCH_RIGHT         (MT9V03X_W - 1)                   // 搜索区间：右终点
#define BRIDGE_SEARCH_TOP           (0)                               // 搜索区间：上起点
#define BRIDGE_SEARCH_BOTTOM        (MT9V03X_H - 1)                   // 搜索区间：下终点
#define BRIDGE_MIN_WIDTH            (5)                               // 连续黑色线最短长度
#define BRIDGE_MIN_HEIGHT           (5)                               // 连续黑色线上下最低宽度
#define BRIDGE_MIN_AREA             (400)                             // 黑色块最小面积
#define BRIDGE_CONNECT_GAP          (5)                               // 行间黑色矩形错开长度

//=========================== 左边线独立参数 /* 参数尚未调整 */ ===========================
#define BRIDGE_LEFT_MIN_EDGE_LENGTH          (10)              // 拟合边线最小实际长度
#define BRIDGE_LEFT_MAX_EDGE_LENGTH          (100)             // 拟合边线最大实际长度
#define BRIDGE_LEFT_MIN_EDGE_X               (30)              // 拟合边线端点最小横坐标
#define BRIDGE_LEFT_MAX_EDGE_X               (140)             // 拟合边线端点最大横坐标
#define BRIDGE_LEFT_TARGET_EDGE_X            (MT9V03X_W / 2)   // 左边线期望对齐横坐标
#define BRIDGE_LEFT_REFERENCE_ROW            (0)               // 固定测量行，0 使用内部默认值

#define BRIDGE_LEFT_ALIGNED_ANGLE_D10        (5)               // 正确对准时的视觉角度，单位 0.1 度
#define BRIDGE_LEFT_ALIGNED_DISTANCE_PX      (-6)              // 正确对准时的视觉距离，单位像素

#define BRIDGE_LEFT_ANGLE_GAIN               (2.0f)            // 角度内环误差增益
#define BRIDGE_LEFT_DISTANCE_TO_ANGLE_GAIN   (5.0f)            // 距离外环增益，单位 0.1 度/像素
#define BRIDGE_LEFT_ANGLE_DIRECTION          (1.0f)            // 角度内环修正方向
#define BRIDGE_LEFT_DISTANCE_TO_ANGLE_DIRECTION (-1.0f)         // 距离误差转换为目标角度的方向
#define BRIDGE_LEFT_ANGLE_DEADBAND_D10       (10)              // 角度误差死区，单位 0.1 度
#define BRIDGE_LEFT_DISTANCE_DEADBAND_PX     (3)               // 距离误差死区，单位像素
#define BRIDGE_LEFT_TARGET_ANGLE_LIMIT_D10   (60)              // 距离外环目标角度偏移限制，单位 0.1 度
#define BRIDGE_LEFT_YAW_OFFSET_LIMIT_D10     (120)              // 最大航向修正量，单位 0.1 度
#define BRIDGE_LEFT_CONTROL_GAIN_PER_DEG     (5.0f)            // 每 1 度对应的底盘 angle 控制量
#define BRIDGE_LEFT_CONTROL_DIRECTION        (1.0f)            // 底盘控制方向
#define BRIDGE_LEFT_CONTROL_LIMIT            (120)              // 底盘 angle 控制量限制

#define BRIDGE_LEFT_ALIGN_DISTANCE_MIN_PX    (0)             // 判断对齐的最小距离
#define BRIDGE_LEFT_ALIGN_DISTANCE_MAX_PX    (30)               // 判断对齐的最大距离
#define BRIDGE_LEFT_ALIGN_ANGLE_MIN_D10      (-50)            // 判断对齐的最小角度
#define BRIDGE_LEFT_ALIGN_ANGLE_MAX_D10      (50)              // 判断对齐的最大角度

//=========================== 右边线独立参数 ===========================
#define BRIDGE_RIGHT_MIN_EDGE_LENGTH         (10)              // 拟合边线最小实际长度
#define BRIDGE_RIGHT_MAX_EDGE_LENGTH         (100)             // 拟合边线最大实际长度
#define BRIDGE_RIGHT_MIN_EDGE_X              (40)              // 拟合边线端点最小横坐标
#define BRIDGE_RIGHT_MAX_EDGE_X              (140)             // 拟合边线端点最大横坐标
#define BRIDGE_RIGHT_TARGET_EDGE_X           (MT9V03X_W / 2)   // 右边线期望对齐横坐标
#define BRIDGE_RIGHT_REFERENCE_ROW           (30)               // 固定测量行，0 使用内部默认值

#define BRIDGE_RIGHT_ALIGNED_ANGLE_D10       (0)               // 正确对准时的视觉角度，单位 0.1 度
#define BRIDGE_RIGHT_ALIGNED_DISTANCE_PX     (-15)              // 正确对准时的视觉距离，单位像素

#define BRIDGE_RIGHT_ANGLE_GAIN              (2.0f)            // 角度内环误差增益
#define BRIDGE_RIGHT_DISTANCE_TO_ANGLE_GAIN  (5.0f)            // 距离外环增益，单位 0.1 度/像素

#define BRIDGE_RIGHT_ANGLE_DIRECTION         (-1.0f)           // 角度内环修正方向
#define BRIDGE_RIGHT_DISTANCE_TO_ANGLE_DIRECTION (1.0f)         // 距离误差转换为目标角度的方向

#define BRIDGE_RIGHT_ANGLE_DEADBAND_D10      (10)              // 角度误差死区，单位 0.1 度
#define BRIDGE_RIGHT_DISTANCE_DEADBAND_PX    (2)               // 距离误差死区，单位像素

#define BRIDGE_RIGHT_TARGET_ANGLE_LIMIT_D10  (60)              // 距离外环目标角度偏移限制，单位 0.1 度
#define BRIDGE_RIGHT_YAW_OFFSET_LIMIT_D10    (120)              // 最大航向修正量，单位 0.1 度

#define BRIDGE_RIGHT_CONTROL_GAIN_PER_DEG    (5.0f)            // 每 1 度对应的底盘 angle 控制量
#define BRIDGE_RIGHT_CONTROL_DIRECTION       (1.0f)            // 底盘控制方向
#define BRIDGE_RIGHT_CONTROL_LIMIT           (120)              // 底盘 angle 控制量限制

#define BRIDGE_RIGHT_ALIGN_DISTANCE_MIN_PX   (-20)             // 判断对齐的最小距离
#define BRIDGE_RIGHT_ALIGN_DISTANCE_MAX_PX   (-10)               // 判断对齐的最大距离
#define BRIDGE_RIGHT_ALIGN_ANGLE_MIN_D10     (-25)            // 判断对齐的最小角度
#define BRIDGE_RIGHT_ALIGN_ANGLE_MAX_D10     (25)              // 判断对齐的最大角度度

// 根据 BRIDGE_EDGE_TYPE 选用左边线或右边线参数
#define BRIDGE_SIDE_PARAM(left_param, right_param)    ((BRIDGE_EDGE_TYPE == CAMERA_BRIDGE_EDGE_LEFT) ? (left_param) : (right_param))
#define BRIDGE_MIN_EDGE_LENGTH       BRIDGE_SIDE_PARAM(BRIDGE_LEFT_MIN_EDGE_LENGTH,       BRIDGE_RIGHT_MIN_EDGE_LENGTH)
#define BRIDGE_MAX_EDGE_LENGTH       BRIDGE_SIDE_PARAM(BRIDGE_LEFT_MAX_EDGE_LENGTH,       BRIDGE_RIGHT_MAX_EDGE_LENGTH)
#define BRIDGE_MIN_EDGE_X            BRIDGE_SIDE_PARAM(BRIDGE_LEFT_MIN_EDGE_X,            BRIDGE_RIGHT_MIN_EDGE_X)
#define BRIDGE_MAX_EDGE_X            BRIDGE_SIDE_PARAM(BRIDGE_LEFT_MAX_EDGE_X,            BRIDGE_RIGHT_MAX_EDGE_X)
#define BRIDGE_TARGET_EDGE_X         BRIDGE_SIDE_PARAM(BRIDGE_LEFT_TARGET_EDGE_X,         BRIDGE_RIGHT_TARGET_EDGE_X)
#define BRIDGE_REFERENCE_ROW         BRIDGE_SIDE_PARAM(BRIDGE_LEFT_REFERENCE_ROW,         BRIDGE_RIGHT_REFERENCE_ROW)
#define BRIDGE_ALIGNED_ANGLE_D10     BRIDGE_SIDE_PARAM(BRIDGE_LEFT_ALIGNED_ANGLE_D10,     BRIDGE_RIGHT_ALIGNED_ANGLE_D10)
#define BRIDGE_ALIGNED_DISTANCE_PX   BRIDGE_SIDE_PARAM(BRIDGE_LEFT_ALIGNED_DISTANCE_PX,   BRIDGE_RIGHT_ALIGNED_DISTANCE_PX)
#define BRIDGE_ANGLE_GAIN            BRIDGE_SIDE_PARAM(BRIDGE_LEFT_ANGLE_GAIN,            BRIDGE_RIGHT_ANGLE_GAIN)
#define BRIDGE_DISTANCE_TO_ANGLE_GAIN BRIDGE_SIDE_PARAM(BRIDGE_LEFT_DISTANCE_TO_ANGLE_GAIN, BRIDGE_RIGHT_DISTANCE_TO_ANGLE_GAIN)
#define BRIDGE_ANGLE_DIRECTION       BRIDGE_SIDE_PARAM(BRIDGE_LEFT_ANGLE_DIRECTION,       BRIDGE_RIGHT_ANGLE_DIRECTION)
#define BRIDGE_DISTANCE_TO_ANGLE_DIRECTION BRIDGE_SIDE_PARAM(BRIDGE_LEFT_DISTANCE_TO_ANGLE_DIRECTION, BRIDGE_RIGHT_DISTANCE_TO_ANGLE_DIRECTION)
#define BRIDGE_ANGLE_DEADBAND_D10    BRIDGE_SIDE_PARAM(BRIDGE_LEFT_ANGLE_DEADBAND_D10,    BRIDGE_RIGHT_ANGLE_DEADBAND_D10)
#define BRIDGE_DISTANCE_DEADBAND_PX  BRIDGE_SIDE_PARAM(BRIDGE_LEFT_DISTANCE_DEADBAND_PX,  BRIDGE_RIGHT_DISTANCE_DEADBAND_PX)
#define BRIDGE_TARGET_ANGLE_LIMIT_D10 BRIDGE_SIDE_PARAM(BRIDGE_LEFT_TARGET_ANGLE_LIMIT_D10, BRIDGE_RIGHT_TARGET_ANGLE_LIMIT_D10)
#define BRIDGE_YAW_OFFSET_LIMIT_D10  BRIDGE_SIDE_PARAM(BRIDGE_LEFT_YAW_OFFSET_LIMIT_D10,  BRIDGE_RIGHT_YAW_OFFSET_LIMIT_D10)
#define BRIDGE_CONTROL_GAIN_PER_DEG  BRIDGE_SIDE_PARAM(BRIDGE_LEFT_CONTROL_GAIN_PER_DEG,  BRIDGE_RIGHT_CONTROL_GAIN_PER_DEG)
#define BRIDGE_CONTROL_DIRECTION     BRIDGE_SIDE_PARAM(BRIDGE_LEFT_CONTROL_DIRECTION,     BRIDGE_RIGHT_CONTROL_DIRECTION)
#define BRIDGE_CONTROL_LIMIT         BRIDGE_SIDE_PARAM(BRIDGE_LEFT_CONTROL_LIMIT,         BRIDGE_RIGHT_CONTROL_LIMIT)
#define BRIDGE_ALIGN_DISTANCE_MIN_PX BRIDGE_SIDE_PARAM(BRIDGE_LEFT_ALIGN_DISTANCE_MIN_PX, BRIDGE_RIGHT_ALIGN_DISTANCE_MIN_PX)
#define BRIDGE_ALIGN_DISTANCE_MAX_PX BRIDGE_SIDE_PARAM(BRIDGE_LEFT_ALIGN_DISTANCE_MAX_PX, BRIDGE_RIGHT_ALIGN_DISTANCE_MAX_PX)
#define BRIDGE_ALIGN_ANGLE_MIN_D10   BRIDGE_SIDE_PARAM(BRIDGE_LEFT_ALIGN_ANGLE_MIN_D10,   BRIDGE_RIGHT_ALIGN_ANGLE_MIN_D10)
#define BRIDGE_ALIGN_ANGLE_MAX_D10   BRIDGE_SIDE_PARAM(BRIDGE_LEFT_ALIGN_ANGLE_MAX_D10,   BRIDGE_RIGHT_ALIGN_ANGLE_MAX_D10)

//=========================== 单边桥离开检测参数 ===========================
#define BRIDGE_EXIT_BINARY_THRESHOLD   (85)    // 离桥检测固定二值化阈值
#define BRIDGE_EXIT_CHECK_ROW          (100)    // 离桥检测矩形起始行
#define BRIDGE_EXIT_CHECK_ROW_COUNT    (25)     // 从起始行向上检查的行数
#define BRIDGE_EXIT_CHECK_COLUMN       (55)     // 离桥检测矩形起始列
#define BRIDGE_EXIT_CHECK_COLUMN_COUNT (73)     // 从起始列向右检查的列数
#define BRIDGE_EXIT_WHITE_DOT_COUNT    (1400)   // 判断离桥所需的白色像素数量
#define BRIDGE_EXIT_CONFIRM_FRAMES     (5)      // 连续满足要求的帧数
#define BRIDGE_EXIT_CHECK_DELAY_MS     (3000)    // 冲桥后延迟开始离桥检测的时间

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

// IPC 接收核0的车速和视觉工作模式
static void appipc_speed_callback(uint32 data)
{
    appipc_core0_data_t core0_data;

    if(appipc_decode_core0_data(data, &core0_data))
    {
        core0_car_speed = core0_data.car_speed;
        function_option = core0_data.vision_detect_mode;
        vision_phase_bab = core0_data.vision_phase_bab;
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
        .edge_type                = BRIDGE_EDGE_TYPE,
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
        .target_edge_x            = BRIDGE_TARGET_EDGE_X,
        .reference_row            = BRIDGE_REFERENCE_ROW
    };                                                // 单边桥识别参数结构体
    CameraBridgeControlParams_t bridge_control_params =
    {
        .aligned_angle_d10        = BRIDGE_ALIGNED_ANGLE_D10,
        .aligned_distance_px      = BRIDGE_ALIGNED_DISTANCE_PX,
        .angle_gain               = BRIDGE_ANGLE_GAIN,
        .distance_to_angle_gain   = BRIDGE_DISTANCE_TO_ANGLE_GAIN,
        .angle_direction          = BRIDGE_ANGLE_DIRECTION,
        .distance_to_angle_direction = BRIDGE_DISTANCE_TO_ANGLE_DIRECTION,
        .angle_deadband_d10       = BRIDGE_ANGLE_DEADBAND_D10,
        .distance_deadband_px     = BRIDGE_DISTANCE_DEADBAND_PX,
        .target_angle_limit_d10   = BRIDGE_TARGET_ANGLE_LIMIT_D10,
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


    while(true)
    {
        //=========================== 执行单边桥与颠簸路段检测程序 ===========================
        // 核心视觉步骤判断条件，目前有| 0 空闲 | 1 单边桥-颠簸路段 | 2 跳跃 | 后续可能增加的步骤：返回三级台阶
        if (function_option == VISION_BRIDGE_BUMP)
        {
            // 更新单边桥内部状态，并且确定运行时变量状态，主要是计时相关变量
            if(last_vision_phase_bab != vision_phase_bab)
            {
                last_vision_phase_bab = vision_phase_bab;

                // 状态恢复
                if(vision_phase_bab == VISION_PHASE_BAB_BRIDGE_EXIT_CHECK)
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
                // 数据处理与对齐判断部分
                independent_fps = camera_fps_counter_update(&camera_fps, sys_ms);  // 独立 FPS 计算
                camera_bridge_calculate_control(&bridge_result, &bridge_control_params, &bridge_control_result); // 单边桥实际控制换算器
                bridge_aligned = (uint8)(
                    bridge_result.valid &&                                          // 数据有效
                    (BRIDGE_ALIGN_DISTANCE_MIN_PX <= bridge_result.distance_px) &&  // 水平误差 大于 左 阈值
                    (bridge_result.distance_px <= BRIDGE_ALIGN_DISTANCE_MAX_PX) &&  // 水平误差 小于 右 阈值
                    (BRIDGE_ALIGN_ANGLE_MIN_D10 <= bridge_result.angle_d10) &&      // 角度误差 大于 最小阈值
                    (bridge_result.angle_d10 <= BRIDGE_ALIGN_ANGLE_MAX_D10)         // 角度误差 小于 最大阈值
                );  // 单边桥是否对齐判断 5个条件

                // 入桥 IPC 发送部分
                appipc_send_bridge_data(bridge_result.valid, bridge_aligned, 0,
                                        (uint8)bridge_result.bottom, bridge_control_result.control_value);  // IPC 发送数据
                // 串口部分
                sprintf(txt, "Valid %d |Aligned %d |Bottom %d |Distance %d |Angle %d |CtrlAng %d |FPS %d\r\n",
                        bridge_result.valid, bridge_aligned, bridge_result.bottom,
                        bridge_result.distance_px, bridge_result.angle_d10,
                        bridge_control_result.control_value, independent_fps);
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
        debug_image_screen_display(jump_params, &bridge_result, bridge_exit_params, independent_fps, core0_car_speed);
    }
}

