#ifndef CAMERA_H
#define CAMERA_H

#include "zf_common_typedef.h"

// TFT180 横屏中的摄像头显示区域
// MT9V03X 为 188x120，等比例缩放到 160x102 后上下各保留约 13 像素
#define IMAGE_X                 (0u)
#define IMAGE_Y                 (13u)
#define IMAGE_DISPLAY_WIDTH     (160u)
#define IMAGE_DISPLAY_HEIGHT    (102u)

// WiFi SPI 图传默认分频：每处理 10 帧发送 1 帧
#define CAMERA_WIFI_IMAGE_SEND_DIV_DEFAULT     (10U)


// =================================== 结构体 ===================================
// 摄像头帧率统计结构体
typedef struct
{
    uint32 last_time_ms;            // 上一次完成 FPS 更新的系统时间，单位 ms
    uint32 frame_count;             // 当前 1 秒统计窗口内已经处理的帧数
    uint32 fps;                     // 最近一次计算得到的 FPS
} fps_counter_t;

// 跳跃检测参数
typedef struct
{
    uint8 binary_threshold;         // 当前统一使用的固定二值化阈值
    uint16 check_row;               // 检测矩形的起始行，后续从该行向上检查
    uint16 check_row_count;         // 从起始行向上检查的行数量
    uint16 check_column;            // 检测矩形的起始列，后续从该列向右检查
    uint16 check_column_count;      // 从起始列向右检查的列数量
    uint16 otsu_roi_row;            // 大津法 ROI 底部行，从该行开始向上取区域
    uint16 otsu_roi_row_count;      // 大津法 ROI 行数
    uint16 otsu_roi_column;         // 大津法 ROI 左侧起始列
    uint16 otsu_roi_column_count;   // 大津法 ROI 列数
    uint32 dot_type;                // 检测像素类型：CAMERA_IMAGE_DOT_BLACK 或 CAMERA_IMAGE_DOT_WHITE
    uint32 dot_count;               // 矩形检测区域内的目标颜色像素总数阈值
    uint32 cooldown_time_ms;        // 跳跃触发后的冷却时间，单位 ms
    uint32 multi_frame;             // 连续检测到目标的帧数阈值，1 表示单帧触发
    uint32 steps;                   // 已执行的识别步骤
} JumpDetectParams_t;

// 单边桥对准控制阶段
typedef enum
{
    CAMERA_BRIDGE_ALIGN_TRACK = 0,   // 根据检测中线持续进行对准
    CAMERA_BRIDGE_ALIGN_BLIND_TURN,  // 边线即将丢失时使用可靠历史控制盲转
    CAMERA_BRIDGE_ALIGN_COMPLETE     // 连续入框或保底条件已经确认对准
} CameraBridgeAlignPhase_t;

// 单边桥赛道边线识别参数
typedef struct
{
    uint8 binary_threshold;         // 固定阈值模式使用的阈值，也是大津法失效时的备用阈值
    uint8 use_otsu_threshold;       // 1 使用 ROI 大津法自动阈值 | 0 使用固定阈值
    float threshold_filter_alpha;   // 自动阈值低通滤波旧值权重，范围 0.0~1.0
    uint16 roi_top;                 // ROI 最上行
    uint16 roi_bottom;              // ROI 最下行
    uint16 roi_left;                // ROI 最左列
    uint16 roi_right;               // ROI 最右列
    uint16 min_lane_width;          // 左右边线之间允许的最小宽度
    uint16 max_lane_width;          // 左右边线之间允许的最大宽度
    uint16 ransac_iterations;       // 每条边线执行的 RANSAC 采样次数
    uint16 min_line_point_count;    // 单条有效拟合边线所需的最少内点数量
    uint16 min_y_span;              // 单条有效边线需要覆盖的最小纵向范围
    uint8 ransac_distance_px;       // RANSAC 内点到候选直线的最大距离
    uint8 row_step;                 // 纵向采样行间隔，数值越大处理速度越快
    uint8 stable_pixel_count;       // 跳变两侧必须连续保持黑色和白色的像素数
    uint8 single_edge_hold_frames;  // 暂时只识别到单边线时允许使用历史赛道宽度的帧数
    uint8 lost_hold_frames;         // 双边线完全丢失后允许保持上一结果的帧数
    float lane_width_filter_alpha;  // 赛道宽度模型低通滤波旧值权重，范围 0.0~1.0
} CameraBridgeParams_t;

// 单边桥赛道边线识别结果
typedef struct
{
    uint8 valid;                    // 1 表示识别成功，0 表示当前帧没有有效目标
    uint8 estimated;                // 1 表示当前结果由单边补线或短时丢线保持得到
    uint16 top;                     // 当前有效路径最上方的纵坐标
    uint16 bottom;                  // 当前有效路径最下方的纵坐标
    uint16 point_count;             // 左右拟合线中较少一侧的 RANSAC 内点数量
    uint16 left_x1;                 // 左拟合线上端横坐标
    uint16 left_y1;                 // 左拟合线上端纵坐标
    uint16 left_x2;                 // 左拟合线下端横坐标
    uint16 left_y2;                 // 左拟合线下端纵坐标
    uint16 right_x1;                // 右拟合线上端横坐标
    uint16 right_y1;                // 右拟合线上端纵坐标
    uint16 right_x2;                // 右拟合线下端横坐标
    uint16 right_y2;                // 右拟合线下端纵坐标
    uint16 center_x1;               // 中线上端横坐标
    uint16 center_y1;               // 中线上端纵坐标
    uint16 center_x2;               // 中线下端横坐标
    uint16 center_y2;               // 中线下端纵坐标
} CameraBridgeResult_t;

// 单边桥对准控制参数
typedef struct
{
    uint16 target_center_x;          // 车辆视觉中心对应的目标横坐标
    uint16 far_tolerance_px;         // 中线上端允许的横向误差
    uint16 near_tolerance_px;        // 中线下端允许的横向误差
    uint16 fallback_far_tolerance_px;   // 保底冲刺允许的中线上端横向误差
    uint16 fallback_near_tolerance_px;  // 保底冲刺允许的中线下端横向误差
    uint16 fallback_timeout_ms;          // 连续处于保底范围后强制完成对准的时间
    uint8 fallback_estimated_grace_frames; // 保底计时允许连续出现的估算结果帧数
    uint16 control_deadband_d10;     // 预瞄航向角控制死区，单位 0.1 度
    uint16 lookahead_min_px;         // 大角度时使用的最小预瞄距离
    uint16 lookahead_max_px;         // 小角度时使用的最大预瞄距离
    uint16 lookahead_full_angle_d10; // 达到最小预瞄距离对应的中线角度，单位 0.1 度
    uint8 complete_confirm_frames;   // 完成对准所需的连续新鲜入框帧数
    uint8 lost_reset_frames;         // 连续丢失目标后复位对准过程的帧数
    uint16 near_enter_angle_d10;     // 进入近对准模式的最大中线角度，单位 0.1 度
    uint16 near_exit_angle_d10;      // 退出近对准模式的中线角度，单位 0.1 度
    float near_line_filter_alpha;    // 近对准模式中线滤波旧值权重，范围 0.0~1.0
    float near_yaw_gain;             // 近对准模式预瞄航向角增益
    uint16 near_control_deadband_d10; // 近对准模式控制死区，单位 0.1 度
    int16 near_yaw_slew_limit_d10;   // 近对准模式每帧航向修正量最大变化
    float near_control_gain_per_deg; // 近对准模式每 1 度对应的底盘控制量
    uint8 near_reverse_confirm_frames; // 近对准模式反向控制所需连续确认帧数
    float line_filter_alpha;         // 中线模型低通滤波旧值权重，范围 0.0~1.0
    float yaw_gain;                  // 预瞄航向角增益
    float yaw_direction;             // 预瞄航向角修正方向
    int16 yaw_offset_limit_d10;      // 航向角修正量限制，单位 0.1 度
    int16 yaw_slew_limit_d10;        // 每帧航向修正量允许的最大变化，单位 0.1 度
    float control_gain_per_deg;      // 每 1 度航向修正转换成的底盘 angle 控制量
    float control_direction;         // 底盘控制方向
    int16 control_limit;             // 底盘 angle 控制量的最大绝对值
    uint16 blind_trigger_angle_d10;   // 允许进入盲转准备的最小中线角度，单位 0.1 度
    uint16 blind_turn_time_ms;        // 单次盲转持续时间
    uint8 blind_edge_margin_px;       // 边线接近画幅边界时允许触发盲转的距离
    uint8 blind_reliable_frames;      // 保存盲转控制前所需的连续可靠帧数
    uint8 blind_estimated_frames;     // 连续使用补线结果后触发盲转的帧数
    uint8 blind_control_percent;      // 盲转控制量占可靠视觉控制量的百分比
} CameraBridgeAlignParams_t;

// 单边桥对准控制运行状态
typedef struct
{
    CameraBridgeAlignPhase_t phase; // 内部对准阶段，完成后保持锁存
    uint8 complete_frame_count;     // 新鲜中线连续进入对准框的帧数
    uint8 fallback_timer_active;    // 保底冲刺计时器是否已经启动
    uint8 fallback_estimated_count; // 保底计时期间连续出现的估算结果帧数
    uint8 lost_frame_count;         // 连续丢失有效中线的帧数
    uint8 line_filter_initialized;  // 中线模型滤波值是否已经初始化
    uint8 near_mode_active;         // 当前是否正在使用近对准控制参数
    uint8 near_reverse_count;       // 当前反向控制已经连续确认的帧数
    int8 near_control_sign;         // 近对准模式最近接受的非零控制方向
    int8 near_pending_sign;         // 当前等待确认的反向控制方向
    uint8 blind_reliable_count;     // 当前连续获得可靠大角度控制的帧数
    uint8 blind_estimated_count;    // 当前连续使用单边补线结果的帧数
    uint32 phase_start_time_ms;     // 当前盲转阶段的开始时间
    uint32 fallback_start_time_ms;  // 连续进入保底范围的开始时间
    float filtered_slope;           // 滤波后的中线 x/y 斜率
    float filtered_intercept;       // 滤波后的中线截距
    int16 previous_yaw_offset_d10;  // 上一帧输出的航向修正量
    int16 previous_control_value;   // 上一帧输出的底盘控制量
    int16 blind_reference_control;  // 连续可靠视觉帧得到的盲转参考控制量
    int16 blind_control_value;      // 当前盲转阶段实际使用的底盘控制量
    uint16 held_bottom_y;           // 保持或盲转阶段沿用的路径下端纵坐标
} CameraBridgeAlignState_t;

// 单边桥对准控制结果
typedef struct
{
    uint8 valid;                    // 1 表示当前对准控制结果有效
    uint8 point_inside;             // 1 表示中线上下端点均位于红色对准框内
    uint8 aligned;                  // 1 表示连续入框或保底条件已经满足
    uint8 near_mode;                // 1 表示当前使用近对准控制参数
    CameraBridgeAlignPhase_t phase; // 当前帧的对准判断阶段
    uint16 active_x;                // 当前预瞄目标点横坐标
    uint16 active_y;                // 当前预瞄目标点纵坐标
    uint16 lookahead_px;            // 当前根据中线角度选择的预瞄距离
    int16 heading_error_d10;        // 中线相对车辆前向的角度误差，单位 0.1 度
    int16 lateral_error_px;         // 中线在图像底部的横向位置误差
    int16 lookahead_error_px;       // 预瞄目标点的横向误差
    int16 yaw_offset_d10;           // 预瞄误差生成的航向修正量，单位 0.1 度
    int16 control_value;            // 最终底盘 angle 控制量
    uint16 bottom_y;                // 当前控制阶段提供给核心0的路径下端纵坐标
} CameraBridgeAlignResult_t;

// 单边桥离开检测阶段
typedef enum
{
    CAMERA_BRIDGE_EXIT_WAIT_WHITE = 0,  // 等待检测区域连续出现白色
    CAMERA_BRIDGE_EXIT_WAIT_BLACK,      // 白色已经确认，等待检测区域变为黑色
    CAMERA_BRIDGE_EXIT_COMPLETE         // 已经确认进入颠簸路段
} CameraBridgeExitStage_t;

// 单边桥离开检测参数
typedef struct
{
    uint8 binary_threshold;         // 固定二值化阈值
    uint16 check_row;               // 检测矩形的起始行，后续从该行向上检查
    uint16 check_row_count;         // 从起始行向上检查的行数量
    uint16 check_column;            // 检测矩形的起始列，后续从该列向右检查
    uint16 check_column_count;      // 从起始列向右检查的列数量
    uint32 white_dot_count;         // 确认白色区域所需的白色像素数量
    uint32 black_dot_count;         // 确认颠簸路段所需的黑色像素数量
    uint8 white_confirm_frames;     // 确认白色区域所需的连续帧数
    uint8 white_frame_count;        // 当前连续检测到白色区域的帧数
    CameraBridgeExitStage_t stage;  // 当前离桥视觉检测阶段
    uint8 exited;                   // 1 表示常规离桥流程或全黑保底判断已经确认离桥
} BridgeExitParams_t;

// WiFi SPI 图像叠加类型
typedef enum
{
    CAMERA_WIFI_OVERLAY_NONE = 0,       // 只显示二值图像
    CAMERA_WIFI_OVERLAY_JUMP,           // 显示跳跃检测矩形
    CAMERA_WIFI_OVERLAY_BRIDGE          // 显示单边桥 ROI、对准框和拟合线
} CameraWifiOverlayType_t;

// WiFi SPI 图像叠加参数；结构体只保存引用，不复制视觉参数和结果
typedef struct
{
    CameraWifiOverlayType_t type;
    const JumpDetectParams_t *jump_params;
    const CameraBridgeParams_t *bridge_params;
    const CameraBridgeResult_t *bridge_result;
    const CameraBridgeAlignParams_t *bridge_align_params;
} CameraWifiOverlay_t;


// =================================== 函数 ===================================
// 摄像头基础接口
uint8 camera_has_frame(void);           // 是否有新帧
void camera_init(void);                 // 初始化摄像头模块 如果摄像头初始化失败，函数会一直重试并慢速闪烁 LED

// 屏幕调试接口
uint8 camera_debug_on_screen(void);     // 有新处理帧时显示图像；1 已刷新 | 0 没有新帧

/**
 * 初始化 WiFi SPI 图传，并通过 TCP 将逐飞助手连接到上位机。
 * @param wifi_ssid   WiFi 名称
 * @param pass_word   WiFi 密码；无密码时可传入 NULL
 * @param target_ip   上位机 IP 地址字符串
 * @param target_port 上位机端口字符串
 * @param local_port  WiFi 模块本地端口字符串
 *
 * @return 0 初始化成功 | 非 0 WiFi 或 TCP 连接失败
 */
uint8 camera_wifi_spi_init(char *wifi_ssid, char *pass_word, char *target_ip, char *target_port, char *local_port);

/**
 * 将最近一次处理完成的二值图像压缩后通过 WiFi SPI 发送到逐飞助手。
 * 图像按 1bit/像素打包，模块繁忙时直接跳过当前发送帧，不阻塞视觉算法。
 * 可使用逐飞助手 XY 边线协议叠加检测框、赛道边线、拟合中线等调试元素。
 * 边线颜色由逐飞助手上位机分配，嵌入端协议不提供 RGB 颜色设置。
 * 本函数不采集新帧也不重复执行视觉算法，同一处理帧只会参与一次发送分频。
 * @param send_div 发送分频；0 或 1 表示每个处理帧都发送，N 表示每 N 帧发送 1 帧
 * @param overlay  叠加参数；传入 NULL 或 CAMERA_WIFI_OVERLAY_NONE 时只发送图像
 */
void camera_debug_on_wifi_spi(uint16 send_div, const CameraWifiOverlay_t *overlay);

// 独立 FPS 计算接口
void camera_fps_counter_init(fps_counter_t *counter, uint32 time_ms);       // FPS 计算初始化
uint32 camera_fps_counter_update(fps_counter_t *counter, uint32 time_ms);   // 更新 FPS 计数器

// 跳跃检测参数接口
uint16 camera_jump_check_row_from_speed(uint16 car_speed, int8 aggressive_coeff); // 根据车速计算跳跃检测行

// 检测接口
/**
 * 单边桥检测接口
 * @param bridge_params 单边桥识别参数结构体
 * @param bridge_result 当前帧左右边线及中线检测结果输出地址
 *
 * @return 1 已处理一个新帧 | 0 当前没有新帧或参数无效
 */
uint8 camera_bridge_processing(const CameraBridgeParams_t *bridge_params, CameraBridgeResult_t *bridge_result);

/**
 * 复位单边桥边线跟踪和对准控制运行状态
 * @param align_state 单边桥对准控制运行状态
 */
void camera_bridge_align_reset(CameraBridgeAlignState_t *align_state);

/**
 * 根据拟合中线生成自适应预瞄目标并计算单边桥对准控制量
 * @param time_ms        当前系统毫秒时间
 * @param bridge_result  单边桥识别结果结构体
 * @param align_params   单边桥对准控制参数结构体
 * @param align_state    单边桥对准控制运行状态
 * @param align_result   单边桥对准控制结果输出地址
 *
 * @note 预瞄点同时包含横向位置与中线方向，避免两个独立控制量相互抵消。
 * @note 接近对准时短时保持航向，大角度丢线前保存可靠控制并定时重新确认。
 * @note 严格条件未触发时，连续处于保底范围达到设定时间也会确认对准。
 * @note 对准完成结果不锁存，后续帧仍会根据当前中线重新判断。
 * @return 1 当前帧控制结果有效 | 0 识别无效或参数非法
 */
uint8 camera_bridge_align_update(uint32 time_ms, const CameraBridgeResult_t *bridge_result, const CameraBridgeAlignParams_t *align_params, CameraBridgeAlignState_t *align_state, CameraBridgeAlignResult_t *align_result);

/**
 * 根据赛道拟合中线持续计算航向控制量
 * @param time_ms        当前系统毫秒时间
 * @param bridge_result  当前帧赛道边线及中线识别结果
 * @param align_params   中线跟踪控制参数
 * @param align_state    中线跟踪控制运行状态
 * @param align_result   中线跟踪控制结果输出地址
 *
 * @note 仅输出实时控制量，不判断是否对齐，也不进入盲转状态。
 * @return 1 当前帧控制结果有效 | 0 识别无效或参数非法
 */
uint8 camera_lane_follow_update(uint32 time_ms, const CameraBridgeResult_t *bridge_result, const CameraBridgeAlignParams_t *align_params, CameraBridgeAlignState_t *align_state, CameraBridgeAlignResult_t *align_result);

/**
 * 检查当前已处理二值帧中指定矩形区域的白色像素数量
 * @param check_row          检测矩形最下方行坐标
 * @param check_row_count    从起始行向上检测的行数
 * @param check_column       检测矩形最左侧列坐标
 * @param check_column_count 从起始列向右检测的列数
 * @param white_dot_count    触发所需的白色像素数量
 *
 * @note 该函数不获取新帧，应在摄像头处理接口成功后对同一张二值图调用。
 * @return 1 白色像素数量达到阈值 | 0 未达到阈值
 */
uint8 camera_processed_white_area_check(uint16 check_row, uint16 check_row_count,
                                        uint16 check_column, uint16 check_column_count,
                                        uint32 white_dot_count);

/**
 * 检查当前已处理二值帧中的单边桥保底离开区域是否全部为黑色
 * @param bridge_exit_params 单边桥离开检测参数结构体，复用其中的检测矩形
 *
 * @note 该函数不获取新帧，应在摄像头处理接口成功后对同一张二值图调用。
 * @return 1 检测矩形全部为黑色 | 0 检测矩形未全部变黑或参数无效
 */
uint8 camera_bridge_failsafe_exit_check(const BridgeExitParams_t *bridge_exit_params);

/**
 * 单边桥离开检测接口
 * 同一检测区域先连续确认白色，再检测大面积黑色并锁存离桥状态；区域全黑时直接保底确认离桥。
 * @param bridge_exit_params 单边桥离开检测参数结构体
 *
 * @return 1 已经确认离桥 | 0 尚未完成或当前没有新帧
 */
uint8 camera_bridge_exit_processing(BridgeExitParams_t *bridge_exit_params);

/**
 * 跳跃检测接口
 * @param time_ms     当前系统毫秒时间，用于跳跃冷却判断
 * @param jump_params 跳跃检测参数结构体指针
 *
 * @return 1 跳跃 | 0 不跳跃
 */
uint8 camera_jump_processing(uint32 time_ms, JumpDetectParams_t *jump_params);

#endif
