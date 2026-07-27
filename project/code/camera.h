#ifndef CAMERA_H
#define CAMERA_H

#include "zf_common_typedef.h"

// 屏幕显示坐标
#define IMAGE_X                 (0)
#define IMAGE_Y                 (120)
#define IMAGE_DISPLAY_WIDTH     (188)
#define IMAGE_DISPLAY_HEIGHT    (120)


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
    CAMERA_BRIDGE_ALIGN_TRACK = 0,   // 根据拟合中线持续进行对准
    CAMERA_BRIDGE_ALIGN_COMPLETE     // 远近中线点均已完成对准
} CameraBridgeAlignPhase_t;

// 单边桥赛道边线识别参数
typedef struct
{
    uint16 search_top;              // 搜索区域最上行
    uint16 search_bottom;           // 搜索区域最下行
    uint16 left_edge_min_x;         // 左边线允许搜索的最小横坐标
    uint16 left_edge_max_x;         // 左边线允许搜索的最大横坐标
    uint16 right_edge_min_x;        // 右边线允许搜索的最小横坐标
    uint16 right_edge_max_x;        // 右边线允许搜索的最大横坐标
    uint16 min_lane_width;          // 左右边线之间允许的最小宽度
    uint16 max_lane_width;          // 左右边线之间允许的最大宽度
    uint16 max_edge_jump;           // 相邻行锚点边线允许的最大横向变化
    uint16 min_point_count;         // 拟合所需的最少有效行锚点数量
    uint16 min_y_span;              // 有效边线点需要覆盖的最小纵向范围
    uint16 center_residual_limit;   // 中线拟合允许的最大横向残差
    uint16 width_residual_limit;    // 半宽拟合允许的最大横向残差
    uint8 row_step;                 // 相邻行锚点之间的行间隔
    uint8 edge_window;              // 单侧灰度均值使用的像素窗口宽度
    uint8 min_edge_contrast;        // 黑白边缘两侧所需的最小平均灰度差
    uint8 local_search_radius;      // 上一帧模型附近的局部搜索半径
    uint8 max_missing_rows;         // 路径允许连续缺失的行锚点数量
} CameraBridgeParams_t;

// 单边桥赛道边线识别结果
typedef struct
{
    uint8 valid;                    // 1 表示识别成功，0 表示当前帧没有有效目标
    uint16 top;                     // 当前有效路径最上方的纵坐标
    uint16 bottom;                  // 当前有效路径最下方的纵坐标
    uint16 point_count;             // 参与最终拟合的有效行锚点数量
    uint16 left_x1;                 // 左拟合边线上端点横坐标
    uint16 left_y1;                 // 左拟合边线上端点纵坐标
    uint16 left_x2;                 // 左拟合边线下端点横坐标
    uint16 left_y2;                 // 左拟合边线下端点纵坐标
    uint16 right_x1;                // 右拟合边线上端点横坐标
    uint16 right_y1;                // 右拟合边线上端点纵坐标
    uint16 right_x2;                // 右拟合边线下端点横坐标
    uint16 right_y2;                // 右拟合边线下端点纵坐标
    uint16 center_x1;               // 中线拟合后的上端点横坐标
    uint16 center_y1;               // 中线拟合后的上端点纵坐标
    uint16 center_x2;               // 中线拟合后的下端点横坐标
    uint16 center_y2;               // 中线拟合后的下端点纵坐标
    float center_slope;             // 中线模型 x = center_slope*y + center_intercept
    float center_intercept;         // 中线模型截距
    float half_width_slope;         // 赛道半宽模型 half_width = slope*y + intercept
    float half_width_intercept;     // 赛道半宽模型截距
    float center_error;             // 中线最终拟合均方误差
    float width_error;              // 半宽最终拟合均方误差
} CameraBridgeResult_t;

// 单边桥对准控制参数
typedef struct
{
    uint16 target_center_x;         // 赛道中线期望对齐的横坐标
    uint16 lookahead_row;           // 计算转向控制误差的前视行
    uint16 far_check_row;           // 判断远处中线是否对齐的纵坐标
    uint16 near_check_row;          // 判断近处中线是否对齐的纵坐标
    uint16 far_tolerance_px;        // 远处中线允许的横向误差
    uint16 near_tolerance_px;       // 近处中线允许的横向误差
    uint16 control_deadband_px;     // 前视点控制误差死区
    uint8 complete_confirm_frames;  // 远近中线点完成对准的连续确认帧数
    uint8 lost_reset_frames;        // 连续丢失目标后复位对准过程的帧数
    float point_filter_alpha;       // 目标点横坐标低通滤波旧值权重，范围 0.0~1.0
    float point_gain_d10_per_px;    // 每像素误差产生的航向修正量，单位 0.1 度/像素
    float point_direction;          // 目标点横向误差修正方向
    int16 yaw_offset_limit_d10;     // 航向角修正量限制，单位 0.1 度
    int16 yaw_slew_limit_d10;       // 每帧航向修正量允许的最大变化，单位 0.1 度
    float control_gain_per_deg;     // 每 1 度航向修正转换成的底盘 angle 控制量
    float control_direction;        // 底盘控制方向
    int16 control_limit;            // 底盘 angle 控制量的最大绝对值
} CameraBridgeAlignParams_t;

// 单边桥对准控制运行状态
typedef struct
{
    CameraBridgeAlignPhase_t phase; // 当前对准控制阶段
    uint8 complete_frame_count;     // 整条线已经连续满足要求的帧数
    uint8 lost_frame_count;         // 连续丢失有效拟合线的帧数
    uint8 point_filter_initialized; // 目标点滤波值是否已经初始化
    float filtered_point_x;         // 滤波后的当前目标点横坐标
    int16 previous_yaw_offset_d10;  // 上一帧输出的航向修正量
} CameraBridgeAlignState_t;

// 单边桥对准控制结果
typedef struct
{
    uint8 valid;                    // 1 表示当前对准控制结果有效
    uint8 point_inside;             // 1 表示远近两个中线检查点均位于容差内
    uint8 aligned;                  // 1 表示已经完成赛道中线对准
    CameraBridgeAlignPhase_t phase; // 当前对准控制阶段
    uint16 active_x;                // 当前前视中线点横坐标
    uint16 active_y;                // 当前前视中线点纵坐标
    int16 point_error_px;           // 前视中线点相对目标中心的有符号横向误差
    int16 yaw_offset_d10;           // 目标点误差生成的航向修正量，单位 0.1 度
    int16 control_value;            // 最终底盘 angle 控制量
} CameraBridgeAlignResult_t;

// 单边桥离开检测参数
typedef struct
{
    uint8 binary_threshold;         // 固定二值化阈值
    uint16 check_row;               // 检测矩形的起始行，后续从该行向上检查
    uint16 check_row_count;         // 从起始行向上检查的行数量
    uint16 check_column;            // 检测矩形的起始列，后续从该列向右检查
    uint16 check_column_count;      // 从起始列向右检查的列数量
    uint32 white_dot_count;         // 矩形检测区域内的白色像素总数阈值
    uint8 confirm_frame_count;      // 确认离开所需的连续有效帧数
    uint8 continuous_frame_count;   // 当前已经连续检测到白色的帧数
    uint8 exited;                   // 1 表示已经确认离开单边桥
} BridgeExitParams_t;

// 颠簸路段离开检测参数
typedef struct
{
    uint8 binary_threshold;             // 固定二值化阈值
    uint16 check_row;                   // 检测矩形的起始行，后续从该行向上检查
    uint16 check_row_count;             // 从起始行向上检查的行数量
    uint16 check_column;                // 检测矩形的起始列，后续从该列向右检查
    uint16 check_column_count;          // 从起始列向右检查的列数量
    uint32 black_dot_count;             // 确认看到黑色凸起所需的黑色像素数量
    uint32 white_dot_count;             // 确认驶出颠簸路段所需的白色像素数量
    uint8 black_confirm_frame_count;    // 确认看到黑色凸起所需的连续帧数
    uint8 white_confirm_frame_count;    // 确认驶出所需的连续白色帧数
    uint8 black_continuous_frame_count; // 当前连续检测到黑色凸起的帧数
    uint8 white_continuous_frame_count; // 当前连续检测到白色出口的帧数
    uint8 bump_seen;                    // 1 表示已经确认进入颠簸路段
    uint8 exited;                       // 1 表示已经确认离开颠簸路段
} BumpExitParams_t;


// =================================== 函数 ===================================
// 摄像头基础接口
uint8 camera_has_frame(void);           // 是否有新帧
void camera_init(void);                 // 初始化摄像头模块 如果摄像头初始化失败，函数会一直重试并慢速闪烁 LED

// 屏幕调试接口
void camera_debug_on_screen(void);      // 显示图像

// 独立 FPS 计算接口
void camera_fps_counter_init(fps_counter_t *counter, uint32 time_ms);       // FPS 计算初始化
uint32 camera_fps_counter_update(fps_counter_t *counter, uint32 time_ms);   // 更新 FPS 计数器

// 跳跃检测参数接口
uint16 camera_jump_check_row_from_speed(uint16 car_speed, int8 aggressive_coeff); // 根据车速计算跳跃检测行

// 检测接口
/**
 * 单边桥检测接口
 * @param bridge_params 单边桥识别参数结构体
 * @param bridge_result 当前帧左右边线及中线拟合结果输出地址
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
 * 根据拟合中线前视点计算单边桥对准控制量
 * @param bridge_result  单边桥识别结果结构体
 * @param align_params   单边桥对准控制参数结构体
 * @param align_state    单边桥对准控制运行状态
 * @param align_result   单边桥对准控制结果输出地址
 *
 * @return 1 当前帧控制结果有效 | 0 识别无效或参数非法
 */
uint8 camera_bridge_align_update(const CameraBridgeResult_t *bridge_result, const CameraBridgeAlignParams_t *align_params, CameraBridgeAlignState_t *align_state, CameraBridgeAlignResult_t *align_result);

/**
 * 单边桥离开检测接口
 * 固定统计检测区域内的白色像素，连续满足指定帧数后锁存离开状态。
 * @param bridge_exit_params 单边桥离开检测参数结构体
 *
 * @return 1 已经确认离开单边桥 | 0 尚未离开或当前没有新帧
 */
uint8 camera_bridge_exit_processing(BridgeExitParams_t *bridge_exit_params);

/**
 * 颠簸路段离开检测接口
 * 先确认检测区域内出现黑色凸起，允许出口检测后，再连续检测指定帧数的白色区域。
 * @param bump_exit_params 颠簸路段离开检测参数结构体
 * @param exit_check_enabled 1 允许判断白色出口 | 0 仅确认黑色凸起
 *
 * @return 1 已经确认离开颠簸路段 | 0 尚未离开或当前没有新帧
 */
uint8 camera_bump_exit_processing(BumpExitParams_t *bump_exit_params, uint8 exit_check_enabled);

/**
 * 跳跃检测接口
 * @param time_ms     当前系统毫秒时间，用于跳跃冷却判断
 * @param jump_params 跳跃检测参数结构体指针
 *
 * @return 1 跳跃 | 0 不跳跃
 */
uint8 camera_jump_processing(uint32 time_ms, JumpDetectParams_t *jump_params);

#endif
