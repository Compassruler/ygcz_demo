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

// 单边桥识别边线类型
typedef enum
{
    CAMERA_BRIDGE_EDGE_LEFT = 0,   // 识别候选区域左侧边线
    CAMERA_BRIDGE_EDGE_RIGHT       // 识别候选区域右侧边线
} CameraBridgeEdgeType_t;

// 单边桥矩形识别参数
typedef struct
{
    uint8 binary_threshold;         // 固定二值化阈值
    CameraBridgeEdgeType_t edge_type; // 当前需要识别的边线类型
    uint16 search_left;             // 搜索区域最左列
    uint16 search_right;            // 搜索区域最右列
    uint16 search_top;              // 搜索区域最上行
    uint16 search_bottom;           // 搜索区域最下行
    uint16 min_width;               // 候选连通区域的最小横向宽度
    uint16 min_height;              // 候选连通区域及其目标边线的最小纵向高度
    uint16 min_edge_length;         // 拟合边线允许的最小实际长度，单位像素
    uint16 max_edge_length;         // 拟合边线允许的最大实际长度，单位像素
    uint16 min_edge_x;              // 拟合边线端点允许的最小横坐标
    uint16 max_edge_x;              // 拟合边线端点允许的最大横坐标
    uint32 min_area;                // 候选连通区域内黑色像素总数下限
    uint16 connect_gap;             // 相邻两行黑色像素允许的横向连接间隔
    uint16 target_edge_x;           // 当前目标边线期望对齐的横坐标
    uint16 reference_row;           // 固定位置测量行，0 表示使用内部默认行
} CameraBridgeParams_t;

// 单边桥矩形识别结果
typedef struct
{
    uint8 valid;                    // 1 表示识别成功，0 表示当前帧没有有效目标
    uint16 left;                    // 最佳候选连通区域最左坐标
    uint16 right;                   // 最佳候选连通区域最右坐标
    uint16 top;                     // 最佳候选连通区域最上坐标
    uint16 bottom;                  // 最佳候选连通区域最下坐标
    uint32 area;                    // 最佳候选连通区域的黑色像素总数
    uint16 edge_x;                  // 拟合边线在 reference_row 处的横坐标
    int16 distance_px;              // 拟合边线相对 target_edge_x 的有符号像素距离
    int16 angle_d10;                // 拟合边线相对垂直方向的角度，单位 0.1 度
    uint16 edge_x1;                 // 实际边线拟合后的上端点横坐标
    uint16 edge_y1;                 // 实际边线上端点的纵坐标
    uint16 edge_x2;                 // 实际边线拟合后的下端点横坐标
    uint16 edge_y2;                 // 实际边线下端点的纵坐标
    uint16 reference_row;           // 本次计算 edge_x 时使用的参考行
    uint16 edge_point_count;        // 参与边线拟合的轮廓点数量
    float edge_slope;               // 拟合模型 x = edge_slope*y + edge_intercept 的斜率
    float edge_intercept;           // 拟合模型 x = edge_slope*y + edge_intercept 的截距
} CameraBridgeResult_t;

// 单边桥控制换算参数
typedef struct
{
    int16 aligned_angle_d10;        // 小车正确对准时的视觉角度，单位 0.1 度
    int16 aligned_distance_px;      // 小车正确对准时的视觉距离，单位像素
    float angle_gain;               // 角度内环误差增益
    float distance_to_angle_gain;   // 距离外环增益，单位 0.1 度/像素
    float angle_direction;          // 角度内环修正方向
    float distance_to_angle_direction; // 距离误差转换为目标角度的方向
    int16 angle_deadband_d10;       // 角度误差死区，单位 0.1 度
    int16 distance_deadband_px;     // 距离误差死区，单位像素
    int16 target_angle_limit_d10;   // 距离外环生成的目标角度偏移限制，单位 0.1 度
    int16 yaw_offset_limit_d10;     // 航向角修正量限制，单位 0.1 度
    float control_gain_per_deg;     // 每 1 度航向修正转换成的底盘 angle 控制量
    float control_direction;        // 底盘控制方向
    int16 control_limit;            // 底盘 angle 控制量的最大绝对值
} CameraBridgeControlParams_t;

// 单边桥控制换算结果
typedef struct
{
    int16 distance_error_px;        // 应用校准和死区后的距离误差，单位像素
    int16 target_angle_d10;         // 距离外环生成的目标视觉角度，单位 0.1 度
    int16 angle_error_d10;          // 目标角度与实际角度之间的内环误差，单位 0.1 度
    int16 yaw_offset_d10;           // 角度内环生成的航向角修正量，单位 0.1 度
    int16 control_value;            // 最终底盘 angle 控制量
} CameraBridgeControlResult_t;

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
 * @param bridge_result 当前帧的单边桥识别结果输出地址，不进行跨帧滤波
 */
uint8 camera_bridge_processing(const CameraBridgeParams_t *bridge_params, CameraBridgeResult_t *bridge_result);

/**
 * 单边桥控制换算接口
 * @param bridge_result  单边桥识别结果结构体
 * @param control_params 单边桥控制换算参数结构体
 * @param control_result 单边桥控制换算结果输出地址
 *
 * @return 1 换算完成 | 0 识别无效或参数非法
 */
uint8 camera_bridge_calculate_control(const CameraBridgeResult_t *bridge_result, const CameraBridgeControlParams_t *control_params, CameraBridgeControlResult_t *control_result);

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
