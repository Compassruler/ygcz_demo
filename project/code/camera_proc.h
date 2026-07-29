#ifndef CAMERA_PROC_H
#define CAMERA_PROC_H

#include "camera.h"
#include "zf_device_mt9v03x.h"

#define CAMERA_IMAGE_DOT_BLACK         (0)    // 检测二值图中的黑色像素，像素值为 0
#define CAMERA_IMAGE_DOT_WHITE         (1)    // 检测二值图中的白色像素，像素值为 255
#define CAMERA_DOT_TYPE_LIST_COUNT     (3)    // 跳跃检测像素类型序列长度
#define LANE_MAX_POINT_NUM   800              // 八邻域搜索点数

// ==================================================== 公共函数 ====================================================
/**
 * 对灰度图像进行固定阈值二值化处理，直接修改原数组
 * @param image     待处理的图像数组
 * @param threshold 二值化阈值
 */
void camproc_pub_thresh_bin(uint8 image[MT9V03X_H][MT9V03X_W], uint8 threshold);

// 八邻域搜索得到的赛道像素点结构体
typedef struct
{
    uint16 x;   // 像素点横坐标（图像列方向）
    uint16 y;    // 像素点纵坐标（图像行方向）

}LanePoint_t;

// 函数功能：采用BFS八邻域算法搜索连续黑色赛道区域
// 输入参数：
// image   - 二值化图像
// start_x - 搜索起始点横坐标
// start_y - 搜索起始点纵坐标
// point   - 保存搜索到的赛道像素点
// 返回值：搜索到的像素点数量
uint16 camproc_lane_search_8neighbor(const uint8 image[MT9V03X_H][MT9V03X_W], uint16 start_x, uint16 start_y, LanePoint_t point[]);

// 函数功能：根据赛道像素点计算赛道中心横坐标
// 输入参数：
// point    - 八邻域搜索得到的像素点
// count    - 像素点数量
// center_x - 输出赛道中心坐标
void camproc_lane_center_calculate(LanePoint_t point[], uint16 count, int16 *center_x);


/**
 * 在指定矩形区域内执行指定颜色像素总量检测
 * @param image              待检测的二值图像数组
 * @param check_row          起始检测行的纵向坐标，范围应小于 `MT9V03X_H`
 * @param check_row_count    从起始检测行开始，继续向上检查的行数
 * @param check_column       起始检测列的横向坐标，范围应小于 `MT9V03X_W`
 * @param check_column_count 从起始检测列开始，继续向右检查的列数
 * @param dot_count          整个矩形区域内需要达到的指定颜色像素总数阈值
 * @param dot_type           需要统计的像素类型 CAMERA_IMAGE_DOT_BLACK | CAMERA_IMAGE_DOT_WHITE
 *
 * @return 1 矩形内像素数量达到要求 | 0 像素数量不满足或条件非法
 */
uint8 camproc_pub_check_area(uint8 image[MT9V03X_H][MT9V03X_W], uint16 check_row, uint16 check_row_count, uint16 check_column, uint16 check_column_count, uint32 dot_count, uint32 dot_type);

// ==================================================== 单边桥和颠簸路段函数 ====================================================
/**
 * 使用固定阈值或 ROI 大津法生成单边桥二值图
 * @param image  待处理的灰度图像，处理完成后原地保存二值图
 * @param params 单边桥识别参数结构体
 *
 * @return 1 二值化完成 | 0 参数非法
 */
uint8 camproc_bridge_prepare_binary(uint8 image[MT9V03X_H][MT9V03X_W], const CameraBridgeParams_t *params);

/**
 * 复位自动阈值、赛道宽度模型和临时丢线保持状态
 */
void camproc_bridge_detect_reset(void);

/**
 * 在二值图 ROI 中提取黑白边缘点，使用 RANSAC 拟合左右边线并生成赛道中线
 * @param image  待检测的二值图像数组
 * @param params 单边桥识别参数结构体
 * @param result 识别结果输出结构体，函数会在每次调用开始时清空该结构体
 *
 * @return 1 成功提取左右边线和中线 | 0 未找到或者参数非法
 */
uint8 camproc_bridge_detect(const uint8 image[MT9V03X_H][MT9V03X_W], const CameraBridgeParams_t *params, CameraBridgeResult_t *result);

/**
 * 复位单边桥边线跟踪和对准控制运行状态
 * @param align_state 单边桥对准控制运行状态
 */
void camproc_bridge_align_reset(CameraBridgeAlignState_t *align_state);

/**
 * 根据拟合中线计算自适应预瞄目标与底盘 angle 控制量
 * @param time_ms        当前系统毫秒时间
 * @param bridge_result  单边桥识别结果结构体
 * @param align_params   单边桥对准控制参数结构体
 * @param align_state    单边桥对准控制运行状态
 * @param align_result   单边桥对准控制结果输出地址
 *
 * @note 预瞄点同时包含中线横向位置和方向信息，避免两项控制量相互抵消。
 * @note 接近对准时短时保持航向，大角度丢线前保存可靠控制并定时重新确认。
 * @note 严格条件未触发时，连续处于保底范围达到设定时间也会确认对准。
 * @note 对准完成结果不锁存，后续帧仍会根据当前中线重新判断。
 * @return 1 当前帧控制结果有效 | 0 识别无效或参数非法
 */
uint8 camproc_bridge_align_update(uint32 time_ms, const CameraBridgeResult_t *bridge_result, const CameraBridgeAlignParams_t *align_params, CameraBridgeAlignState_t *align_state, CameraBridgeAlignResult_t *align_result);

/**
 * 颠簸路段离开检测，先确认黑色凸起，再连续确认白色出口
 * @param image              待检测的二值图像数组
 * @param bump_exit_params   颠簸路段离开检测参数结构体
 * @param exit_check_enabled 1 允许判断白色出口 | 0 仅确认黑色凸起
 *
 * @return 1 已确认离开颠簸路段 | 0 尚未离开
 */
uint8 camproc_bump_exit_detect(uint8 image[MT9V03X_H][MT9V03X_W], BumpExitParams_t *bump_exit_params, uint8 exit_check_enabled);

// ==================================================== 跳跃检测、过滤、切换函数 ====================================================
/**
 * 自适应 Row - Speed 对照函数
 * @param car_speed  实际车速
 * @param coeff      调节参数，控制对照结构体中的速度细节
 * 
 * @return Row 最终选择值
 */
uint16 camproc_jump_adaptive_row(uint16 car_speed, int16 coeff);

/**
 * 对跳跃检测结果进行单次触发冷却过滤
 * @param time_ms          当前系统毫秒时间
 * @param cooldown_time_ms 冷却时间，单位 ms
 */
uint8 camproc_jump_cooldown_filter(uint32 time_ms, uint32 cooldown_time_ms);

// 按跳跃触发次数依次切换检测像素类型，并增加计数
uint8 camproc_jump_dot_type_switch(void);

// 获取当前已经触发成功的跳跃次数
uint32 camproc_jump_get_steps(void);

// 复位跳跃检测序列，并返回初始检测像素类型
uint8 camproc_jump_dot_type_reset(void);

#endif
