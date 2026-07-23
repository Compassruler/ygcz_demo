#ifndef CAMERA_PROC_H
#define CAMERA_PROC_H

#include "camera.h"
#include "zf_device_mt9v03x.h"

#define CAMERA_IMAGE_DOT_BLACK         (0)    // 检测二值图中的黑色像素，像素值为 0
#define CAMERA_IMAGE_DOT_WHITE         (1)    // 检测二值图中的白色像素，像素值为 255
#define CAMERA_DOT_TYPE_LIST_COUNT     (3)    // 跳跃检测像素类型序列长度

// ==================================================== 公共函数 ====================================================
/**
 * 对灰度图像进行固定阈值二值化处理，直接修改原数组
 * @param image     待处理的图像数组
 * @param threshold 二值化阈值
 */
void camproc_pub_thresh_bin(uint8 image[MT9V03X_H][MT9V03X_W], uint8 threshold);

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

/**
 * 颠簸路段离开检测，先确认黑色凸起，再连续确认白色出口
 * @param image              待检测的二值图像数组
 * @param bump_exit_params   颠簸路段离开检测参数结构体
 * @param exit_check_enabled 1 允许判断白色出口 | 0 仅确认黑色凸起
 *
 * @return 1 已确认离开颠簸路段 | 0 尚未离开
 */
uint8 camproc_bump_exit_detect(uint8 image[MT9V03X_H][MT9V03X_W], BumpExitParams_t *bump_exit_params, uint8 exit_check_enabled);

// ==================================================== 单边桥函数 ====================================================
/**
 * 从二值图中搜索多个黑色连通区域，将候选轮廓简化为四边形，并按参数拟合左侧或右侧边线
 * @param image  待检测的二值图像数组
 * @param params 单边桥识别参数结构体
 * @param result 识别结果输出结构体，函数会在每次调用开始时清空该结构体
 *
 * @return 1 成功找到并拟合 | 0 未找到或者参数非法
 */
uint8 camproc_bridge_detect(const uint8 image[MT9V03X_H][MT9V03X_W], const CameraBridgeParams_t *params, CameraBridgeResult_t *result);

/**
 * 根据单边桥识别结果计算底盘 angle 控制量
 * @param bridge_result  单边桥识别结果结构体
 * @param control_params 单边桥控制换算参数结构体
 * @param control_result 单边桥控制换算结果输出地址
 *
 * @return 1 换算完成 | 0 识别无效或参数非法
 */
uint8 camproc_bridge_calc_ctrl(const CameraBridgeResult_t *bridge_result, const CameraBridgeControlParams_t *control_params, CameraBridgeControlResult_t *control_result);

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
