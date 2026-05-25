#ifndef CAMERA_H
#define CAMERA_H

#include "zf_common_headfile.h"
#include "zf_device_mt9v03x.h"


/**
 * @brief 查询摄像头是否采集到新的一帧图像。
 *
 * 本函数直接返回 MT9V03X 驱动中的 `mt9v03x_finish_flag`。
 *
 * @return uint8
 *         - 0：当前没有新帧；
 *         - 非 0：当前已有一帧新图像可处理。
 *
 * @note 该函数只查询标志位，不会清除标志位。
 */
uint8 camera_has_frame(void);


/**
 * @brief 发送一帧摄像头图像到逐飞助手上位机。
 *
 * 如果检测到 `mt9v03x_finish_flag` 为 1，函数会：
 * 1. 清除帧完成标志；
 * 2. 将 `mt9v03x_image` 复制到内部图像副本 `image_copy`；
 * 3. 对图像副本做固定阈值二值化；
 * 4. 执行黑色/白色孤立噪点过滤；
 * 5. 配置逐飞助手摄像头图像信息；
 * 6. 通过逐飞助手协议发送图像；
 * 7. 已发送帧计数加 1。
 *
 * @note 发送的是二值化后的图像副本，不会直接修改原始 `mt9v03x_image`。
 * @note 调用本函数前，应先初始化逐飞助手通信接口，例如无线串口接口。
 * @note 无线串口带宽有限，发送整帧图像可能会明显阻塞主循环。
 */
void camera_send_frame(void);


/**
 * @brief 获取已经发送的摄像头帧数。
 *
 * @return uint32 已通过 `camera_send_frame()` 发送的帧数量。
 *
 * @note 该计数只在 `camera_send_frame()` 实际发送图像后增加。
 */
uint32 camera_get_frame_count(void);


/**
 * @brief 初始化 MT9V03X 摄像头模块。
 *
 * 该函数会初始化错误提示 LED，并持续尝试初始化 MT9V03X 摄像头。
 *
 * @note 如果摄像头初始化失败，函数会一直重试并慢速闪烁 LED。
 * @note 本函数不初始化屏幕、无线串口或逐飞助手协议接口。
 */
void camera_init(void);

/**
 * @brief 处理一帧摄像头图像并显示到 IPS200 屏幕，用于图像调试观察。
 *
 * 如果检测到摄像头有新帧，本函数会先复制新图像到内部副本，并完成：
 * 1. 大津法自动阈值二值化；
 * 2. 黑色/白色孤立噪点过滤；
 * 3. 将处理后的图像显示到 IPS200 指定位置。
 *
 * 如果当前没有新帧，但内部已经有处理过的图像副本，本函数会继续显示最近一次处理结果。
 *
 * @param x               图像显示区域左上角 x 坐标。
 * @param y               图像显示区域左上角 y 坐标。
 * @param display_width   图像显示宽度。
 * @param display_height  图像显示高度。
 *
 * @return void
 *
 * @note 本函数只负责屏幕显示调试，不进行跳跃检测，也不进行无线发送。
 * @note 若需要同时检测与显示，建议先调用 `camera_processing()`，再调用本函数显示同一帧处理结果。
 * @note 本函数会显示处理后的二值图像，原始 `mt9v03x_image` 不会被直接修改。
 */
void camera_debug_on_screen(uint16 x, uint16 y, uint16 display_width, uint16 display_height);


/**
 * @brief 处理一帧摄像头图像并返回跳跃检测结果。
 *
 * 如果检测到摄像头有新帧，本函数会：
 * 1. 清除帧完成标志；
 * 2. 复制 `mt9v03x_image` 到内部图像副本；
 * 3. 使用大津法自动阈值完成二值化；
 * 4. 执行黑色/白色孤立噪点过滤；
 * 5. 调用 `camera_image_check_jump_area()` 判断矩形区域内黑色像素总数是否达到阈值。
 *
 * @param row             跳跃检测的起始行纵向坐标。
 * @param row_total       从起始行开始向上检查的行数。
 * @param colum           跳跃检测的起始列横向坐标。
 * @param colum_total     从起始列开始向右检查的列数。
 * @param black_pix_count 矩形检测区域内需要达到的黑色像素总数阈值。
 *
 * @return uint8
 *         - 1：本次检测到新帧，并且检测到跳跃特征；
 *         - 0：当前没有新帧，或本次处理后未检测到跳跃特征。
 *
 * @note 本函数不显示图像，也不进行无线发送。
 * @note 本函数只有在检测到新帧时才会执行处理；如果没有新帧，直接返回 0。
 * @note 本函数会在内部图像副本上处理，原始 `mt9v03x_image` 不会被直接修改。
 */
uint8 camera_processing(uint16 row, uint16 row_total, uint16 colum, uint16 colum_total, uint16 black_pix_count);
#endif
