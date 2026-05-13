#ifndef CAMERA_H
#define CAMERA_H

#include "zf_common_headfile.h"
#include "zf_device_mt9v03x.h"

/**
 * @brief 初始化摄像头无线传图模块。
 *
 * 该函数会完成以下工作：
 * 1. 初始化错误提示 LED；
 * 2. 初始化无线串口模块；
 * 3. 初始化逐飞助手传输接口，并指定使用无线串口；
 * 4. 初始化 MT9V03X 摄像头；
 * 5. 配置逐飞助手摄像头图像发送参数。
 *
 * @note 如果无线串口初始化失败，函数会进入死循环并快速闪烁 LED。
 * @note 如果摄像头初始化失败，函数会一直重试并慢速闪烁 LED。
 */
void camera_wireless_init(void);


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
uint8 camera_wireless_has_frame(void);


/**
 * @brief 发送一帧摄像头图像到逐飞助手上位机。
 *
 * 如果检测到 `mt9v03x_finish_flag` 为 1，函数会：
 * 1. 清除帧完成标志；
 * 2. 将 `mt9v03x_image` 复制到内部图像副本 `image_copy`；
 * 3. 对图像副本做固定阈值二值化；
 * 4. 通过逐飞助手协议发送图像；
 * 5. 已发送帧计数加 1。
 *
 * @note 发送的是二值化后的图像副本，不会直接修改原始 `mt9v03x_image`。
 * @note 无线串口带宽有限，发送整帧图像可能会明显阻塞主循环。
 */
void camera_wireless_send_frame(void);


/**
 * @brief 获取已经发送的摄像头帧数。
 *
 * @return uint32 已通过 `camera_wireless_send_frame()` 发送的帧数量。
 *
 * @note 该计数只在实际发送图像后增加。
 */
uint32 camera_wireless_get_frame_count(void);


/**
 * @brief 初始化摄像头屏幕显示模块。
 *
 * 该函数只初始化屏幕和 MT9V03X 摄像头，不初始化无线串口和逐飞助手。
 * 适用于放弃上位机图传、改用 IPS200 屏幕查看处理后图像的场景。
 *
 * @note 如果摄像头初始化失败，函数会一直重试并慢速闪烁 LED。
 */
void camera_wireless_screen_init(void);

/**
 * @brief 处理一帧摄像头图像并显示到 IPS200 屏幕。
 *
 * 如果检测到摄像头有新帧，本函数会：
 * 1. 清除帧完成标志；
 * 2. 复制 `mt9v03x_image` 到内部图像副本；
 * 3. 使用固定阈值完成二值化；
 * 4. 执行黑色/白色孤立噪点过滤；
 * 5. 将处理后的图像显示到 IPS200 指定位置。
 *
 * @param x              图像显示区域左上角 x 坐标。
 * @param y              图像显示区域左上角 y 坐标。
 * @param display_width  图像显示宽度。
 * @param display_height 图像显示高度。
 * @param threshold      固定二值化阈值。
 *
 * @return uint8
 *         - 1：本次检测到新帧并完成显示；
 *         - 0：当前没有新帧，未刷新屏幕。
 *
 * @note 本函数不进行无线发送，不会调用 `seekfree_assistant_camera_send()`。
 */
uint8 camera_wireless_show_processed_frame_on_screen(uint16 x, uint16 y, uint16 display_width, uint16 display_height, uint8 threshold);
#endif
