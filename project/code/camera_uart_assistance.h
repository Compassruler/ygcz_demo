#ifndef CAMERA_UART_ASSISTANCE_H
#define CAMERA_UART_ASSISTANCE_H

#include "zf_common_typedef.h"

/**
 * @brief 初始化摄像头调参使用的上位机串口助手通道。
 *
 * 该函数会完成以下工作：
 * 1. 初始化无线串口模块；
 * 2. 将逐飞助手的数据收发接口绑定到无线串口。
 *
 * @return void
 *
 * @note 本函数只负责上位机调参通信，不负责摄像头初始化、屏幕初始化或图传初始化。
 * @note 使用逐飞助手调参时，不建议在同一无线串口上混发普通字符串，避免破坏协议帧。
 */
void camera_uart_assistance_init(void);


/**
 * @brief 从逐飞助手上位机读取并更新跳跃检测参数。
 *
 * 该函数会调用 `seekfree_assistant_data_analysis()` 解析上位机下发的数据，
 * 并按照以下通道关系更新参数：
 *
 * - 参数通道 1：`check_row`，跳跃检测起始行；
 * - 参数通道 2：`check_row_count`，从起始行开始向上检查的行数；
 * - 参数通道 3：`black_count`，单行需要达到的黑色像素数量阈值。
 *
 * @param check_row       跳跃检测起始行变量地址。
 * @param check_row_count 向上检查行数变量地址。
 * @param black_count     黑色像素数量阈值变量地址。
 *
 * @return uint8
 *         - bit0 为 1：`check_row` 被上位机更新；
 *         - bit1 为 1：`check_row_count` 被上位机更新；
 *         - bit2 为 1：`black_count` 被上位机更新；
 *         - 0：本次没有参数更新，或传入了空指针。
 *
 * @note 函数内部会自动限制参数范围，避免超过 MT9V03X 图像宽高。
 * @note 上位机下发的参数在逐飞助手协议中是 float，本函数会转换为 uint16 使用。
 */
uint8 camera_uart_assistance_update_jump_param(uint16 *check_row, uint16 *check_row_count, uint16 *black_count);

#endif
