#ifndef _appipc_h_
#define _appipc_h_

#include "zf_common_typedef.h"

// 发送成功
#define APPIPC_OK       (0u)

// IPC 通道忙，上一条数据尚未被接收端释放
#define APPIPC_BUSY     (1u)

// IPC 接收回调函数类型
// 注意：该回调在 IPC 中断中执行，只建议做变量赋值，不要延时或执行复杂外设操作。
typedef void (*appipc_callback_t)(uint32 data);

// 单边桥视觉数据解包结果
typedef struct
{
    uint8 valid;             // 1 表示当前识别结果有效
    int8 distance_px;        // 右边线相对目标横坐标的距离，单位像素
    int16 angle_d10;         // 右边线相对垂直方向的角度，单位 0.1 度
} appipc_bridge_data_t;

// 初始化核心0接收端
// 使用场景：核心0调用，用于接收核心1发送过来的 uint32 跳跃标志或视觉数据。
// 参数说明：callback 接收到数据后调用的回调函数。
void  appipc_rx_init(appipc_callback_t callback);

// 发送通用 uint32 数据
// 使用场景：核心1调用，用于向核心0发送跳跃标志、视觉数据或状态量。
// 返回值：APPIPC_OK 表示发送成功，APPIPC_BUSY 表示通道忙。
uint8 appipc_send_u32(uint32 data);

// 打包并发送单边桥视觉数据
// 数据格式：bit31 为有效标志，bit23~16 为 int8 距离，bit15~0 为 int16 角度。
// distance_px 会被限制在 -127~127；valid 为 0 时发送数据 0。
// 返回值：APPIPC_OK 表示发送成功，APPIPC_BUSY 表示通道忙。
uint8 appipc_send_bridge_data(uint8 valid, int16 distance_px, int16 angle_d10);

// 解包核心1发送的单边桥视觉数据
// 返回值：1 表示数据有效，0 表示参数为空或数据中的有效标志为 0。
uint8 appipc_decode_bridge_data(uint32 data, appipc_bridge_data_t *bridge_data);

// 初始化速度接收端
// 使用场景：核心1调用，用于接收核心0发送过来的 uint32 速度数据。
// 参数说明：callback 接收到速度数据后调用的回调函数。
void  appipc_speed_rx_init(appipc_callback_t callback);

// 发送速度数据
// 使用场景：核心0调用，用于向核心1发送车速快照。
// 返回值：APPIPC_OK 表示发送成功，APPIPC_BUSY 表示通道忙。
uint8 appipc_send_speed_u32(uint32 data);

#endif
