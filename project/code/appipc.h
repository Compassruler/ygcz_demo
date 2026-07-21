#ifndef _appipc_h_
#define _appipc_h_

#include "zf_common_typedef.h"

// 发送成功
#define APPIPC_OK       (0u)

// IPC 通道忙，上一条数据尚未被接收端释放
#define APPIPC_BUSY     (1u)

// 视觉工作模式，与核心0的 vision_detect_mode 保持一致
#define APPIPC_VISION_MODE_IDLE      (0u)
#define APPIPC_VISION_MODE_BRIDGE    (1u)
#define APPIPC_VISION_MODE_JUMP      (2u)

// IPC 接收回调函数类型
// 注意：该回调在 IPC 中断中执行，只建议做变量赋值，不要延时或执行复杂外设操作。
typedef void (*appipc_callback_t)(uint32 data);

// 单边桥视觉数据解包结果
typedef struct
{
    uint8 valid;             // 1 表示当前识别结果有效
    uint8 aligned;           // 1 表示小车已经满足单边桥对齐标准
    uint8 bottom_y;          // 单边桥候选区域最下端纵坐标，用于估算前向距离
    int16 control_value;     // 核心1计算后的底盘航向控制量
} appipc_bridge_data_t;

// 核心0发送给核心1的运行状态
typedef struct
{
    uint16 car_speed;          // 当前车速绝对值
    uint8 vision_detect_mode;  // 0 空闲，1 单边桥，2 跳跃
} appipc_core0_data_t;

// 初始化核心0接收端
// 使用场景：核心0调用，用于接收核心1发送过来的 uint32 跳跃标志或视觉数据。
// 参数说明：callback 接收到数据后调用的回调函数。
void  appipc_rx_init(appipc_callback_t callback);

// 发送通用 uint32 数据
// 使用场景：核心1调用，用于向核心0发送跳跃标志、视觉数据或状态量。
// 返回值：APPIPC_OK 表示发送成功，APPIPC_BUSY 表示通道忙。
uint8 appipc_send_u32(uint32 data);

// 打包并发送单边桥视觉数据
// 数据格式：bit31 为有效标志，bit30 为对齐标志，bit23~16 为 bottom_y，bit15~0 为 int16 控制量。
// valid 为 0 时发送数据 0。
// 返回值：APPIPC_OK 表示发送成功，APPIPC_BUSY 表示通道忙。
uint8 appipc_send_bridge_data(uint8 valid, uint8 aligned, uint8 bottom_y, int16 control_value);

// 解包核心1发送的单边桥视觉数据
// 返回值：1 表示数据有效，0 表示参数为空或数据中的有效标志为 0。
uint8 appipc_decode_bridge_data(uint32 data, appipc_bridge_data_t *bridge_data);

// 初始化核心0状态接收端
// 使用场景：核心1调用，用于接收核心0发送过来的车速和视觉工作模式。
// 参数说明：callback 接收到状态数据后调用的回调函数。
void  appipc_speed_rx_init(appipc_callback_t callback);

// 发送通用 uint32 核心0状态数据
// 返回值：APPIPC_OK 表示发送成功，APPIPC_BUSY 表示通道忙。
uint8 appipc_send_speed_u32(uint32 data);

// 打包并发送核心0状态
// 数据格式：bit23~16 为视觉工作模式，bit15~0 为 uint16 车速绝对值。
// 返回值：APPIPC_OK 表示发送成功，APPIPC_BUSY 表示通道忙。
uint8 appipc_send_core0_data(uint16 car_speed, uint8 vision_detect_mode);

// 解包核心0发送的车速和视觉工作模式
// 返回值：1 表示解包成功，0 表示参数为空或工作模式超出范围。
uint8 appipc_decode_core0_data(uint32 data, appipc_core0_data_t *core0_data);

#endif
