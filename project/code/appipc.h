#ifndef _appipc_h_
#define _appipc_h_

#include "zf_common_typedef.h"

// 发送成功
#define APPIPC_OK       (0u)

// IPC 通道忙，上一条数据尚未被接收端释放
#define APPIPC_BUSY     (1u)

// IPC 接收回调函数类型
// 注意：该回调在 IPC 中断中执行，只建议做变量赋值，不要延时或执行复杂外设操作
typedef void (*appipc_callback_t)(uint32 data);

// 初始化 IPC 接收端
// 使用场景：核心0调用，用于接收核心1发送过来的 uint32 数据
// 参数说明：callback 接收到数据后调用的回调函数
void  appipc_rx_init(appipc_callback_t callback);

// 发送一个 uint32 数据
// 使用场景：核心1调用，用于向核心0发送状态量、标志位、计数值等小数据
// 返回值：APPIPC_OK 表示发送成功，APPIPC_BUSY 表示通道忙
uint8 appipc_send_u32(uint32 data);

#endif
