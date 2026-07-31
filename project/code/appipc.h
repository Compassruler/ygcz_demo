#ifndef _appipc_h_
#define _appipc_h_

#include "zf_common_typedef.h"

// 发送成功
#define APPIPC_OK       (0u)

// IPC 通道忙，上一条数据尚未被接收端释放
#define APPIPC_BUSY     (1u)

// 视觉工作模式，与核心0的 vision_detect_mode 保持一致
#define VISION_IDLE           (0u)
#define VISION_BRIDGE_BUMP    (1u)
#define VISION_JUMP           (2u)
#define VISION_BACK           (3u)

// 单边桥与颠簸路段工作子状态，由核心0统一控制并同步给核心1
#define VISION_PHASE_BAB_BRIDGE_ALIGN         (0u)
#define VISION_PHASE_BAB_BRIDGE_EXIT_CHECK    (1u)
#define VISION_PHASE_BAB_BUMP_DISTANCE        (2u)
#define VISION_PHASE_BAB_COMPLETE             (3u)

// IPC 接收回调函数类型
// 注意：该回调在 IPC 中断中执行，只建议做变量赋值，不要延时或执行复杂外设操作。
typedef void (*appipc_callback_t)(uint32 data);

// 单边桥与颠簸路段视觉数据解包结果
typedef struct
{
    uint8 valid;             // 1 表示当前识别结果有效
    uint8 aligned;           // 1 表示小车已经满足单边桥对齐标准
    uint8 exited;            // 1 表示核心1已经确认颠簸路段积分完成，可结束视觉阶段
    uint8 bottom_y;          // 单边桥候选区域最下端纵坐标，用于估算前向距离
    int16 control_value;     // 核心1计算后的底盘航向控制量
    uint8 force_blind;       // 1 表示核心0需要使用 IMU 强制完成大角度盲转
    uint8 blind_release;     // 1 表示可靠视觉已经恢复，可以提前结束强制盲转
    uint8 fresh_target;      // 1 表示连续获得了新鲜且非补线的赛道目标
    uint8 vision_bump_start; // 1 表示视觉已经确认离开单边桥，核心0可以开始颠簸路段距离积分
} appipc_bridge_data_t;

// 核心0发送给核心1的运行状态
typedef struct
{
    uint16 car_speed;          // 当前车速绝对值
    uint8 vision_detect_mode;  // 0 空闲，1 单边桥与颠簸路段，2 跳跃，3 三级台阶返回
    uint8 vision_phase_bab;    // 单边桥与颠簸路段工作子状态
    uint8 vision_binary_threshold; // 当前统一图像二值化阈值，范围 1 ~ 255
    uint8 vision_bump_finish;  // 1 表示核心0距离积分已经确认通过颠簸路段
} appipc_core0_data_t;

// 初始化核心0接收端
// 使用场景：核心0调用，用于接收核心1发送过来的 uint32 跳跃标志或视觉数据。
// 参数说明：callback 接收到数据后调用的回调函数。
void  appipc_rx_init(appipc_callback_t callback);

// 发送通用 uint32 数据
// 使用场景：核心1调用，用于向核心0发送跳跃标志、视觉数据或状态量。
// 返回值：APPIPC_OK 表示发送成功，APPIPC_BUSY 表示通道忙。
uint8 appipc_send_u32(uint32 data);

// 打包并发送单边桥与颠簸路段视觉数据
// 数据格式：bit31 为有效标志，bit30 为对齐标志，bit29 为视觉完成应答标志，bit28 为强制盲转标志，
//           bit27 为提前结束盲转标志，bit26 为新鲜赛道目标标志，bit25 为颠簸积分开始标志，
//           bit23~16 为 bottom_y，bit15~0 为 int16 控制量。
// 视觉完成应答独立于有效标志，仅在核心0积分完成后由核心1返回。
// 返回值：APPIPC_OK 表示发送成功，APPIPC_BUSY 表示通道忙。
uint8 appipc_send_bridge_data(uint8 valid, uint8 aligned, uint8 exited, uint8 bottom_y,
                              int16 control_value, uint8 force_blind,
                              uint8 blind_release, uint8 fresh_target,
                              uint8 vision_bump_start);

// 解包核心1发送的单边桥与颠簸路段视觉数据
// 返回值：1 表示解包成功，0 表示参数为空。
uint8 appipc_decode_bridge_data(uint32 data, appipc_bridge_data_t *bridge_data);

// 初始化核心0状态接收端
// 使用场景：核心1调用，用于接收核心0发送过来的车速、二值化阈值、视觉工作模式和 BAB 子状态。
// 参数说明：callback 接收到状态数据后调用的回调函数。
void  appipc_speed_rx_init(appipc_callback_t callback);

// 发送通用 uint32 核心0状态数据
// 返回值：APPIPC_OK 表示发送成功，APPIPC_BUSY 表示通道忙。
uint8 appipc_send_speed_u32(uint32 data);

// 打包并发送核心0状态
// 数据格式：bit31 为颠簸积分完成标志，bit30~28 为 BAB 子状态，bit27~24 为视觉工作模式，
//           bit23~16 为统一图像二值化阈值，bit15~0 为 uint16 车速绝对值。
// 返回值：APPIPC_OK 表示发送成功，APPIPC_BUSY 表示通道忙。
uint8 appipc_send_core0_data(uint16 car_speed, uint8 vision_detect_mode, uint8 vision_phase_bab,
                             uint8 vision_binary_threshold, uint8 vision_bump_finish);

// 解包核心0发送的车速、二值化阈值、视觉工作模式和 BAB 子状态
// 返回值：1 表示解包成功，0 表示参数为空或状态值超出范围。
uint8 appipc_decode_core0_data(uint32 data, appipc_core0_data_t *core0_data);

#endif
