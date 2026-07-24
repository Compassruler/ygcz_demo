#include "camera.h"
#include "camera_proc.h"
#include "screen.h"
#include "zf_device_mt9v03x.h"
#include "zf_driver_delay.h"
#include "zf_driver_gpio.h"
#include <string.h>

#define LED1                                 (P19_0)                     // 摄像头初始化失败蓝色 LED
#define CAMERA_BINARY_THRESHOLD_DEFAULT      (100u)                      // 默认 固定二值化 阈值

static uint8 image_copy[MT9V03X_H][MT9V03X_W];  // 复制的帧，所有后续图像处理均使用该帧

// 摄像头新帧复制并进行最基本二值化处理
static uint8 camera_frame_cpy_and_basic_processing(uint8 threshold)
{
    // 是否采集到一个新帧
    if(!mt9v03x_finish_flag)
    {
        return 0;
    }
    mt9v03x_finish_flag = 0;

    // 复制新帧到 image_copy
    memcpy(image_copy[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);

    // 图像最基本二值化处理
    camproc_pub_thresh_bin(image_copy, threshold);

    return 1;
}

uint8 camera_has_frame(void)
{
    return mt9v03x_finish_flag; // 直接读取逐飞封装的 新帧标志位
}

void camera_init(void)
{
    // 初始化摄像头失败，则闪烁蓝色 LED
    gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL);

    while(mt9v03x_init())
    {
        gpio_toggle_level(LED1);
        system_delay_ms(500);
    }
}

void camera_debug_on_screen(void)
{
    screen_show_camera_image(IMAGE_X, IMAGE_Y, image_copy[0], IMAGE_DISPLAY_WIDTH, IMAGE_DISPLAY_HEIGHT);
}

void camera_fps_counter_init(fps_counter_t *counter, uint32 time_ms)
{
    // 如果没有传入有效结构体
    if(NULL == counter)
    {
        return;
    }

    // 所有相关数据清零恢复
    counter->last_time_ms = time_ms;
    counter->frame_count = 0;
    counter->fps = 0;
}

uint32 camera_fps_counter_update(fps_counter_t *counter, uint32 time_ms)
{
    // 如果没有传入有效结构体
    if(NULL == counter)
    {
        return 0;
    }

    // 帧计数++
    counter->frame_count++;

    // 现在的时间 - 上一次计算时间 大于 1 秒，完成一次 FPS 计算，结构体写入数据
    if(time_ms - counter->last_time_ms >= 1000)
    {
        counter->last_time_ms = time_ms;
        counter->fps = counter->frame_count;
        counter->frame_count = 0;
    }

    // 返回 FPS
    return counter->fps;
}

uint16 camera_jump_check_row_from_speed(uint16 car_speed, int8 aggressive_coeff)
{
    return camproc_jump_adaptive_row(car_speed, aggressive_coeff);
}

uint8 camera_bridge_processing(const CameraBridgeParams_t *bridge_params, CameraBridgeResult_t *bridge_result)
{
    // 如果没有有效结构体
    if((NULL == bridge_params) || (NULL == bridge_result))
    {
        return 0;
    }

    // 是否有效进行 摄像头新帧复制并进行最基本二值化处理
    if(!camera_frame_cpy_and_basic_processing(bridge_params->binary_threshold))
    {
        return 0;
    }

    camproc_bridge_detect(image_copy, bridge_params, bridge_result);

    return 1;  // 返回的不是成功识别单边桥，这里最终的结果在 CameraBridgeResult_t 中
}

void camera_bridge_align_reset(CameraBridgeAlignState_t *align_state)
{
    camproc_bridge_align_reset(align_state);
}

uint8 camera_bridge_align_update(const CameraBridgeResult_t *bridge_result, const CameraBridgeAlignParams_t *align_params, CameraBridgeAlignState_t *align_state, CameraBridgeAlignResult_t *align_result)
{
    return camproc_bridge_align_update(bridge_result, align_params, align_state, align_result);
}

uint8 camera_bridge_exit_processing(BridgeExitParams_t *bridge_exit_params)
{
    uint8 white_detected = 0;  // 当前帧的白色像素是否达到阈值

    // 如果没有有效结构体
    if(NULL == bridge_exit_params)
    {
        return 0;
    }

    // 离开状态锁存后始终返回 1，便于 IPC 发送失败时继续重试
    if(bridge_exit_params->exited)
    {
        return 1;
    }

    // 是否有效进行 摄像头新帧复制并进行最基本二值化处理
    if(!camera_frame_cpy_and_basic_processing(bridge_exit_params->binary_threshold))
    {
        return 0;
    }

    // 固定统计矩形区域内的白色像素总数
    white_detected = camproc_pub_check_area(
        image_copy,
        bridge_exit_params->check_row,
        bridge_exit_params->check_row_count,
        bridge_exit_params->check_column,
        bridge_exit_params->check_column_count,
        bridge_exit_params->white_dot_count,
        CAMERA_IMAGE_DOT_WHITE
    );

    // 任意一帧不满足要求，连续帧计数立即清零
    if(!white_detected)
    {
        bridge_exit_params->continuous_frame_count = 0;
        return 0;
    }

    if(bridge_exit_params->continuous_frame_count < bridge_exit_params->confirm_frame_count)
    {
        bridge_exit_params->continuous_frame_count++;
    }

    if(bridge_exit_params->continuous_frame_count >= bridge_exit_params->confirm_frame_count)
    {
        bridge_exit_params->exited = 1;
    }

    return bridge_exit_params->exited;
}

uint8 camera_bump_exit_processing(BumpExitParams_t *bump_exit_params, uint8 exit_check_enabled)
{
    if(NULL == bump_exit_params)
    {
        return 0;
    }

    // 离开状态锁存后始终返回 1，便于 IPC 发送失败时继续重试
    if(bump_exit_params->exited)
    {
        return 1;
    }

    if(!camera_frame_cpy_and_basic_processing(bump_exit_params->binary_threshold))
    {
        return 0;
    }

    return camproc_bump_exit_detect(image_copy, bump_exit_params, exit_check_enabled);
}

uint8 camera_jump_processing(uint32 time_ms, JumpDetectParams_t *jump_params)
{
    // 如果没有有效结构体
    if(NULL == jump_params)
    {
        return 0;
    }

    uint8 jump_detected = 0;                                 // 内部跳跃检测标志，不是最终的返回值
    static uint32 multi_frame_count = 0;                     // 多帧计数
    uint32 required_frame_count = jump_params->multi_frame;  // 需要的多帧检测数量

    // 如果已执行的跳跃识别次数大于跳跃检测类型列表，不再检测
    if(jump_params->steps >= CAMERA_DOT_TYPE_LIST_COUNT)
    {
        mt9v03x_finish_flag = 0;    // 丢弃当前新帧，避免标志位一直保持为 1
        multi_frame_count = 0;
        return 0;
    }

    // 是否有效进行 摄像头新帧复制并进行最基本二值化处理
    if(!camera_frame_cpy_and_basic_processing(CAMERA_BINARY_THRESHOLD_DEFAULT))
    {
        return 0;
    }

    // 矩形检测：统计指定区域内的黑色或白色像素总数
    jump_detected = camproc_pub_check_area(
        image_copy,
        jump_params->check_row,
        jump_params->check_row_count,
        jump_params->check_column,
        jump_params->check_column_count,
        jump_params->dot_count,
        jump_params->dot_type
    );

    // 多帧确认要求连续检测到目标，期间只要有一帧未检测到就重新计数
    if(!jump_detected)
    {
        multi_frame_count = 0;
        return 0;
    }

    multi_frame_count++;

    if(multi_frame_count < required_frame_count)
    {
        return 0;
    }

    multi_frame_count = 0;

    // 通过多帧确认后，再进入触发冷却过滤，避免连续重复触发
    jump_detected = camproc_jump_cooldown_filter(time_ms, jump_params->cooldown_time_ms);

    // 最终跳跃确认后
    if(jump_detected)
    {
        jump_params->dot_type = camproc_jump_dot_type_switch();  // 切换检测点类型，跳跃计数++
        jump_params->steps = camproc_jump_get_steps();           // 更新跳跃计数

        return 1;
    }

    return 0;
}
