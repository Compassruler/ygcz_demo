#include "camera.h"
#include "camera_proc.h"
#include "screen.h"
#include "zf_device_mt9v03x.h"
#include "zf_device_wifi_spi.h"
#include "zf_driver_delay.h"
#include "zf_driver_gpio.h"
#include "seekfree_assistant.h"
#include <stddef.h>
#include <string.h>

#define LED1                                 (P19_0)                     // 摄像头初始化失败蓝色 LED
#define CAMERA_BINARY_THRESHOLD_DEFAULT      (100u)                      // 默认 固定二值化 阈值
#define CAMERA_WIFI_IMAGE_WIDTH              ((MT9V03X_W / 8u) * 8u)    // 二值图宽度必须为 8 的整数倍
#define CAMERA_WIFI_IMAGE_HEIGHT             (MT9V03X_H)
#define CAMERA_WIFI_IMAGE_X_OFFSET           ((MT9V03X_W - CAMERA_WIFI_IMAGE_WIDTH) / 2u)
#define CAMERA_WIFI_IMAGE_SIZE               (CAMERA_WIFI_IMAGE_WIDTH * CAMERA_WIFI_IMAGE_HEIGHT / 8u)
#define CAMERA_WIFI_BOUNDARY_MAX             (5u)                        // 保证完整数据包不超过一次 SPI 传输上限
#define CAMERA_WIFI_BOUNDARY_POINT_COUNT     (120u)                      // 每条折线使用 120 个 XY 点

typedef struct
{
    uint8 x[CAMERA_WIFI_BOUNDARY_POINT_COUNT];
    uint8 y[CAMERA_WIFI_BOUNDARY_POINT_COUNT];
} CameraWifiBoundaryPoints_t;

typedef struct
{
    seekfree_assistant_camera_struct information;
    uint8 image[CAMERA_WIFI_IMAGE_SIZE];
    seekfree_assistant_camera_dot_struct boundary_information;
    CameraWifiBoundaryPoints_t boundary[CAMERA_WIFI_BOUNDARY_MAX];
} CameraWifiImagePacket_t;

typedef char CameraWifiPacketSizeCheck[
    (sizeof(CameraWifiImagePacket_t) <= WIFI_SPI_TRANSFER_SIZE) ? 1 : -1];

static uint8 image_copy[MT9V03X_H][MT9V03X_W];  // 复制的帧，所有后续图像处理和屏幕显示均使用该帧
static CameraWifiImagePacket_t camera_wifi_packet;
static uint32 camera_processed_frame_id = 0;     // 最近一次完成基础处理的图像序号
static uint32 camera_screen_last_frame_id = 0;   // TFT180 已经显示的图像序号
static uint32 camera_wifi_last_frame_id = 0;     // WiFi 图传已经检查过的图像序号
static uint16 camera_wifi_frame_count = 0;       // WiFi 图传分频计数
static uint8 camera_wifi_spi_ready = 0;          // WiFi SPI 图传初始化完成标志

// 标记内部图像已经完成基础处理，可供屏幕或 WiFi 图传复用
static void camera_processed_frame_mark(void)
{
    camera_processed_frame_id++;
}

// 将 8bit 二值图压缩为逐飞助手使用的 1bit 二值图，高位像素在前
static void camera_wifi_pack_binary_image(void)
{
    uint16 y = 0;
    uint16 byte_x = 0;
    uint16 pixel_x = 0;
    uint8 bit = 0;
    uint8 packed_pixel = 0;
    uint8 *output = camera_wifi_packet.image;

    for(y = 0; y < CAMERA_WIFI_IMAGE_HEIGHT; y++)
    {
        for(byte_x = 0; byte_x < (CAMERA_WIFI_IMAGE_WIDTH / 8u); byte_x++)
        {
            packed_pixel = 0;
            pixel_x = (uint16)(CAMERA_WIFI_IMAGE_X_OFFSET + byte_x * 8u);

            for(bit = 0; bit < 8u; bit++)
            {
                packed_pixel <<= 1;
                if(image_copy[y][pixel_x + bit])
                {
                    packed_pixel |= 1u;
                }
            }

            *output++ = packed_pixel;
        }
    }
}

// 将摄像头横坐标换算到裁剪后的 WiFi 图像坐标，并自动限制到图像内
static uint8 camera_wifi_map_x(uint16 x)
{
    if(x <= CAMERA_WIFI_IMAGE_X_OFFSET)
    {
        return 0;
    }

    if(x >= CAMERA_WIFI_IMAGE_X_OFFSET + CAMERA_WIFI_IMAGE_WIDTH)
    {
        return CAMERA_WIFI_IMAGE_WIDTH - 1u;
    }

    return (uint8)(x - CAMERA_WIFI_IMAGE_X_OFFSET);
}

static uint8 camera_wifi_map_y(uint16 y)
{
    return (uint8)((y < CAMERA_WIFI_IMAGE_HEIGHT) ? y : (CAMERA_WIFI_IMAGE_HEIGHT - 1u));
}

// 用固定数量的点描述一条直线，便于逐飞助手稳定绘制
static void camera_wifi_boundary_add_line(
    uint8 *boundary_count,
    uint16 x1,
    uint16 y1,
    uint16 x2,
    uint16 y2)
{
    CameraWifiBoundaryPoints_t *boundary = NULL;
    uint16 point = 0;
    int32 x = 0;
    int32 y = 0;

    if((NULL == boundary_count) || (*boundary_count >= CAMERA_WIFI_BOUNDARY_MAX))
    {
        return;
    }

    boundary = &camera_wifi_packet.boundary[*boundary_count];
    for(point = 0; point < CAMERA_WIFI_BOUNDARY_POINT_COUNT; point++)
    {
        x = (int32)x1 +
            ((int32)x2 - (int32)x1) * (int32)point /
            (int32)(CAMERA_WIFI_BOUNDARY_POINT_COUNT - 1u);
        y = (int32)y1 +
            ((int32)y2 - (int32)y1) * (int32)point /
            (int32)(CAMERA_WIFI_BOUNDARY_POINT_COUNT - 1u);
        boundary->x[point] = camera_wifi_map_x((uint16)x);
        boundary->y[point] = camera_wifi_map_y((uint16)y);
    }

    (*boundary_count)++;
}

// 用一组闭合 XY 点描述矩形或梯形
static void camera_wifi_boundary_add_quadrilateral(
    uint8 *boundary_count,
    const uint16 vertex_x[4],
    const uint16 vertex_y[4])
{
    CameraWifiBoundaryPoints_t *boundary = NULL;
    uint16 point = 0;
    uint16 segment = 0;
    uint16 segment_point = 0;
    uint16 next_vertex = 0;
    int32 x = 0;
    int32 y = 0;

    if((NULL == boundary_count) ||
       (NULL == vertex_x) ||
       (NULL == vertex_y) ||
       (*boundary_count >= CAMERA_WIFI_BOUNDARY_MAX))
    {
        return;
    }

    boundary = &camera_wifi_packet.boundary[*boundary_count];
    for(point = 0; point < CAMERA_WIFI_BOUNDARY_POINT_COUNT; point++)
    {
        segment = point / (CAMERA_WIFI_BOUNDARY_POINT_COUNT / 4u);
        segment_point = point % (CAMERA_WIFI_BOUNDARY_POINT_COUNT / 4u);
        next_vertex = (segment + 1u) % 4u;

        x = (int32)vertex_x[segment] +
            ((int32)vertex_x[next_vertex] - (int32)vertex_x[segment]) *
            (int32)segment_point /
            (int32)(CAMERA_WIFI_BOUNDARY_POINT_COUNT / 4u - 1u);
        y = (int32)vertex_y[segment] +
            ((int32)vertex_y[next_vertex] - (int32)vertex_y[segment]) *
            (int32)segment_point /
            (int32)(CAMERA_WIFI_BOUNDARY_POINT_COUNT / 4u - 1u);
        boundary->x[point] = camera_wifi_map_x((uint16)x);
        boundary->y[point] = camera_wifi_map_y((uint16)y);
    }

    (*boundary_count)++;
}

static void camera_wifi_boundary_add_rectangle(
    uint8 *boundary_count,
    uint16 left,
    uint16 top,
    uint16 right,
    uint16 bottom)
{
    const uint16 vertex_x[4] = {left, right, right, left};
    const uint16 vertex_y[4] = {top, top, bottom, bottom};

    camera_wifi_boundary_add_quadrilateral(boundary_count, vertex_x, vertex_y);
}

static void camera_wifi_add_jump_overlay(
    uint8 *boundary_count,
    const JumpDetectParams_t *jump_params)
{
    if((NULL == jump_params) ||
       (jump_params->check_row >= MT9V03X_H) ||
       (0u == jump_params->check_row_count) ||
       (jump_params->check_row_count > jump_params->check_row + 1u) ||
       (jump_params->check_column >= MT9V03X_W) ||
       (0u == jump_params->check_column_count) ||
       (jump_params->check_column_count > MT9V03X_W - jump_params->check_column))
    {
        return;
    }

    camera_wifi_boundary_add_rectangle(
        boundary_count,
        jump_params->check_column,
        (uint16)(jump_params->check_row - jump_params->check_row_count + 1u),
        (uint16)(jump_params->check_column + jump_params->check_column_count - 1u),
        jump_params->check_row
    );
}

static void camera_wifi_add_bridge_overlay(
    uint8 *boundary_count,
    const CameraBridgeParams_t *bridge_params,
    const CameraBridgeResult_t *bridge_result,
    const CameraBridgeAlignParams_t *align_params)
{
    uint16 far_y = 0;
    uint16 near_y = MT9V03X_H - 1u;
    uint16 far_left = 0;
    uint16 far_right = 0;
    uint16 near_left = 0;
    uint16 near_right = 0;
    uint16 vertex_x[4];
    uint16 vertex_y[4];

    // 第一条边线保留给对准范围，使上位机按首通道颜色显示该梯形
    if((NULL != align_params) &&
       (align_params->target_center_x < MT9V03X_W) &&
       (0u != align_params->far_tolerance_px) &&
       (0u != align_params->near_tolerance_px))
    {
        if((NULL != bridge_result) &&
           bridge_result->valid &&
           (bridge_result->center_y1 < bridge_result->center_y2) &&
           (bridge_result->center_y2 < MT9V03X_H))
        {
            far_y = bridge_result->center_y1;
            near_y = bridge_result->center_y2;
        }

        far_left = (align_params->target_center_x > align_params->far_tolerance_px) ?
            (uint16)(align_params->target_center_x - align_params->far_tolerance_px) : 0u;
        far_right = ((uint32)align_params->target_center_x + align_params->far_tolerance_px >= MT9V03X_W) ?
            (MT9V03X_W - 1u) :
            (uint16)(align_params->target_center_x + align_params->far_tolerance_px);
        near_left = (align_params->target_center_x > align_params->near_tolerance_px) ?
            (uint16)(align_params->target_center_x - align_params->near_tolerance_px) : 0u;
        near_right = ((uint32)align_params->target_center_x + align_params->near_tolerance_px >= MT9V03X_W) ?
            (MT9V03X_W - 1u) :
            (uint16)(align_params->target_center_x + align_params->near_tolerance_px);

        vertex_x[0] = far_left;
        vertex_x[1] = far_right;
        vertex_x[2] = near_right;
        vertex_x[3] = near_left;
        vertex_y[0] = far_y;
        vertex_y[1] = far_y;
        vertex_y[2] = near_y;
        vertex_y[3] = near_y;
        camera_wifi_boundary_add_quadrilateral(boundary_count, vertex_x, vertex_y);
    }

    // 第二条边线保留给单边桥 ROI
    if((NULL != bridge_params) &&
       (bridge_params->roi_top < bridge_params->roi_bottom) &&
       (bridge_params->roi_bottom < MT9V03X_H) &&
       (bridge_params->roi_left < bridge_params->roi_right) &&
       (bridge_params->roi_right < MT9V03X_W))
    {
        camera_wifi_boundary_add_rectangle(
            boundary_count,
            bridge_params->roi_left,
            bridge_params->roi_top,
            bridge_params->roi_right,
            bridge_params->roi_bottom
        );
    }

    if((NULL == bridge_result) ||
       !bridge_result->valid ||
       (bridge_result->left_x1 >= MT9V03X_W) ||
       (bridge_result->left_x2 >= MT9V03X_W) ||
       (bridge_result->right_x1 >= MT9V03X_W) ||
       (bridge_result->right_x2 >= MT9V03X_W) ||
       (bridge_result->center_x1 >= MT9V03X_W) ||
       (bridge_result->center_x2 >= MT9V03X_W) ||
       (bridge_result->left_y1 >= MT9V03X_H) ||
       (bridge_result->left_y2 >= MT9V03X_H) ||
       (bridge_result->right_y1 >= MT9V03X_H) ||
       (bridge_result->right_y2 >= MT9V03X_H) ||
       (bridge_result->center_y1 >= MT9V03X_H) ||
       (bridge_result->center_y2 >= MT9V03X_H))
    {
        return;
    }

    camera_wifi_boundary_add_line(
        boundary_count,
        bridge_result->left_x1,
        bridge_result->left_y1,
        bridge_result->left_x2,
        bridge_result->left_y2
    );
    camera_wifi_boundary_add_line(
        boundary_count,
        bridge_result->center_x1,
        bridge_result->center_y1,
        bridge_result->center_x2,
        bridge_result->center_y2
    );
    camera_wifi_boundary_add_line(
        boundary_count,
        bridge_result->right_x1,
        bridge_result->right_y1,
        bridge_result->right_x2,
        bridge_result->right_y2
    );
}

// 生成逐飞助手边线协议，并返回本帧实际需要发送的总字节数
static uint32 camera_wifi_prepare_overlay(const CameraWifiOverlay_t *overlay)
{
    uint8 boundary_count = 0;

    if(NULL != overlay)
    {
        if(CAMERA_WIFI_OVERLAY_JUMP == overlay->type)
        {
            camera_wifi_add_jump_overlay(&boundary_count, overlay->jump_params);
        }
        else if(CAMERA_WIFI_OVERLAY_BRIDGE == overlay->type)
        {
            camera_wifi_add_bridge_overlay(
                &boundary_count,
                overlay->bridge_params,
                overlay->bridge_result,
                overlay->bridge_align_params
            );
        }
    }

    camera_wifi_packet.information.camera_type =
        (uint8)((SEEKFREE_ASSISTANT_BINARY << 5) | boundary_count);

    if(0u == boundary_count)
    {
        return (uint32)offsetof(CameraWifiImagePacket_t, boundary_information);
    }

    camera_wifi_packet.boundary_information.head = SEEKFREE_ASSISTANT_SEND_HEAD;
    camera_wifi_packet.boundary_information.function = SEEKFREE_ASSISTANT_CAMERA_DOT_FUNCTION;
    camera_wifi_packet.boundary_information.dot_type =
        (uint8)(((uint8)XY_BOUNDARY << 6) | boundary_count);
    camera_wifi_packet.boundary_information.length = sizeof(seekfree_assistant_camera_dot_struct);
    camera_wifi_packet.boundary_information.dot_num = CAMERA_WIFI_BOUNDARY_POINT_COUNT;
    camera_wifi_packet.boundary_information.valid_flag = (uint8)((1u << boundary_count) - 1u);
    camera_wifi_packet.boundary_information.reserve = 0;

    return (uint32)(
        offsetof(CameraWifiImagePacket_t, boundary) +
        (size_t)boundary_count * sizeof(CameraWifiBoundaryPoints_t)
    );
}

// 摄像头新帧复制
static uint8 camera_frame_copy(void)
{
    // 是否采集到一个新帧
    if(!mt9v03x_finish_flag)
    {
        return 0;
    }
    mt9v03x_finish_flag = 0;

    // 复制新帧到 image_copy
    memcpy(image_copy[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);

    // 复制期间摄像头中断产生了新帧，本次图像可能被覆盖，留到下一轮重新复制
    if(mt9v03x_finish_flag)
    {
        return 0;
    }

    return 1;
}

// 摄像头新帧复制并进行最基本二值化处理
static uint8 camera_frame_cpy_and_basic_processing(uint8 threshold)
{
    if(!camera_frame_copy())
    {
        return 0;
    }

    // 图像最基本二值化处理
    camproc_pub_thresh_bin(image_copy, threshold);
    camera_processed_frame_mark();

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

    camera_screen_last_frame_id = camera_processed_frame_id;
}

uint8 camera_debug_on_screen(void)
{
    if((0u == camera_processed_frame_id) ||
       (camera_screen_last_frame_id == camera_processed_frame_id))
    {
        return 0;
    }

    screen_show_camera_image(IMAGE_X, IMAGE_Y, image_copy[0], IMAGE_DISPLAY_WIDTH, IMAGE_DISPLAY_HEIGHT);
    camera_screen_last_frame_id = camera_processed_frame_id;

    return 1;
}

uint8 camera_wifi_spi_init(char *wifi_ssid, char *pass_word, char *target_ip, char *target_port, char *local_port)
{
    uint8 return_state = 0;

    camera_wifi_spi_ready = 0;
    camera_wifi_frame_count = 0;
    camera_wifi_last_frame_id = camera_processed_frame_id;

    camera_wifi_packet.information.head = SEEKFREE_ASSISTANT_SEND_HEAD;
    camera_wifi_packet.information.function = SEEKFREE_ASSISTANT_CAMERA_FUNCTION;
    camera_wifi_packet.information.camera_type = (uint8)(SEEKFREE_ASSISTANT_BINARY << 5);
    camera_wifi_packet.information.length = sizeof(seekfree_assistant_camera_struct);
    camera_wifi_packet.information.image_width = CAMERA_WIFI_IMAGE_WIDTH;
    camera_wifi_packet.information.image_height = CAMERA_WIFI_IMAGE_HEIGHT;

    return_state = wifi_spi_init(wifi_ssid, pass_word);
    if(return_state)
    {
        return return_state;
    }

    return_state = wifi_spi_socket_connect("TCP", target_ip, target_port, local_port);
    if(return_state)
    {
        return return_state;
    }

    camera_wifi_spi_ready = 1;

    return 0;
}

void camera_debug_on_wifi_spi(uint16 send_div, const CameraWifiOverlay_t *overlay)
{
    uint32 send_length = 0;

    if(!camera_wifi_spi_ready ||
       (0 == camera_processed_frame_id) ||
       (camera_wifi_last_frame_id == camera_processed_frame_id))
    {
        return;
    }

    // 每个处理帧只参与一次分频，避免主循环重复发送同一幅图像
    camera_wifi_last_frame_id = camera_processed_frame_id;

    if(0 == send_div)
    {
        send_div = 1;
    }

    if(camera_wifi_frame_count)
    {
        camera_wifi_frame_count--;
        return;
    }

    camera_wifi_frame_count = (uint16)(send_div - 1u);

    // 模块忙时直接跳过本帧，避免底层最长 1 秒的阻塞等待
    if(!gpio_get_level(WIFI_SPI_INT_PIN))
    {
        return;
    }

    camera_wifi_pack_binary_image();
    send_length = camera_wifi_prepare_overlay(overlay);
    (void)wifi_spi_send_buffer((const uint8 *)&camera_wifi_packet, send_length);
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

    // 复制新帧，并使用单边桥专用阈值策略生成完整二值图
    if(!camera_frame_copy())
    {
        return 0;
    }

    if(!camproc_bridge_prepare_binary(image_copy, bridge_params))
    {
        return 0;
    }
    camera_processed_frame_mark();

    // RANSAC 边线算法只处理 bridge_params 指定的 ROI
    camproc_bridge_detect(image_copy, bridge_params, bridge_result);

    return 1;  // 返回的不是成功识别单边桥，这里最终的结果在 CameraBridgeResult_t 中
}

void camera_bridge_align_reset(CameraBridgeAlignState_t *align_state)
{
    camproc_bridge_detect_reset();
    camproc_bridge_align_reset(align_state);
}

uint8 camera_bridge_align_update(uint32 time_ms, const CameraBridgeResult_t *bridge_result, const CameraBridgeAlignParams_t *align_params, CameraBridgeAlignState_t *align_state, CameraBridgeAlignResult_t *align_result)
{
    return camproc_bridge_align_update(time_ms, bridge_result, align_params, align_state, align_result);
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
