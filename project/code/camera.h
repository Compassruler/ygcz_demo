#ifndef CAMERA_H
#define CAMERA_H

#include "zf_common_headfile.h"
#include "camera_image_processing.h"
#include "zf_device_mt9v03x.h"

// 屏幕显示坐标
#define IMAGE_X                 (0)
#define IMAGE_Y                 (120)
#define IMAGE_DISPLAY_WIDTH     (188)
#define IMAGE_DISPLAY_HEIGHT    (120)

// WiFi SPI 图传默认分频：2 表示每 2 帧发送 1 帧
#define CAMERA_WIFI_IMAGE_SEND_DIV_DEFAULT     (2U)


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
 * @return uint32 已通过图传函数发送的帧数量。
 *
 * @note 该计数会在 `camera_send_frame()` 或 `camera_debug_on_wifi_spi()` 实际发送图像后增加。
 */
uint32 camera_get_frame_count(void);


/**
 * @brief 初始化 WiFi SPI 图传通道，并将逐飞助手绑定到 WiFi SPI。
 *
 * 本函数会参考 vision 模块中的 WiFi SPI 图传流程完成：
 * 1. 初始化 WiFi SPI 模块并连接指定 WiFi；
 * 2. 建立 UDP Socket 连接；
 * 3. 将逐飞助手发送/接收接口切换到 `SEEKFREE_ASSISTANT_WIFI_SPI`；
 * 4. 绑定内部处理图像 `image_copy` 到逐飞助手摄像头显示通道。
 *
 * @param wifi_ssid   需要连接的 WiFi 名称。
 * @param pass_word   WiFi 密码；如果目标 WiFi 无密码，可传入 NULL。
 * @param target_ip   上位机 IP 地址字符串。
 * @param target_port 上位机端口号字符串，需与逐飞助手接收端一致。
 * @param local_port  本机端口号字符串，通常可使用 "6666"。
 *
 * @return uint8
 *         - 0：初始化成功；
 *         - 非 0：WiFi 连接或 Socket 连接失败。
 *
 * @note 本函数不初始化 MT9V03X 摄像头，摄像头仍需调用 `camera_init()` 初始化。
 * @note 本函数使用 UDP 方式连接，适合图像调试这种低延迟、允许少量丢包的场景。
 */
uint8 camera_wifi_spi_init(char *wifi_ssid, char *pass_word, char *target_ip, char *target_port, char *local_port);


/**
 * @brief 初始化 WiFi SPI 逐飞助手通道，用于上位机图传或参数调试。
 *
 * 本函数是 `camera_wifi_spi_init()` 的语义化封装，适合在主函数中表达
 * “本次使用 WiFi SPI 作为逐飞助手调参通道”。
 *
 * @param wifi_ssid   需要连接的 WiFi 名称。
 * @param pass_word   WiFi 密码；如果目标 WiFi 无密码，可传入 NULL。
 * @param target_ip   上位机 IP 地址字符串。
 * @param target_port 上位机端口号字符串。
 * @param local_port  本机端口号字符串，通常可使用 "6666"。
 *
 * @return uint8
 *         - 0：初始化成功；
 *         - 非 0：WiFi 连接或 Socket 连接失败。
 *
 * @note 逐飞助手同一时间只有一组发送/接收接口。调用本函数后，
 *       参数调试、图传和示波器都会走 WiFi SPI。
 */
uint8 camera_assistant_wifi_spi_init(char *wifi_ssid, char *pass_word, char *target_ip, char *target_port, char *local_port);


/**
 * @brief 解析逐飞助手接收到的参数数据包。
 *
 * 调参时应在主循环中周期性调用本函数。调用后，若上位机修改了某个通道，
 * 对应的 `camera_assistant_parameter_read_xxx()` 函数即可读到新值。
 *
 * @return void
 *
 * @note 本函数只解析数据包，不决定参数含义；每个通道对应什么变量由主文件自行设置。
 */
void camera_assistant_parameter_update(void);


/**
 * @brief 从逐飞助手指定通道读取 float 参数，并自动限幅。
 *
 * @param channel   上位机参数通道号，范围为 1~8。
 * @param value     读取成功后写入的新参数值。
 * @param min_value 允许的最小值。
 * @param max_value 允许的最大值。
 *
 * @return uint8
 *         - 1：该通道收到新参数，且已写入 `value`；
 *         - 0：该通道没有新参数，或参数指针/通道号非法。
 */
uint8 camera_assistant_parameter_read_float(uint8 channel, float *value, float min_value, float max_value);


/**
 * @brief 从逐飞助手指定通道读取 int16 参数，并自动四舍五入和限幅。
 *
 * @param channel   上位机参数通道号，范围为 1~8。
 * @param value     读取成功后写入的新参数值。
 * @param min_value 允许的最小值。
 * @param max_value 允许的最大值。
 *
 * @return uint8
 *         - 1：该通道收到新参数，且已写入 `value`；
 *         - 0：该通道没有新参数，或参数指针/通道号非法。
 */
uint8 camera_assistant_parameter_read_int16(uint8 channel, int16 *value, int16 min_value, int16 max_value);


/**
 * @brief 从逐飞助手指定通道读取 uint16 参数，并自动四舍五入和限幅。
 *
 * @param channel   上位机参数通道号，范围为 1~8。
 * @param value     读取成功后写入的新参数值。
 * @param min_value 允许的最小值。
 * @param max_value 允许的最大值。
 *
 * @return uint8
 *         - 1：该通道收到新参数，且已写入 `value`；
 *         - 0：该通道没有新参数，或参数指针/通道号非法。
 */
uint8 camera_assistant_parameter_read_uint16(uint8 channel, uint16 *value, uint16 min_value, uint16 max_value);


/**
 * @brief 从逐飞助手指定通道读取 uint32 参数，并自动四舍五入和限幅。
 *
 * @param channel   上位机参数通道号，范围为 1~8。
 * @param value     读取成功后写入的新参数值。
 * @param min_value 允许的最小值。
 * @param max_value 允许的最大值。
 *
 * @return uint8
 *         - 1：该通道收到新参数，且已写入 `value`；
 *         - 0：该通道没有新参数，或参数指针/通道号非法。
 */
uint8 camera_assistant_parameter_read_uint32(uint8 channel, uint32 *value, uint32 min_value, uint32 max_value);


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
 * // 一般无需调整，已固定在函数中
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
void camera_debug_on_screen();  // (uint16 x, uint16 y, uint16 display_width, uint16 display_height);


/**
 * @brief 将处理后的摄像头图像通过 WiFi SPI 发送到逐飞助手上位机。
 *
 * 本函数的封装方式类似 `camera_debug_on_screen()`：
 * 如果当前存在新帧，会先复制并处理新帧；如果没有新帧，但内部已有处理后的图像，
 * 则直接复用最近一次处理结果进行发送。
 *
 * @param send_div 图传分频系数：
 *                 - 0 或 1：每次调用都尝试发送；
 *                 - 2：每 2 次调用发送 1 次；
 *                 - N：每 N 次调用发送 1 次。
 *
 * @return void
 *
 * @note 调用本函数前必须先调用 `camera_wifi_spi_init()`。
 * @note 若需要同时执行视觉判断、屏幕显示和 WiFi 图传，建议先调用 `camera_processing()`，
 *       再调用 `camera_debug_on_screen()` 和本函数，这样三者共用同一帧处理后的 `image_copy`。
 * @note WiFi SPI 带宽高于无线串口，但整帧图像仍可能占用时间，建议使用分频发送。
 */
void camera_debug_on_wifi_spi(uint16 send_div);


// 计算帧率
uint32 calc_fps(uint32 time_ms, uint32 *frame_count, uint32 *fps);

/**
 * @brief 处理一帧摄像头图像并返回跳跃检测结果。
 *
 * 如果检测到摄像头有新帧，本函数会：
 * 1. 清除帧完成标志；
 * 2. 复制 `mt9v03x_image` 到内部图像副本；
 * 3. 使用大津法自动阈值完成二值化；
 * 4. 执行黑色/白色孤立噪点过滤；
 * 5. 根据 `jump_params->algo_type` 选择严格检测或矩形区域总量检测；
 * 6. 根据 `jump_params->multi_frame` 做连续多帧确认；
 * 7. 根据 `jump_params->cooldown_time_ms` 做单次触发冷却过滤；
 * 8. 当跳跃触发成功时，自动切换 `jump_params->dot_type`，用于下一次检测相反颜色。
 *
 * @param time_ms     当前系统毫秒时间，用于跳跃冷却判断。
 * @param jump_params 跳跃检测参数结构体指针，包含检测区域、算法类型、像素类型、像素阈值、
 *                    连续检测帧数和冷却时间。函数可能会在触发成功后修改其中的 `dot_type`。
 *
 * @return uint8
 *         - 1：本次检测到新帧，并且检测到跳跃特征；
 *         - 0：当前没有新帧、参数指针为空，或本次处理后未检测到跳跃特征。
 *
 * @note 本函数不显示图像，也不进行无线发送。
 * @note 本函数只有在检测到新帧时才会执行处理；如果没有新帧，直接返回 0。
 * @note 本函数会在内部图像副本上处理，原始 `mt9v03x_image` 不会被直接修改。
 * @note 矩形区域检测可以通过 `jump_params->dot_type` 选择统计黑色或白色像素。
 * @note 严格检测当前仍以黑色像素为判断目标，并将 `jump_params->dot_count` 同时作为行阈值和列阈值。
 */
uint8 camera_processing(uint32 time_ms, JumpDetectParams_t *jump_params);

/**
 * @brief 使用 ROI 大津法处理一帧摄像头图像，并返回跳跃检测结果。
 *
 * 本函数是 `camera_processing()` 的 ROI 大津法版本。函数会使用
 * `jump_params->otsu_roi_*` 计算大津法阈值，再完成图像二值化，
 * 最后执行与 `camera_processing()` 相同的跳跃检测逻辑。
 *
 * @param time_ms     当前系统毫秒时间。
 * @param jump_params 跳跃检测参数结构体指针，包含大津法 ROI 参数。
 *
 * @return uint8
 *         - 1：检测到跳跃，并且允许本次触发；
 *         - 0：当前没有新帧、参数非法，或本次未检测到跳跃。
 */
uint8 camera_processing_roi(uint32 time_ms, JumpDetectParams_t *jump_params);
#endif
