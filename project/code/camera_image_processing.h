#ifndef CAMERA_IMAGE_PROCESSING_H
#define CAMERA_IMAGE_PROCESSING_H

#include "zf_common_typedef.h"
#include "zf_device_mt9v03x.h"

#define CAMERA_JUMP_ALGO_STRICT       (0)    // 严格检测：行检测和列检测必须同时通过
#define CAMERA_JUMP_ALGO_AREA         (1)    // 矩形检测：矩形区域内指定颜色像素总数达到阈值

#define CAMERA_IMAGE_DOT_BLACK        (0)    // 检测二值图中的黑色像素，像素值为 0
#define CAMERA_IMAGE_DOT_WHITE        (1)    // 检测二值图中的白色像素，像素值为 255

#define CAMERA_BRIDGE_REFERENCE_ROW_DEFAULT       ((MT9V03X_H * 3u) / 4u) // 默认在图像高度 3/4 处计算位置偏差
#define CAMERA_BRIDGE_MIN_EDGE_POINTS_DEFAULT     (15u)                   // 默认至少使用 15 行连续轮廓拟合边线
#define CAMERA_BRIDGE_FIT_RESIDUAL_PX_DEFAULT     (2u)                    // 默认允许轮廓点偏离拟合线 2 像素


#define CAMERA_ROW_SPEED_RULE_COUNT     (5u)
#define CAMERA_ROW_AGGRESSIVE_BIAS      (10u)

typedef struct 
{
    uint16 algo_type;               // 算法类型选择：CAMERA_JUMP_ALGO_STRICT 或 CAMERA_JUMP_ALGO_AREA
    uint16 check_row;               // 检测矩形的起始行，后续从该行向上检查
    uint16 check_row_count;         // 从起始行向上检查的行数量
    uint16 check_column;            // 检测矩形的起始列，后续从该列向右检查
    uint16 check_column_count;      // 从起始列向右检查的列数量
    uint16 otsu_roi_row;            // 大津法 ROI 底部行，从该行开始向上取区域
    uint16 otsu_roi_row_count;      // 大津法 ROI 行数
    uint16 otsu_roi_column;         // 大津法 ROI 左侧起始列
    uint16 otsu_roi_column_count;   // 大津法 ROI 列数
    uint32 dot_type;                // 检测像素类型：CAMERA_IMAGE_DOT_BLACK 或 CAMERA_IMAGE_DOT_WHITE
    uint32 dot_count;               // 矩形检测时的像素总数阈值；严格检测时作为行/列阈值使用
    uint32 cooldown_time_ms;        // 跳跃触发后的冷却时间，单位 ms
    uint32 multi_frame;             // 连续检测到目标的帧数阈值，0 或 1 表示单帧触发
    uint32 steps;                   // 已执行的识别步骤

} JumpDetectParams_t;               // 跳跃检测参数结构体

/**
 * @brief 单边桥简化识别参数。
 *
 * 核心算法在二值图中从 `search_bottom` 向 `search_top` 扫描，
 * 寻找左右两侧均为白色、并且能够在相邻行间连续连接的黑色区域。
 */
typedef struct
{
    uint8 binary_threshold;         // 上层复制摄像头图像后使用的固定二值化阈值
    uint16 search_left;             // 搜索区域最左列
    uint16 search_right;            // 搜索区域最右列
    uint16 search_top;              // 搜索区域最上行
    uint16 search_bottom;           // 搜索区域最下行
    uint16 min_width;               // 每一行候选黑段需要达到的最小宽度
    uint16 min_height;              // 候选区域需要连续达到的最小行数
    uint32 min_area;                // 候选区域内黑色像素总数下限
    uint16 connect_gap;             // 相邻两行黑段允许的最大横向间隔
    uint16 target_edge_x;           // 右边线期望对准的横坐标
    uint16 reference_row;           // 固定位置测量行；设置为 0 时使用 CAMERA_BRIDGE_REFERENCE_ROW_DEFAULT
    uint16 min_edge_points;         // 拟合边线的最少连续点数；设置为 0 时使用默认值
    uint16 fit_residual_px;          // 拟合点最大横向残差；设置为 0 时使用默认值
} CameraBridgeParams_t;

/**
 * @brief 单边桥简化识别结果。
 */
typedef struct
{
    uint8 valid;                    // 1 表示识别成功，0 表示当前图像没有有效目标
    uint16 left;                    // 候选黑色区域最左坐标
    uint16 right;                   // 候选黑色区域最右坐标
    uint16 top;                     // 候选黑色区域最上坐标
    uint16 bottom;                  // 候选黑色区域最下坐标
    uint32 area;                    // 跟踪到的候选黑色像素总数
    uint16 right_edge_x;            // 拟合右边线在固定 reference_row 处的横坐标
    int16 distance_px;              // 右边线相对 target_edge_x 的有符号像素距离
    int16 angle_d10;                // 右边线相对垂直方向的角度，单位 0.1 度
    uint16 edge_x1;                 // 拟合右边线的上端点横坐标
    uint16 edge_y1;                 // 拟合右边线的上端点纵坐标
    uint16 edge_x2;                 // 拟合右边线的下端点横坐标
    uint16 edge_y2;                 // 拟合右边线的下端点纵坐标
    uint16 reference_row;           // 本次计算 right_edge_x 时使用的固定参考行
    uint16 edge_point_count;        // 最终参与稳健拟合的右轮廓点数量
    float edge_slope;               // 拟合模型 x = edge_slope*y + edge_intercept 的斜率
    float edge_intercept;           // 拟合模型 x = edge_slope*y + edge_intercept 的截距
} CameraBridgeResult_t;

// 跳跃触发成功后，下一次检测像素类型会按该列表循环切换
static const uint32 dot_type_list[] =
{
    CAMERA_IMAGE_DOT_WHITE,
    CAMERA_IMAGE_DOT_BLACK,
    CAMERA_IMAGE_DOT_WHITE,
};

#define CAMERA_DOT_TYPE_LIST_COUNT     (sizeof(dot_type_list) / sizeof(dot_type_list[0]))



typedef struct
{
    uint16 max_speed;
    uint16 check_row;
}CameraRowSpeedRule_t;

extern const CameraRowSpeedRule_t camera_row_speed_rules[CAMERA_ROW_SPEED_RULE_COUNT];

uint16 camera_check_row_from_speed(uint16 car_speed, int8 aggressive_coeff);
void camera_jump_params_set_row_by_speed(JumpDetectParams_t *jump_params, uint16 car_speed, int8 aggressive_coeff);



/**
 * @brief 对 MT9V03X 灰度图像进行固定阈值二值化处理。
 *
 * 该函数会遍历整幅图像，并根据传入阈值 `threshold`
 * 将每个像素转换为黑白两种值：
 *
 * - 原像素值 > threshold：置为 255，表示白色；
 * - 原像素值 <= threshold：置为 0，表示黑色。
 *
 * @param image     待处理的图像数组，尺寸必须为 `MT9V03X_H * MT9V03X_W`。
 *                  函数会直接修改该数组内容。
 * @param threshold 二值化阈值，范围通常为 0~255。
 *
 * @return void
 *
 * @note 这是固定阈值二值化，适合光照比较稳定的场景。
 * @note 函数会原地修改图像，不会保留原始灰度数据。
 * @note 如果需要保留原图，应先复制图像，再调用本函数处理副本。
 */
void vision_binary_fixed(uint8 image[MT9V03X_H][MT9V03X_W], uint8 threshold);


/**
 * @brief 使用大津法自动计算阈值并完成二值化处理。
 *
 * 该函数会先统计整幅 MT9V03X 灰度图像的 0~255 灰度直方图，
 * 再根据类间方差最大原则自动计算二值化阈值，最后调用固定阈值
 * 二值化逻辑将图像转换为 0/255 二值图。
 *
 * @param image 待处理的灰度图像数组，尺寸必须为 `MT9V03X_H * MT9V03X_W`。
 *              函数会直接修改该数组内容。
 *
 * @return uint8 大津法计算出的二值化阈值。
 *
 * @note 调用后原始灰度图会被覆盖为二值图。
 * @note 大津法适合黑白灰度差异明显、光照相对均匀的场景。
 * @note 如果需要保留原图，应先复制图像，再调用本函数处理副本。
 */
uint8 camera_image_binary_otsu(uint8 image[MT9V03X_H][MT9V03X_W]);


/**
 * @brief 从固定 ROI 区域计算大津法阈值，并用该阈值二值化整幅图像。
 *
 * ROI 从 `roi_row` 行开始向上取 `roi_row_count` 行，
 * 从 `roi_column` 列开始向右取 `roi_column_count` 列。
 * 如果 ROI 参数非法，或者 ROI 内灰度区分度不足，函数会回退到
 * `camera_image_binary_otsu()` 对整幅图像计算阈值。
 *
 * @param image            待处理的图像数组，函数会直接修改该数组内容。
 * @param roi_row          ROI 底部行坐标。
 * @param roi_row_count    ROI 行数，从 `roi_row` 开始向上统计。
 * @param roi_column       ROI 左侧起始列坐标。
 * @param roi_column_count ROI 列数，从 `roi_column` 开始向右统计。
 *
 * @return uint8 实际用于二值化的大津法阈值。
 */
uint8 camera_image_binary_otsu_roi(uint8 image[MT9V03X_H][MT9V03X_W], uint16 roi_row, uint16 roi_row_count, uint16 roi_column, uint16 roi_column_count);


/**
 * @brief 去除二值图像中的孤立黑色噪点。
 *
 * 该函数用于处理已经二值化后的 MT9V03X 图像，默认约定：
 *
 * - 像素值 0：黑色目标；
 * - 像素值 255：非黑色背景。
 *
 * 函数会检查每个黑色像素周围 3x3 邻域内的黑色像素数量。
 * 如果黑色数量过少，就认为该黑点是孤立噪声，并将其改为 255。
 *
 * @param image 待处理的二值图像数组，尺寸必须为 `MT9V03X_H * MT9V03X_W`。
 *              函数会直接修改该数组内容。
 *
 * @return void
 *
 * @note 调用本函数前，应先完成二值化处理。
 * @note 当前过滤强度为：3x3 邻域内黑点数量小于等于 2 时删除中心黑点。
 * @note 该函数适合去除零散小黑点；如果黑线非常细，过滤强度不宜继续加大。
 */
void camera_image_filter_isolated_black(uint8 image[MT9V03X_H][MT9V03X_W]);


/**
 * @brief 去除二值图像中的孤立白色噪点。
 *
 * 该函数用于处理已经二值化后的 MT9V03X 图像，默认约定：
 *
 * - 像素值 0：黑色目标；
 * - 像素值 255：非黑色背景。
 *
 * 函数会检查每个白色像素周围 3x3 邻域内的白色像素数量。
 * 如果白色数量过少，就认为该白点是孤立噪声，并将其改为 0。
 *
 * @param image 待处理的二值图像数组，尺寸必须为 `MT9V03X_H * MT9V03X_W`。
 *              函数会直接修改该数组内容。
 *
 * @return void
 *
 * @note 调用本函数前，应先完成二值化处理。
 * @note 当前过滤强度为：3x3 邻域内白点数量小于等于 2 时删除中心白点。
 * @note 该函数适合去除黑色区域中的零散小白点；如果白色线条很细，过滤强度不宜继续加大。
 */
void camera_image_filter_isolated_white(uint8 image[MT9V03X_H][MT9V03X_W]);


/**
 * @brief 从二值图中识别最靠近画面下方的独立黑色区域，并拟合其右边线。
 *
 * 函数从搜索区域底部向上逐行扫描。每行只保留左右均有白色像素包围、
 * 且宽度达到 `min_width` 的黑色连续段；相邻行黑段在
 * `connect_gap` 范围内发生连接时，将其归入同一个候选区域。
 * 第一个同时达到最小高度和最小面积的候选区域被认为是距离画面下方
 * 最近的目标。算法收集各行黑段的最右端点，先使用 3 点中值滤波抑制单行锯齿，
 * 再搜索长度足够、拟合残差较小并且方向更接近垂直的连续轮廓段。
 * 找到初始线段后会剔除横向残差超限的离群点并执行第二次拟合，避免矩形上边缘、
 * 下边缘和二值化突出点进入右边线模型。
 *
 * @param image  已完成二值化的 MT9V03X 图像，黑色为 0，白色为非 0。
 * @param params 单边桥识别参数。
 * @param result 识别结果输出地址。函数会在每次调用开始时清空该结构体。
 *
 * @return uint8
 *         - 1：找到有效候选区域并成功拟合右边线；
 *         - 0：参数非法、未找到有效目标，或右边线无法拟合。
 *
 * @note 本函数不会修改输入图像，也不负责摄像头取帧和二值化。
 * @note `reference_row`、`min_edge_points` 和 `fit_residual_px` 设置为 0 时使用默认值。
 * @note `distance_px` 大于 0 表示右边线位于目标横坐标右侧。
 * @note `angle_d10` 大于 0 表示边线朝画面右下方倾斜。
 */
uint8 camera_image_bridge_detect(const uint8 image[MT9V03X_H][MT9V03X_W], const CameraBridgeParams_t *params, CameraBridgeResult_t *result);


/**
 * @brief 在指定矩形区域内执行跳跃特征的行检测。
 *
 * 该函数从 `check_row` 开始向上检查 `check_row_count` 行，
 * 每一行只统计从 `check_column` 开始向右的 `check_column_count` 个像素。
 * 只有每一行的黑色像素数量都大于或等于 `black_count`，才返回检测通过。
 *
 * @param image              待检测的二值图像数组，尺寸必须为 `MT9V03X_H * MT9V03X_W`。
 * @param check_row          起始检测行的纵向坐标，范围应小于 `MT9V03X_H`。
 * @param check_row_count    从起始检测行开始，继续向上检查的行数。
 * @param check_column       起始检测列的横向坐标，范围应小于 `MT9V03X_W`。
 * @param check_column_count 从起始检测列开始，继续向右检查的列数。
 * @param black_count        每一行需要达到的黑色像素数量阈值。
 *
 * @return uint8
 *         - 1：行检测通过；
 *         - 0：行检测未通过。
 *
 * @note 调用本函数前，应先完成二值化处理。
 */
uint8 camera_image_check_jump_rows(uint8 image[MT9V03X_H][MT9V03X_W], uint16 check_row, uint16 check_row_count, uint16 check_column, uint16 check_column_count, uint16 black_count);


/**
 * @brief 在指定矩形区域内执行跳跃特征的列检测。
 *
 * 该函数从 `check_column` 开始向右检查 `check_column_count` 列，
 * 每一列只统计从 `check_row` 开始向上的 `check_row_count` 个像素。
 * 只有每一列的黑色像素数量都大于或等于 `black_count`，才返回检测通过。
 *
 * @param image              待检测的二值图像数组，尺寸必须为 `MT9V03X_H * MT9V03X_W`。
 * @param check_row          起始检测行的纵向坐标，范围应小于 `MT9V03X_H`。
 * @param check_row_count    从起始检测行开始，继续向上检查的行数。
 * @param check_column       起始检测列的横向坐标，范围应小于 `MT9V03X_W`。
 * @param check_column_count 从起始检测列开始，继续向右检查的列数。
 * @param black_count        每一列需要达到的黑色像素数量阈值。
 *
 * @return uint8
 *         - 1：列检测通过；
 *         - 0：列检测未通过。
 *
 * @note 调用本函数前，应先完成二值化处理。
 */
uint8 camera_image_check_jump_columns(uint8 image[MT9V03X_H][MT9V03X_W], uint16 check_row, uint16 check_row_count, uint16 check_column, uint16 check_column_count, uint16 black_count);


/**
 * @brief 在指定矩形区域内执行指定颜色像素总量检测。
 *
 * 该函数从 `check_row` 开始向上取 `check_row_count` 行，
 * 从 `check_column` 开始向右取 `check_column_count` 列，形成一个矩形检测区域。
 * 只要该矩形区域内指定颜色像素总数大于或等于 `dot_count`，函数立即返回检测通过。
 *
 * @param image              待检测的二值图像数组，尺寸必须为 `MT9V03X_H * MT9V03X_W`。
 * @param check_row          起始检测行的纵向坐标，范围应小于 `MT9V03X_H`。
 * @param check_row_count    从起始检测行开始，继续向上检查的行数。
 * @param check_column       起始检测列的横向坐标，范围应小于 `MT9V03X_W`。
 * @param check_column_count 从起始检测列开始，继续向右检查的列数。
 * @param dot_count          整个矩形区域内需要达到的指定颜色像素总数阈值。
 * @param dot_type           需要统计的像素类型：
 *                           - CAMERA_IMAGE_DOT_BLACK：统计黑色像素，像素值为 0；
 *                           - CAMERA_IMAGE_DOT_WHITE：统计白色像素，像素值为 255。
 *
 * @return uint8
 *         - 1：矩形区域内指定颜色像素总数达到阈值；
 *         - 0：指定颜色像素总数未达到阈值、检测区域越界，或 `dot_type` 非法。
 *
 * @note 调用本函数前，应先完成二值化处理。
 * @note 当前检测区域为从 `check_row` 向上、从 `check_column` 向右形成的矩形区域。
 * @note 若 `dot_count` 大于矩形区域像素总数，函数会直接返回 0。
 */
uint8 camera_image_check_jump_area(uint8 image[MT9V03X_H][MT9V03X_W], uint16 check_row, uint16 check_row_count, uint16 check_column, uint16 check_column_count, uint32 dot_count, uint32 dot_type);


/**
 * @brief 综合行检测和列检测判断是否出现跳跃特征。
 *
 * 该函数会在同一个矩形区域内分别调用：
 *
 * 1. `camera_image_check_jump_rows()`：要求区域内每一行黑点数量达标；
 * 2. `camera_image_check_jump_columns()`：要求区域内每一列黑点数量达标。
 *
 * 只有行检测和列检测同时通过，函数才返回跳跃标志。
 *
 * @param image              待检测的二值图像数组，尺寸必须为 `MT9V03X_H * MT9V03X_W`。
 * @param check_row          起始检测行的纵向坐标，范围应小于 `MT9V03X_H`。
 * @param check_row_count    从起始检测行开始，继续向上检查的行数。
 * @param row_black_count    每一行需要达到的黑色像素数量阈值。
 * @param check_column       起始检测列的横向坐标，范围应小于 `MT9V03X_W`。
 * @param check_column_count 从起始检测列开始，继续向右检查的列数。
 * @param column_black_count 每一列需要达到的黑色像素数量阈值。
 *
 * @return uint8
 *         - 1：检测到跳跃特征；
 *         - 0：未检测到跳跃特征。
 *
 * @note 调用本函数前，应先完成二值化处理。
 * @note 当前检测区域为从 `check_row` 向上、从 `check_column` 向右形成的矩形区域。
 */
uint8 camera_image_check_jump_strict(uint8 image[MT9V03X_H][MT9V03X_W], uint16 check_row, uint16 check_row_count, uint16 row_black_count, uint16 check_column, uint16 check_column_count, uint16 column_black_count);


/**
 * @brief 对跳跃检测结果进行单次触发冷却过滤。
 *
 * 当检测到跳跃后，本函数会记录触发时间。在冷却时间未结束前，
 * 即使底层视觉算法继续检测到跳跃，也会返回 0，避免连续重复触发。
 *
 * @param time_ms          当前系统毫秒时间。
 * @param cooldown_time_ms 冷却时间，单位 ms。
 * @param jump_detected    原始跳跃检测结果，1 表示检测到跳跃。
 *
 * @return uint8
 *         - 1：本次允许触发跳跃；
 *         - 0：未检测到跳跃，或处于冷却时间内。
 */
uint8 camera_image_jump_trigger_filter(uint32 time_ms, uint32 cooldown_time_ms, uint8 jump_detected);


/**
 * @brief 按跳跃触发次数循环切换检测像素类型。
 *
 * 本函数用于在跳跃触发成功后，记录一次触发次数，并用触发次数对
 * `dot_type_list` 的长度取模，得到下一次需要检测的像素类型。
 * 例如列表为 `{BLACK, BLACK, WHITE}`，且初始检测类型为列表第 0 项时，
 * 后续检测顺序为：黑色 -> 黑色 -> 白色 -> 黑色 -> ...
 * 
 * 
 * @return uint8
 *         - 返回 `dot_type_list[jump_trigger_count % CAMERA_DOT_TYPE_LIST_COUNT]`；
 *         - 每调用一次，内部触发次数自动加 1。
 *
 * @note 该函数只返回检测类型，不会直接修改任何 `JumpDetectParams_t` 参数；
 *       使用时需要将返回值赋给 `jump_params->dot_type` 或 `jump_params.dot_type`。
 * @note 本函数应只在一次跳跃被确认触发后调用；如果在未触发时频繁调用，
 *       会导致检测类型提前切换。
 */
uint8 camera_dot_type_switch(void);

/**
 * @brief 获取当前已经触发成功的跳跃次数。
 *
 * @return uint32 当前跳跃触发计数。
 */
uint32 camera_dot_type_get_steps(void);

/**
 * @brief 复位跳跃检测序列，并返回初始检测像素类型。
 *
 * @return uint8 `dot_type_list` 的第一个检测像素类型。
 *
 * @note 调用后应将返回值同步赋给 `jump_params.dot_type`，
 *       这样内部计数和外部当前检测类型才能保持一致。
 */
uint8 camera_dot_type_reset(void);

#endif
