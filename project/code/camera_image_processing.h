#ifndef CAMERA_IMAGE_PROCESSING_H
#define CAMERA_IMAGE_PROCESSING_H

#include "zf_common_headfile.h"

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
 * @brief 根据指定行向上的黑色像素增长趋势判断是否出现跳跃特征。
 *
 * 该函数用于处理已经二值化后的 MT9V03X 图像，默认约定：
 *
 * - 像素值 0：黑色目标；
 * - 像素值 255：非黑色背景。
 *
 * 判断规则：
 * 1. 先统计 `check_row` 指定行的整行黑色像素数量；
 * 2. 如果该行黑色像素数量小于或等于 `black_count`，则直接返回不跳跃；
 * 3. 如果该行黑色像素数量超过 `black_count`，则从上一行开始继续向上检查；
 * 4. 向上检查 `check_row_count` 行，要求每一行黑色像素数量大于或等于上一行；
 * 5. 若向上检查的所有行都满足增长趋势，则返回跳跃标志，否则返回不跳跃标志。
 *
 * @param image           待检测的二值图像数组，尺寸必须为 `MT9V03X_H * MT9V03X_W`。
 * @param check_row       起始检测行的纵向坐标，范围应小于 `MT9V03X_H`。
 * @param check_row_count 从起始检测行的上一行开始，继续向上检查的行数。
 * @param black_count     起始检测行需要超过的黑色像素数量阈值。
 *
 * @return uint8
 *         - 1：检测到跳跃特征；
 *         - 0：未检测到跳跃特征。
 *
 * @note 调用本函数前，应先完成二值化处理。
 * @note 当前检测范围是整行宽度 `MT9V03X_W`，不是只检测图像中间区域。
 * @note 传入的 `check_row_count` 不应大于 `check_row`，否则可检查行数不足时会返回 0。
 */
uint8 camera_image_check_jump(uint8 image[MT9V03X_H][MT9V03X_W], uint16 check_row, uint16 check_row_count, uint16 black_count);

#endif
