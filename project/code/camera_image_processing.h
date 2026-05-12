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

#endif
