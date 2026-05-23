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
 * @brief 在指定矩形区域内执行黑色像素总量检测。
 *
 * 该函数从 `check_row` 开始向上取 `check_row_count` 行，
 * 从 `check_column` 开始向右取 `check_column_count` 列，形成一个矩形检测区域。
 * 只要该矩形区域内黑色像素总数大于或等于 `black_count`，函数立即返回检测通过。
 *
 * @param image              待检测的二值图像数组，尺寸必须为 `MT9V03X_H * MT9V03X_W`。
 * @param check_row          起始检测行的纵向坐标，范围应小于 `MT9V03X_H`。
 * @param check_row_count    从起始检测行开始，继续向上检查的行数。
 * @param check_column       起始检测列的横向坐标，范围应小于 `MT9V03X_W`。
 * @param check_column_count 从起始检测列开始，继续向右检查的列数。
 * @param black_count        整个矩形区域内需要达到的黑色像素总数阈值。
 *
 * @return uint8
 *         - 1：矩形区域内黑色像素总数达到阈值；
 *         - 0：矩形区域内黑色像素总数未达到阈值，或检测区域越界。
 *
 * @note 调用本函数前，应先完成二值化处理。
 * @note 当前检测区域为从 `check_row` 向上、从 `check_column` 向右形成的矩形区域。
 * @note 若 `black_count` 大于矩形区域像素总数，函数会直接返回 0。
 */
uint8 camera_image_check_jump_area(uint8 image[MT9V03X_H][MT9V03X_W], uint16 check_row, uint16 check_row_count, uint16 check_column, uint16 check_column_count, uint32 black_count);


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

#endif
