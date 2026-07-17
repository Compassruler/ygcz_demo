#include "camera_image_processing.h"
#include <math.h>
#include <string.h>

static uint32 jump_trigger_count = 0;

#define CAMERA_BRIDGE_RAD_TO_D10       (572.9577951f)
#define CAMERA_BRIDGE_MAX_ABS_SLOPE    (1.0f)
#define CAMERA_BRIDGE_SLOPE_WEIGHT     (2.0f)

typedef struct
{
    uint16 left;
    uint16 right;
} CameraBridgeRun_t;

typedef struct
{
    uint8 active;
    uint16 previous_left;
    uint16 previous_right;
    uint16 left;
    uint16 right;
    uint16 top;
    uint16 bottom;
    uint16 point_count;
    uint32 area;
    uint16 edge_x[MT9V03X_H];
    uint16 edge_y[MT9V03X_H];
} CameraBridgeCandidate_t;

typedef struct
{
    uint8 valid;
    uint16 start_index;
    uint16 end_index;
    uint16 point_count;
    uint16 top;
    uint16 bottom;
    float slope;
    float intercept;
    float mean_square_error;
    float score;
} CameraBridgeFit_t;

static uint8 camera_bridge_find_row_run(
    const uint8 image[MT9V03X_H][MT9V03X_W],
    uint16 row,
    uint16 scan_left,
    uint16 scan_right,
    const CameraBridgeParams_t *params,
    uint8 require_connection,
    uint16 previous_left,
    uint16 previous_right,
    CameraBridgeRun_t *result_run)
{
    uint8 found = 0;
    uint16 x = scan_left;
    uint16 run_left = 0;
    uint16 run_right = 0;
    uint16 run_width = 0;
    uint16 best_width = 0;
    uint16 center_distance = 0;
    uint16 best_center_distance = 0xFFFFu;
    uint16 current_center_twice = 0;
    uint16 previous_center_twice = previous_left + previous_right;

    while(x <= scan_right)
    {
        while((x <= scan_right) && (0 != image[row][x]))
        {
            x++;
        }

        if(x > scan_right)
        {
            break;
        }

        run_left = x;

        while((x <= scan_right) && (0 == image[row][x]))
        {
            x++;
        }

        run_right = x - 1u;
        run_width = run_right - run_left + 1u;

        if(run_width < params->min_width)
        {
            continue;
        }

        // 黑段左右必须紧邻白色，避免把延伸到画面边缘的黑色背景作为目标。
        if((0 == image[row][run_left - 1u]) || (0 == image[row][run_right + 1u]))
        {
            continue;
        }

        if(require_connection)
        {
            if(((uint32)run_left > ((uint32)previous_right + params->connect_gap)) ||
               (((uint32)run_right + params->connect_gap) < previous_left))
            {
                continue;
            }

            current_center_twice = run_left + run_right;
            if(current_center_twice >= previous_center_twice)
            {
                center_distance = current_center_twice - previous_center_twice;
            }
            else
            {
                center_distance = previous_center_twice - current_center_twice;
            }

            if((!found) ||
               (center_distance < best_center_distance) ||
               ((center_distance == best_center_distance) && (run_width > best_width)))
            {
                found = 1;
                best_center_distance = center_distance;
                best_width = run_width;
                result_run->left = run_left;
                result_run->right = run_right;
            }
        }
        else if((!found) || (run_width > best_width))
        {
            found = 1;
            best_width = run_width;
            result_run->left = run_left;
            result_run->right = run_right;
        }
    }

    return found;
}

static void camera_bridge_candidate_reset(CameraBridgeCandidate_t *candidate)
{
    memset(candidate, 0, sizeof(*candidate));
}

static void camera_bridge_candidate_add_run(CameraBridgeCandidate_t *candidate, uint16 row, const CameraBridgeRun_t *run)
{
    uint16 run_width = run->right - run->left + 1u;
    uint16 point_index = candidate->point_count;

    if(point_index >= MT9V03X_H)
    {
        return;
    }

    if(!candidate->active)
    {
        candidate->active = 1;
        candidate->left = run->left;
        candidate->right = run->right;
        candidate->top = row;
        candidate->bottom = row;
    }
    else
    {
        candidate->top = row;

        if(run->left < candidate->left)
        {
            candidate->left = run->left;
        }

        if(run->right > candidate->right)
        {
            candidate->right = run->right;
        }
    }

    candidate->previous_left = run->left;
    candidate->previous_right = run->right;
    candidate->edge_x[point_index] = run->right;
    candidate->edge_y[point_index] = row;
    candidate->point_count++;
    candidate->area += run_width;
}

static uint16 camera_bridge_median3(uint16 value_a, uint16 value_b, uint16 value_c)
{
    if(value_a > value_b)
    {
        uint16 temp = value_a;
        value_a = value_b;
        value_b = temp;
    }

    if(value_b > value_c)
    {
        value_b = value_c;
    }

    return (value_a > value_b) ? value_a : value_b;
}

static uint16 camera_bridge_get_min_edge_points(const CameraBridgeParams_t *params)
{
    uint16 min_edge_points = params->min_edge_points;

    if(0u == min_edge_points)
    {
        min_edge_points = CAMERA_BRIDGE_MIN_EDGE_POINTS_DEFAULT;
    }

    if(min_edge_points < 3u)
    {
        min_edge_points = 3u;
    }

    return min_edge_points;
}

static float camera_bridge_get_residual_limit(const CameraBridgeParams_t *params)
{
    uint16 residual_limit = params->fit_residual_px;

    if(0u == residual_limit)
    {
        residual_limit = CAMERA_BRIDGE_FIT_RESIDUAL_PX_DEFAULT;
    }

    return (float)residual_limit;
}

static uint16 camera_bridge_get_reference_row(const CameraBridgeParams_t *params)
{
    uint16 reference_row = params->reference_row;

    if(0u == reference_row)
    {
        reference_row = CAMERA_BRIDGE_REFERENCE_ROW_DEFAULT;
    }

    if(reference_row < params->search_top)
    {
        reference_row = params->search_top;
    }
    else if(reference_row > params->search_bottom)
    {
        reference_row = params->search_bottom;
    }

    return reference_row;
}

static void camera_bridge_fit_from_sums(
    uint16 point_count,
    uint32 sum_x,
    uint32 sum_y,
    uint32 sum_xy,
    uint32 sum_yy,
    uint32 sum_xx,
    uint16 top,
    uint16 bottom,
    CameraBridgeFit_t *fit)
{
    float count = 0.0f;
    float centered_xx = 0.0f;
    float centered_xy = 0.0f;
    float centered_yy = 0.0f;
    float square_error = 0.0f;

    memset(fit, 0, sizeof(*fit));

    if((point_count < 2u) || (top >= bottom))
    {
        return;
    }

    count = (float)point_count;
    centered_xx = (float)sum_xx - ((float)sum_x * (float)sum_x) / count;
    centered_xy = (float)sum_xy - ((float)sum_x * (float)sum_y) / count;
    centered_yy = (float)sum_yy - ((float)sum_y * (float)sum_y) / count;

    if(centered_yy < 0.5f)
    {
        return;
    }

    fit->slope = centered_xy / centered_yy;
    fit->intercept = ((float)sum_x - fit->slope * (float)sum_y) / count;
    square_error = centered_xx - fit->slope * centered_xy;
    if(square_error < 0.0f)
    {
        square_error = 0.0f;
    }

    fit->mean_square_error = square_error / count;
    fit->point_count = point_count;
    fit->top = top;
    fit->bottom = bottom;
    fit->valid = 1;
}

static uint8 camera_bridge_find_best_edge_fit(
    const CameraBridgeCandidate_t *candidate,
    const CameraBridgeParams_t *params,
    const uint16 filtered_x[MT9V03X_H],
    CameraBridgeFit_t *best_fit)
{
    uint16 start_index = 0;
    uint16 end_index = 0;
    uint16 point_count = 0;
    uint16 min_edge_points = camera_bridge_get_min_edge_points(params);
    uint32 sum_x = 0;
    uint32 sum_y = 0;
    uint32 sum_xy = 0;
    uint32 sum_yy = 0;
    uint32 sum_xx = 0;
    float residual_limit = camera_bridge_get_residual_limit(params);
    float residual_limit_square = residual_limit * residual_limit;
    float abs_slope = 0.0f;
    CameraBridgeFit_t current_fit = {0};

    memset(best_fit, 0, sizeof(*best_fit));

    if(candidate->point_count < min_edge_points)
    {
        return 0;
    }

    for(start_index = 0; start_index + min_edge_points <= candidate->point_count; start_index++)
    {
        sum_x = 0;
        sum_y = 0;
        sum_xy = 0;
        sum_yy = 0;
        sum_xx = 0;

        for(end_index = start_index; end_index < candidate->point_count; end_index++)
        {
            uint32 point_x = filtered_x[end_index];
            uint32 point_y = candidate->edge_y[end_index];

            sum_x += point_x;
            sum_y += point_y;
            sum_xy += point_x * point_y;
            sum_yy += point_y * point_y;
            sum_xx += point_x * point_x;
            point_count = end_index - start_index + 1u;

            if(point_count < min_edge_points)
            {
                continue;
            }

            camera_bridge_fit_from_sums(
                point_count,
                sum_x,
                sum_y,
                sum_xy,
                sum_yy,
                sum_xx,
                candidate->edge_y[end_index],
                candidate->edge_y[start_index],
                &current_fit);

            if(!current_fit.valid || (current_fit.mean_square_error > residual_limit_square))
            {
                continue;
            }

            abs_slope = (current_fit.slope < 0.0f) ? -current_fit.slope : current_fit.slope;
            if(abs_slope > CAMERA_BRIDGE_MAX_ABS_SLOPE)
            {
                continue;
            }

            current_fit.start_index = start_index;
            current_fit.end_index = end_index;
            current_fit.score = (float)point_count /
                                (1.0f + CAMERA_BRIDGE_SLOPE_WEIGHT * abs_slope);

            if((!best_fit->valid) ||
               (current_fit.score > best_fit->score + 0.001f) ||
               (((current_fit.score + 0.001f) >= best_fit->score) &&
                (current_fit.mean_square_error < best_fit->mean_square_error)))
            {
                *best_fit = current_fit;
            }
        }
    }

    return best_fit->valid;
}

static uint8 camera_bridge_refit_inliers(
    const CameraBridgeCandidate_t *candidate,
    const CameraBridgeParams_t *params,
    const uint16 filtered_x[MT9V03X_H],
    const CameraBridgeFit_t *initial_fit,
    CameraBridgeFit_t *refined_fit)
{
    uint16 index = 0;
    uint16 first_inlier = 0;
    uint16 last_inlier = 0;
    uint16 inlier_count = 0;
    uint16 min_edge_points = camera_bridge_get_min_edge_points(params);
    uint32 sum_x = 0;
    uint32 sum_y = 0;
    uint32 sum_xy = 0;
    uint32 sum_yy = 0;
    uint32 sum_xx = 0;
    float residual_limit = camera_bridge_get_residual_limit(params);
    float predicted_x = 0.0f;
    float residual = 0.0f;

    memset(refined_fit, 0, sizeof(*refined_fit));

    for(index = initial_fit->start_index; index <= initial_fit->end_index; index++)
    {
        uint32 point_x = filtered_x[index];
        uint32 point_y = candidate->edge_y[index];

        predicted_x = initial_fit->slope * (float)point_y + initial_fit->intercept;
        residual = (float)point_x - predicted_x;
        if(residual < 0.0f)
        {
            residual = -residual;
        }

        if(residual > residual_limit)
        {
            continue;
        }

        if(0u == inlier_count)
        {
            first_inlier = index;
        }

        last_inlier = index;
        inlier_count++;
        sum_x += point_x;
        sum_y += point_y;
        sum_xy += point_x * point_y;
        sum_yy += point_y * point_y;
        sum_xx += point_x * point_x;
    }

    if(inlier_count < min_edge_points)
    {
        return 0;
    }

    camera_bridge_fit_from_sums(
        inlier_count,
        sum_x,
        sum_y,
        sum_xy,
        sum_yy,
        sum_xx,
        candidate->edge_y[last_inlier],
        candidate->edge_y[first_inlier],
        refined_fit);

    if(!refined_fit->valid)
    {
        return 0;
    }

    if(((refined_fit->slope < 0.0f) ? -refined_fit->slope : refined_fit->slope) >
       CAMERA_BRIDGE_MAX_ABS_SLOPE)
    {
        memset(refined_fit, 0, sizeof(*refined_fit));
        return 0;
    }

    refined_fit->start_index = first_inlier;
    refined_fit->end_index = last_inlier;

    return 1;
}

static uint16 camera_bridge_round_and_limit_x(float value)
{
    int32 rounded_value = 0;

    if(0.0f <= value)
    {
        rounded_value = (int32)(value + 0.5f);
    }
    else
    {
        rounded_value = (int32)(value - 0.5f);
    }

    if(rounded_value < 0)
    {
        rounded_value = 0;
    }
    else if(rounded_value >= MT9V03X_W)
    {
        rounded_value = MT9V03X_W - 1;
    }

    return (uint16)rounded_value;
}

static uint8 camera_bridge_candidate_make_result(
    const CameraBridgeCandidate_t *candidate,
    const CameraBridgeParams_t *params,
    CameraBridgeResult_t *result)
{
    uint16 index = 0;
    uint16 previous_index = 0;
    uint16 next_index = 0;
    uint16 reference_row = 0;
    uint16 filtered_x[MT9V03X_H] = {0};
    float reference_y = 0.0f;
    float angle_d10 = 0.0f;
    CameraBridgeFit_t initial_fit = {0};
    CameraBridgeFit_t refined_fit = {0};

    if((!candidate->active) ||
       (candidate->point_count < params->min_height) ||
       (candidate->area < params->min_area))
    {
        return 0;
    }

    // 对相邻三行的右端点做中值滤波，消除二值化边缘上的单行尖峰。
    for(index = 0; index < candidate->point_count; index++)
    {
        previous_index = (0u == index) ? index : (index - 1u);
        next_index = ((index + 1u) < candidate->point_count) ? (index + 1u) : index;
        filtered_x[index] = camera_bridge_median3(
            candidate->edge_x[previous_index],
            candidate->edge_x[index],
            candidate->edge_x[next_index]);
    }

    if(!camera_bridge_find_best_edge_fit(candidate, params, filtered_x, &initial_fit) ||
       !camera_bridge_refit_inliers(candidate, params, filtered_x, &initial_fit, &refined_fit))
    {
        return 0;
    }

    reference_row = camera_bridge_get_reference_row(params);
    reference_y = (float)reference_row;
    angle_d10 = atanf(refined_fit.slope) * CAMERA_BRIDGE_RAD_TO_D10;

    result->left = candidate->left;
    result->right = candidate->right;
    result->top = candidate->top;
    result->bottom = candidate->bottom;
    result->area = candidate->area;
    result->right_edge_x = camera_bridge_round_and_limit_x(
        refined_fit.slope * reference_y + refined_fit.intercept);
    result->distance_px = (int16)((int32)result->right_edge_x - (int32)params->target_edge_x);
    result->edge_y1 = refined_fit.top;
    result->edge_x1 = camera_bridge_round_and_limit_x(
        refined_fit.slope * refined_fit.top + refined_fit.intercept);
    result->edge_y2 = refined_fit.bottom;
    result->edge_x2 = camera_bridge_round_and_limit_x(
        refined_fit.slope * refined_fit.bottom + refined_fit.intercept);
    result->reference_row = reference_row;
    result->edge_point_count = refined_fit.point_count;
    result->edge_slope = refined_fit.slope;
    result->edge_intercept = refined_fit.intercept;

    if(0.0f <= angle_d10)
    {
        result->angle_d10 = (int16)(angle_d10 + 0.5f);
    }
    else
    {
        result->angle_d10 = (int16)(angle_d10 - 0.5f);
    }

    result->valid = 1;

    return 1;
}

uint8 camera_image_bridge_detect(
    const uint8 image[MT9V03X_H][MT9V03X_W],
    const CameraBridgeParams_t *params,
    CameraBridgeResult_t *result)
{
    int16 row = 0;
    uint16 scan_left = 0;
    uint16 scan_right = 0;
    uint32 search_area = 0;
    CameraBridgeRun_t run = {0};
    CameraBridgeCandidate_t candidate = {0};

    if(NULL == result)
    {
        return 0;
    }

    memset(result, 0, sizeof(*result));

    if((NULL == image) || (NULL == params))
    {
        return 0;
    }

    if((params->search_left >= MT9V03X_W) ||
       (params->search_right >= MT9V03X_W) ||
       (params->search_top >= MT9V03X_H) ||
       (params->search_bottom >= MT9V03X_H) ||
       (params->search_left > params->search_right) ||
       (params->search_top > params->search_bottom) ||
       (0 == params->min_width) ||
       (params->min_height < 2u) ||
       (0 == params->min_area) ||
       (params->connect_gap >= MT9V03X_W) ||
       (params->target_edge_x >= MT9V03X_W) ||
       ((0u != params->reference_row) && (params->reference_row >= MT9V03X_H)) ||
       (params->min_edge_points > MT9V03X_H) ||
       (params->fit_residual_px >= MT9V03X_W))
    {
        return 0;
    }

    scan_left = params->search_left;
    scan_right = params->search_right;

    // 为左右白色邻点检查预留一列，防止访问图像边界之外。
    if(0 == scan_left)
    {
        scan_left = 1;
    }

    if(scan_right >= (MT9V03X_W - 1u))
    {
        scan_right = MT9V03X_W - 2u;
    }

    if((scan_left > scan_right) ||
       (params->min_width > (scan_right - scan_left + 1u)) ||
       (params->min_height > (params->search_bottom - params->search_top + 1u)))
    {
        return 0;
    }

    search_area = (uint32)(scan_right - scan_left + 1u) *
                  (uint32)(params->search_bottom - params->search_top + 1u);
    if(params->min_area > search_area)
    {
        return 0;
    }

    camera_bridge_candidate_reset(&candidate);

    for(row = (int16)params->search_bottom; row >= (int16)params->search_top; row--)
    {
        if(candidate.active)
        {
            if(camera_bridge_find_row_run(
                image,
                (uint16)row,
                scan_left,
                scan_right,
                params,
                1,
                candidate.previous_left,
                candidate.previous_right,
                &run))
            {
                camera_bridge_candidate_add_run(&candidate, (uint16)row, &run);
                continue;
            }

            if(camera_bridge_candidate_make_result(&candidate, params, result))
            {
                return 1;
            }

            camera_bridge_candidate_reset(&candidate);
        }

        if(camera_bridge_find_row_run(
            image,
            (uint16)row,
            scan_left,
            scan_right,
            params,
            0,
            0,
            0,
            &run))
        {
            camera_bridge_candidate_add_run(&candidate, (uint16)row, &run);
        }
    }

    return camera_bridge_candidate_make_result(&candidate, params, result);
}

void vision_binary_fixed(uint8 image[MT9V03X_H][MT9V03X_W], uint8 threshold)
{
    uint16 x = 0;
    uint16 y = 0;

    for(y = 0; y < MT9V03X_H; y++)
    {
        for(x = 0; x < MT9V03X_W; x++)
        {
            if(image[y][x] > threshold)
            {
                image[y][x] = 255;
            }
            else
            {
                image[y][x] = 0;
            }
        }
    }
}

uint8 camera_image_binary_otsu(uint8 image[MT9V03X_H][MT9V03X_W])
{
    uint16 x = 0;
    uint16 y = 0;
    uint16 threshold_temp = 0;
    uint8 threshold = 0;
    uint32 total = MT9V03X_IMAGE_SIZE;
    uint32 sum = 0;
    uint32 sum_background = 0;
    uint32 weight_background = 0;
    uint32 weight_foreground = 0;
    uint32 histogram[256] = {0};
    double mean_background = 0;
    double mean_foreground = 0;
    double between_class_variance = 0;
    double max_between_class_variance = 0;

    for(y = 0; y < MT9V03X_H; y++)
    {
        for(x = 0; x < MT9V03X_W; x++)
        {
            histogram[image[y][x]]++;
        }
    }

    for(threshold_temp = 0; threshold_temp < 256; threshold_temp++)
    {
        sum += threshold_temp * histogram[threshold_temp];
    }

    for(threshold_temp = 0; threshold_temp < 256; threshold_temp++)
    {
        weight_background += histogram[threshold_temp];
        if(weight_background == 0)
        {
            continue;
        }

        weight_foreground = total - weight_background;
        if(weight_foreground == 0)
        {
            break;
        }

        sum_background += threshold_temp * histogram[threshold_temp];
        mean_background = (double)sum_background / weight_background;
        mean_foreground = (double)(sum - sum_background) / weight_foreground;
        between_class_variance = (double)weight_background *
                                 (double)weight_foreground *
                                 (mean_background - mean_foreground) *
                                 (mean_background - mean_foreground);

        if(between_class_variance > max_between_class_variance)
        {
            max_between_class_variance = between_class_variance;
            threshold = (uint8)threshold_temp;
        }
    }

    vision_binary_fixed(image, threshold);

    return threshold;
}

uint8 camera_image_binary_otsu_roi(uint8 image[MT9V03X_H][MT9V03X_W], uint16 roi_row, uint16 roi_row_count, uint16 roi_column, uint16 roi_column_count)
{
    uint16 x = 0;
    uint16 y = 0;
    uint16 checked_rows = 0;
    uint16 checked_columns = 0;
    uint16 threshold_temp = 0;
    uint8  threshold = 0;
    uint32 total = 0;
    uint32 sum = 0;
    uint32 sum_background = 0;
    uint32 weight_background = 0;
    uint32 weight_foreground = 0;
    uint32 histogram[256] = {0};
    double mean_background = 0;
    double mean_foreground = 0;
    double between_class_variance = 0;
    double max_between_class_variance = 0;

    if((MT9V03X_H <= roi_row) || (MT9V03X_W <= roi_column))
    {
        return camera_image_binary_otsu(image);
    }

    if((0 == roi_row_count) || (0 == roi_column_count))
    {
        return camera_image_binary_otsu(image);
    }

    if((roi_row + 1) < roi_row_count)
    {
        return camera_image_binary_otsu(image);
    }

    if((MT9V03X_W - roi_column) < roi_column_count)
    {
        return camera_image_binary_otsu(image);
    }

    total = (uint32)roi_row_count * (uint32)roi_column_count;

    for(checked_rows = 0; checked_rows < roi_row_count; checked_rows++)
    {
        y = roi_row - checked_rows;

        for(checked_columns = 0; checked_columns < roi_column_count; checked_columns++)
        {
            x = roi_column + checked_columns;
            histogram[image[y][x]]++;
        }
    }

    for(threshold_temp = 0; threshold_temp < 256; threshold_temp++)
    {
        sum += threshold_temp * histogram[threshold_temp];
    }

    for(threshold_temp = 0; threshold_temp < 256; threshold_temp++)
    {
        weight_background += histogram[threshold_temp];
        if(weight_background == 0)
        {
            continue;
        }

        weight_foreground = total - weight_background;
        if(weight_foreground == 0)
        {
            break;
        }

        sum_background += threshold_temp * histogram[threshold_temp];
        mean_background = (double)sum_background / weight_background;
        mean_foreground = (double)(sum - sum_background) / weight_foreground;
        between_class_variance = (double)weight_background *
                                 (double)weight_foreground *
                                 (mean_background - mean_foreground) *
                                 (mean_background - mean_foreground);

        if(between_class_variance > max_between_class_variance)
        {
            max_between_class_variance = between_class_variance;
            threshold = (uint8)threshold_temp;
        }
    }

    if(0 == max_between_class_variance)
    {
        return camera_image_binary_otsu(image);
    }

    vision_binary_fixed(image, threshold);

    return threshold;
}

void camera_image_filter_isolated_black(uint8 image[MT9V03X_H][MT9V03X_W])
{
    uint16 x = 0;
    uint16 y = 0;
    int16 dx = 0;
    int16 dy = 0;
    uint8 black_count = 0;
    static uint8 image_temp[MT9V03X_H][MT9V03X_W];

    memcpy(image_temp[0], image[0], MT9V03X_IMAGE_SIZE);

    for(y = 1; y < MT9V03X_H - 1; y++)
    {
        for(x = 1; x < MT9V03X_W - 1; x++)
        {
            if(image_temp[y][x] == 0)
            {
                black_count = 0;

                for(dy = -1; dy <= 1; dy++)
                {
                    for(dx = -1; dx <= 1; dx++)
                    {
                        if(image_temp[y + dy][x + dx] == 0)
                        {
                            black_count++;
                        }
                    }
                }

                if(black_count <= 2)
                {
                    image[y][x] = 255;
                }
            }
        }
    }
}

void camera_image_filter_isolated_white(uint8 image[MT9V03X_H][MT9V03X_W])
{
    uint16 x = 0;
    uint16 y = 0;
    int16 dx = 0;
    int16 dy = 0;
    uint8 white_count = 0;
    static uint8 image_temp[MT9V03X_H][MT9V03X_W];

    memcpy(image_temp[0], image[0], MT9V03X_IMAGE_SIZE);

    for(y = 1; y < MT9V03X_H - 1; y++)
    {
        for(x = 1; x < MT9V03X_W - 1; x++)
        {
            if(image_temp[y][x] == 255)
            {
                white_count = 0;

                for(dy = -1; dy <= 1; dy++)
                {
                    for(dx = -1; dx <= 1; dx++)
                    {
                        if(image_temp[y + dy][x + dx] == 255)
                        {
                            white_count++;
                        }
                    }
                }

                if(white_count <= 2)
                {
                    image[y][x] = 0;
                }
            }
        }
    }
}


uint8 camera_image_check_jump_rows(uint8 image[MT9V03X_H][MT9V03X_W], uint16 check_row, uint16 check_row_count, uint16 check_column, uint16 check_column_count, uint16 black_count)
{
    uint16 x = 0;
    int16 y = 0;
    uint16 checked_columns = 0;
    uint16 current_black_count = 0;
    uint16 checked_rows = 0;

    if((MT9V03X_H <= check_row) || (MT9V03X_W <= check_column))
    {
        return 0;
    }

    if((0 == check_row_count) || (0 == check_column_count))
    {
        return 0;
    }

    for(y = (int16)check_row; (0 <= y) && (checked_rows < check_row_count); y--)
    {
        current_black_count = 0;
        checked_columns = 0;

        for(x = check_column; (x < MT9V03X_W) && (checked_columns < check_column_count); x++)
        {
            if(image[y][x] == 0)
            {
                current_black_count++;
            }

            checked_columns++;
        }

        if(checked_columns != check_column_count)
        {
            return 0;
        }

        if(current_black_count < black_count)
        {
            return 0;
        }

        checked_rows++;
    }

    return (checked_rows == check_row_count);
}

uint8 camera_image_check_jump_columns(uint8 image[MT9V03X_H][MT9V03X_W], uint16 check_row, uint16 check_row_count, uint16 check_column, uint16 check_column_count, uint16 black_count)
{
    uint16 x = 0;
    int16 y = 0;
    uint16 checked_columns = 0;
    uint16 current_black_count = 0;
    uint16 checked_rows = 0;

    if((MT9V03X_H <= check_row) || (MT9V03X_W <= check_column))
    {
        return 0;
    }

    if((0 == check_row_count) || (0 == check_column_count))
    {
        return 0;
    }

    for(x = check_column; (x < MT9V03X_W) && (checked_columns < check_column_count); x++)
    {
        current_black_count = 0;
        checked_rows = 0;

        for(y = (int16)check_row; (0 <= y) && (checked_rows < check_row_count); y--)
        {
            if(image[y][x] == 0)
            {
                current_black_count++;
            }

            checked_rows++;
        }

        if(checked_rows != check_row_count)
        {
            return 0;
        }

        if(current_black_count < black_count)
        {
            return 0;
        }

        checked_columns++;
    }

    return (checked_columns == check_column_count);
}

uint8 camera_image_check_jump_area(uint8 image[MT9V03X_H][MT9V03X_W], uint16 check_row, uint16 check_row_count, uint16 check_column, uint16 check_column_count, uint32 dot_count, uint32 dot_type)
{
    uint16 x = 0;
    uint16 y = 0;
    uint16 checked_rows = 0;
    uint16 checked_columns = 0;
    uint8 target_dot_value = 0;
    uint32 current_dot_count = 0;
    uint32 check_area_size = 0;

    if((MT9V03X_H <= check_row) || (MT9V03X_W <= check_column))
    {
        return 0;
    }

    if((0 == check_row_count) || (0 == check_column_count))
    {
        return 0;
    }

    if((check_row + 1) < check_row_count)
    {
        return 0;
    }

    if((MT9V03X_W - check_column) < check_column_count)
    {
        return 0;
    }

    if(CAMERA_IMAGE_DOT_BLACK == dot_type)
    {
        target_dot_value = 0;
    }
    else if(CAMERA_IMAGE_DOT_WHITE == dot_type)
    {
        target_dot_value = 255;
    }
    else
    {
        return 0;
    }

    check_area_size = (uint32)check_row_count * (uint32)check_column_count;
    if(check_area_size < dot_count)
    {
        return 0;
    }

    if(0 == dot_count)
    {
        return 1;
    }

    for(checked_rows = 0; checked_rows < check_row_count; checked_rows++)
    {
        y = check_row - checked_rows;

        for(checked_columns = 0; checked_columns < check_column_count; checked_columns++)
        {
            x = check_column + checked_columns;

            if(image[y][x] == target_dot_value)
            {
                current_dot_count++;

                if(current_dot_count >= dot_count)
                {
                    return 1;
                }
            }
        }
    }

    return 0;
}

uint8 camera_image_check_jump_strict(uint8 image[MT9V03X_H][MT9V03X_W], uint16 check_row, uint16 check_row_count, uint16 row_black_count, uint16 check_column, uint16 check_column_count, uint16 column_black_count)
{
    if(!camera_image_check_jump_rows(image, check_row, check_row_count, check_column, check_column_count, row_black_count))
    {
        return 0;
    }

    return camera_image_check_jump_columns(image, check_row, check_row_count, check_column, check_column_count, column_black_count);
}


// 跳跃触发冷却时间检�?
uint8 camera_image_jump_trigger_filter(uint32 time_ms, uint32 cooldown_time_ms, uint8 jump_detected)
{
    static uint32 last_jump_time = 0;
    static uint8 has_triggered = 0;

    if(!jump_detected)
    {
        return 0;
    }

    if(has_triggered && ((time_ms - last_jump_time) < cooldown_time_ms))
    {
        return 0;
    }

    has_triggered = 1;
    last_jump_time = time_ms;

    return 1;
}

uint8 camera_dot_type_switch(void)
{
    if(jump_trigger_count < CAMERA_DOT_TYPE_LIST_COUNT)
    {
        jump_trigger_count++;
    }

    if(jump_trigger_count >= CAMERA_DOT_TYPE_LIST_COUNT)
    {
        return (uint8)dot_type_list[CAMERA_DOT_TYPE_LIST_COUNT - 1];
    }

    return (uint8)dot_type_list[jump_trigger_count];
}

uint32 camera_dot_type_get_steps(void)
{
    return jump_trigger_count;
}

uint8 camera_dot_type_reset(void)
{
    jump_trigger_count = 0;
    return (uint8)dot_type_list[0];
}




const CameraRowSpeedRule_t camera_row_speed_rules[CAMERA_ROW_SPEED_RULE_COUNT] =
{
    // 旧版自适应算法
    
    {116u, 115u},
    {127u, 105u},
    {141u, 95u},
    {162u, 85u},
    {200u, 75u},
    

    // 当前适用的算法
    /*{116u, 105U},
    {127u, 95u},
    {141u, 85u},
    {162u, 65u},
    {200u, 65u},
    */

};

uint16 camera_check_row_from_speed(uint16 car_speed, int8 aggressive_coeff)
{
    int16 row_bias = (int16)aggressive_coeff * (int16)CAMERA_ROW_AGGRESSIVE_BIAS;
    uint8 i = 0;

    for(i = 0; i < CAMERA_ROW_SPEED_RULE_COUNT; i++)
    {
        int16 max_speed = (int16)camera_row_speed_rules[i].max_speed;

        if(i < (CAMERA_ROW_SPEED_RULE_COUNT - 1u))
        {
            max_speed += row_bias;
        }

        if(car_speed <= max_speed)
        {
            return camera_row_speed_rules[i].check_row;
        }
    }

    return camera_row_speed_rules[CAMERA_ROW_SPEED_RULE_COUNT - 1u].check_row;
}

void camera_jump_params_set_row_by_speed(JumpDetectParams_t *jump_params, uint16 car_speed, int8 aggressive_coeff)
{
    if(NULL == jump_params)
    {
        return;
    }

    jump_params->check_row = camera_check_row_from_speed(car_speed, aggressive_coeff);
}
