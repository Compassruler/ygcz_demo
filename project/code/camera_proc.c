#include "camera_proc.h"
#include <math.h>
#include <string.h>

// ==================================================== 参数调节 ====================================================
#define CAMERA_BRIDGE_EDGE_CANDIDATE_COUNT      (3u)    // 每行每侧最多保留的边缘候选数量
#define CAMERA_BRIDGE_PAIR_STATE_COUNT          (9u)    // 每行左右候选最多产生的组合数量
#define CAMERA_BRIDGE_INVALID_STATE_INDEX       (0xFFu)
#define CAMERA_BRIDGE_POSITION_COST_WEIGHT      (8)     // 相邻行边线位置变化代价
#define CAMERA_BRIDGE_WIDTH_COST_WEIGHT         (4)     // 相邻行赛道宽度变化代价
#define CAMERA_BRIDGE_MODEL_COST_WEIGHT         (3)     // 与上一帧预测模型偏差代价
#define CAMERA_ROW_SPEED_RULE_COUNT              (5u)

// 跳跃次序列表
static const uint32 dot_type_list[CAMERA_DOT_TYPE_LIST_COUNT] =
{
    CAMERA_IMAGE_DOT_WHITE,  // 1
    CAMERA_IMAGE_DOT_BLACK,  // 2
    CAMERA_IMAGE_DOT_WHITE,  // 3
};

typedef struct
{
    uint16 max_speed;  // 最高速度
    uint16 check_row;  // 检测的 Row
} CamprocRowSpeedRule_t;  // 自适应 Row - Speed 结构体

// 自适应 Row - Speed 结构体列表
static const CamprocRowSpeedRule_t camproc_row_speed_rules[CAMERA_ROW_SPEED_RULE_COUNT] =
{
    // 旧版自适应算法
    {116, 115},
    {127, 105},
    {141, 95},
    {162, 85},
    {200, 75},
    
    // 新版自适应算法
    /*
    {116, 105},
    {127, 95},
    {141, 85},
    {162, 65},
    {200, 65},
    */

};


// ==================================================== 公共函数 ====================================================
void camproc_pub_thresh_bin(uint8 image[MT9V03X_H][MT9V03X_W], uint8 threshold)
{
    uint16 x = 0;  // 遍历中的 x 坐标
    uint16 y = 0;  // 遍历中的 y 坐标

    // 遍历图像
    for(y = 0; y < MT9V03X_H; y++)
    {
        for(x = 0; x < MT9V03X_W; x++)
        {
            image[y][x] = (image[y][x] > threshold) ? 255 : 0;  // 大于为 白色 否则为 黑色
        }
    }
}


// ==================================================== 单边桥函数 ====================================================
typedef struct
{
    uint8 valid;
    uint8 x;
    uint16 strength;
} CameraBridgeEdgeCandidate_t;

typedef struct
{
    uint8 valid;
    uint8 left_x;
    uint8 right_x;
    uint8 previous_index;
    uint8 path_count;
    uint16 strength;
    int32 cost;
} CameraBridgePairState_t;

typedef struct
{
    uint8 y;
    uint8 left_x;
    uint8 right_x;
    uint8 weight;
} CameraBridgeSample_t;

static CameraBridgePairState_t camera_bridge_pair_states[MT9V03X_H][CAMERA_BRIDGE_PAIR_STATE_COUNT];
static uint8 camera_bridge_state_count[MT9V03X_H];
static uint8 camera_bridge_anchor_y[MT9V03X_H];

static uint8 camera_bridge_previous_model_valid = 0;
static float camera_bridge_previous_center_slope = 0.0f;
static float camera_bridge_previous_center_intercept = 0.0f;
static float camera_bridge_previous_half_width_slope = 0.0f;
static float camera_bridge_previous_half_width_intercept = 0.0f;

static uint16 camera_bridge_round_and_limit_x(float value)
{
    int32 rounded_value = 0;

    rounded_value = (value >= 0.0f) ?
        (int32)(value + 0.5f) : (int32)(value - 0.5f);

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

static uint16 camera_bridge_abs_diff_u16(uint16 value_a, uint16 value_b)
{
    return (value_a >= value_b) ?
        (uint16)(value_a - value_b) : (uint16)(value_b - value_a);
}

static float camera_bridge_abs_float(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static void camera_bridge_track_reset(void)
{
    camera_bridge_previous_model_valid = 0;
    camera_bridge_previous_center_slope = 0.0f;
    camera_bridge_previous_center_intercept = 0.0f;
    camera_bridge_previous_half_width_slope = 0.0f;
    camera_bridge_previous_half_width_intercept = 0.0f;
}

static void camera_bridge_sort_candidates(CameraBridgeEdgeCandidate_t candidates[CAMERA_BRIDGE_EDGE_CANDIDATE_COUNT])
{
    uint8 i = 0;
    uint8 j = 0;
    CameraBridgeEdgeCandidate_t temporary = {0};

    for(i = 0; i < CAMERA_BRIDGE_EDGE_CANDIDATE_COUNT; i++)
    {
        for(j = (uint8)(i + 1u); j < CAMERA_BRIDGE_EDGE_CANDIDATE_COUNT; j++)
        {
            if((!candidates[i].valid && candidates[j].valid) ||
               (candidates[i].valid && candidates[j].valid &&
                (candidates[j].strength > candidates[i].strength)))
            {
                temporary = candidates[i];
                candidates[i] = candidates[j];
                candidates[j] = temporary;
            }
        }
    }
}

static void camera_bridge_add_candidate(
    CameraBridgeEdgeCandidate_t candidates[CAMERA_BRIDGE_EDGE_CANDIDATE_COUNT],
    uint16 x,
    uint16 strength,
    uint8 min_spacing)
{
    uint8 index = 0;

    // 同一条边缘附近只保留最强响应，避免三个候选集中在相邻像素。
    for(index = 0; index < CAMERA_BRIDGE_EDGE_CANDIDATE_COUNT; index++)
    {
        if(candidates[index].valid &&
           (camera_bridge_abs_diff_u16(candidates[index].x, x) <= min_spacing))
        {
            if(strength > candidates[index].strength)
            {
                candidates[index].x = (uint8)x;
                candidates[index].strength = strength;
                camera_bridge_sort_candidates(candidates);
            }

            return;
        }
    }

    for(index = 0; index < CAMERA_BRIDGE_EDGE_CANDIDATE_COUNT; index++)
    {
        if(!candidates[index].valid)
        {
            candidates[index].valid = 1;
            candidates[index].x = (uint8)x;
            candidates[index].strength = strength;
            camera_bridge_sort_candidates(candidates);
            return;
        }
    }

    if(strength > candidates[CAMERA_BRIDGE_EDGE_CANDIDATE_COUNT - 1u].strength)
    {
        candidates[CAMERA_BRIDGE_EDGE_CANDIDATE_COUNT - 1u].x = (uint8)x;
        candidates[CAMERA_BRIDGE_EDGE_CANDIDATE_COUNT - 1u].strength = strength;
        camera_bridge_sort_candidates(candidates);
    }
}

static uint8 camera_bridge_find_edge_candidates(
    const uint8 image[MT9V03X_H][MT9V03X_W],
    uint16 y,
    uint16 search_start,
    uint16 search_end,
    uint8 left_edge,
    const CameraBridgeParams_t *params,
    CameraBridgeEdgeCandidate_t candidates[CAMERA_BRIDGE_EDGE_CANDIDATE_COUNT])
{
    uint8 index = 0;
    uint8 offset = 0;
    uint8 candidate_count = 0;
    uint16 x = 0;
    uint16 left_sum = 0;
    uint16 right_sum = 0;
    uint16 strength = 0;
    int32 signed_strength = 0;

    memset(candidates, 0, sizeof(CameraBridgeEdgeCandidate_t) * CAMERA_BRIDGE_EDGE_CANDIDATE_COUNT);

    if(search_start < params->edge_window)
    {
        search_start = params->edge_window;
    }

    if(search_end >= (MT9V03X_W - params->edge_window))
    {
        search_end = MT9V03X_W - params->edge_window - 1u;
    }

    if(search_start > search_end)
    {
        return 0;
    }

    for(offset = 1; offset <= params->edge_window; offset++)
    {
        left_sum += image[y][search_start - offset];
        right_sum += image[y][search_start + offset];
    }

    for(x = search_start; x <= search_end; x++)
    {
        // 左边线为黑到白，右边线为白到黑，分别保留对应方向的灰度突变。
        signed_strength = left_edge ?
            (int32)right_sum - left_sum : (int32)left_sum - right_sum;

        if(signed_strength >= (int32)params->min_edge_contrast * params->edge_window)
        {
            strength = (uint16)signed_strength;
            camera_bridge_add_candidate(candidates, x, strength, params->edge_window);
        }

        if(x < search_end)
        {
            left_sum = (uint16)(
                left_sum + image[y][x] - image[y][x - params->edge_window]);
            right_sum = (uint16)(
                right_sum + image[y][x + params->edge_window + 1u] - image[y][x + 1u]);
        }
    }

    for(index = 0; index < CAMERA_BRIDGE_EDGE_CANDIDATE_COUNT; index++)
    {
        if(candidates[index].valid)
        {
            candidate_count++;
        }
    }

    return candidate_count;
}

static uint8 camera_bridge_collect_side_candidates(
    const uint8 image[MT9V03X_H][MT9V03X_W],
    uint16 y,
    uint8 left_edge,
    uint8 use_previous_model,
    const CameraBridgeParams_t *params,
    CameraBridgeEdgeCandidate_t candidates[CAMERA_BRIDGE_EDGE_CANDIDATE_COUNT])
{
    uint16 search_start = left_edge ? params->left_edge_min_x : params->right_edge_min_x;
    uint16 search_end = left_edge ? params->left_edge_max_x : params->right_edge_max_x;
    uint16 predicted_x = 0;
    uint16 local_start = 0;
    uint16 local_end = 0;
    float center_x = 0.0f;
    float half_width = 0.0f;

    if(use_previous_model && camera_bridge_previous_model_valid)
    {
        center_x = camera_bridge_previous_center_slope * (float)y +
                   camera_bridge_previous_center_intercept;
        half_width = camera_bridge_previous_half_width_slope * (float)y +
                     camera_bridge_previous_half_width_intercept;
        predicted_x = camera_bridge_round_and_limit_x(
            left_edge ? center_x - half_width : center_x + half_width);

        local_start = (predicted_x > params->local_search_radius) ?
            (uint16)(predicted_x - params->local_search_radius) : 0u;
        local_end = (uint16)(predicted_x + params->local_search_radius);

        if(local_start < search_start) local_start = search_start;
        if(local_end > search_end) local_end = search_end;

        if(camera_bridge_find_edge_candidates(
            image, y, local_start, local_end, left_edge, params, candidates))
        {
            return 1;
        }
    }

    return camera_bridge_find_edge_candidates(
        image, y, search_start, search_end, left_edge, params, candidates);
}

static uint8 camera_bridge_build_path(
    const uint8 image[MT9V03X_H][MT9V03X_W],
    const CameraBridgeParams_t *params,
    uint8 use_previous_model,
    CameraBridgeSample_t samples[MT9V03X_H],
    uint16 *sample_count)
{
    int16 y = 0;
    int16 best_anchor_index = -1;
    uint8 left_index = 0;
    uint8 right_index = 0;
    uint8 pair_index = 0;
    uint8 previous_index = 0;
    uint8 best_state_index = CAMERA_BRIDGE_INVALID_STATE_INDEX;
    uint8 anchor_count = 0;
    uint8 pair_count = 0;
    uint8 best_path_count = 0;
    uint16 lane_width = 0;
    uint16 previous_width = 0;
    uint16 left_jump = 0;
    uint16 right_jump = 0;
    uint16 width_jump = 0;
    uint16 row_gap = 0;
    uint16 gap_step_count = 0;
    uint16 allowed_jump = 0;
    uint16 max_pair_strength = (uint16)(2u * params->edge_window * 255u);
    uint16 path_sample_count = 0;
    int32 base_cost = 0;
    int32 transition_cost = 0;
    int32 total_cost = 0;
    int32 best_cost = 0x7FFFFFFF;
    uint16 predicted_left = 0;
    uint16 predicted_right = 0;
    float predicted_center = 0.0f;
    float predicted_half_width = 0.0f;
    CameraBridgeEdgeCandidate_t left_candidates[CAMERA_BRIDGE_EDGE_CANDIDATE_COUNT];
    CameraBridgeEdgeCandidate_t right_candidates[CAMERA_BRIDGE_EDGE_CANDIDATE_COUNT];
    CameraBridgePairState_t *state = NULL;
    CameraBridgePairState_t *previous_state = NULL;

    if(NULL == sample_count)
    {
        return 0;
    }

    *sample_count = 0;

    for(y = (int16)params->search_bottom; y >= (int16)params->search_top;
        y = (int16)(y - params->row_step))
    {
        if(!camera_bridge_collect_side_candidates(
               image, (uint16)y, 1, use_previous_model, params, left_candidates) ||
           !camera_bridge_collect_side_candidates(
               image, (uint16)y, 0, use_previous_model, params, right_candidates))
        {
            continue;
        }

        memset(camera_bridge_pair_states[anchor_count], 0,
               sizeof(camera_bridge_pair_states[anchor_count]));
        pair_count = 0;

        for(left_index = 0;
            left_index < CAMERA_BRIDGE_EDGE_CANDIDATE_COUNT;
            left_index++)
        {
            if(!left_candidates[left_index].valid)
            {
                continue;
            }

            for(right_index = 0;
                right_index < CAMERA_BRIDGE_EDGE_CANDIDATE_COUNT;
                right_index++)
            {
                if(!right_candidates[right_index].valid ||
                   (left_candidates[left_index].x >= right_candidates[right_index].x))
                {
                    continue;
                }

                lane_width = (uint16)(
                    right_candidates[right_index].x - left_candidates[left_index].x);
                if((lane_width < params->min_lane_width) ||
                   (lane_width > params->max_lane_width))
                {
                    continue;
                }

                state = &camera_bridge_pair_states[anchor_count][pair_count];
                state->valid = 1;
                state->left_x = left_candidates[left_index].x;
                state->right_x = right_candidates[right_index].x;
                state->strength = (uint16)(
                    left_candidates[left_index].strength +
                    right_candidates[right_index].strength);
                state->previous_index = CAMERA_BRIDGE_INVALID_STATE_INDEX;
                state->path_count = 1;
                base_cost = (int32)max_pair_strength - state->strength;

                if(use_previous_model && camera_bridge_previous_model_valid)
                {
                    predicted_center =
                        camera_bridge_previous_center_slope * (float)y +
                        camera_bridge_previous_center_intercept;
                    predicted_half_width =
                        camera_bridge_previous_half_width_slope * (float)y +
                        camera_bridge_previous_half_width_intercept;
                    predicted_left = camera_bridge_round_and_limit_x(
                        predicted_center - predicted_half_width);
                    predicted_right = camera_bridge_round_and_limit_x(
                        predicted_center + predicted_half_width);

                    base_cost +=
                        (camera_bridge_abs_diff_u16(state->left_x, predicted_left) +
                         camera_bridge_abs_diff_u16(state->right_x, predicted_right)) *
                        CAMERA_BRIDGE_MODEL_COST_WEIGHT;
                }

                state->cost = base_cost;

                if(anchor_count > 0u)
                {
                    row_gap = (uint16)(
                        camera_bridge_anchor_y[anchor_count - 1u] - (uint16)y);
                    gap_step_count = (uint16)(
                        (row_gap + params->row_step - 1u) / params->row_step);

                    if(gap_step_count <= (uint16)params->max_missing_rows + 1u)
                    {
                        allowed_jump = (uint16)(params->max_edge_jump * gap_step_count);

                        for(previous_index = 0;
                            previous_index < camera_bridge_state_count[anchor_count - 1u];
                            previous_index++)
                        {
                            previous_state =
                                &camera_bridge_pair_states[anchor_count - 1u][previous_index];
                            left_jump = camera_bridge_abs_diff_u16(
                                state->left_x, previous_state->left_x);
                            right_jump = camera_bridge_abs_diff_u16(
                                state->right_x, previous_state->right_x);

                            if((left_jump > allowed_jump) || (right_jump > allowed_jump))
                            {
                                continue;
                            }

                            previous_width = (uint16)(
                                previous_state->right_x - previous_state->left_x);
                            width_jump = camera_bridge_abs_diff_u16(lane_width, previous_width);
                            transition_cost =
                                (int32)(left_jump + right_jump) *
                                    CAMERA_BRIDGE_POSITION_COST_WEIGHT +
                                (int32)width_jump * CAMERA_BRIDGE_WIDTH_COST_WEIGHT;
                            total_cost = previous_state->cost + base_cost + transition_cost;

                            if(((uint16)previous_state->path_count + 1u > state->path_count) ||
                               (((uint16)previous_state->path_count + 1u == state->path_count) &&
                                (total_cost < state->cost)))
                            {
                                state->path_count = (uint8)(previous_state->path_count + 1u);
                                state->previous_index = previous_index;
                                state->cost = total_cost;
                            }
                        }
                    }
                }

                pair_count++;
            }
        }

        if(0u == pair_count)
        {
            continue;
        }

        camera_bridge_anchor_y[anchor_count] = (uint8)y;
        camera_bridge_state_count[anchor_count] = pair_count;
        anchor_count++;
    }

    for(pair_index = 0; pair_index < anchor_count; pair_index++)
    {
        for(previous_index = 0;
            previous_index < camera_bridge_state_count[pair_index];
            previous_index++)
        {
            state = &camera_bridge_pair_states[pair_index][previous_index];

            if((state->path_count > best_path_count) ||
               ((state->path_count == best_path_count) && (state->cost < best_cost)))
            {
                best_path_count = state->path_count;
                best_cost = state->cost;
                best_anchor_index = pair_index;
                best_state_index = previous_index;
            }
        }
    }

    if((best_anchor_index < 0) ||
       (best_state_index == CAMERA_BRIDGE_INVALID_STATE_INDEX) ||
       (best_path_count < params->min_point_count))
    {
        return 0;
    }

    while((best_anchor_index >= 0) &&
          (best_state_index != CAMERA_BRIDGE_INVALID_STATE_INDEX) &&
          (path_sample_count < MT9V03X_H))
    {
        state = &camera_bridge_pair_states[best_anchor_index][best_state_index];
        samples[path_sample_count].y = camera_bridge_anchor_y[best_anchor_index];
        samples[path_sample_count].left_x = state->left_x;
        samples[path_sample_count].right_x = state->right_x;
        samples[path_sample_count].weight = (uint8)(
            1u + state->strength / (2u * params->edge_window * 64u));
        if(samples[path_sample_count].weight > 4u)
        {
            samples[path_sample_count].weight = 4u;
        }

        path_sample_count++;
        best_state_index = state->previous_index;
        best_anchor_index--;
    }

    *sample_count = path_sample_count;
    return path_sample_count >= params->min_point_count;
}

static uint8 camera_bridge_fit_model(
    const CameraBridgeSample_t samples[MT9V03X_H],
    const uint8 inlier_mask[MT9V03X_H],
    uint16 sample_count,
    uint8 fit_center,
    float *slope,
    float *intercept,
    float *mean_square_error,
    uint16 *inlier_count)
{
    uint16 index = 0;
    uint16 count = 0;
    float y = 0.0f;
    float value = 0.0f;
    float weight = 0.0f;
    float weight_sum = 0.0f;
    float sum_y = 0.0f;
    float sum_value = 0.0f;
    float sum_yy = 0.0f;
    float sum_y_value = 0.0f;
    float denominator = 0.0f;
    float error = 0.0f;
    float error_sum = 0.0f;

    if((NULL == slope) || (NULL == intercept) ||
       (NULL == mean_square_error) || (NULL == inlier_count))
    {
        return 0;
    }

    for(index = 0; index < sample_count; index++)
    {
        if(!inlier_mask[index])
        {
            continue;
        }

        y = (float)samples[index].y;
        value = fit_center ?
            ((float)samples[index].left_x + samples[index].right_x) * 0.5f :
            ((float)samples[index].right_x - samples[index].left_x) * 0.5f;
        weight = (float)samples[index].weight;
        weight_sum += weight;
        sum_y += weight * y;
        sum_value += weight * value;
        sum_yy += weight * y * y;
        sum_y_value += weight * y * value;
        count++;
    }

    if((count < 2u) || (weight_sum <= 0.0f))
    {
        return 0;
    }

    denominator = weight_sum * sum_yy - sum_y * sum_y;
    if(camera_bridge_abs_float(denominator) < 0.0001f)
    {
        return 0;
    }

    *slope = (weight_sum * sum_y_value - sum_y * sum_value) / denominator;
    *intercept = (sum_value - *slope * sum_y) / weight_sum;

    for(index = 0; index < sample_count; index++)
    {
        if(!inlier_mask[index])
        {
            continue;
        }

        value = fit_center ?
            ((float)samples[index].left_x + samples[index].right_x) * 0.5f :
            ((float)samples[index].right_x - samples[index].left_x) * 0.5f;
        error = value - (*slope * (float)samples[index].y + *intercept);
        error_sum += error * error;
    }

    *mean_square_error = error_sum / (float)count;
    *inlier_count = count;
    return 1;
}

static uint8 camera_bridge_fit_path(
    const CameraBridgeSample_t samples[MT9V03X_H],
    uint16 sample_count,
    const CameraBridgeParams_t *params,
    CameraBridgeResult_t *result)
{
    uint16 index = 0;
    uint16 inlier_count = 0;
    uint16 top = MT9V03X_H;
    uint16 bottom = 0;
    uint8 inlier_mask[MT9V03X_H];
    float center_slope = 0.0f;
    float center_intercept = 0.0f;
    float half_width_slope = 0.0f;
    float half_width_intercept = 0.0f;
    float center_error = 0.0f;
    float width_error = 0.0f;
    float center_residual = 0.0f;
    float width_residual = 0.0f;
    float center_top = 0.0f;
    float center_bottom = 0.0f;
    float half_width_top = 0.0f;
    float half_width_bottom = 0.0f;
    float left_top = 0.0f;
    float left_bottom = 0.0f;
    float right_top = 0.0f;
    float right_bottom = 0.0f;

    memset(inlier_mask, 1, sizeof(inlier_mask));

    if(!camera_bridge_fit_model(
           samples, inlier_mask, sample_count, 1,
           &center_slope, &center_intercept, &center_error, &inlier_count) ||
       !camera_bridge_fit_model(
           samples, inlier_mask, sample_count, 0,
           &half_width_slope, &half_width_intercept, &width_error, &inlier_count))
    {
        return 0;
    }

    // 第一次拟合后删除偏离中线或赛道宽度模型的点，再进行最终拟合。
    for(index = 0; index < sample_count; index++)
    {
        center_residual =
            ((float)samples[index].left_x + samples[index].right_x) * 0.5f -
            (center_slope * samples[index].y + center_intercept);
        width_residual =
            ((float)samples[index].right_x - samples[index].left_x) * 0.5f -
            (half_width_slope * samples[index].y + half_width_intercept);

        if((camera_bridge_abs_float(center_residual) > params->center_residual_limit) ||
           (camera_bridge_abs_float(width_residual) > params->width_residual_limit))
        {
            inlier_mask[index] = 0;
        }
    }

    if(!camera_bridge_fit_model(
           samples, inlier_mask, sample_count, 1,
           &center_slope, &center_intercept, &center_error, &inlier_count) ||
       (inlier_count < params->min_point_count) ||
       !camera_bridge_fit_model(
           samples, inlier_mask, sample_count, 0,
           &half_width_slope, &half_width_intercept, &width_error, &inlier_count) ||
       (inlier_count < params->min_point_count))
    {
        return 0;
    }

    for(index = 0; index < sample_count; index++)
    {
        if(!inlier_mask[index])
        {
            continue;
        }

        if(samples[index].y < top) top = samples[index].y;
        if(samples[index].y > bottom) bottom = samples[index].y;
    }

    if((top >= MT9V03X_H) || (bottom <= top) ||
       ((bottom - top) < params->min_y_span))
    {
        return 0;
    }

    center_top = center_slope * (float)top + center_intercept;
    center_bottom = center_slope * (float)bottom + center_intercept;
    half_width_top = half_width_slope * (float)top + half_width_intercept;
    half_width_bottom = half_width_slope * (float)bottom + half_width_intercept;

    if((half_width_top <= 0.0f) || (half_width_bottom <= 0.0f) ||
       ((half_width_top * 2.0f) < params->min_lane_width) ||
       ((half_width_top * 2.0f) > params->max_lane_width) ||
       ((half_width_bottom * 2.0f) < params->min_lane_width) ||
       ((half_width_bottom * 2.0f) > params->max_lane_width) ||
       ((half_width_bottom + params->width_residual_limit) < half_width_top))
    {
        return 0;
    }

    left_top = center_top - half_width_top;
    left_bottom = center_bottom - half_width_bottom;
    right_top = center_top + half_width_top;
    right_bottom = center_bottom + half_width_bottom;

    if((left_top < 0.0f) || (left_bottom < 0.0f) ||
       (right_top >= MT9V03X_W) || (right_bottom >= MT9V03X_W) ||
       (left_top >= right_top) || (left_bottom >= right_bottom))
    {
        return 0;
    }

    result->top = top;
    result->bottom = bottom;
    result->point_count = inlier_count;
    result->left_x1 = camera_bridge_round_and_limit_x(left_top);
    result->left_y1 = top;
    result->left_x2 = camera_bridge_round_and_limit_x(left_bottom);
    result->left_y2 = bottom;
    result->right_x1 = camera_bridge_round_and_limit_x(right_top);
    result->right_y1 = top;
    result->right_x2 = camera_bridge_round_and_limit_x(right_bottom);
    result->right_y2 = bottom;
    result->center_x1 = camera_bridge_round_and_limit_x(center_top);
    result->center_y1 = top;
    result->center_x2 = camera_bridge_round_and_limit_x(center_bottom);
    result->center_y2 = bottom;
    result->center_slope = center_slope;
    result->center_intercept = center_intercept;
    result->half_width_slope = half_width_slope;
    result->half_width_intercept = half_width_intercept;
    result->center_error = center_error;
    result->width_error = width_error;
    result->valid = 1;

    return 1;
}

static uint8 camera_bridge_detect_once(
    const uint8 image[MT9V03X_H][MT9V03X_W],
    const CameraBridgeParams_t *params,
    uint8 use_previous_model,
    CameraBridgeResult_t *result)
{
    uint16 sample_count = 0;
    CameraBridgeSample_t samples[MT9V03X_H];

    if(!camera_bridge_build_path(
           image, params, use_previous_model, samples, &sample_count))
    {
        return 0;
    }

    return camera_bridge_fit_path(samples, sample_count, params, result);
}

uint8 camproc_bridge_detect(
    const uint8 image[MT9V03X_H][MT9V03X_W],
    const CameraBridgeParams_t *params,
    CameraBridgeResult_t *result)
{
    uint16 maximum_point_count = 0;

    if(NULL == result)
    {
        return 0;
    }

    memset(result, 0, sizeof(*result));

    if((NULL == image) || (NULL == params))
    {
        return 0;
    }

    if((params->search_top >= params->search_bottom) ||
       (params->search_bottom >= MT9V03X_H) ||
       (params->left_edge_min_x >= params->left_edge_max_x) ||
       (params->left_edge_max_x >= MT9V03X_W) ||
       (params->right_edge_min_x >= params->right_edge_max_x) ||
       (params->right_edge_max_x >= MT9V03X_W) ||
       (params->min_lane_width >= params->max_lane_width) ||
       (params->max_lane_width >= MT9V03X_W) ||
       (0u == params->max_edge_jump) ||
       (params->min_point_count < 2u) ||
       (0u == params->min_y_span) ||
       (0u == params->center_residual_limit) ||
       (0u == params->width_residual_limit) ||
       (0u == params->row_step) ||
       (0u == params->edge_window) ||
       (params->edge_window > 8u) ||
       (0u == params->min_edge_contrast) ||
       (params->local_search_radius < params->edge_window))
    {
        return 0;
    }

    maximum_point_count = (uint16)(
        (params->search_bottom - params->search_top) / params->row_step + 1u);
    if((params->min_point_count > maximum_point_count) ||
       (params->min_lane_width >
        (params->right_edge_max_x - params->left_edge_min_x)))
    {
        return 0;
    }

    // 上一帧模型附近优先局部搜索；失败后同一帧立即执行完整搜索。
    if(camera_bridge_previous_model_valid &&
       camera_bridge_detect_once(image, params, 1, result))
    {
        camera_bridge_previous_center_slope = result->center_slope;
        camera_bridge_previous_center_intercept = result->center_intercept;
        camera_bridge_previous_half_width_slope = result->half_width_slope;
        camera_bridge_previous_half_width_intercept = result->half_width_intercept;
        return 1;
    }

    memset(result, 0, sizeof(*result));
    if(camera_bridge_detect_once(image, params, 0, result))
    {
        camera_bridge_previous_model_valid = 1;
        camera_bridge_previous_center_slope = result->center_slope;
        camera_bridge_previous_center_intercept = result->center_intercept;
        camera_bridge_previous_half_width_slope = result->half_width_slope;
        camera_bridge_previous_half_width_intercept = result->half_width_intercept;
        return 1;
    }

    camera_bridge_track_reset();
    return 0;
}

static int16 camproc_bridge_yaw_offset_to_control(
    int16 yaw_offset_d10,
    const CameraBridgeAlignParams_t *align_params)
{
    float control_output = 0.0f;
    int32 control_value = 0;

    control_output = (float)yaw_offset_d10 * 0.1f
                   * align_params->control_gain_per_deg
                   * align_params->control_direction;
    control_value = (control_output >= 0.0f) ?
        (int32)(control_output + 0.5f) : (int32)(control_output - 0.5f);

    if(control_value > align_params->control_limit)
    {
        control_value = align_params->control_limit;
    }
    else if(control_value < -align_params->control_limit)
    {
        control_value = -align_params->control_limit;
    }

    return (int16)control_value;
}

void camproc_bridge_align_reset(CameraBridgeAlignState_t *align_state)
{
    if(NULL == align_state)
    {
        return;
    }

    memset(align_state, 0, sizeof(*align_state));
    align_state->phase = CAMERA_BRIDGE_ALIGN_TRACK;
    camera_bridge_track_reset();
}

uint8 camproc_bridge_align_update(
    const CameraBridgeResult_t *bridge_result,
    const CameraBridgeAlignParams_t *align_params,
    CameraBridgeAlignState_t *align_state,
    CameraBridgeAlignResult_t *align_result)
{
    uint16 active_x = 0;
    uint16 far_x = 0;
    uint16 near_x = 0;
    uint8 center_line_inside = 0;
    float point_error = 0.0f;
    float yaw_offset = 0.0f;
    int32 yaw_offset_d10 = 0;
    int32 yaw_change_d10 = 0;
    int32 far_error = 0;
    int32 near_error = 0;

    if(NULL == align_result)
    {
        return 0;
    }

    memset(align_result, 0, sizeof(*align_result));

    if((NULL == bridge_result) || (NULL == align_params) || (NULL == align_state))
    {
        return 0;
    }

    if((align_params->target_center_x >= MT9V03X_W) ||
       (align_params->lookahead_row >= MT9V03X_H) ||
       (align_params->far_check_row >= MT9V03X_H) ||
       (align_params->near_check_row >= MT9V03X_H) ||
       (align_params->far_check_row >= align_params->near_check_row) ||
       (0u == align_params->far_tolerance_px) ||
       (0u == align_params->near_tolerance_px) ||
       (0u == align_params->complete_confirm_frames) ||
       (0u == align_params->lost_reset_frames) ||
       (align_params->point_filter_alpha < 0.0f) ||
       (align_params->point_filter_alpha > 1.0f) ||
       (align_params->point_gain_d10_per_px <= 0.0f) ||
       (0.0f == align_params->point_direction) ||
       (align_params->yaw_offset_limit_d10 <= 0) ||
       (align_params->yaw_slew_limit_d10 <= 0) ||
       (align_params->control_gain_per_deg <= 0.0f) ||
       (0.0f == align_params->control_direction) ||
       (align_params->control_limit <= 0))
    {
        return 0;
    }

    align_result->phase = align_state->phase;

    // 对准完成后锁存状态，等待核心0切换到下一阶段。
    if(CAMERA_BRIDGE_ALIGN_COMPLETE == align_state->phase)
    {
        align_result->valid = 1;
        align_result->point_inside = 1;
        align_result->aligned = 1;
        return 1;
    }

    if(!bridge_result->valid ||
       (align_params->lookahead_row < bridge_result->top) ||
       (align_params->lookahead_row > bridge_result->bottom))
    {
        align_state->complete_frame_count = 0;

        if(align_state->lost_frame_count < align_params->lost_reset_frames)
        {
            align_state->lost_frame_count++;
        }

        align_state->previous_yaw_offset_d10 = 0;
        align_state->point_filter_initialized = 0;

        if(align_state->lost_frame_count >= align_params->lost_reset_frames)
        {
            camproc_bridge_align_reset(align_state);
        }

        align_result->phase = align_state->phase;
        return 0;
    }

    align_state->lost_frame_count = 0;
    align_result->valid = 1;
    active_x = camera_bridge_round_and_limit_x(
        bridge_result->center_slope * align_params->lookahead_row +
        bridge_result->center_intercept);
    far_x = camera_bridge_round_and_limit_x(
        bridge_result->center_slope * align_params->far_check_row +
        bridge_result->center_intercept);
    near_x = camera_bridge_round_and_limit_x(
        bridge_result->center_slope * align_params->near_check_row +
        bridge_result->center_intercept);

    align_result->active_x = active_x;
    align_result->active_y = align_params->lookahead_row;

    far_error = (int32)far_x - align_params->target_center_x;
    near_error = (int32)near_x - align_params->target_center_x;
    center_line_inside = (uint8)(
        (align_params->far_check_row >= bridge_result->top) &&
        (align_params->near_check_row <= bridge_result->bottom) &&
        (far_error >= -(int32)align_params->far_tolerance_px) &&
        (far_error <=  (int32)align_params->far_tolerance_px) &&
        (near_error >= -(int32)align_params->near_tolerance_px) &&
        (near_error <=  (int32)align_params->near_tolerance_px));
    align_result->point_inside = center_line_inside;

    if(center_line_inside)
    {
        if(align_state->complete_frame_count < align_params->complete_confirm_frames)
        {
            align_state->complete_frame_count++;
        }
    }
    else
    {
        align_state->complete_frame_count = 0;
    }

    if(align_state->complete_frame_count >= align_params->complete_confirm_frames)
    {
        align_state->phase = CAMERA_BRIDGE_ALIGN_COMPLETE;
        align_state->previous_yaw_offset_d10 = 0;
        align_result->valid = 1;
        align_result->point_inside = 1;
        align_result->aligned = 1;
        align_result->phase = CAMERA_BRIDGE_ALIGN_COMPLETE;
        return 1;
    }

    if(!align_state->point_filter_initialized)
    {
        align_state->filtered_point_x = (float)active_x;
        align_state->point_filter_initialized = 1;
    }
    else
    {
        align_state->filtered_point_x =
              align_params->point_filter_alpha * align_state->filtered_point_x
            + (1.0f - align_params->point_filter_alpha) * (float)active_x;
    }

    point_error = align_state->filtered_point_x - align_params->target_center_x;
    if(point_error > align_params->control_deadband_px)
    {
        point_error -= align_params->control_deadband_px;
    }
    else if(point_error < -(float)align_params->control_deadband_px)
    {
        point_error += align_params->control_deadband_px;
    }
    else
    {
        point_error = 0.0f;
    }

    align_result->point_error_px = (point_error >= 0.0f) ?
        (int16)(point_error + 0.5f) : (int16)(point_error - 0.5f);
    yaw_offset = align_params->point_direction
               * align_params->point_gain_d10_per_px
               * point_error;
    yaw_offset_d10 = (yaw_offset >= 0.0f) ?
        (int32)(yaw_offset + 0.5f) : (int32)(yaw_offset - 0.5f);

    if(yaw_offset_d10 > align_params->yaw_offset_limit_d10)
    {
        yaw_offset_d10 = align_params->yaw_offset_limit_d10;
    }
    else if(yaw_offset_d10 < -align_params->yaw_offset_limit_d10)
    {
        yaw_offset_d10 = -align_params->yaw_offset_limit_d10;
    }

    yaw_change_d10 = yaw_offset_d10 - align_state->previous_yaw_offset_d10;
    if(yaw_change_d10 > align_params->yaw_slew_limit_d10)
    {
        yaw_offset_d10 =
            align_state->previous_yaw_offset_d10 + align_params->yaw_slew_limit_d10;
    }
    else if(yaw_change_d10 < -align_params->yaw_slew_limit_d10)
    {
        yaw_offset_d10 =
            align_state->previous_yaw_offset_d10 - align_params->yaw_slew_limit_d10;
    }

    align_state->previous_yaw_offset_d10 = (int16)yaw_offset_d10;
    align_result->yaw_offset_d10 = (int16)yaw_offset_d10;
    align_result->control_value = camproc_bridge_yaw_offset_to_control(
        align_result->yaw_offset_d10, align_params);

    return 1;
}


// ==================================================== 跳跃检测、过滤、切换函数 ====================================================
static uint32 jump_trigger_count = 0;  // 已经触发的跳跃次数

uint16 camproc_jump_adaptive_row(uint16 car_speed, int16 coeff)
{
    // 遍历自适应参数列表
    for(uint8 i = 0; i < CAMERA_ROW_SPEED_RULE_COUNT; i++)
    {
        int16 max_speed = (int16)camproc_row_speed_rules[i].max_speed + coeff;  // 每个对应的最高速度为加上 系数的 最高速度

        if(car_speed <= max_speed)
        {
            // 实际速度 小于 最高速度，则返回当前的 Row 值，要求结构体中的排列顺序为 低速 -> 高速
            return camproc_row_speed_rules[i].check_row;
        }
    }

    // 均不能满足则返回最近速度
    return camproc_row_speed_rules[CAMERA_ROW_SPEED_RULE_COUNT - 1].check_row;
}

uint8 camproc_pub_check_area(uint8 image[MT9V03X_H][MT9V03X_W], uint16 check_row, uint16 check_row_count, uint16 check_column, uint16 check_column_count, uint32 dot_count, uint32 dot_type)
{
    uint16 x = 0;                   // 循环中 检查的点的 x 坐标
    uint16 y = 0;                   // 循环中 检查的点的 y 坐标
    uint16 checked_rows = 0;        // 已检查的行的数量
    uint16 checked_columns = 0;     // 已检查的列的数量
    uint32 current_dot_count = 0;   // 矩形中已经计数的符合条件的点的数量

    uint8 target_dot_value = (dot_type == CAMERA_IMAGE_DOT_BLACK) ? 0 : 255;        // 检测点具体值类型
    uint32 check_area_size = (uint32)check_row_count * (uint32)check_column_count;  // 检测区域面积

    /* 已移除所有安全限制 */

    // 遍历矩形
    for(checked_rows = 0; checked_rows < check_row_count; checked_rows++)
    {
        y = check_row - checked_rows;  // 当前循环检测的 y 坐标

        for(checked_columns = 0; checked_columns < check_column_count; checked_columns++)
        {
            x = check_column + checked_columns;  // 当前循环检测的 x 坐标

            // 当 (x, y) 处的像素点与 目标值相同时
            if(image[y][x] == target_dot_value)
            {
                current_dot_count++;

                if(current_dot_count >= dot_count)
                {
                    return 1;  // 如果计数点数量大于阈值，直接返回
                }
            }
        }
    }

    return 0;
}

uint8 camproc_bump_exit_detect(uint8 image[MT9V03X_H][MT9V03X_W], BumpExitParams_t *bump_exit_params, uint8 exit_check_enabled)
{
    uint8 black_detected = 0;
    uint8 white_detected = 0;

    // 必须先连续看到黑色凸起，防止入口处的白色地面被误判为出口
    if(!bump_exit_params->bump_seen)
    {
        black_detected = camproc_pub_check_area(
            image,
            bump_exit_params->check_row,
            bump_exit_params->check_row_count,
            bump_exit_params->check_column,
            bump_exit_params->check_column_count,
            bump_exit_params->black_dot_count,
            CAMERA_IMAGE_DOT_BLACK
        );

        if(!black_detected)
        {
            bump_exit_params->black_continuous_frame_count = 0;
            return 0;
        }

        if(bump_exit_params->black_continuous_frame_count < bump_exit_params->black_confirm_frame_count)
        {
            bump_exit_params->black_continuous_frame_count++;
        }

        if(bump_exit_params->black_continuous_frame_count >= bump_exit_params->black_confirm_frame_count)
        {
            bump_exit_params->bump_seen = 1;
        }

        return 0;
    }

    // 最短行驶时间结束前不进行白色出口判断
    if(!exit_check_enabled)
    {
        bump_exit_params->white_continuous_frame_count = 0;
        return 0;
    }

    white_detected = camproc_pub_check_area(
        image,
        bump_exit_params->check_row,
        bump_exit_params->check_row_count,
        bump_exit_params->check_column,
        bump_exit_params->check_column_count,
        bump_exit_params->white_dot_count,
        CAMERA_IMAGE_DOT_WHITE
    );

    if(!white_detected)
    {
        bump_exit_params->white_continuous_frame_count = 0;
        return 0;
    }

    if(bump_exit_params->white_continuous_frame_count < bump_exit_params->white_confirm_frame_count)
    {
        bump_exit_params->white_continuous_frame_count++;
    }

    if(bump_exit_params->white_continuous_frame_count >= bump_exit_params->white_confirm_frame_count)
    {
        bump_exit_params->exited = 1;
    }

    return bump_exit_params->exited;
}

uint8 camproc_jump_cooldown_filter(uint32 time_ms, uint32 cooldown_time_ms)
{
    static uint32 last_jump_time = 0;  // 上一次跳跃时间
    static uint8 has_triggered = 0;    // 是否已经触发，避免第一次跳跃时发生错误

    if(has_triggered && ((time_ms - last_jump_time) < cooldown_time_ms))
    {
        return 0;  // 时间不满足且不是第一次跳跃
    }

    has_triggered = 1;  // 第一次跳跃时， time_ms 可能小于 cooldown_time_ms，故设置保护静态变量
    last_jump_time = time_ms;

    return 1;
}

uint8 camproc_jump_dot_type_switch(void)
{
    // 当跳跃计数小于列表长度，切换即增加一次跳跃次数
    if(jump_trigger_count < CAMERA_DOT_TYPE_LIST_COUNT)
    {
        jump_trigger_count++;
    }

    // 跳完之后停止在最后一个检测位置
    if(jump_trigger_count >= CAMERA_DOT_TYPE_LIST_COUNT)
    {
        return (uint8)dot_type_list[CAMERA_DOT_TYPE_LIST_COUNT - 1];
    }

    // 正常就返回跳跃列表
    return (uint8)dot_type_list[jump_trigger_count];
}

uint32 camproc_jump_get_steps(void)
{
    // 获取当前跳跃次数
    return jump_trigger_count;
}

uint8 camproc_jump_dot_type_reset(void)
{
    // 重置跳跃
    jump_trigger_count = 0;
    return (uint8)dot_type_list[0];
}
