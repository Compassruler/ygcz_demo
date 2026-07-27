#include "camera_proc.h"
#include <string.h>

// ==================================================== 参数调节 ====================================================
#define CAMERA_BRIDGE_ENDPOINT_AVERAGE_COUNT    (3u)    // 上下端点分别使用的平均采样点数量
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

// 八邻域方向
static const int8 lane_neighbor_8[8][2]=
{
    {-1,-1}, {0,-1}, {1,-1},
    {-1,0} ,         {1,0},
    {-1,1} ,  {0,1}, {1,1}
};

// 判断是否为黑色
static uint8 lane_is_black(const uint8 image[MT9V03X_H][MT9V03X_W], int16 x, int16 y)
{
    if(x < 0 || x >= MT9V03X_W || y < 0 || y >= MT9V03X_H) return 0;

    return (image[y][x] == 0);
}

uint16 camproc_lane_search_8neighbor(const uint8 image[MT9V03X_H][MT9V03X_W], uint16 start_x, uint16 start_y, LanePoint_t point[])
{
    static uint8 visited[MT9V03X_H][MT9V03X_W];

    LanePoint_t queue[LANE_MAX_POINT_NUM];

    uint16 head=0;
    uint16 tail=0;

    uint16 count=0;

    memset(visited,0,sizeof(visited));

    if(!lane_is_black(image,start_x,start_y)) return 0; 

    queue[tail].x=start_x;
    queue[tail].y=start_y;

    tail++;

    visited[start_y][start_x]=1;

    while(head < tail)
    {
        LanePoint_t p=queue[head++];

        if(count < LANE_MAX_POINT_NUM)
        {
            point[count]=p;
            count++;
        }

        for(uint8 i=0;i<8;i++)
        {
            int16 nx=p.x+lane_neighbor_8[i][0];
            int16 ny=p.y+lane_neighbor_8[i][1];

            if(!lane_is_black(image,nx,ny)) continue;
            if(visited[ny][nx]) continue;

            visited[ny][nx]=1;

            if(tail < LANE_MAX_POINT_NUM)
            {
                queue[tail].x=nx;
                queue[tail].y=ny;

                tail++;
            }
        }
    }
    return count;
}

void camproc_lane_center_calculate(LanePoint_t point[], uint16 count, int16 *center_x)
{
    uint32 sum_x=0;
    
    if(count==0)
    {
        *center_x=MT9V03X_W/2;
        return;
    }

    for(uint16 i=0;i<count;i++)
    {
        sum_x+=point[i].x;
    }

    *center_x=sum_x/count;
}

// ==================================================== 单边桥和颠簸路段函数 ====================================================
typedef struct
{
    uint8 y;
    uint8 left_x;
    uint8 right_x;
} CameraBridgeSample_t;

static uint16 camera_bridge_abs_diff_u16(uint16 value_a, uint16 value_b)
{
    return (value_a >= value_b) ?
        (uint16)(value_a - value_b) : (uint16)(value_b - value_a);
}

// 从画面左侧向内寻找连续黑色到连续白色的稳定跳变
static uint8 camera_bridge_find_left_edge(
    const uint8 image[MT9V03X_H][MT9V03X_W],
    uint16 y,
    const CameraBridgeParams_t *params,
    uint8 *left_x)
{
    uint16 x = 0;
    uint8 offset = 0;
    uint8 stable = 0;

    for(x = params->roi_left + params->stable_pixel_count;
        x <= (params->roi_right - params->stable_pixel_count + 1u);
        x++)
    {
        stable = 1;

        for(offset = 0; offset < params->stable_pixel_count; offset++)
        {
            if((0u != image[y][x - offset - 1u]) ||
               (255u != image[y][x + offset]))
            {
                stable = 0;
                break;
            }
        }

        if(stable)
        {
            *left_x = (uint8)x;
            return 1;
        }
    }

    return 0;
}

// 从画面右侧向内寻找连续黑色到连续白色的稳定跳变
static uint8 camera_bridge_find_right_edge(
    const uint8 image[MT9V03X_H][MT9V03X_W],
    uint16 y,
    const CameraBridgeParams_t *params,
    uint8 *right_x)
{
    int16 x = 0;
    uint8 offset = 0;
    uint8 stable = 0;

    for(x = (int16)(params->roi_right - params->stable_pixel_count);
        x >= (int16)(params->roi_left + params->stable_pixel_count - 1u);
        x--)
    {
        stable = 1;

        for(offset = 0; offset < params->stable_pixel_count; offset++)
        {
            if((255u != image[y][(uint16)x - offset]) ||
               (0u != image[y][(uint16)x + offset + 1u]))
            {
                stable = 0;
                break;
            }
        }

        if(stable)
        {
            *right_x = (uint8)x;
            return 1;
        }
    }

    return 0;
}

// 在当前行同时取得左右边线，并检查赛道宽度
static uint8 camera_bridge_find_edge_pair(
    const uint8 image[MT9V03X_H][MT9V03X_W],
    uint16 y,
    const CameraBridgeParams_t *params,
    CameraBridgeSample_t *sample)
{
    uint8 left_x = 0;
    uint8 right_x = 0;
    uint16 lane_width = 0;

    if(!camera_bridge_find_left_edge(
           image, y, params, &left_x) ||
       !camera_bridge_find_right_edge(
           image, y, params, &right_x) ||
       (left_x >= right_x))
    {
        return 0;
    }

    lane_width = (uint16)right_x - left_x;
    if((lane_width < params->min_lane_width) ||
       (lane_width > params->max_lane_width))
    {
        return 0;
    }

    sample->y = (uint8)y;
    sample->left_x = left_x;
    sample->right_x = right_x;
    return 1;
}

// 保存当前最长的连续双边线段
static void camera_bridge_update_best_segment(
    const CameraBridgeSample_t current_samples[MT9V03X_H],
    uint16 current_count,
    CameraBridgeSample_t best_samples[MT9V03X_H],
    uint16 *best_count)
{
    uint16 index = 0;
    uint16 current_span = 0;
    uint16 best_span = 0;

    if(0u == current_count)
    {
        return;
    }

    current_span = (uint16)(
        current_samples[0].y - current_samples[current_count - 1u].y);

    if(0u != *best_count)
    {
        best_span = (uint16)(
            best_samples[0].y - best_samples[*best_count - 1u].y);
    }

    if((current_count < *best_count) ||
       ((current_count == *best_count) && (current_span <= best_span)))
    {
        return;
    }

    for(index = 0; index < current_count; index++)
    {
        best_samples[index] = current_samples[index];
    }

    *best_count = current_count;
}

uint8 camproc_bridge_detect(
    const uint8 image[MT9V03X_H][MT9V03X_W],
    const CameraBridgeParams_t *params,
    CameraBridgeResult_t *result)
{
    int16 y = 0;
    uint8 pair_valid = 0;
    uint8 missing_rows = 0;
    uint16 current_count = 0;
    uint16 best_count = 0;
    uint16 maximum_point_count = 0;
    uint16 allowed_jump = 0;
    uint16 average_count = 0;
    uint16 index = 0;
    uint32 far_left_sum = 0;
    uint32 far_right_sum = 0;
    uint32 far_y_sum = 0;
    uint32 near_left_sum = 0;
    uint32 near_right_sum = 0;
    uint32 near_y_sum = 0;
    CameraBridgeSample_t sample = {0};
    CameraBridgeSample_t current_samples[MT9V03X_H];
    CameraBridgeSample_t best_samples[MT9V03X_H];

    if(NULL == result)
    {
        return 0;
    }

    memset(result, 0, sizeof(*result));

    if((NULL == image) || (NULL == params) ||
       (params->roi_top >= params->roi_bottom) ||
       (params->roi_bottom >= MT9V03X_H) ||
       (params->roi_left >= params->roi_right) ||
       (params->roi_right >= MT9V03X_W) ||
       (params->min_lane_width >= params->max_lane_width) ||
       (0u == params->max_edge_jump) ||
       (params->min_point_count < 2u) ||
       (0u == params->min_y_span) ||
       (0u == params->row_step) ||
       (0u == params->stable_pixel_count) ||
       ((uint16)params->stable_pixel_count * 2u >=
        (params->roi_right - params->roi_left + 1u)))
    {
        return 0;
    }

    maximum_point_count = (uint16)(
        ((uint16)(params->roi_bottom - params->roi_top) /
         (uint16)params->row_step) + 1u);
    if(params->min_point_count > maximum_point_count)
    {
        return 0;
    }

    // 从画面下方向上逐行扫描，低视角下超出画面的近处边线会自然被跳过
    for(y = (int16)params->roi_bottom;
        y >= (int16)params->roi_top;
        y -= params->row_step)
    {
        pair_valid = camera_bridge_find_edge_pair(
            image, (uint16)y, params, &sample);

        if(pair_valid && (0u != current_count))
        {
            allowed_jump = (uint16)(
                params->max_edge_jump * (uint16)(missing_rows + 1u));

            if((camera_bridge_abs_diff_u16(
                    sample.left_x,
                    current_samples[current_count - 1u].left_x) > allowed_jump) ||
               (camera_bridge_abs_diff_u16(
                    sample.right_x,
                    current_samples[current_count - 1u].right_x) > allowed_jump))
            {
                camera_bridge_update_best_segment(
                    current_samples, current_count, best_samples, &best_count);
                current_count = 0;
                missing_rows = 0;
            }
        }

        if(pair_valid)
        {
            current_samples[current_count] = sample;
            current_count++;
            missing_rows = 0;
        }
        else if(0u != current_count)
        {
            missing_rows++;

            if(missing_rows > params->max_missing_rows)
            {
                camera_bridge_update_best_segment(
                    current_samples, current_count, best_samples, &best_count);
                current_count = 0;
                missing_rows = 0;
            }
        }
    }

    camera_bridge_update_best_segment(
        current_samples, current_count, best_samples, &best_count);

    if((best_count < params->min_point_count) ||
       ((uint16)(best_samples[0].y - best_samples[best_count - 1u].y) <
        params->min_y_span))
    {
        return 0;
    }

    average_count = (best_count < CAMERA_BRIDGE_ENDPOINT_AVERAGE_COUNT) ?
        best_count : CAMERA_BRIDGE_ENDPOINT_AVERAGE_COUNT;

    // 分别平均连续边线段的上端和下端，减少单行抖动对控制的影响
    for(index = 0; index < average_count; index++)
    {
        near_left_sum += best_samples[index].left_x;
        near_right_sum += best_samples[index].right_x;
        near_y_sum += best_samples[index].y;

        far_left_sum += best_samples[best_count - 1u - index].left_x;
        far_right_sum += best_samples[best_count - 1u - index].right_x;
        far_y_sum += best_samples[best_count - 1u - index].y;
    }

    result->top = best_samples[best_count - 1u].y;
    result->bottom = best_samples[0].y;
    result->point_count = best_count;
    result->left_x1 = (uint16)(far_left_sum / average_count);
    result->left_y1 = (uint16)(far_y_sum / average_count);
    result->left_x2 = (uint16)(near_left_sum / average_count);
    result->left_y2 = (uint16)(near_y_sum / average_count);
    result->right_x1 = (uint16)(far_right_sum / average_count);
    result->right_y1 = result->left_y1;
    result->right_x2 = (uint16)(near_right_sum / average_count);
    result->right_y2 = result->left_y2;
    result->center_x1 = (uint16)((result->left_x1 + result->right_x1) / 2u);
    result->center_y1 = result->left_y1;
    result->center_x2 = (uint16)((result->left_x2 + result->right_x2) / 2u);
    result->center_y2 = result->left_y2;
    result->valid = 1;

    return 1;
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
}

uint8 camproc_bridge_align_update(
    const CameraBridgeResult_t *bridge_result,
    const CameraBridgeAlignParams_t *align_params,
    CameraBridgeAlignState_t *align_state,
    CameraBridgeAlignResult_t *align_result)
{
    uint16 active_x = 0;
    uint16 active_y = 0;
    uint16 vertical_span = 0;
    uint16 tilt_error_abs = 0;
    uint8 center_line_inside = 0;
    float point_error = 0.0f;
    float error_gain = 0.0f;
    float yaw_offset = 0.0f;
    int32 tilt_error_numerator = 0;
    int32 normalized_tilt_error = 0;
    int32 yaw_offset_d10 = 0;
    int32 yaw_change_d10 = 0;
    int32 far_error = 0;
    int32 near_error = 0;
    int32 far_residual = 0;
    int32 near_residual = 0;

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
       (0u == align_params->far_tolerance_px) ||
       (0u == align_params->near_tolerance_px) ||
       (0u == align_params->tilt_reference_span) ||
       (align_params->tilt_reference_span > MT9V03X_H) ||
       (0u == align_params->tilt_enter_threshold_px) ||
       (align_params->tilt_exit_threshold_px >= align_params->tilt_enter_threshold_px) ||
       (0u == align_params->complete_confirm_frames) ||
       (0u == align_params->lost_reset_frames) ||
       (align_params->point_filter_alpha < 0.0f) ||
       (align_params->point_filter_alpha > 1.0f) ||
       (align_params->point_gain_d10_per_px <= 0.0f) ||
       (align_params->tilt_gain_d10_per_px <= 0.0f) ||
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

    if(!bridge_result->valid ||
       (bridge_result->center_y1 >= bridge_result->center_y2))
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

    // 中线中点用于横向位置控制，上下端点差值用于倾角控制
    active_x = (uint16)(
        (bridge_result->center_x1 + bridge_result->center_x2 + 1u) / 2u);
    active_y = (uint16)(
        (bridge_result->center_y1 + bridge_result->center_y2 + 1u) / 2u);
    align_result->active_x = active_x;
    align_result->active_y = active_y;

    far_error = (int32)bridge_result->center_x1 - align_params->target_center_x;
    near_error = (int32)bridge_result->center_x2 - align_params->target_center_x;
    vertical_span = bridge_result->center_y2 - bridge_result->center_y1;
    tilt_error_numerator =
        ((int32)bridge_result->center_x1 - bridge_result->center_x2)
        * align_params->tilt_reference_span;

    // 将不同长度中线的倾斜量统一换算到固定纵向跨度
    normalized_tilt_error = (tilt_error_numerator >= 0) ?
        (tilt_error_numerator + (int32)(vertical_span / 2u)) / (int32)vertical_span :
        (tilt_error_numerator - (int32)(vertical_span / 2u)) / (int32)vertical_span;
    align_result->tilt_error_px = (int16)normalized_tilt_error;

    center_line_inside = (uint8)(
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
        align_state->tilt_control_active = 0;
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

    tilt_error_abs = (uint16)((normalized_tilt_error >= 0) ?
        normalized_tilt_error : -normalized_tilt_error);

    // 使用迟滞阈值切换控制模式，避免在倾角阈值附近反复跳变
    if(align_state->tilt_control_active)
    {
        if(tilt_error_abs <= align_params->tilt_exit_threshold_px)
        {
            align_state->tilt_control_active = 0;
        }
    }
    else if(tilt_error_abs >= align_params->tilt_enter_threshold_px)
    {
        align_state->tilt_control_active = 1;
    }

    align_result->tilt_control_active = align_state->tilt_control_active;

    // 中线明显倾斜时优先修正方向，基本竖直后再修正中点位置
    if(align_state->tilt_control_active)
    {
        point_error = (float)normalized_tilt_error;
        error_gain = align_params->tilt_gain_d10_per_px;
    }
    else
    {
        point_error = align_state->filtered_point_x - align_params->target_center_x;
        error_gain = align_params->point_gain_d10_per_px;
    }

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

    // 尚未对齐但控制误差落入死区时，使用超出容差最多的端点继续修正
    if((0.0f == point_error) && !center_line_inside)
    {
        if(far_error > (int32)align_params->far_tolerance_px)
        {
            far_residual = far_error - align_params->far_tolerance_px;
        }
        else if(far_error < -(int32)align_params->far_tolerance_px)
        {
            far_residual = far_error + align_params->far_tolerance_px;
        }

        if(near_error > (int32)align_params->near_tolerance_px)
        {
            near_residual = near_error - align_params->near_tolerance_px;
        }
        else if(near_error < -(int32)align_params->near_tolerance_px)
        {
            near_residual = near_error + align_params->near_tolerance_px;
        }

        point_error = ((far_residual >= 0 ? far_residual : -far_residual) >=
                       (near_residual >= 0 ? near_residual : -near_residual)) ?
            (float)far_residual : (float)near_residual;
        error_gain = align_params->point_gain_d10_per_px;
    }

    align_result->point_error_px = (point_error >= 0.0f) ?
        (int16)(point_error + 0.5f) : (int16)(point_error - 0.5f);
    yaw_offset = align_params->point_direction
               * error_gain
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
