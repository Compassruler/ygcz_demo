#include "camera_proc.h"
#include <math.h>
#include <string.h>

// ==================================================== 参数调节 ====================================================
#define CAMERA_BRIDGE_MAX_EDGE_POINTS           (1024u) // 单侧边线最多保存的候选点数量
#define CAMERA_BRIDGE_MAX_POINTS_PER_ROW        (8u)    // 每行单侧最多保存的黑白跳变数量
#define CAMERA_BRIDGE_RAD_TO_D10                 (572.9578f)
#define CAMERA_BRIDGE_CONTROL_STABLE_DELTA       (4)     // 连续可靠控制量允许的最大变化
#define CAMERA_ROW_SPEED_RULE_COUNT              (5u)
#define CAMERA_LANE_VISITED_BYTE_COUNT           ((MT9V03X_H * MT9V03X_W + 7u) / 8u)

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

// 使用位图记录访问状态，避免为整幅图像常驻分配一字节/像素的数组
static uint8 lane_visited_get(const uint8 visited[], uint16 x, uint16 y)
{
    uint32 index = (uint32)y * MT9V03X_W + x;

    return (uint8)((visited[index >> 3] >> (index & 0x07u)) & 0x01u);
}

static void lane_visited_set(uint8 visited[], uint16 x, uint16 y)
{
    uint32 index = (uint32)y * MT9V03X_W + x;

    visited[index >> 3] |= (uint8)(1u << (index & 0x07u));
}

uint16 camproc_lane_search_8neighbor(const uint8 image[MT9V03X_H][MT9V03X_W], uint16 start_x, uint16 start_y, LanePoint_t point[])
{
    static uint8 visited[CAMERA_LANE_VISITED_BYTE_COUNT];

    uint16 head=0;
    uint16 tail=0;

    memset(visited,0,sizeof(visited));

    if(NULL == point) return 0;
    if(!lane_is_black(image,start_x,start_y)) return 0; 

    point[tail].x=start_x;
    point[tail].y=start_y;

    tail++;

    lane_visited_set(visited,start_x,start_y);

    while(head < tail)
    {
        LanePoint_t p=point[head++];

        for(uint8 i=0;i<8;i++)
        {
            int16 nx=p.x+lane_neighbor_8[i][0];
            int16 ny=p.y+lane_neighbor_8[i][1];

            if(!lane_is_black(image,nx,ny)) continue;
            if(lane_visited_get(visited,(uint16)nx,(uint16)ny)) continue;

            lane_visited_set(visited,(uint16)nx,(uint16)ny);

            if(tail < LANE_MAX_POINT_NUM)
            {
                point[tail].x=(uint16)nx;
                point[tail].y=(uint16)ny;

                tail++;
            }
        }
    }
    return tail;
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
    uint8 x;
    uint8 y;
} CameraBridgeEdgePoint_t;

typedef struct
{
    uint8 valid;
    uint16 point_count;
    uint16 top;
    uint16 bottom;
    float slope;
    float intercept;
} CameraBridgeLine_t;

typedef struct
{
    uint8 threshold_initialized;
    float filtered_threshold;
    uint8 lane_width_initialized;
    float lane_width_slope;
    float lane_width_intercept;
    uint8 single_edge_frame_count;
    uint8 lost_frame_count;
    uint32 random_seed;
    CameraBridgeResult_t last_result;
} CameraBridgeDetectState_t;

static CameraBridgeDetectState_t camera_bridge_detect_state;
static CameraBridgeEdgePoint_t camera_bridge_left_points[CAMERA_BRIDGE_MAX_EDGE_POINTS];
static CameraBridgeEdgePoint_t camera_bridge_right_points[CAMERA_BRIDGE_MAX_EDGE_POINTS];

static uint8 camera_bridge_calculate_otsu_threshold(
    const uint8 image[MT9V03X_H][MT9V03X_W],
    const CameraBridgeParams_t *params,
    uint8 *threshold)
{
    uint16 x = 0;
    uint16 y = 0;
    uint16 value = 0;
    uint32 histogram[256] = {0};
    uint32 total_count = 0;
    uint32 background_count = 0;
    uint32 foreground_count = 0;
    uint32 total_sum = 0;
    uint32 background_sum = 0;
    float background_mean = 0.0f;
    float foreground_mean = 0.0f;
    float mean_difference = 0.0f;
    float variance = 0.0f;
    float maximum_variance = -1.0f;
    uint8 best_threshold = 0;
    uint8 threshold_found = 0;

    for(y = params->roi_top; y <= params->roi_bottom; y++)
    {
        for(x = params->roi_left; x <= params->roi_right; x++)
        {
            histogram[image[y][x]]++;
        }
    }

    total_count =
        (uint32)(params->roi_bottom - params->roi_top + 1u) *
        (uint32)(params->roi_right - params->roi_left + 1u);

    for(value = 0; value < 256u; value++)
    {
        total_sum += (uint32)value * histogram[value];
    }

    for(value = 0; value < 255u; value++)
    {
        background_count += histogram[value];
        background_sum += (uint32)value * histogram[value];

        if(0u == background_count)
        {
            continue;
        }

        foreground_count = total_count - background_count;
        if(0u == foreground_count)
        {
            break;
        }

        background_mean = (float)background_sum / background_count;
        foreground_mean =
            (float)(total_sum - background_sum) / foreground_count;
        mean_difference = background_mean - foreground_mean;
        variance =
            (float)background_count * (float)foreground_count *
            mean_difference * mean_difference;

        if(variance > maximum_variance)
        {
            maximum_variance = variance;
            best_threshold = (uint8)value;
            threshold_found = 1;
        }
    }

    if(!threshold_found)
    {
        return 0;
    }

    *threshold = best_threshold;
    return 1;
}

uint8 camproc_bridge_prepare_binary(
    uint8 image[MT9V03X_H][MT9V03X_W],
    const CameraBridgeParams_t *params)
{
    uint8 threshold = 0;
    int32 filtered_threshold = 0;

    if((NULL == image) || (NULL == params) ||
       (params->roi_top >= params->roi_bottom) ||
       (params->roi_bottom >= MT9V03X_H) ||
       (params->roi_left >= params->roi_right) ||
       (params->roi_right >= MT9V03X_W) ||
       (params->threshold_filter_alpha < 0.0f) ||
       (params->threshold_filter_alpha > 1.0f))
    {
        return 0;
    }

    threshold = params->binary_threshold;

    if(params->use_otsu_threshold)
    {
        if(!camera_bridge_calculate_otsu_threshold(image, params, &threshold))
        {
            threshold = params->binary_threshold;
        }

        if(!camera_bridge_detect_state.threshold_initialized)
        {
            camera_bridge_detect_state.filtered_threshold = (float)threshold;
            camera_bridge_detect_state.threshold_initialized = 1;
        }
        else
        {
            camera_bridge_detect_state.filtered_threshold =
                  params->threshold_filter_alpha *
                  camera_bridge_detect_state.filtered_threshold
                + (1.0f - params->threshold_filter_alpha) *
                  (float)threshold;
        }

        filtered_threshold =
            (int32)(camera_bridge_detect_state.filtered_threshold + 0.5f);
        if(filtered_threshold < 0)
        {
            filtered_threshold = 0;
        }
        else if(filtered_threshold > 255)
        {
            filtered_threshold = 255;
        }

        threshold = (uint8)filtered_threshold;
    }
    else
    {
        camera_bridge_detect_state.threshold_initialized = 0;
    }

    camproc_pub_thresh_bin(image, threshold);
    return 1;
}

void camproc_bridge_detect_reset(void)
{
    memset(&camera_bridge_detect_state, 0, sizeof(camera_bridge_detect_state));
    camera_bridge_detect_state.random_seed = 0x6D2B79F5u;
}

// 读取 3x3 邻域多数值，抑制二值图中的孤立噪点
static uint8 camera_bridge_filtered_white(
    const uint8 image[MT9V03X_H][MT9V03X_W],
    uint16 x,
    uint16 y)
{
    int16 offset_x = 0;
    int16 offset_y = 0;
    int16 sample_x = 0;
    int16 sample_y = 0;
    uint8 white_count = 0;
    uint8 sample_count = 0;

    for(offset_y = -1; offset_y <= 1; offset_y++)
    {
        sample_y = (int16)y + offset_y;
        if((sample_y < 0) || (sample_y >= MT9V03X_H))
        {
            continue;
        }

        for(offset_x = -1; offset_x <= 1; offset_x++)
        {
            sample_x = (int16)x + offset_x;
            if((sample_x < 0) || (sample_x >= MT9V03X_W))
            {
                continue;
            }

            if(255u == image[sample_y][sample_x])
            {
                white_count++;
            }
            sample_count++;
        }
    }

    return (uint8)((uint16)white_count * 2u > sample_count);
}

// 从每个采样行提取黑到白和白到黑的稳定跳变点
static void camera_bridge_extract_edge_points(
    const uint8 image[MT9V03X_H][MT9V03X_W],
    const CameraBridgeParams_t *params,
    uint16 *left_count,
    uint16 *right_count)
{
    int16 x = 0;
    int16 y = 0;
    uint8 offset = 0;
    uint8 stable = 0;
    uint8 row_point_count = 0;
    uint8 filtered_row[MT9V03X_W];

    *left_count = 0;
    *right_count = 0;

    for(y = (int16)params->roi_bottom;
        y >= (int16)params->roi_top;
        y -= params->row_step)
    {
        for(x = (int16)params->roi_left;
            x <= (int16)params->roi_right;
            x++)
        {
            filtered_row[x] = camera_bridge_filtered_white(
                image, (uint16)x, (uint16)y) ? 255u : 0u;
        }

        // 左边线优先保留每行靠左的黑到白跳变
        row_point_count = 0;
        for(x = (int16)(params->roi_left + params->stable_pixel_count);
            x <= (int16)(params->roi_right - params->stable_pixel_count + 1u);
            x++)
        {
            stable = 1;
            for(offset = 0; offset < params->stable_pixel_count; offset++)
            {
                if((0u != filtered_row[x - offset - 1]) ||
                   (255u != filtered_row[x + offset]))
                {
                    stable = 0;
                    break;
                }
            }

            if(stable &&
               (row_point_count < CAMERA_BRIDGE_MAX_POINTS_PER_ROW) &&
               (*left_count < CAMERA_BRIDGE_MAX_EDGE_POINTS))
            {
                camera_bridge_left_points[*left_count].x = (uint8)x;
                camera_bridge_left_points[*left_count].y = (uint8)y;
                (*left_count)++;
                row_point_count++;
            }
        }

        // 右边线从右向左扫描，优先保留每行靠右的白到黑跳变
        row_point_count = 0;
        for(x = (int16)(params->roi_right - params->stable_pixel_count + 1u);
            x >= (int16)(params->roi_left + params->stable_pixel_count);
            x--)
        {
            stable = 1;
            for(offset = 0; offset < params->stable_pixel_count; offset++)
            {
                if((255u != filtered_row[x - offset - 1]) ||
                   (0u != filtered_row[x + offset]))
                {
                    stable = 0;
                    break;
                }
            }

            if(stable &&
               (row_point_count < CAMERA_BRIDGE_MAX_POINTS_PER_ROW) &&
               (*right_count < CAMERA_BRIDGE_MAX_EDGE_POINTS))
            {
                camera_bridge_right_points[*right_count].x = (uint8)x;
                camera_bridge_right_points[*right_count].y = (uint8)y;
                (*right_count)++;
                row_point_count++;
            }
        }
    }
}

static uint32 camera_bridge_random_u32(void)
{
    if(0u == camera_bridge_detect_state.random_seed)
    {
        camera_bridge_detect_state.random_seed = 0x6D2B79F5u;
    }

    camera_bridge_detect_state.random_seed =
          camera_bridge_detect_state.random_seed * 1664525u
        + 1013904223u;
    return camera_bridge_detect_state.random_seed;
}

static uint8 camera_bridge_point_near_pair(
    const CameraBridgeEdgePoint_t *point,
    const CameraBridgeEdgePoint_t *point_a,
    const CameraBridgeEdgePoint_t *point_b,
    uint8 distance_px)
{
    int32 delta_x = (int32)point_b->x - point_a->x;
    int32 delta_y = (int32)point_b->y - point_a->y;
    int32 point_delta_x = (int32)point->x - point_a->x;
    int32 point_delta_y = (int32)point->y - point_a->y;
    int64 cross = (int64)delta_x * point_delta_y -
                  (int64)delta_y * point_delta_x;
    int64 line_length_square =
        (int64)delta_x * delta_x + (int64)delta_y * delta_y;
    int64 distance_limit_square =
        (int64)distance_px * distance_px * line_length_square;

    return (uint8)(cross * cross <= distance_limit_square);
}

static uint8 camera_bridge_refit_line(
    const CameraBridgeEdgePoint_t points[],
    uint16 point_count,
    const CameraBridgeEdgePoint_t *seed_a,
    const CameraBridgeEdgePoint_t *seed_b,
    const CameraBridgeParams_t *params,
    CameraBridgeLine_t *line)
{
    uint8 pass = 0;
    uint16 index = 0;
    uint16 inlier_count = 0;
    uint16 top = 0;
    uint16 bottom = 0;
    float slope = 0.0f;
    float intercept = 0.0f;
    float distance_limit = 0.0f;
    float residual = 0.0f;
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_yy = 0.0f;
    float sum_yx = 0.0f;
    float denominator = 0.0f;

    if(seed_a->y == seed_b->y)
    {
        return 0;
    }

    slope = ((float)seed_b->x - seed_a->x) /
            ((float)seed_b->y - seed_a->y);
    intercept = (float)seed_a->x - slope * seed_a->y;

    // 两次重新拟合可消除随机种子点对最终直线参数的直接影响
    for(pass = 0; pass < 2u; pass++)
    {
        inlier_count = 0;
        top = MT9V03X_H;
        bottom = 0;
        sum_x = 0.0f;
        sum_y = 0.0f;
        sum_yy = 0.0f;
        sum_yx = 0.0f;
        distance_limit =
            params->ransac_distance_px * sqrtf(1.0f + slope * slope);

        for(index = 0; index < point_count; index++)
        {
            residual = fabsf(
                (float)points[index].x -
                (slope * points[index].y + intercept));

            if(residual <= distance_limit)
            {
                inlier_count++;
                sum_x += points[index].x;
                sum_y += points[index].y;
                sum_yy += (float)points[index].y * points[index].y;
                sum_yx += (float)points[index].y * points[index].x;

                if(points[index].y < top)
                {
                    top = points[index].y;
                }
                if(points[index].y > bottom)
                {
                    bottom = points[index].y;
                }
            }
        }

        if((inlier_count < params->min_line_point_count) ||
           ((bottom - top) < params->min_y_span))
        {
            return 0;
        }

        denominator =
            (float)inlier_count * sum_yy - sum_y * sum_y;
        if(fabsf(denominator) < 0.001f)
        {
            return 0;
        }

        slope =
            ((float)inlier_count * sum_yx - sum_y * sum_x) /
            denominator;
        intercept = (sum_x - slope * sum_y) / inlier_count;
    }

    line->valid = 1;
    line->point_count = inlier_count;
    line->top = top;
    line->bottom = bottom;
    line->slope = slope;
    line->intercept = intercept;
    return 1;
}

// RANSAC 不依赖相邻行位置，因此大角度边线也可以参与同一条直线拟合
static uint8 camera_bridge_fit_line_ransac(
    const CameraBridgeEdgePoint_t points[],
    uint16 point_count,
    const CameraBridgeParams_t *params,
    uint8 prefer_right_edge,
    CameraBridgeLine_t *line)
{
    uint16 iteration = 0;
    uint16 index = 0;
    uint16 index_a = 0;
    uint16 index_b = 0;
    uint16 minimum_seed_span = 0;
    uint16 inlier_count = 0;
    uint16 top = 0;
    uint16 bottom = 0;
    uint16 span = 0;
    uint16 best_inlier_count = 0;
    uint16 best_span = 0;
    uint16 best_bottom = 0;
    int16 seed_span = 0;
    float reference_x = 0.0f;
    float best_reference_x = 0.0f;
    uint8 better_model = 0;
    CameraBridgeEdgePoint_t best_a = {0};
    CameraBridgeEdgePoint_t best_b = {0};

    memset(line, 0, sizeof(*line));

    if(point_count < params->min_line_point_count)
    {
        return 0;
    }

    minimum_seed_span = params->min_y_span / 3u;
    if(minimum_seed_span < 2u)
    {
        minimum_seed_span = 2u;
    }

    for(iteration = 0; iteration < params->ransac_iterations; iteration++)
    {
        index_a = (uint16)(camera_bridge_random_u32() % point_count);
        index_b = (uint16)(camera_bridge_random_u32() % point_count);
        if(index_a == index_b)
        {
            index_b = (uint16)((index_b + 1u) % point_count);
        }

        seed_span =
            (int16)points[index_a].y - (int16)points[index_b].y;
        if(seed_span < 0)
        {
            seed_span = (int16)-seed_span;
        }
        if((uint16)seed_span < minimum_seed_span)
        {
            continue;
        }

        inlier_count = 0;
        top = MT9V03X_H;
        bottom = 0;

        for(index = 0; index < point_count; index++)
        {
            if(camera_bridge_point_near_pair(
                   &points[index],
                   &points[index_a],
                   &points[index_b],
                   params->ransac_distance_px))
            {
                inlier_count++;
                if(points[index].y < top)
                {
                    top = points[index].y;
                }
                if(points[index].y > bottom)
                {
                    bottom = points[index].y;
                }
            }
        }

        span = bottom - top;
        if((inlier_count < params->min_line_point_count) ||
           (span < params->min_y_span))
        {
            continue;
        }

        reference_x =
            points[index_a].x +
            ((float)points[index_b].x - points[index_a].x) *
            ((float)params->roi_bottom - points[index_a].y) /
            ((float)points[index_b].y - points[index_a].y);

        better_model = 0;
        if((inlier_count > best_inlier_count) ||
           ((inlier_count == best_inlier_count) && (span > best_span)) ||
           ((inlier_count == best_inlier_count) &&
            (span == best_span) && (bottom > best_bottom)))
        {
            better_model = 1;
        }
        else if((inlier_count == best_inlier_count) &&
                (span == best_span) &&
                (bottom == best_bottom))
        {
            better_model = prefer_right_edge ?
                (uint8)(reference_x > best_reference_x) :
                (uint8)(reference_x < best_reference_x);
        }

        if(better_model)
        {
            best_inlier_count = inlier_count;
            best_span = span;
            best_bottom = bottom;
            best_reference_x = reference_x;
            best_a = points[index_a];
            best_b = points[index_b];
        }
    }

    if(0u == best_inlier_count)
    {
        return 0;
    }

    return camera_bridge_refit_line(
        points, point_count, &best_a, &best_b, params, line);
}

static float camera_bridge_line_x(const CameraBridgeLine_t *line, uint16 y)
{
    return line->slope * y + line->intercept;
}

static uint16 camera_bridge_clamp_x(float x)
{
    if(x <= 0.0f)
    {
        return 0;
    }
    if(x >= (float)(MT9V03X_W - 1u))
    {
        return MT9V03X_W - 1u;
    }

    return (uint16)(x + 0.5f);
}

static uint8 camera_bridge_hold_last_result(
    const CameraBridgeParams_t *params,
    CameraBridgeResult_t *result)
{
    if(camera_bridge_detect_state.last_result.valid &&
       (camera_bridge_detect_state.lost_frame_count <
        params->lost_hold_frames))
    {
        camera_bridge_detect_state.lost_frame_count++;
        *result = camera_bridge_detect_state.last_result;
        result->estimated = 1;
        return 1;
    }

    camera_bridge_detect_state.last_result.valid = 0;
    camera_bridge_detect_state.lane_width_initialized = 0;
    camera_bridge_detect_state.single_edge_frame_count = 0;
    return 0;
}

uint8 camproc_bridge_detect(
    const uint8 image[MT9V03X_H][MT9V03X_W],
    const CameraBridgeParams_t *params,
    CameraBridgeResult_t *result)
{
    uint8 left_valid = 0;
    uint8 right_valid = 0;
    uint8 both_edges_valid = 0;
    uint16 left_point_count = 0;
    uint16 right_point_count = 0;
    uint16 common_top = 0;
    uint16 common_bottom = 0;
    uint16 line_point_count = 0;
    float left_top_x = 0.0f;
    float left_bottom_x = 0.0f;
    float right_top_x = 0.0f;
    float right_bottom_x = 0.0f;
    float lane_width_top = 0.0f;
    float lane_width_bottom = 0.0f;
    float width_slope = 0.0f;
    float width_intercept = 0.0f;
    CameraBridgeLine_t left_line = {0};
    CameraBridgeLine_t right_line = {0};

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
       (0u == params->ransac_iterations) ||
       (0u == params->ransac_distance_px) ||
       (params->min_line_point_count < 2u) ||
       (0u == params->min_y_span) ||
       (0u == params->row_step) ||
       (0u == params->stable_pixel_count) ||
       (params->lane_width_filter_alpha < 0.0f) ||
       (params->lane_width_filter_alpha > 1.0f) ||
       ((uint16)params->stable_pixel_count * 2u >=
        (params->roi_right - params->roi_left + 1u)))
    {
        return 0;
    }

    camera_bridge_extract_edge_points(
        image, params, &left_point_count, &right_point_count);

    left_valid = camera_bridge_fit_line_ransac(
        camera_bridge_left_points,
        left_point_count,
        params,
        0,
        &left_line);
    right_valid = camera_bridge_fit_line_ransac(
        camera_bridge_right_points,
        right_point_count,
        params,
        1,
        &right_line);

    both_edges_valid = (uint8)(left_valid && right_valid);

    if(both_edges_valid)
    {
        common_top = (left_line.top > right_line.top) ?
            left_line.top : right_line.top;
        common_bottom = (left_line.bottom < right_line.bottom) ?
            left_line.bottom : right_line.bottom;

        if((common_bottom <= common_top) ||
           ((common_bottom - common_top) < params->min_y_span))
        {
            return camera_bridge_hold_last_result(params, result);
        }

        left_top_x = camera_bridge_line_x(&left_line, common_top);
        left_bottom_x = camera_bridge_line_x(&left_line, common_bottom);
        right_top_x = camera_bridge_line_x(&right_line, common_top);
        right_bottom_x = camera_bridge_line_x(&right_line, common_bottom);
        lane_width_top = right_top_x - left_top_x;
        lane_width_bottom = right_bottom_x - left_bottom_x;

        // 透视下赛道远端宽度不能大于近端宽度，排除起点横线造成的 V 形误拟合
        if((lane_width_top < params->min_lane_width) ||
           (lane_width_top > params->max_lane_width) ||
           (lane_width_bottom < params->min_lane_width) ||
           (lane_width_bottom > params->max_lane_width) ||
           (lane_width_top > lane_width_bottom))
        {
            return camera_bridge_hold_last_result(params, result);
        }

        // 双边线有效时更新随纵坐标变化的赛道宽度模型
        width_slope = right_line.slope - left_line.slope;
        width_intercept = right_line.intercept - left_line.intercept;
        if(!camera_bridge_detect_state.lane_width_initialized)
        {
            camera_bridge_detect_state.lane_width_slope = width_slope;
            camera_bridge_detect_state.lane_width_intercept = width_intercept;
            camera_bridge_detect_state.lane_width_initialized = 1;
        }
        else
        {
            camera_bridge_detect_state.lane_width_slope =
                  params->lane_width_filter_alpha *
                  camera_bridge_detect_state.lane_width_slope
                + (1.0f - params->lane_width_filter_alpha) *
                  width_slope;
            camera_bridge_detect_state.lane_width_intercept =
                  params->lane_width_filter_alpha *
                  camera_bridge_detect_state.lane_width_intercept
                + (1.0f - params->lane_width_filter_alpha) *
                  width_intercept;
        }

        camera_bridge_detect_state.single_edge_frame_count = 0;
        result->estimated = 0;
    }
    else if(camera_bridge_detect_state.lane_width_initialized &&
            (camera_bridge_detect_state.single_edge_frame_count <
             params->single_edge_hold_frames) &&
            (left_valid || right_valid))
    {
        // 一侧短暂出画时，使用之前学到的透视宽度模型补出缺失边线
        if(left_valid)
        {
            right_line = left_line;
            right_line.slope += camera_bridge_detect_state.lane_width_slope;
            right_line.intercept +=
                camera_bridge_detect_state.lane_width_intercept;
        }
        else
        {
            left_line = right_line;
            left_line.slope -= camera_bridge_detect_state.lane_width_slope;
            left_line.intercept -=
                camera_bridge_detect_state.lane_width_intercept;
        }

        common_top = left_valid ? left_line.top : right_line.top;
        common_bottom = left_valid ? left_line.bottom : right_line.bottom;
        left_top_x = camera_bridge_line_x(&left_line, common_top);
        left_bottom_x = camera_bridge_line_x(&left_line, common_bottom);
        right_top_x = camera_bridge_line_x(&right_line, common_top);
        right_bottom_x = camera_bridge_line_x(&right_line, common_bottom);
        lane_width_top = right_top_x - left_top_x;
        lane_width_bottom = right_bottom_x - left_bottom_x;

        // 补线结果同样必须满足远端不宽于近端，防止继续沿用错误的透视模型
        if((lane_width_top < params->min_lane_width) ||
           (lane_width_top > params->max_lane_width) ||
           (lane_width_bottom < params->min_lane_width) ||
           (lane_width_bottom > params->max_lane_width) ||
           (lane_width_top > lane_width_bottom))
        {
            return camera_bridge_hold_last_result(params, result);
        }

        camera_bridge_detect_state.single_edge_frame_count++;
        result->estimated = 1;
    }
    else
    {
        return camera_bridge_hold_last_result(params, result);
    }

    line_point_count = (left_line.point_count < right_line.point_count) ?
        left_line.point_count : right_line.point_count;

    result->top = common_top;
    result->bottom = common_bottom;
    result->point_count = line_point_count;
    result->left_x1 = camera_bridge_clamp_x(left_top_x);
    result->left_y1 = common_top;
    result->left_x2 = camera_bridge_clamp_x(left_bottom_x);
    result->left_y2 = common_bottom;
    result->right_x1 = camera_bridge_clamp_x(right_top_x);
    result->right_y1 = common_top;
    result->right_x2 = camera_bridge_clamp_x(right_bottom_x);
    result->right_y2 = common_bottom;
    result->center_x1 =
        camera_bridge_clamp_x((left_top_x + right_top_x) * 0.5f);
    result->center_y1 = common_top;
    result->center_x2 =
        camera_bridge_clamp_x((left_bottom_x + right_bottom_x) * 0.5f);
    result->center_y2 = common_bottom;
    result->valid = 1;

    camera_bridge_detect_state.lost_frame_count = 0;
    camera_bridge_detect_state.last_result = *result;
    return 1;
}

static int16 camproc_bridge_yaw_offset_to_control(
    int16 yaw_offset_d10,
    float control_gain_per_deg,
    const CameraBridgeAlignParams_t *align_params)
{
    float control_output = 0.0f;
    int32 control_value = 0;

    control_output = (float)yaw_offset_d10 * 0.1f
                   * control_gain_per_deg
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
    uint32 time_ms,
    const CameraBridgeResult_t *bridge_result,
    const CameraBridgeAlignParams_t *align_params,
    CameraBridgeAlignState_t *align_state,
    CameraBridgeAlignResult_t *align_result)
{
    uint16 vertical_span = 0;
    uint16 lookahead_px = 0;
    uint16 active_y = 0;
    uint8 position_inside = 0;
    uint8 strict_inside = 0;
    uint8 fallback_inside = 0;
    uint8 fallback_triggered = 0;
    uint8 edge_near_border = 0;
    uint8 control_sign_changed = 0;
    uint8 control_stable = 0;
    int8 control_sign = 0;
    float raw_slope = 0.0f;
    float raw_intercept = 0.0f;
    float raw_heading_d10 = 0.0f;
    float heading_d10 = 0.0f;
    float heading_abs_d10 = 0.0f;
    float active_line_filter_alpha = 0.0f;
    float active_yaw_gain = 0.0f;
    float active_control_gain_per_deg = 0.0f;
    float lookahead_ratio = 0.0f;
    float line_norm = 0.0f;
    float projection_y = 0.0f;
    float projection_x = 0.0f;
    float required_lookahead = 0.0f;
    float active_x = 0.0f;
    float active_y_float = 0.0f;
    float bottom_center_x = 0.0f;
    float lookahead_error = 0.0f;
    float forward_distance = 0.0f;
    float yaw_offset = 0.0f;
    int32 far_error = 0;
    int32 near_error = 0;
    int32 far_error_abs = 0;
    int32 near_error_abs = 0;
    int32 yaw_offset_d10 = 0;
    int32 yaw_change_d10 = 0;
    int32 control_delta = 0;
    int32 scaled_control = 0;
    uint16 active_control_deadband_d10 = 0;
    int16 active_yaw_slew_limit_d10 = 0;

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
       (0u == align_params->fallback_far_tolerance_px) ||
       (0u == align_params->fallback_near_tolerance_px) ||
       (0u == align_params->fallback_timeout_ms) ||
       (0u == align_params->fallback_estimated_grace_frames) ||
       (0u == align_params->lookahead_min_px) ||
       (align_params->lookahead_min_px > align_params->lookahead_max_px) ||
       (align_params->lookahead_max_px >= MT9V03X_H) ||
       (0u == align_params->lookahead_full_angle_d10) ||
       (0u == align_params->complete_confirm_frames) ||
       (0u == align_params->lost_reset_frames) ||
       (0u == align_params->near_enter_angle_d10) ||
       (align_params->near_enter_angle_d10 >=
        align_params->near_exit_angle_d10) ||
       (align_params->near_line_filter_alpha < 0.0f) ||
       (align_params->near_line_filter_alpha > 1.0f) ||
       (align_params->near_yaw_gain <= 0.0f) ||
       (0u == align_params->near_control_deadband_d10) ||
       (align_params->near_yaw_slew_limit_d10 <= 0) ||
       (align_params->near_control_gain_per_deg <= 0.0f) ||
       (0u == align_params->near_reverse_confirm_frames) ||
       (align_params->line_filter_alpha < 0.0f) ||
       (align_params->line_filter_alpha > 1.0f) ||
       (align_params->yaw_gain <= 0.0f) ||
       (0.0f == align_params->yaw_direction) ||
       (align_params->yaw_offset_limit_d10 <= 0) ||
       (align_params->yaw_slew_limit_d10 <= 0) ||
       (align_params->control_gain_per_deg <= 0.0f) ||
       (0.0f == align_params->control_direction) ||
       (align_params->control_limit <= 0) ||
       (0u == align_params->blind_trigger_angle_d10) ||
       (0u == align_params->blind_turn_time_ms) ||
       (0u == align_params->blind_edge_margin_px) ||
       (align_params->blind_edge_margin_px >= (MT9V03X_W / 2u)) ||
       (0u == align_params->blind_reliable_frames) ||
       (0u == align_params->blind_estimated_frames) ||
       (0u == align_params->blind_control_percent) ||
       (align_params->blind_control_percent > 100u))
    {
        return 0;
    }

    align_result->phase = align_state->phase;
    align_result->bottom_y = align_state->held_bottom_y;
    align_result->near_mode = align_state->near_mode_active;

    // 对准完成后持续锁存结果，保证核心0能够稳定收到连续确认信号
    if(CAMERA_BRIDGE_ALIGN_COMPLETE == align_state->phase)
    {
        align_result->valid = 1;
        align_result->point_inside = 1;
        align_result->aligned = 1;
        align_result->control_value = 0;
        return 1;
    }

    // 盲转阶段沿用可靠视觉控制，时间结束后重新读取当前新鲜图像
    if(CAMERA_BRIDGE_ALIGN_BLIND_TURN == align_state->phase)
    {
        align_state->near_mode_active = 0;
        align_state->near_reverse_count = 0;
        align_state->near_control_sign = 0;
        align_state->near_pending_sign = 0;
        align_state->fallback_timer_active = 0;
        align_state->fallback_start_time_ms = 0;
        align_state->fallback_estimated_count = 0;
        align_result->valid = 1;
        align_result->near_mode = 0;
        align_result->control_value = align_state->blind_control_value;

        if((time_ms - align_state->phase_start_time_ms) <
           align_params->blind_turn_time_ms)
        {
            return 1;
        }

        align_state->phase = CAMERA_BRIDGE_ALIGN_TRACK;
        align_state->blind_reliable_count = 0;
        align_state->blind_estimated_count = 0;
        align_state->blind_reference_control = 0;
        align_state->blind_control_value = 0;
        align_state->line_filter_initialized = 0;
        align_state->previous_yaw_offset_d10 = 0;
        align_state->previous_control_value = 0;
        align_result->phase = align_state->phase;
        align_result->control_value = 0;
    }

    if(!bridge_result->valid ||
       (bridge_result->center_y1 >= bridge_result->center_y2))
    {
        align_state->near_mode_active = 0;
        align_state->near_reverse_count = 0;
        align_state->near_control_sign = 0;
        align_state->near_pending_sign = 0;
        align_result->near_mode = 0;
        align_state->complete_frame_count = 0;
        align_state->fallback_timer_active = 0;
        align_state->fallback_start_time_ms = 0;
        align_state->fallback_estimated_count = 0;

        // 大角度可靠方向已经确定后，短暂丢线时继续执行分段盲转
        if((align_state->blind_reliable_count >=
            align_params->blind_reliable_frames) &&
           (0 != align_state->blind_reference_control))
        {
            scaled_control =
                (int32)align_state->blind_reference_control *
                align_params->blind_control_percent / 100;

            if(0 == scaled_control)
            {
                scaled_control =
                    (align_state->blind_reference_control > 0) ? 1 : -1;
            }

            align_state->blind_control_value = (int16)scaled_control;
            align_state->phase = CAMERA_BRIDGE_ALIGN_BLIND_TURN;
            align_state->phase_start_time_ms = time_ms;
            align_state->fallback_timer_active = 0;
            align_state->fallback_start_time_ms = 0;
            align_state->fallback_estimated_count = 0;
            align_state->blind_reliable_count = 0;
            align_state->blind_estimated_count = 0;
            align_state->blind_reference_control = 0;
            align_result->valid = 1;
            align_result->phase = align_state->phase;
            align_result->control_value = align_state->blind_control_value;
            align_result->bottom_y = align_state->held_bottom_y;
            return 1;
        }

        if(align_state->lost_frame_count < align_params->lost_reset_frames)
        {
            align_state->lost_frame_count++;
        }

        align_state->previous_yaw_offset_d10 = 0;
        align_state->line_filter_initialized = 0;

        if(align_state->lost_frame_count >= align_params->lost_reset_frames)
        {
            camproc_bridge_align_reset(align_state);
        }

        align_result->phase = align_state->phase;
        align_result->near_mode = 0;
        return 0;
    }

    align_state->lost_frame_count = 0;
    align_state->held_bottom_y = bridge_result->bottom;
    align_result->valid = 1;
    align_result->bottom_y = align_state->held_bottom_y;

    far_error = (int32)bridge_result->center_x1 - align_params->target_center_x;
    near_error = (int32)bridge_result->center_x2 - align_params->target_center_x;
    far_error_abs = (far_error >= 0) ? far_error : -far_error;
    near_error_abs = (near_error >= 0) ? near_error : -near_error;
    vertical_span = bridge_result->center_y2 - bridge_result->center_y1;
    raw_slope =
        ((float)bridge_result->center_x2 - bridge_result->center_x1) /
        vertical_span;
    raw_intercept =
        (float)bridge_result->center_x1 -
        raw_slope * bridge_result->center_y1;
    raw_heading_d10 = atan2f(-raw_slope, 1.0f) * CAMERA_BRIDGE_RAD_TO_D10;

    position_inside = (uint8)(
        (far_error >= -(int32)align_params->far_tolerance_px) &&
        (far_error <=  (int32)align_params->far_tolerance_px) &&
        (near_error >= -(int32)align_params->near_tolerance_px) &&
        (near_error <=  (int32)align_params->near_tolerance_px));
    align_result->point_inside = position_inside;
    strict_inside = (uint8)(!bridge_result->estimated && position_inside);
    fallback_inside = (uint8)(
        (far_error_abs <= (int32)align_params->fallback_far_tolerance_px) &&
        (near_error_abs <= (int32)align_params->fallback_near_tolerance_px));

    // 小角度且已经靠近目标范围时进入近对准，使用迟滞避免反复切换
    if(!align_state->near_mode_active)
    {
        if(!bridge_result->estimated &&
           fallback_inside &&
           (fabsf(raw_heading_d10) <= align_params->near_enter_angle_d10))
        {
            align_state->near_mode_active = 1;
            align_state->near_reverse_count = 0;
            align_state->near_control_sign = 0;
            align_state->near_pending_sign = 0;
        }
    }
    else if(!fallback_inside ||
            (fabsf(raw_heading_d10) >= align_params->near_exit_angle_d10))
    {
        align_state->near_mode_active = 0;
        align_state->near_reverse_count = 0;
        align_state->near_control_sign = 0;
        align_state->near_pending_sign = 0;
    }
    align_result->near_mode = align_state->near_mode_active;

    // 新鲜中线上下端点连续进入红框后直接完成对准
    if(strict_inside)
    {
        if(align_state->complete_frame_count <
           align_params->complete_confirm_frames)
        {
            align_state->complete_frame_count++;
        }
    }
    else
    {
        align_state->complete_frame_count = 0;
    }

    // 保底范围允许少量估算帧，避免单帧补线反复清空驻留计时
    if(fallback_inside)
    {
        if(bridge_result->estimated)
        {
            if(align_state->fallback_timer_active &&
               (align_state->fallback_estimated_count <
                align_params->fallback_estimated_grace_frames))
            {
                align_state->fallback_estimated_count++;
            }
            else
            {
                align_state->fallback_timer_active = 0;
                align_state->fallback_start_time_ms = 0;
                align_state->fallback_estimated_count = 0;
            }
        }
        else
        {
            align_state->fallback_estimated_count = 0;

            if(!align_state->fallback_timer_active)
            {
                align_state->fallback_timer_active = 1;
                align_state->fallback_start_time_ms = time_ms;
            }
            else if((time_ms - align_state->fallback_start_time_ms) >=
                    align_params->fallback_timeout_ms)
            {
                fallback_triggered = 1;
            }
        }
    }
    else
    {
        align_state->fallback_timer_active = 0;
        align_state->fallback_start_time_ms = 0;
        align_state->fallback_estimated_count = 0;
    }

    if((align_state->complete_frame_count >=
        align_params->complete_confirm_frames) ||
       fallback_triggered)
    {
        align_state->phase = CAMERA_BRIDGE_ALIGN_COMPLETE;
        align_state->previous_yaw_offset_d10 = 0;
        align_state->previous_control_value = 0;
        align_result->point_inside = 1;
        align_result->aligned = 1;
        align_result->phase = align_state->phase;
        align_result->control_value = 0;
        return 1;
    }

    if(align_state->near_mode_active)
    {
        active_line_filter_alpha = align_params->near_line_filter_alpha;
        active_yaw_gain = align_params->near_yaw_gain;
        active_control_deadband_d10 = align_params->near_control_deadband_d10;
        active_yaw_slew_limit_d10 = align_params->near_yaw_slew_limit_d10;
        active_control_gain_per_deg = align_params->near_control_gain_per_deg;
    }
    else
    {
        active_line_filter_alpha = align_params->line_filter_alpha;
        active_yaw_gain = align_params->yaw_gain;
        active_control_deadband_d10 = align_params->control_deadband_d10;
        active_yaw_slew_limit_d10 = align_params->yaw_slew_limit_d10;
        active_control_gain_per_deg = align_params->control_gain_per_deg;
    }

    // 对直线参数而不是单个端点滤波，端点纵坐标变化时仍保持同一条中线
    if(!align_state->line_filter_initialized)
    {
        align_state->filtered_slope = raw_slope;
        align_state->filtered_intercept = raw_intercept;
        align_state->line_filter_initialized = 1;
    }
    else
    {
        align_state->filtered_slope =
              active_line_filter_alpha * align_state->filtered_slope
            + (1.0f - active_line_filter_alpha) * raw_slope;
        align_state->filtered_intercept =
              active_line_filter_alpha * align_state->filtered_intercept
            + (1.0f - active_line_filter_alpha) * raw_intercept;
    }

    heading_d10 =
        atan2f(-align_state->filtered_slope, 1.0f) *
        CAMERA_BRIDGE_RAD_TO_D10;
    heading_abs_d10 = fabsf(heading_d10);
    if(heading_abs_d10 > align_params->lookahead_full_angle_d10)
    {
        heading_abs_d10 = align_params->lookahead_full_angle_d10;
    }

    // 角度越大预瞄距离越短，使大角度接近时仍能快速修正横向位置
    lookahead_ratio =
        heading_abs_d10 / align_params->lookahead_full_angle_d10;
    lookahead_px = (uint16)(
        align_params->lookahead_max_px -
        (align_params->lookahead_max_px - align_params->lookahead_min_px) *
        lookahead_ratio + 0.5f);
    line_norm = sqrtf(
        1.0f +
        align_state->filtered_slope * align_state->filtered_slope);

    // 先把车辆视觉锚点投影到中线，再沿中线向画面上方移动预瞄距离
    projection_y =
        (((float)align_params->target_center_x -
          align_state->filtered_intercept) *
         align_state->filtered_slope +
         (MT9V03X_H - 1u)) /
        (1.0f +
         align_state->filtered_slope * align_state->filtered_slope);
    projection_x =
        align_state->filtered_slope * projection_y +
        align_state->filtered_intercept;
    required_lookahead = (float)lookahead_px;

    // 横向误差很大时投影点可能落在车后，确保最终目标仍位于车辆前方
    if((projection_y - required_lookahead / line_norm) >=
       (MT9V03X_H - 2u))
    {
        required_lookahead =
            (projection_y - (MT9V03X_H - 2u)) * line_norm;
    }

    active_x =
        projection_x -
        align_state->filtered_slope / line_norm * required_lookahead;
    active_y_float =
        projection_y - required_lookahead / line_norm;
    bottom_center_x =
        align_state->filtered_slope * (MT9V03X_H - 1u) +
        align_state->filtered_intercept;
    lookahead_error = active_x - align_params->target_center_x;
    forward_distance = (MT9V03X_H - 1u) - active_y_float;

    if(active_y_float <= 0.0f)
    {
        active_y = 0;
    }
    else if(active_y_float >= (float)(MT9V03X_H - 1u))
    {
        active_y = MT9V03X_H - 1u;
    }
    else
    {
        active_y = (uint16)(active_y_float + 0.5f);
    }

    align_result->active_x = camera_bridge_clamp_x(active_x);
    align_result->active_y = active_y;
    align_result->lookahead_px = lookahead_px;
    align_result->heading_error_d10 = (heading_d10 >= 0.0f) ?
        (int16)(heading_d10 + 0.5f) : (int16)(heading_d10 - 0.5f);
    align_result->lateral_error_px =
        (bottom_center_x >= align_params->target_center_x) ?
        (int16)(bottom_center_x - align_params->target_center_x + 0.5f) :
        (int16)(bottom_center_x - align_params->target_center_x - 0.5f);
    align_result->lookahead_error_px = (lookahead_error >= 0.0f) ?
        (int16)(lookahead_error + 0.5f) : (int16)(lookahead_error - 0.5f);

    // 一个预瞄目标同时包含横向偏差和中线方向，不再叠加两项可能互相抵消的输出
    yaw_offset =
          atan2f(lookahead_error, forward_distance)
        * CAMERA_BRIDGE_RAD_TO_D10
        * active_yaw_gain
        * align_params->yaw_direction;

    if(yaw_offset > active_control_deadband_d10)
    {
        yaw_offset -= active_control_deadband_d10;
    }
    else if(yaw_offset < -(float)active_control_deadband_d10)
    {
        yaw_offset += active_control_deadband_d10;
    }
    else
    {
        yaw_offset = 0.0f;
    }

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
    if(yaw_change_d10 > active_yaw_slew_limit_d10)
    {
        yaw_offset_d10 =
            align_state->previous_yaw_offset_d10 + active_yaw_slew_limit_d10;
    }
    else if(yaw_change_d10 < -active_yaw_slew_limit_d10)
    {
        yaw_offset_d10 =
            align_state->previous_yaw_offset_d10 - active_yaw_slew_limit_d10;
    }

    align_state->previous_yaw_offset_d10 = (int16)yaw_offset_d10;
    align_result->yaw_offset_d10 = (int16)yaw_offset_d10;
    align_result->control_value = camproc_bridge_yaw_offset_to_control(
        align_result->yaw_offset_d10,
        active_control_gain_per_deg,
        align_params);

    // 近对准时反向控制必须连续出现，避免拟合线抖动造成左右快速换向
    control_sign = (align_result->control_value > 0) ? 1 :
                   ((align_result->control_value < 0) ? -1 : 0);
    if(align_state->near_mode_active)
    {
        if(0 == control_sign)
        {
            align_state->near_reverse_count = 0;
            align_state->near_pending_sign = 0;
        }
        else if((0 == align_state->near_control_sign) ||
                (control_sign == align_state->near_control_sign))
        {
            align_state->near_control_sign = control_sign;
            align_state->near_reverse_count = 0;
            align_state->near_pending_sign = 0;
        }
        else
        {
            if(control_sign != align_state->near_pending_sign)
            {
                align_state->near_pending_sign = control_sign;
                align_state->near_reverse_count = 1;
            }
            else if(align_state->near_reverse_count <
                    align_params->near_reverse_confirm_frames)
            {
                align_state->near_reverse_count++;
            }

            if(align_state->near_reverse_count >=
               align_params->near_reverse_confirm_frames)
            {
                align_state->near_control_sign = control_sign;
                align_state->near_reverse_count = 0;
                align_state->near_pending_sign = 0;
            }
            else
            {
                align_result->control_value = 0;
            }
        }
    }

    control_delta =
        (int32)align_result->control_value -
        align_state->previous_control_value;
    if(control_delta < 0)
    {
        control_delta = -control_delta;
    }

    control_sign_changed = (uint8)(
        ((align_state->previous_control_value > 0) &&
         (align_result->control_value < 0)) ||
        ((align_state->previous_control_value < 0) &&
         (align_result->control_value > 0)));
    control_stable = (uint8)(
        (0 == align_state->previous_control_value) ||
        (!control_sign_changed &&
         (control_delta <= CAMERA_BRIDGE_CONTROL_STABLE_DELTA)));

    edge_near_border = (uint8)(
        (bridge_result->left_x1 <= align_params->blind_edge_margin_px) ||
        (bridge_result->left_x2 <= align_params->blind_edge_margin_px) ||
        (bridge_result->right_x1 >=
         (MT9V03X_W - 1u - align_params->blind_edge_margin_px)) ||
        (bridge_result->right_x2 >=
         (MT9V03X_W - 1u - align_params->blind_edge_margin_px)));

    // 只使用连续的新鲜大角度结果保存盲转方向，避免把补线噪声写入历史
    if(!bridge_result->estimated &&
       (heading_abs_d10 >= align_params->blind_trigger_angle_d10) &&
       (0 != align_result->control_value))
    {
        if(control_stable)
        {
            if(align_state->blind_reliable_count <
               align_params->blind_reliable_frames)
            {
                align_state->blind_reliable_count++;
            }

            if(1u == align_state->blind_reliable_count)
            {
                align_state->blind_reference_control =
                    align_result->control_value;
            }
            else
            {
                align_state->blind_reference_control = (int16)(
                    ((int32)align_state->blind_reference_control *
                     (int32)(align_state->blind_reliable_count - 1u) +
                     align_result->control_value) /
                    (int32)align_state->blind_reliable_count);
            }
        }
        else
        {
            align_state->blind_reliable_count = 1;
            align_state->blind_reference_control =
                align_result->control_value;
        }

        align_state->blind_estimated_count = 0;
    }
    else if(!bridge_result->estimated)
    {
        align_state->blind_reliable_count = 0;
        align_state->blind_estimated_count = 0;
        align_state->blind_reference_control = 0;
    }
    else if(align_state->blind_reliable_count >=
            align_params->blind_reliable_frames)
    {
        if(align_state->blind_estimated_count <
           align_params->blind_estimated_frames)
        {
            align_state->blind_estimated_count++;
        }
    }

    // 大角度边线接近画幅，或已经连续使用补线结果时进入短时盲转
    if((align_state->blind_reliable_count >=
        align_params->blind_reliable_frames) &&
       (edge_near_border ||
        (align_state->blind_estimated_count >=
         align_params->blind_estimated_frames)))
    {
        scaled_control =
            (int32)align_state->blind_reference_control *
            align_params->blind_control_percent / 100;

        if(0 == scaled_control)
        {
            scaled_control =
                (align_state->blind_reference_control > 0) ? 1 : -1;
        }

        align_state->blind_control_value = (int16)scaled_control;
        align_state->phase = CAMERA_BRIDGE_ALIGN_BLIND_TURN;
        align_state->phase_start_time_ms = time_ms;
        align_state->near_mode_active = 0;
        align_state->near_reverse_count = 0;
        align_state->near_control_sign = 0;
        align_state->near_pending_sign = 0;
        align_state->fallback_timer_active = 0;
        align_state->fallback_start_time_ms = 0;
        align_state->fallback_estimated_count = 0;
        align_state->blind_reliable_count = 0;
        align_state->blind_estimated_count = 0;
        align_state->blind_reference_control = 0;
        align_result->phase = align_state->phase;
        align_result->near_mode = 0;
        align_result->control_value = align_state->blind_control_value;
        align_state->previous_control_value =
            align_state->blind_control_value;
        return 1;
    }

    align_state->previous_control_value = align_result->control_value;
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
