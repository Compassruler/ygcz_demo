#include "camera_proc.h"
#include <math.h>
#include <string.h>

// ==================================================== 参数调节 ====================================================
#define CAMERA_BRIDGE_RAD_TO_D10                (572.9577951f)
#define CAMERA_BRIDGE_REFERENCE_ROW_DEFAULT     ((MT9V03X_H * 3u) / 4u)
#define CAMERA_BRIDGE_MIN_FILL_PERCENT          (35u)                       // 候选区域的最低黑色填充率
#define CAMERA_BRIDGE_MAX_EDGE_MSE              (9.0f)                      // 右边线允许的最大拟合误差
#define CAMERA_BRIDGE_EDGE_POINT_DISTANCE       (3.0f)                      // 轮廓点到初始右边线的最大横向距离
#define CAMERA_BRIDGE_BOTTOM_PRIORITY_MARGIN    (5)                         // 候选矩形下方优先的位置容差
#define CAMERA_BRIDGE_VISITED_BUFFER_SIZE       ((MT9V03X_IMAGE_SIZE + 7u) / 8u) // 连通域访问标记所需空间
#define CAMERA_BRIDGE_HULL_BUFFER_SIZE          ((MT9V03X_W + MT9V03X_H) * 2u + 2u) // 凸包最大点数
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
    uint8 x;
    uint8 y;
} CameraBridgePoint_t;

typedef struct
{
    uint8 valid;
    uint16 left;
    uint16 right;
    uint16 top;
    uint16 bottom;
    uint16 edge_y1;
    uint16 edge_y2;
    uint16 edge_point_count;
    uint32 area;
    float right_slope;
    float right_intercept;
    float edge_error;
} CameraBridgeCandidate_t;

static uint8 camera_bridge_visited[CAMERA_BRIDGE_VISITED_BUFFER_SIZE];
static uint8 camera_bridge_component_mask[CAMERA_BRIDGE_VISITED_BUFFER_SIZE];
static CameraBridgePoint_t camera_bridge_queue[MT9V03X_IMAGE_SIZE];
static CameraBridgePoint_t camera_bridge_hull[CAMERA_BRIDGE_HULL_BUFFER_SIZE];

static uint8 camera_bridge_is_visited(uint16 x, uint16 y)
{
    uint32 index = (uint32)y * MT9V03X_W + x;

    return (camera_bridge_visited[index >> 3u] & (uint8)(1u << (index & 7u))) != 0u;
}

static void camera_bridge_mark_visited(uint16 x, uint16 y)
{
    uint32 index = (uint32)y * MT9V03X_W + x;

    camera_bridge_visited[index >> 3u] |= (uint8)(1u << (index & 7u));
}

static uint8 camera_bridge_is_component_point(uint16 x, uint16 y)
{
    uint32 index = (uint32)y * MT9V03X_W + x;

    return (camera_bridge_component_mask[index >> 3u] & (uint8)(1u << (index & 7u))) != 0u;
}

static void camera_bridge_mark_component_point(uint16 x, uint16 y)
{
    uint32 index = (uint32)y * MT9V03X_W + x;

    camera_bridge_component_mask[index >> 3u] |= (uint8)(1u << (index & 7u));
}

static void camera_bridge_clear_component_point(uint16 x, uint16 y)
{
    uint32 index = (uint32)y * MT9V03X_W + x;

    camera_bridge_component_mask[index >> 3u] &= (uint8)~(uint8)(1u << (index & 7u));
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

static uint8 camera_bridge_fit_edge(
    const uint16 edge_x[MT9V03X_H],
    uint16 top,
    uint16 bottom,
    float *slope,
    float *intercept,
    uint16 *edge_point_count,
    float *mean_square_error)
{
    uint16 y = 0;
    uint16 index = 0;
    uint32 sum_x = 0;
    uint32 sum_y = 0;
    uint32 sum_xy = 0;
    uint32 sum_yy = 0;
    float point_count = 0.0f;
    float denominator = 0.0f;
    float error_sum = 0.0f;

    if((NULL == slope) || (NULL == intercept) ||
       (NULL == edge_point_count) || (NULL == mean_square_error))
    {
        return 0;
    }

    for(y = top; y <= bottom; y++)
    {
        if(edge_x[y] >= MT9V03X_W)
        {
            continue;
        }

        sum_x += edge_x[y];
        sum_y += y;
        sum_xy += (uint32)edge_x[y] * y;
        sum_yy += (uint32)y * y;
        index++;
    }

    if(index < 2u)
    {
        return 0;
    }

    point_count = (float)index;
    denominator = point_count * (float)sum_yy - (float)sum_y * (float)sum_y;
    if(denominator <= 0.0f)
    {
        return 0;
    }

    *slope = (point_count * (float)sum_xy - (float)sum_x * (float)sum_y) / denominator;
    *intercept = ((float)sum_x - *slope * (float)sum_y) / point_count;

    for(y = top; y <= bottom; y++)
    {
        float error = 0.0f;

        if(edge_x[y] >= MT9V03X_W)
        {
            continue;
        }

        error = (float)edge_x[y] - (*slope * (float)y + *intercept);
        error_sum += error * error;
    }

    *edge_point_count = index;
    *mean_square_error = error_sum / point_count;

    return 1;
}

static int32 camera_bridge_cross_points(CameraBridgePoint_t point_o, CameraBridgePoint_t point_a, CameraBridgePoint_t point_b)
{
    return ((int32)point_a.x - point_o.x) * ((int32)point_b.y - point_o.y) -
           ((int32)point_a.y - point_o.y) * ((int32)point_b.x - point_o.x);
}

// 按坐标顺序扫描当前连通域，生成外轮廓凸包。
static uint16 camera_bridge_build_convex_hull(uint16 left, uint16 right, uint16 top, uint16 bottom)
{
    uint16 x = 0;
    uint16 y = 0;
    uint16 hull_count = 0;
    uint16 upper_start = 0;
    int16 reverse_x = 0;
    int16 reverse_y = 0;
    uint8 skipped_last_point = 0;
    CameraBridgePoint_t point = {0};

    // 构建下半部分凸包。
    for(x = left; x <= right; x++)
    {
        for(y = top; y <= bottom; y++)
        {
            if(!camera_bridge_is_component_point(x, y))
            {
                continue;
            }

            point.x = (uint8)x;
            point.y = (uint8)y;

            while((hull_count >= 2u) &&
                  (camera_bridge_cross_points(camera_bridge_hull[hull_count - 2u],
                                              camera_bridge_hull[hull_count - 1u], point) <= 0))
            {
                hull_count--;
            }

            if(hull_count >= CAMERA_BRIDGE_HULL_BUFFER_SIZE)
            {
                return 0;
            }

            camera_bridge_hull[hull_count] = point;
            hull_count++;
        }
    }

    if(hull_count < 2u)
    {
        return 0;
    }

    // 反向扫描构建上半部分凸包，首尾重复点最终会被移除。
    upper_start = hull_count + 1u;
    for(reverse_x = (int16)right; reverse_x >= (int16)left; reverse_x--)
    {
        for(reverse_y = (int16)bottom; reverse_y >= (int16)top; reverse_y--)
        {
            if(!camera_bridge_is_component_point((uint16)reverse_x, (uint16)reverse_y))
            {
                continue;
            }

            if(!skipped_last_point)
            {
                skipped_last_point = 1;
                continue;
            }

            point.x = (uint8)reverse_x;
            point.y = (uint8)reverse_y;

            while((hull_count >= upper_start) &&
                  (camera_bridge_cross_points(camera_bridge_hull[hull_count - 2u],
                                              camera_bridge_hull[hull_count - 1u], point) <= 0))
            {
                hull_count--;
            }

            if(hull_count >= CAMERA_BRIDGE_HULL_BUFFER_SIZE)
            {
                return 0;
            }

            camera_bridge_hull[hull_count] = point;
            hull_count++;
        }
    }

    if(hull_count > 1u)
    {
        hull_count--;
    }

    return hull_count;
}

// 反复删除对轮廓影响最小的点，将凸包简化为四边形。
static uint16 camera_bridge_simplify_hull_to_quad(uint16 hull_count)
{
    uint16 index = 0;
    uint16 move_index = 0;
    uint16 remove_index = 0;
    uint16 previous_index = 0;
    uint16 next_index = 0;
    uint32 triangle_area = 0;
    uint32 minimum_triangle_area = 0;
    int32 cross_value = 0;

    if(hull_count < 4u)
    {
        return 0;
    }

    while(hull_count > 4u)
    {
        minimum_triangle_area = 0xFFFFFFFFu;

        for(index = 0; index < hull_count; index++)
        {
            previous_index = (0u == index) ? (uint16)(hull_count - 1u) : (uint16)(index - 1u);
            next_index = (index + 1u == hull_count) ? 0u : (index + 1u);
            cross_value = camera_bridge_cross_points(camera_bridge_hull[previous_index],
                                                      camera_bridge_hull[index],
                                                      camera_bridge_hull[next_index]);
            triangle_area = (cross_value < 0) ? (uint32)(-cross_value) : (uint32)cross_value;

            if(triangle_area < minimum_triangle_area)
            {
                minimum_triangle_area = triangle_area;
                remove_index = index;
            }
        }

        for(move_index = remove_index; move_index + 1u < hull_count; move_index++)
        {
            camera_bridge_hull[move_index] = camera_bridge_hull[move_index + 1u];
        }

        hull_count--;
    }

    return hull_count;
}

static uint32 camera_bridge_quad_area_twice(void)
{
    uint8 index = 0;
    uint8 next_index = 0;
    int32 area_twice = 0;

    for(index = 0; index < 4u; index++)
    {
        next_index = (uint8)((index + 1u) & 0x03u);
        area_twice += (int32)camera_bridge_hull[index].x * camera_bridge_hull[next_index].y -
                      (int32)camera_bridge_hull[index].y * camera_bridge_hull[next_index].x;
    }

    return (area_twice < 0) ? (uint32)(-area_twice) : (uint32)area_twice;
}

// 从四条边中选择横坐标最靠右、并具有足够纵向长度的一条边。
static uint8 camera_bridge_find_right_side(uint16 min_height, CameraBridgePoint_t *upper_point, CameraBridgePoint_t *lower_point)
{
    uint8 index = 0;
    uint8 next_index = 0;
    uint8 best_index = 0;
    uint16 vertical_span = 0;
    uint16 best_vertical_span = 0;
    int16 side_score = 0;
    int16 best_side_score = -1;
    CameraBridgePoint_t point_a = {0};
    CameraBridgePoint_t point_b = {0};

    if((NULL == upper_point) || (NULL == lower_point))
    {
        return 0;
    }

    for(index = 0; index < 4u; index++)
    {
        next_index = (uint8)((index + 1u) & 0x03u);
        point_a = camera_bridge_hull[index];
        point_b = camera_bridge_hull[next_index];
        vertical_span = (point_a.y >= point_b.y) ?
            (uint16)(point_a.y - point_b.y) : (uint16)(point_b.y - point_a.y);

        if(vertical_span + 1u < min_height)
        {
            continue;
        }

        side_score = (int16)point_a.x + point_b.x;
        if((side_score > best_side_score) ||
           ((side_score == best_side_score) && (vertical_span > best_vertical_span)))
        {
            best_index = index;
            best_side_score = side_score;
            best_vertical_span = vertical_span;
        }
    }

    if(best_side_score < 0)
    {
        return 0;
    }

    point_a = camera_bridge_hull[best_index];
    point_b = camera_bridge_hull[(best_index + 1u) & 0x03u];

    if(point_a.y <= point_b.y)
    {
        *upper_point = point_a;
        *lower_point = point_b;
    }
    else
    {
        *upper_point = point_b;
        *lower_point = point_a;
    }

    return upper_point->y < lower_point->y;
}

uint8 camproc_bridge_detect(const uint8 image[MT9V03X_H][MT9V03X_W], const CameraBridgeParams_t *params, CameraBridgeResult_t *result)
{
    uint16 x = 0;
    uint16 y = 0;
    uint16 edge_y = 0;
    uint16 reference_row = 0;
    uint16 component_left = 0;
    uint16 component_right = 0;
    uint16 component_top = 0;
    uint16 component_bottom = 0;
    uint16 component_width = 0;
    uint16 component_height = 0;
    uint16 right_point_count = 0;
    uint16 hull_count = 0;
    int16 neighbor_x = 0;
    int16 neighbor_y = 0;
    int16 neighbor_left = 0;
    int16 neighbor_right = 0;
    int16 neighbor_span = 0;
    int16 bottom_difference = 0;
    uint32 queue_head = 0;
    uint32 queue_tail = 0;
    uint32 component_area = 0;
    uint32 component_box_area = 0;
    uint32 quadrilateral_area_twice = 0;
    uint32 search_area = 0;
    uint16 right_edge[MT9V03X_H];
    uint16 filtered_right_edge[MT9V03X_H];
    float rough_right_slope = 0.0f;
    float rough_right_intercept = 0.0f;
    float right_slope = 0.0f;
    float right_intercept = 0.0f;
    float right_error = 0.0f;
    float edge_delta_x = 0.0f;
    float edge_delta_y = 0.0f;
    float edge_length_squared = 0.0f;
    float min_edge_length_squared = 0.0f;
    float max_edge_length_squared = 0.0f;
    float expected_x = 0.0f;
    float point_distance = 0.0f;
    float angle_d10 = 0.0f;
    CameraBridgePoint_t point = {0};
    CameraBridgePoint_t right_upper_point = {0};
    CameraBridgePoint_t right_lower_point = {0};
    CameraBridgeCandidate_t best_candidate = {0};

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
       (0 == params->min_edge_length) ||
       (params->min_edge_length > params->max_edge_length) ||
       (0 == params->min_area) ||
       (params->connect_gap >= MT9V03X_W) ||
       (params->target_edge_x >= MT9V03X_W) ||
       ((0u != params->reference_row) && (params->reference_row >= MT9V03X_H)))
    {
        return 0;
    }

    if((params->min_width > (params->search_right - params->search_left + 1u)) ||
       (params->min_height > (params->search_bottom - params->search_top + 1u)))
    {
        return 0;
    }

    search_area = (uint32)(params->search_right - params->search_left + 1u) *
                  (uint32)(params->search_bottom - params->search_top + 1u);
    if(params->min_area > search_area)
    {
        return 0;
    }

    min_edge_length_squared = (float)params->min_edge_length * (float)params->min_edge_length;
    max_edge_length_squared = (float)params->max_edge_length * (float)params->max_edge_length;

    memset(camera_bridge_visited, 0, sizeof(camera_bridge_visited));
    memset(camera_bridge_component_mask, 0, sizeof(camera_bridge_component_mask));

    // 逐点搜索全部黑色连通区域，而不是只跟踪画面中的第一个黑段。
    for(y = params->search_top; y <= params->search_bottom; y++)
    {
        for(x = params->search_left; x <= params->search_right; x++)
        {
            if((0 != image[y][x]) || camera_bridge_is_visited(x, y))
            {
                continue;
            }

            memset(right_edge, 0xFF, sizeof(right_edge));

            component_left = x;
            component_right = x;
            component_top = y;
            component_bottom = y;
            component_area = 0;
            queue_head = 0;
            queue_tail = 0;

            camera_bridge_mark_visited(x, y);
            camera_bridge_mark_component_point(x, y);
            camera_bridge_queue[queue_tail].x = (uint8)x;
            camera_bridge_queue[queue_tail].y = (uint8)y;
            queue_tail++;

            // 使用八邻域扩展连通区域，connect_gap 允许相邻两行存在少量横向错位。
            while(queue_head < queue_tail)
            {
                point = camera_bridge_queue[queue_head];
                queue_head++;
                component_area++;

                if(point.x < component_left) component_left = point.x;
                if(point.x > component_right) component_right = point.x;
                if(point.y < component_top) component_top = point.y;
                if(point.y > component_bottom) component_bottom = point.y;

                if(right_edge[point.y] >= MT9V03X_W)
                {
                    right_edge[point.y] = point.x;
                }
                else
                {
                    if(point.x > right_edge[point.y]) right_edge[point.y] = point.x;
                }

                for(neighbor_y = (int16)point.y - 1; neighbor_y <= (int16)point.y + 1; neighbor_y++)
                {
                    if((neighbor_y < (int16)params->search_top) ||
                       (neighbor_y > (int16)params->search_bottom))
                    {
                        continue;
                    }

                    neighbor_span = (neighbor_y == (int16)point.y) ? 1 : (int16)params->connect_gap + 1;
                    neighbor_left = (int16)point.x - neighbor_span;
                    neighbor_right = (int16)point.x + neighbor_span;

                    if(neighbor_left < (int16)params->search_left) neighbor_left = (int16)params->search_left;
                    if(neighbor_right > (int16)params->search_right) neighbor_right = (int16)params->search_right;

                    for(neighbor_x = neighbor_left; neighbor_x <= neighbor_right; neighbor_x++)
                    {
                        if((neighbor_x == (int16)point.x) && (neighbor_y == (int16)point.y))
                        {
                            continue;
                        }

                        if((0 == image[neighbor_y][neighbor_x]) &&
                           !camera_bridge_is_visited((uint16)neighbor_x, (uint16)neighbor_y))
                        {
                            camera_bridge_mark_visited((uint16)neighbor_x, (uint16)neighbor_y);
                            camera_bridge_mark_component_point((uint16)neighbor_x, (uint16)neighbor_y);
                            camera_bridge_queue[queue_tail].x = (uint8)neighbor_x;
                            camera_bridge_queue[queue_tail].y = (uint8)neighbor_y;
                            queue_tail++;
                        }
                    }
                }
            }

            component_width = (uint16)(component_right - component_left + 1u);
            component_height = (uint16)(component_bottom - component_top + 1u);

            // 当前连通域仍保留在掩码中，用它生成凸包；随后立即清除掩码。
            hull_count = camera_bridge_build_convex_hull(component_left, component_right,
                                                          component_top, component_bottom);
            for(queue_head = 0; queue_head < queue_tail; queue_head++)
            {
                camera_bridge_clear_component_point(camera_bridge_queue[queue_head].x,
                                                    camera_bridge_queue[queue_head].y);
            }

            if((component_area < params->min_area) ||
               (component_width < params->min_width) ||
               (component_height < params->min_height) ||
               (hull_count < 4u))
            {
                continue;
            }

            component_box_area = (uint32)component_width * component_height;
            if((component_area * 100u) < (component_box_area * CAMERA_BRIDGE_MIN_FILL_PERCENT))
            {
                continue;
            }

            hull_count = camera_bridge_simplify_hull_to_quad(hull_count);
            if(4u != hull_count)
            {
                continue;
            }

            quadrilateral_area_twice = camera_bridge_quad_area_twice();
            if((quadrilateral_area_twice < params->min_area * 2u) ||
               ((component_area * 200u) <
                (quadrilateral_area_twice * CAMERA_BRIDGE_MIN_FILL_PERCENT)))
            {
                continue;
            }

            if(!camera_bridge_find_right_side(params->min_height,
                                              &right_upper_point, &right_lower_point))
            {
                continue;
            }

            // 四边形角点先给出粗略右边线，再筛选真正位于该边附近的逐行轮廓点。
            rough_right_slope = ((float)right_lower_point.x - right_upper_point.x) /
                                ((float)right_lower_point.y - right_upper_point.y);
            rough_right_intercept = (float)right_upper_point.x -
                                    rough_right_slope * right_upper_point.y;
            memset(filtered_right_edge, 0xFF, sizeof(filtered_right_edge));

            for(edge_y = right_upper_point.y; edge_y <= right_lower_point.y; edge_y++)
            {
                if(right_edge[edge_y] >= MT9V03X_W)
                {
                    continue;
                }

                expected_x = rough_right_slope * edge_y + rough_right_intercept;
                point_distance = (float)right_edge[edge_y] - expected_x;
                if(point_distance < 0.0f)
                {
                    point_distance = -point_distance;
                }

                if(point_distance <= CAMERA_BRIDGE_EDGE_POINT_DISTANCE)
                {
                    filtered_right_edge[edge_y] = right_edge[edge_y];
                }
            }

            if(!camera_bridge_fit_edge(filtered_right_edge,
                                       right_upper_point.y, right_lower_point.y,
                                       &right_slope, &right_intercept,
                                       &right_point_count, &right_error) ||
               (right_point_count < params->min_height) ||
               (right_error > CAMERA_BRIDGE_MAX_EDGE_MSE))
            {
                continue;
            }

            // 根据拟合线两个端点之间的实际长度过滤过短或过长的无关边线。
            edge_delta_y = (float)right_lower_point.y - right_upper_point.y;
            edge_delta_x = right_slope * edge_delta_y;
            edge_length_squared = edge_delta_x * edge_delta_x + edge_delta_y * edge_delta_y;

            if((edge_length_squared < min_edge_length_squared) ||
               (edge_length_squared > max_edge_length_squared))
            {
                continue;
            }

            // 明显更靠下的候选优先；底部位置接近时，再比较面积和拟合误差。
            bottom_difference = (int16)component_bottom - (int16)best_candidate.bottom;

            if(!best_candidate.valid ||
               (bottom_difference > CAMERA_BRIDGE_BOTTOM_PRIORITY_MARGIN) ||
               (((bottom_difference >= -CAMERA_BRIDGE_BOTTOM_PRIORITY_MARGIN) &&
                 (bottom_difference <=  CAMERA_BRIDGE_BOTTOM_PRIORITY_MARGIN)) &&
                (component_area > best_candidate.area)) ||
               (((bottom_difference >= -CAMERA_BRIDGE_BOTTOM_PRIORITY_MARGIN) &&
                 (bottom_difference <=  CAMERA_BRIDGE_BOTTOM_PRIORITY_MARGIN)) &&
                (component_area == best_candidate.area) &&
                (right_error < best_candidate.edge_error)))
            {
                best_candidate.valid = 1;
                best_candidate.left = component_left;
                best_candidate.right = component_right;
                best_candidate.top = component_top;
                best_candidate.bottom = component_bottom;
                best_candidate.area = component_area;
                best_candidate.edge_y1 = right_upper_point.y;
                best_candidate.edge_y2 = right_lower_point.y;
                best_candidate.edge_point_count = right_point_count;
                best_candidate.right_slope = right_slope;
                best_candidate.right_intercept = right_intercept;
                best_candidate.edge_error = right_error;
            }
        }
    }

    if(!best_candidate.valid)
    {
        return 0;
    }

    reference_row = params->reference_row;
    if(0u == reference_row)
    {
        reference_row = CAMERA_BRIDGE_REFERENCE_ROW_DEFAULT;
    }

    if(reference_row < params->search_top) reference_row = params->search_top;
    if(reference_row > params->search_bottom) reference_row = params->search_bottom;

    result->left = best_candidate.left;
    result->right = best_candidate.right;
    result->top = best_candidate.top;
    result->bottom = best_candidate.bottom;
    result->area = best_candidate.area;
    result->right_edge_x = camera_bridge_round_and_limit_x(
        best_candidate.right_slope * (float)reference_row + best_candidate.right_intercept);
    result->distance_px = (int16)((int32)result->right_edge_x - (int32)params->target_edge_x);
    result->edge_y1 = best_candidate.edge_y1;
    result->edge_x1 = camera_bridge_round_and_limit_x(
        best_candidate.right_slope * (float)best_candidate.edge_y1 + best_candidate.right_intercept);
    result->edge_y2 = best_candidate.edge_y2;
    result->edge_x2 = camera_bridge_round_and_limit_x(
        best_candidate.right_slope * (float)best_candidate.edge_y2 + best_candidate.right_intercept);
    result->reference_row = reference_row;
    result->edge_point_count = best_candidate.edge_point_count;
    result->edge_slope = best_candidate.right_slope;
    result->edge_intercept = best_candidate.right_intercept;

    angle_d10 = atanf(best_candidate.right_slope) * CAMERA_BRIDGE_RAD_TO_D10;
    result->angle_d10 = (angle_d10 >= 0.0f) ?
        (int16)(angle_d10 + 0.5f) : (int16)(angle_d10 - 0.5f);
    result->valid = 1;

    return 1;
}

// 将视觉角度与距离误差合成为航向角修正量
static int16 camproc_bridge_calculate_yaw_offset_d10(const CameraBridgeResult_t *bridge_result, const CameraBridgeControlParams_t *control_params, CameraBridgeControlResult_t *control_result)
{
    float yaw_offset;
    int32 yaw_offset_d10;

    // 以小车正确对准时的视觉输出为零点
    control_result->angle_error_d10 = bridge_result->angle_d10 - control_params->aligned_angle_d10;
    control_result->distance_error_px = bridge_result->distance_px - control_params->aligned_distance_px;

    // 消除对准位置附近的小幅抖动
    if((control_result->angle_error_d10 >= -control_params->angle_deadband_d10) &&
       (control_result->angle_error_d10 <=  control_params->angle_deadband_d10))
    {
        control_result->angle_error_d10 = 0;
    }

    if((control_result->distance_error_px >= -control_params->distance_deadband_px) &&
       (control_result->distance_error_px <=  control_params->distance_deadband_px))
    {
        control_result->distance_error_px = 0;
    }

    yaw_offset = control_params->angle_direction * control_params->angle_gain * (float)control_result->angle_error_d10
               + control_params->distance_direction * control_params->distance_gain * (float)control_result->distance_error_px;

    // 四舍五入并限制最大航向修正量
    yaw_offset_d10 = (yaw_offset >= 0.0f) ? (int32)(yaw_offset + 0.5f) : (int32)(yaw_offset - 0.5f);

    if(yaw_offset_d10 > control_params->yaw_offset_limit_d10)
    {
        yaw_offset_d10 = control_params->yaw_offset_limit_d10;
    }
    else if(yaw_offset_d10 < -control_params->yaw_offset_limit_d10)
    {
        yaw_offset_d10 = -control_params->yaw_offset_limit_d10;
    }

    return (int16)yaw_offset_d10;
}

// 将航向角修正量转换为底盘 angle 控制量
static int16 camproc_bridge_yaw_offset_to_control(int16 yaw_offset_d10, const CameraBridgeControlParams_t *control_params)
{
    float control_output;
    int32 control_value;

    control_output = (float)yaw_offset_d10 * 0.1f
                   * control_params->control_gain_per_deg
                   * control_params->control_direction;

    control_value = (control_output >= 0.0f) ? (int32)(control_output + 0.5f) : (int32)(control_output - 0.5f);

    if(control_value > control_params->control_limit)
    {
        control_value = control_params->control_limit;
    }
    else if(control_value < -control_params->control_limit)
    {
        control_value = -control_params->control_limit;
    }

    return (int16)control_value;
}

uint8 camproc_bridge_calc_ctrl(const CameraBridgeResult_t *bridge_result, const CameraBridgeControlParams_t *control_params, CameraBridgeControlResult_t *control_result)
{
    if(NULL == control_result)
    {
        return 0;
    }

    memset(control_result, 0, sizeof(CameraBridgeControlResult_t));

    if((NULL == bridge_result) || (NULL == control_params) || !bridge_result->valid)
    {
        return 0;
    }

    control_result->yaw_offset_d10 = camproc_bridge_calculate_yaw_offset_d10(bridge_result, control_params, control_result);
    control_result->control_value = camproc_bridge_yaw_offset_to_control(control_result->yaw_offset_d10, control_params);

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
