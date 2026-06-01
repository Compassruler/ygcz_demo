#ifndef _BUTTON_H_
#define _BUTTON_H_

#include "zf_common_typedef.h"
#include "zf_driver_gpio.h"

/**
 * 使用例程 在 main 文件中
 * 
 * button_init();
 * 
 * 
 * while(true)
 * {
 *     button_update();  // 按钮更新
 * 
 *     if (button_flag[BTN_1])  // 当检测到 按钮 1 (P20.0) 被按下后
 *     {
 *          // 需要执行的操作 
 *     }
 * 
 *     if (button_flag[BTN_2])  // 当检测到 按钮 2 (P20.1) 被按下后
 *     {
 *          // 需要执行的操作 
 *     }
 * 
 * }
 * 
 */

// 按键引脚定义
// 参考 E1_01_button_switch_demo 例程：按键使用上拉输入，按下时为低电平。
#define BUTTON_KEY1_PIN             (P20_0)
#define BUTTON_KEY2_PIN             (P20_1)
#define BUTTON_KEY3_PIN             (P20_2)
#define BUTTON_KEY4_PIN             (P20_3)

// 按键编号，用于访问 button_flag[] 数组
typedef enum
{
    BTN_1 = 0,
    BTN_2,
    BTN_3,
    BTN_4,
    BUTTON_NUMBER,
}button_index_enum;

// 按键单次触发标志位
// 每次调用 button_update() 会先清零。
// 当检测到按键由松开变为按下时，对应标志位置 1。
// 一直按住不会重复置 1，松开后再次按下才会再次触发。
extern uint8 button_flag[BUTTON_NUMBER];

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     button_init
// 参数说明     void
// 返回值       void
// 使用说明     button_init();
// 说明         初始化 KEY1 ~ KEY4 为上拉输入，按下时读取低电平。
//-------------------------------------------------------------------------------------------------------------------
void button_init(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数名称     button_update
// 参数说明     void
// 返回值       void
// 使用说明     button_update();
// 说明         更新按键标志位。每次调用先清零，再检测新的按下动作。
//-------------------------------------------------------------------------------------------------------------------
void button_update(void);

#endif
