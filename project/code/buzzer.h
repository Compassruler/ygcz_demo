#ifndef __BUZZER_H__
#define __BUZZER_H__

#include "zf_common_headfile.h"

/* 蜂鸣器初始化 */
void buzzer_init(void);

/* 蜂鸣器打开 */
void buzzer_on(void);

/* 蜂鸣器关闭 */
void buzzer_off(void);

/* 蜂鸣器翻转 */
void buzzer_toggle(void);

/* 蜂鸣器调试响声
   times: 响次数
   interval_ms: 间隔时间 */
void buzzer_beep(uint16 times, uint16 interval_ms);

#endif