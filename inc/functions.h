/*
 * GPIO 模式切换、限幅线性映射、绝对差和阻塞延时等工程辅助函数。
 * 中文说明按当前实现编写；保留原代码、历史注释和编译条件。
 */
/*
 * functions.h
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */

/* 仅在未定义 FUNCTIONS_H_ 时编译以下代码。 */
#ifndef FUNCTIONS_H_
/* 头文件重复包含保护标记。 */
#define FUNCTIONS_H_



/* 结束当前条件编译分支。 */
#endif /* FUNCTIONS_H_ */

/* 引入 main.h：电调主控制：配置加载、启动、六步换相、过零判断、保护、油门和遥测调度。 */
#include "main.h"
/* 快速切换桥臂或信号引脚模式；F421 实现只更新模式寄存器，不处理 pull_up_down 参数。 */
void gpio_mode_QUICK(gpio_type* gpio_periph, uint32_t mode, uint32_t pull_up_down, uint32_t pin);
/* 设置输入信号引脚的模式和上下拉；F421 直接操作寄存器，F415 使用 GPIO 初始化接口。 */
void gpio_mode_set(uint32_t mode, uint32_t pull_up_down, uint32_t pin);
/* 计算两个整数之差的绝对值，返回比较差值；用于输入稳定性和失步检测。 */
int getAbsDif(int number1, int number2);
/* 使用 UTILITY_TIMER 忙等延时；会清零共享计数器，时间基准取决于当前定时器分频。 */
void delayMicros(uint32_t micros);
/* 临时修改 UTILITY_TIMER 分频进行毫秒级忙等，等待期间喂狗，结束后恢复代码指定的分频。 */
void delayMillis(uint32_t millis);
/* 先将输入限制在给定范围，再作整数线性映射；返回映射结果，要求 in_max 与 in_min 不相等。 */
long map(long x, long in_min, long in_max, long out_min, long out_max);
