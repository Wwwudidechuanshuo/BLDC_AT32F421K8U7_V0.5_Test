/*
 * 三相桥臂状态控制，包含六步换相、浮空、低侧导通、PWM 和制动组合。
 * 中文说明按当前实现编写；保留原代码、历史注释和编译条件。
 */
/*
 * phaseouts.h
 *
 *  Created on: Apr 22, 2020
 *      Author: Alka
 */

/* 仅在未定义 INC_PHASEOUTS_H_ 时编译以下代码。 */
#ifndef INC_PHASEOUTS_H_
/* 头文件重复包含保护标记。 */
#define INC_PHASEOUTS_H_

/* 引入 main.h：电调主控制：配置加载、启动、六步换相、过零判断、保护、油门和遥测调度。 */
#include "main.h"

/* 将 A、B、C 三相全部置为浮空，关闭各相高低侧驱动。 */
void allOff(void);
/* 按 1～6 的步骤号执行六步换相，每步包含一相 PWM、一相低侧导通和一相浮空。 */
void comStep (int newStep);
/* 把三相全部切为低侧导通，实现三相短接制动。 */
void fullBrake(void);
/* 把三相切换到 PWM 模式，供正弦启动等三通道调制逻辑使用。 */
void allpwm(void);
/* 关闭高侧驱动并把低侧切到 PWM 复用模式，用占空比控制制动力。 */
void proportionalBrake(void);
/* 设置有刷电机正向桥臂组合：A/C 相 PWM，B 相低侧导通。 */
void twoChannelForward(void);
/* 设置有刷电机反向桥臂组合：A/C 相低侧导通，B 相 PWM。 */
void twoChannelReverse(void);

/* 将 A 相切为 PWM 驱动；是否启用互补低侧由 comp_pwm 决定，另一种桥型使用使能脚。 */
void phaseAPWM(void);
/* 将 B 相切为 PWM 驱动；是否启用互补低侧由 comp_pwm 决定，另一种桥型使用使能脚。 */
void phaseBPWM(void);
/* 将 C 相切为 PWM 驱动；是否启用互补低侧由 comp_pwm 决定，另一种桥型使用使能脚。 */
void phaseCPWM(void);
/* 结束当前条件编译分支。 */
#endif /* INC_PHASEOUTS_H_ */
