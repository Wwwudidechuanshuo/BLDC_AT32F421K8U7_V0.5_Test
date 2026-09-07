/*
 * 内置比较器的浮空相选择、过零 EXINT 边沿配置及中断屏蔽控制。
 * 中文说明按当前实现编写；保留原代码、历史注释和编译条件。
 */
/*
 * comparator.h
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */

/* 仅在未定义 COMPARATOR_H_ 时编译以下代码。 */
#ifndef COMPARATOR_H_
/* 头文件重复包含保护标记。 */
#define COMPARATOR_H_
/* 结束当前条件编译分支。 */
#endif /* COMPARATOR_H_ */

/* 保留的 PA0 比较器编码；运行路径使用 targets.h 中的 PHASE_*_COMP。 */
#define COMP_PA0 0b1100001
/* 保留的 PA4 比较器编码；运行路径使用 targets.h 中的 PHASE_*_COMP。 */
#define COMP_PA4 0b1000001
/* 保留的 PA5 比较器编码；运行路径使用 targets.h 中的 PHASE_*_COMP。 */
#define COMP_PA5 0b1010001

/* 引入 main.h：电调主控制：配置加载、启动、六步换相、过零判断、保护、油门和遥测调度。 */
#include "main.h"

/* 屏蔽过零 EXINT 中断并清除挂起标志；比较器本身继续工作，仍可轮询输出。 */
void maskPhaseInterrupts(void);
/* 按当前换相步骤选择浮空相，并根据相电压预期变化方向设置比较器中断边沿。 */
void changeCompInput(void);
/* 允许过零 EXINT 中断；本函数只置位中断使能，不清除之前的挂起标志。 */
void enableCompInterrupts(void);

/* 引用其他模块定义的预期浮空相电压上升标志；比较器输出方向与其相反。 */
extern char rising;
/* 引用其他模块定义的当前六步换相步骤，正常范围为 1～6。 */
extern char step;
