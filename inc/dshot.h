/*
 * DShot 油门帧解码、命令执行及双向遥测的校验与 GCR 波形编码。
 * 中文说明按当前实现编写；保留原代码、历史注释和编译条件。
 */
/*
 * dshot.h
 *
 *  Created on: Apr. 22, 2020
 *      Author: Alka
 */

/* 引入 main.h：电调主控制：配置加载、启动、六步换相、过零判断、保护、油门和遥测调度。 */
#include "main.h"

/* 仅在未定义 INC_DSHOT_H_ 时编译以下代码。 */
#ifndef INC_DSHOT_H_
/* 头文件重复包含保护标记。 */
#define INC_DSHOT_H_





/* 从 32 个边沿时间戳还原 16 位 DShot 帧，验证脉宽和校验码，再更新油门或执行命令。 */
void computeDshotDMA(void);
/* 把电周期或扩展遥测值编码成 12 位数据，追加校验、进行 GCR 编码并生成 DMA 输出波形。 */
void make_dshot_package(void);

/* 播放输入识别提示旋律；临时关闭中断以保持鸣音时序。 */
extern void playInputTune(void);
/* 播放第二组输入提示音，依次改变鸣音分频并恢复桥臂关闭状态。 */
extern void playInputTune2(void);
/* 通过扫频鸣音帮助定位电调；循环切换换相步骤并周期性喂狗。 */
extern void playBeaconTune3(void);
/* 把运行中的方向及模式等字段写回配置缓冲区，再保存整个 176 字节配置到 Flash。 */
extern void saveEEpromSettings(void);

/* 引用其他模块定义的双向 DShot 遥测模式标志。 */
extern char dshot_telemetry;
/* 引用其他模块定义的电调已解锁并允许驱动的标志。 */
extern char armed;
/* 引用其他模块定义的保存的电机默认方向反转设置。 */
extern char dir_reversed;
/* 引用其他模块定义的双向油门模式标志。 */
extern char bi_direction;
/* 引用其他模块定义的保留的输入缓冲区解码尺度参数。 */
extern char buffer_divider;
/* 引用其他模块定义的最近一次执行的 DShot 命令编号。 */
extern uint8_t last_dshot_command;
/* 引用其他模块定义的单扇区的估计时间计数，F421 单位 0.5 微秒。 */
extern uint16_t commutation_interval;

//int e_com_time;


/* 结束当前条件编译分支。 */
#endif /* INC_DSHOT_H_ */
