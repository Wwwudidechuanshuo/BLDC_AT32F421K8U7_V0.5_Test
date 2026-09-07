/*
 * 输入信号定时器/DMA 的收发切换，以及输入协议自动识别。
 * 中文说明按当前实现编写；保留原代码、历史注释和编译条件。
 */
/*
 * IO.h
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */

/* 仅在未定义 IO_H_ 时编译以下代码。 */
#ifndef IO_H_
/* 头文件重复包含保护标记。 */
#define IO_H_

/* 结束当前条件编译分支。 */
#endif /* IO_H_ */


/* 引入 main.h：电调主控制：配置加载、启动、六步换相、过零判断、保护、油门和遥测调度。 */
#include "main.h"

/* 把信号定时器切换为遥测 PWM 输出模式，设置输出时基并标记当前处于发送阶段。 */
void changeToOutput(void);
/* 把信号定时器切换为输入捕获模式，恢复接收时基并标记当前处于接收阶段。 */
void changeToInput(void);
/* 重建输入捕获 DMA 传输，将边沿时间戳写入 dma_buffer 并启动下一次接收。 */
void receiveDshotDma(void);
/* 把 gcr 波形数组通过 DMA 写入定时器比较寄存器，从信号线上发出双向 DShot 遥测。 */
void sendDshotDma(void);
/* 统计捕获脉宽的最小值和平均值，识别 DShot 或舵机 PWM，并选择相应时基及 DMA 长度。 */
void detectInput(void);


/* 引用其他模块定义的双向油门模式标志。 */
extern char bi_direction;
/* 引用其他模块定义的已识别输入信号类型的标志。 */
extern char inputSet;
/* 引用其他模块定义的当前输入识别为 DShot 的标志。 */
extern char dshot;
/* 引用其他模块定义的当前输入识别为舵机 PWM 的标志。 */
extern char servoPwm;
/* 引用其他模块定义的待发送串口遥测标志。 */
extern char send_telemetry;
/* 引用其他模块定义的用于保护和遥测的滤波温度，单位摄氏度。 */
extern uint8_t degrees_celsius;

/* 引用其他模块定义的电压通道 ADC 原始值。 */
extern uint16_t ADC_raw_volts;
/* 引用其他模块定义的舵机 PWM 零油门脉宽端点。 */
extern uint16_t servo_low_threshold; // anything below this point considered 0
/* 引用其他模块定义的舵机 PWM 满油门脉宽端点。 */
extern uint16_t servo_high_threshold;  // anything above this point considered 2000 (max)
/* 引用其他模块定义的双向舵机 PWM 中位脉宽。 */
extern uint16_t servo_neutral;
/* 引用其他模块定义的双向油门中位死区参数。 */
extern uint8_t servo_dead_band;


