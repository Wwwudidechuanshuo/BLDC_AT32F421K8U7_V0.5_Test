/*
 * 舵机 PWM 解析、油门端点校准和输入 DMA 完成后的协议分发。
 * 中文说明按当前实现编写；保留原代码、历史注释和编译条件。
 */
/*
 * IO.h
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */

/* 引入 main.h：电调主控制：配置加载、启动、六步换相、过零判断、保护、油门和遥测调度。 */
#include "main.h"

/* 引用其他模块定义的信号线当前方向，1 为发送遥测，0 为接收输入。 */
extern char out_put;
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
/* 引用其他模块定义的保留的爬车模式参数，部分超时和占空比逻辑仍引用它。 */
extern char crawler_mode;

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


