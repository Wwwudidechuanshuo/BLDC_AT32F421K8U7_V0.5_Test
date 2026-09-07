/*
 * ADC 采样及 DMA 搬运接口，包含正常采样与 BEMF 诊断采样两套配置。
 * 中文说明按当前实现编写；保留原代码、历史注释和编译条件。
 */
/*
 * ADC.h
 *
 *  Created on: May 20, 2020
 *      Author: Alka
 */

/* 引入 main.h：电调主控制：配置加载、启动、六步换相、过零判断、保护、油门和遥测调度。 */
#include "main.h"
/* 引入 targets.h：硬件目标配置：选择 MCU、输入捕获资源、三相驱动引脚、比较器通道和默认参数。 */
#include "targets.h"


/* 仅在未定义 ADC_H_ 时编译以下代码。 */
#ifndef ADC_H_
/* 头文件重复包含保护标记。 */
#define ADC_H_



/* 按 ADC 扫描顺序取出 DMA 数据，更新温度、电压、电流及可选模拟油门的原始值。 */
void ADC_DMA_Callback(void);
/* 预留的 ADC DMA 启用接口；当前工程未提供此函数的实现。 */
void enableADC_DMA(void);
/* 预留的 ADC 激活接口；当前工程未提供此函数的实现。 */
void activateADC(void);
/* 配置普通 ADC 扫描和 DMA 循环搬运；当前序列依次为 PA3、PA6、内部温度通道。 */
void ADC_Init(void);
/* 切换为四路 BEMF 诊断采样，DMA 写入 adc_diagnose_data；会重新配置 ADC 和 DMA1 通道 1。 */
void ADC_Init_Detector(void);
/* 引用其他模块定义的BEMF 诊断 ADC 的四路采样缓冲区。 */
extern uint16_t adc_diagnose_data[4];


/* 结束当前条件编译分支。 */
#endif /* ADC_H_ */
