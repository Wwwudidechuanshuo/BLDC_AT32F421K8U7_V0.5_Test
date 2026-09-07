/*
 * 串口遥测组帧、CRC8 校验和 USART1 DMA 发送配置。
 * 中文说明按当前实现编写；保留原代码、历史注释和编译条件。
 */
/*
 * serial_telemetry.h
 *
 *  Created on: May 13, 2020
 *      Author: Alka
 */


/* 引入 main.h：电调主控制：配置加载、启动、六步换相、过零判断、保护、油门和遥测调度。 */
#include "main.h"

/* 仅在未定义 SERIAL_TELEMETRY_H_ 时编译以下代码。 */
#ifndef SERIAL_TELEMETRY_H_
/* 头文件重复包含保护标记。 */
#define SERIAL_TELEMETRY_H_

/* 按温度、电压、电流、耗电量和电转速顺序组装 10 字节遥测帧，多字节字段高字节先发。 参数依次为摄氏度、0.01 V、0.01 A、mAh、电转速/100。 */
void makeTelemPackage(uint8_t temp,
                      /* 参数 voltage：电压字段，当前主程序以 0.01 V 为单位传入。 */
		              uint16_t voltage,
                      /* 参数 current：电流字段，当前主程序以约 0.01 A 为单位传入。 */
					  uint16_t current,
                      /* 参数 consumption：累计耗电量字段，单位 mAh。 */
					  uint16_t consumption,
                      /* 参数 e_rpm：电转速字段，当前主程序按实际电转速的 1/100 传入。 */
					  uint16_t e_rpm);


/* 初始化 USART1 遥测发送和 DMA；发送引脚为 PB6，具体 DMA 通道随 MCU 适配代码变化。 */
void telem_UART_Init(void);
/* 装载遥测帧长度并启动串口发送 DMA，完成后的通道清理由中断处理。 */
void send_telem_DMA(void);

/* 保留的通道 4 遥测初始化声明；当前工程未提供对应名称的实现。 */
void telem_UART_Init_CH4(void);
/* 保留的通道 4 遥测发送声明；当前工程未提供对应名称的实现。 */
void send_telem_DMA_CH4(void);

/* 结束当前条件编译分支。 */
#endif /* SERIAL_TELEMETRY_H_ */
