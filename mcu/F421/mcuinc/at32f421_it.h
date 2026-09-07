/*
 * F421 外设中断适配：ADC/输入/串口 DMA、过零和控制定时器回调；ARM 内核异常保持原样。
 * 中文说明按当前实现编写；保留原代码、历史注释和编译条件。
 */
/**
  ******************************************************************************
  * File   : Templates/at32f4xx_it.h 
  * Version: V1.3.0
  * Date   : 2021-03-18
  * Brief  : Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
/* 仅在未定义 __AT32F421_IT_H 时编译以下代码。 */
#ifndef __AT32F421_IT_H
/* 头文件重复包含保护标记。 */
#define __AT32F421_IT_H

/* Includes ------------------------------------------------------------------*/
/* 引入 at32f421.h：芯片或运行库接口。 */
#include "at32f421.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
/* F421 ADC/比较器共用中断入口；本实现只处理 EXINT 过零标志并调用 interruptRoutine。 */
void ADC1_CMP_IRQHandler(void);
/* 清除 F421 周期任务定时器标志并执行 tenKhzRoutine。 */
void TMR14_GLOBAL_IRQHandler(void);
/* 清除 TMR15 溢出和通道 1 标志；当前未执行额外业务回调。 */
void TMR15_GLOBAL_IRQHandler(void);
/* 清除 F421 延迟换相定时器状态并执行 PeriodElapsedCallback。 */
void TMR16_GLOBAL_IRQHandler(void);
/* 辅助定时器中断声明；当前 F421 中断源文件未提供此入口实现。 */
void TMR17_GLOBAL_IRQHandler(void);
/* 检查并清除 TMR3 通道 1 捕获和溢出标志；协议数据主要由 DMA 路径处理。 */
void TMR3_GLOBAL_IRQHandler(void);
/* 处理 F421 输入 DMA：舵机半传输时切捕获沿，整帧完成后交给 transfercomplete。 */
void DMA1_Channel5_4_IRQHandler(void);
/* 处理 ADC DMA 完成事件，把新一组采样值交给 ADC_DMA_Callback，并清除相关标志。 */
void DMA1_Channel1_IRQHandler(void);
/* 处理 F421 串口遥测 DMA 通道 2 的完成或错误事件，清标志并关闭通道。 */
void DMA1_Channel3_2_IRQHandler(void);
/* 结束当前条件编译分支。 */
#endif /* __AT32F4XX_IT_H */
