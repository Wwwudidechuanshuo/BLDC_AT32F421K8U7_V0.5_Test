/*
 * F415 外设中断适配；这是另一套 MCU 实现，当前 F421 目标不编译此源文件。
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
/* 仅在未定义 __AT32F415_IT_H 时编译以下代码。 */
#ifndef __AT32F415_IT_H
/* 头文件重复包含保护标记。 */
#define __AT32F415_IT_H

/* Includes ------------------------------------------------------------------*/
/* 引入 at32f415.h：芯片或运行库接口。 */
#include "at32f415.h"

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
/* F415 比较器中断入口：清除过零 EXINT 标志后进入过零过滤和换相调度。 */
void CMP1_IRQHandler(void);
/* F415 间隔定时器中断声明；当前 F415 中断源文件未提供此入口实现。 */
void TMR4_GLOBAL_IRQHandler(void);
/* F415 延迟换相共用中断入口，清除状态后执行 PeriodElapsedCallback。 */
void TMR1_TRG_HALL_TMR11_IRQHandler(void);
/* F415 周期控制任务共用中断入口，清除状态后执行 tenKhzRoutine。 */
void TMR1_BRK_TMR9_IRQHandler(void);
/* 清除 F415 辅助计时器 TMR10 的溢出和通道 1 标志。 */
void TMR1_OVF_TMR10_IRQHandler(void);
/* 检查并清除 TMR3 通道 1 捕获和溢出标志；协议数据主要由 DMA 路径处理。 */
void TMR3_GLOBAL_IRQHandler(void);
/* 处理 F415 串口发送 DMA 通道 4 的状态；代码中错误分支检测的标志编号仍按现有实现保留。 */
void DMA1_Channel4_IRQHandler(void);
/* 处理 F415 输入 DMA 的半传输、整帧完成和错误事件，并调用协议处理回调。 */
void DMA1_Channel6_IRQHandler(void);
/* 结束当前条件编译分支。 */
#endif /* __AT32F4XX_IT_H */
