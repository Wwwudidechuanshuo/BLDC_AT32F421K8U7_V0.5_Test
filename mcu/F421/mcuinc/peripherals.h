/*
 * 本 MCU 的时钟和外设适配初始化；属于工程代码，底层标准库保持原样。
 * 中文说明按当前实现编写；保留原代码、历史注释和编译条件。
 */
/*
 * peripherals.h
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */

/* 仅在未定义 PERIPHERALS_H_ 时编译以下代码。 */
#ifndef PERIPHERALS_H_
/* 头文件重复包含保护标记。 */
#define PERIPHERALS_H_



/* 结束当前条件编译分支。 */
#endif /* PERIPHERALS_H_ */

/* 引入 main.h：电调主控制：配置加载、启动、六步换相、过零判断、保护、油门和遥测调度。 */
#include "main.h"


/* 预留的跳转后初始化钩子；当前实现为空。 */
void initAfterJump(void);
/* 按依赖顺序初始化系统时钟、GPIO、DMA、PWM、各用途定时器、比较器和可选串口遥测。 */
void initCorePeripherals(void);
//void SystemClock_Config(void);
/* 预留通用 GPIO 初始化入口；当前实际引脚配置分散在具体外设初始化函数中。 */
void MX_GPIO_Init(void);
/* 开启 DMA 控制器时钟，并设置输入捕获 DMA 中断优先级。 */
void MX_DMA_Init(void);
//static void MX_ADC_Init(void);
/* 配置本 MCU 的比较器输入和中断入口并启用比较器；浮空相选择由 changeCompInput 完成。 */
void AT_COMP_Init(void);
/* 配置三相 PWM 的周期、输出模式、预装载、死区和引脚复用；极性按目标宏选择。 */
void TIM1_Init(void);
/* 配置 F421 的换相间隔计时器 TMR6；120 MHz 下分频 60，计数单位为 0.5 微秒。 */
void TIM6_Init(void);
/* 选择内部高速时钟作为 PLL 来源，设置总线分频并等待时钟稳定，最后更新系统时钟变量。 */
void system_clock_config(void);
/* 配置并启动独立看门狗，之后运行流程需定期写重载命令。 */
void MX_IWDG_Init(void);
/* 配置 F421 通用辅助计时器 TMR17，初始化为微秒级计数，供延时和耗时测量使用。 */
void TIM17_Init(void);
/* 配置 F421 周期控制任务的 TMR14 时基和中断，供 tenKhzRoutine 调度使用。 */
void TIM14_Init(void);
/* 配置 F421 延迟换相定时器 TMR16；120 MHz 下计数单位为 0.5 微秒。 */
void TIM16_Init(void);
//static void MX_USART1_UART_Init(void);

/* 配置输入信号定时器及其 DMA 请求，具体定时器和引脚由 targets.h 选择。 */
void UN_TIM_Init(void);

/* 配置可选 RGB 状态灯引脚；仅在 USE_RGB_LED 编译分支中使用。 */
void LED_GPIO_init(void);

