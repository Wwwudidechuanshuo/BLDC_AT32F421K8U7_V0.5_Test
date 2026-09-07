/*
 * F421 外设中断适配：ADC/输入/串口 DMA、过零和控制定时器回调；ARM 内核异常保持原样。
 * 中文说明按当前实现编写；保留原代码、历史注释和编译条件。
 */
/* 处理信号 DMA 一帧完成事件：自动识别协议、解码油门、切换双向遥测收发并检查解锁条件。 */
extern void transfercomplete(void);
/* 换相延时到期回调：关闭本次定时中断，执行换相，并按模式重新允许过零中断。 */
extern void PeriodElapsedCallback(void);
/* 处理过零中断：拒绝过早事件并过滤错误电平，计算提前角和延时后安排下一次换相。 */
extern void interruptRoutine(void);
/* 遗留 PWM 更新回调声明；当前工程中未找到该名称的实现。 */
extern void doPWMChanges(void);
/* 周期性控制任务：解锁、油门到占空比映射、电流/低速控制、斜率限制、遥测及失联处理。 */
extern void tenKhzRoutine(void);
/* 把 gcr 波形数组通过 DMA 写入定时器比较寄存器，从信号线上发出双向 DShot 遥测。 */
extern void sendDshotDma(void);
/* 重建输入捕获 DMA 传输，将边沿时间戳写入 dma_buffer 并启动下一次接收。 */
extern void receiveDshotDma(void);
/* 遗留输入边沿回调声明；当前工程中未找到该名称的实现。 */
extern void signalEdgeRoutine(void);


/* 引用其他模块定义的待发送串口遥测标志。 */
extern char send_telemetry;
/* 引用其他模块定义的保留的遥测完成状态。 */
extern char telemetry_done;
/* 引用其他模块定义的当前输入识别为舵机 PWM 的标志。 */
extern char servoPwm;


/* Includes ------------------------------------------------------------------*/
/* 引入 at32f421_it.h：F421 外设中断适配：ADC/输入/串口 DMA、过零和控制定时器回调；ARM 内核异常保持原样。 */
#include "at32f421_it.h"
/* 引入 main.h：电调主控制：配置加载、启动、六步换相、过零判断、保护、油门和遥测调度。 */
#include "main.h"
/* 引入 targets.h：硬件目标配置：选择 MCU、输入捕获资源、三相驱动引脚、比较器通道和默认参数。 */
#include "targets.h"
/* 引入 adc.h：芯片或运行库接口。 */
#include "adc.h"
/** @addtogroup AT32F421_StdPeriph_Templates
  * @{
  */

/** @addtogroup GPIO_LED_Toggle
  * @{
  */

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
    /* Go to infinite loop when Hard Fault exception occurs */
    while (1)
    {
    }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
    /* Go to infinite loop when Memory Manage exception occurs */
    while (1)
    {
    }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
    /* Go to infinite loop when Bus Fault exception occurs */
    while (1)
    {
    }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
    /* Go to infinite loop when Usage Fault exception occurs */
    while (1)
    {
    }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}

/**
 * @brief 处理 ADC DMA 完成事件，把新一组采样值交给 ADC_DMA_Callback，并清除相关标志。
 */
void DMA1_Channel1_IRQHandler(void){
    /* 检查指定 DMA 事件标志，区分半传输、整帧完成或传输错误。 */
    if(dma_flag_get(DMA1_FDT1_FLAG) == SET)		
		{
        /* 清除对应 DMA 通道的已处理状态标志。 */
	    DMA1->clr = DMA1_GL1_FLAG;
        /* 按 ADC 扫描顺序取出 DMA 数据，更新温度、电压、电流及可选模拟油门的原始值。 */
	    ADC_DMA_Callback();
           /* 检查指定 DMA 事件标志，区分半传输、整帧完成或传输错误。 */
		   if(dma_flag_get(DMA1_DTERR1_FLAG) == SET)
		  {
                /* 清除对应 DMA 通道的已处理状态标志。 */
				DMA1->clr = DMA1_GL1_FLAG;
			}
}
}

/**
 * @brief 处理 F421 串口遥测 DMA 通道 2 的完成或错误事件，清标志并关闭通道。
 */
void DMA1_Channel3_2_IRQHandler(void)
{
      /* 检查指定 DMA 事件标志，区分半传输、整帧完成或传输错误。 */
	  if(dma_flag_get(DMA1_FDT2_FLAG) == SET)
	  {
        /* 清除对应 DMA 通道的已处理状态标志。 */
	    DMA1->clr = DMA1_GL2_FLAG;
            /* 将DMA1_CHANNEL2 的DMA 通道使能清零。 */
			DMA1_CHANNEL2->ctrl_bit.chen = FALSE;   
	  }
      /* 检查指定 DMA 事件标志，区分半传输、整帧完成或传输错误。 */
	  if(dma_flag_get(DMA1_DTERR2_FLAG) == SET)
	  {
          /* 清除对应 DMA 通道的已处理状态标志。 */
		  DMA1->clr = DMA1_GL2_FLAG;
          /* 将DMA1_CHANNEL2 的DMA 通道使能清零。 */
		  DMA1_CHANNEL2->ctrl_bit.chen = FALSE;
	  }
}

/**
 * @brief 处理 F421 输入 DMA：舵机半传输时切捕获沿，整帧完成后交给 transfercomplete。
 */
void DMA1_Channel5_4_IRQHandler(void)
{
/* 仅在定义 USE_TIMER_15_CHANNEL_1 时编译以下代码。 */
#ifdef USE_TIMER_15_CHANNEL_1
    /* 检查指定 DMA 事件标志，区分半传输、整帧完成或传输错误。 */
	if(dma_flag_get(DMA1_HDT5_FLAG) == SET){
            /* 当当前输入识别为舵机 PWM 的标志非零时进入此分支。 */
			if(servoPwm){
            /* 更新IC_TIMER_REGISTER 的通道 1 主输出/输入捕获极性。 */
			IC_TIMER_REGISTER->cctrl_bit.c1p = TMR_INPUT_FALLING_EDGE;	
            /* 清除对应 DMA 通道的已处理状态标志。 */
			DMA1->clr = DMA1_HDT5_FLAG;
			}
		}

    /* 检查指定 DMA 事件标志，区分半传输、整帧完成或传输错误。 */
    if(dma_flag_get(DMA1_FDT5_FLAG) == SET)		
		{
      /* 清除对应 DMA 通道的已处理状态标志。 */
      DMA1->clr = DMA1_GL5_FLAG;
            /* 将INPUT_DMA_CHANNEL 的DMA 通道使能清零。 */
			INPUT_DMA_CHANNEL->ctrl_bit.chen = FALSE;
          /* 处理信号 DMA 一帧完成事件：自动识别协议、解码油门、切换双向遥测收发并检查解锁条件。 */
		  transfercomplete();

		  }
           /* 检查指定 DMA 事件标志，区分半传输、整帧完成或传输错误。 */
		   if(dma_flag_get(DMA1_DTERR5_FLAG) == SET)
		  {
                /* 清除对应 DMA 通道的已处理状态标志。 */
				DMA1->clr = DMA1_GL5_FLAG;
			}
/* 结束当前条件编译分支。 */
#endif
/* 仅在定义 USE_TIMER_3_CHANNEL_1 时编译以下代码。 */
#ifdef USE_TIMER_3_CHANNEL_1
        /* 检查指定 DMA 事件标志，区分半传输、整帧完成或传输错误。 */
		if(dma_flag_get(DMA1_HDT4_FLAG) == SET){ 
            /* 当当前输入识别为舵机 PWM 的标志非零时进入此分支。 */
			if(servoPwm){
            /* 更新IC_TIMER_REGISTER 的通道 1 主输出/输入捕获极性。 */
			IC_TIMER_REGISTER->cctrl_bit.c1p = TMR_INPUT_FALLING_EDGE;
            /* 清除对应 DMA 通道的已处理状态标志。 */
			DMA1->clr = DMA1_HDT4_FLAG;
			}
		}

    /* 检查指定 DMA 事件标志，区分半传输、整帧完成或传输错误。 */
    if(dma_flag_get(DMA1_FDT4_FLAG) == SET)		
		{
      /* 清除对应 DMA 通道的已处理状态标志。 */
      DMA1->clr = DMA1_GL4_FLAG;
            /* 将INPUT_DMA_CHANNEL 的DMA 通道使能清零。 */
			INPUT_DMA_CHANNEL->ctrl_bit.chen = FALSE;
          /* 处理信号 DMA 一帧完成事件：自动识别协议、解码油门、切换双向遥测收发并检查解锁条件。 */
		  transfercomplete();

		  }
           /* 检查指定 DMA 事件标志，区分半传输、整帧完成或传输错误。 */
		   if(dma_flag_get(DMA1_DTERR4_FLAG) == SET)
		  {
                /* 清除对应 DMA 通道的已处理状态标志。 */
				DMA1->clr = DMA1_GL4_FLAG;
			}
/* 结束当前条件编译分支。 */
#endif
}

/**
  * @brief This function handles ADC and COMP interrupts (COMP interrupts through EXTI lines 21 and 22).
  */
/**
 * @brief F421 ADC/比较器共用中断入口；本实现只处理 EXINT 过零标志并调用 interruptRoutine。
 */
void ADC1_CMP_IRQHandler(void)
{
      /* 确认挂起的是本目标的比较器过零 EXINT 事件。 */
	  if((EXINT->intsts & EXTI_LINE) != (uint32_t)RESET)
	  {
            /* 向目标 EXINT 挂起位写 1 清除，避免重复进入同一次过零中断。 */
			EXINT->intsts = EXTI_LINE;
        /* 处理过零中断：拒绝过早事件并过滤错误电平，计算提前角和延时后安排下一次换相。 */
	    interruptRoutine();
	  }
}
/* 按过零超时情况限制占空比，再把调整后的值写入三个 PWM 通道；可选换相瞬间降占空比。 */
void setPwmRatio(void);

/**
 * @brief F421 TMR1 共用中断入口；本实现处理溢出事件，调用 setPwmRatio 更新 PWM。
 */
void TMR1_BRK_OVF_TRG_HALL_IRQHandler(void)
{
  /* 确认对应定时器事件已经发生，再处理并清除标志。 */
  if(tmr_flag_get(TMR1, TMR_OVF_FLAG) != RESET)
  {
        /* 按过零超时情况限制占空比，再把调整后的值写入三个 PWM 通道；可选换相瞬间降占空比。 */
		setPwmRatio();
    /* 清除已处理的定时器事件标志。 */
    tmr_flag_clear(TMR1, TMR_OVF_FLAG);
  }
}

/* 间隔定时器溢出的兜底处理，避免主循环阻塞后遗漏过零超时；仅在 running 时执行。 */
void IntervalTimerOverflow(void);
/**
 * @brief 清除 F421 间隔定时器溢出标志并调用过零超时兜底处理。
 */
void TMR6_GLOBAL_IRQHandler(void)
{
        /* 确认对应定时器事件已经发生，再处理并清除标志。 */
		if((TMR6->ists & TMR_OVF_FLAG) != (uint16_t)RESET)
	  {
            /* 清除已处理的定时器事件标志；具体清除位由当前写入值决定。 */
			TMR6->ists = (uint16_t)~TMR_OVF_FLAG;
	  }
        /* 间隔定时器溢出的兜底处理，避免主循环阻塞后遗漏过零超时；仅在 running 时执行。 */
		IntervalTimerOverflow();
}

/**
  * @brief This function handles TIM6 global and DAC underrun error interrupts.
  */
/**
 * @brief 清除 F421 周期任务定时器标志并执行 tenKhzRoutine。
 */
void TMR14_GLOBAL_IRQHandler(void)
{
      /* 清除已处理的定时器事件标志；具体清除位由当前写入值决定。 */
      TMR14->ists = (uint16_t)~TMR_OVF_FLAG;
        /* 周期性控制任务：解锁、油门到占空比映射、电流/低速控制、斜率限制、遥测及失联处理。 */
	    tenKhzRoutine();
}

/**
  * @brief This function handles TIM14 global interrupt.
  */
/**
 * @brief 清除 F421 延迟换相定时器状态并执行 PeriodElapsedCallback。
 */
void TMR16_GLOBAL_IRQHandler(void)
{
        /* 清除已处理的定时器事件标志；具体清除位由当前写入值决定。 */
		TMR16->ists = 0x00;
        /* 换相延时到期回调：关闭本次定时中断，执行换相，并按模式重新允许过零中断。 */
		PeriodElapsedCallback();
}

/**
 * @brief 清除 TMR15 溢出和通道 1 标志；当前未执行额外业务回调。
 */
void TMR15_GLOBAL_IRQHandler(void)
{
    /* 清除已处理的定时器事件标志；具体清除位由当前写入值决定。 */
	TMR15->ists = (uint16_t)~TMR_OVF_FLAG;
    /* 清除已处理的定时器事件标志；具体清除位由当前写入值决定。 */
	TMR15->ists = (uint16_t)~TMR_C1_FLAG;

}



/**
  * @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
  */
/**
 * @brief 保留的串口中断入口；当前函数没有业务处理，遥测发送使用 DMA。
 */
void USART1_IRQHandler(void)
{

}

/**
 * @brief 检查并清除 TMR3 通道 1 捕获和溢出标志；协议数据主要由 DMA 路径处理。
 */
void TMR3_GLOBAL_IRQHandler(void)
{
            /* 确认对应定时器事件已经发生，再处理并清除标志。 */
			if((TMR3->ists & TMR_C1_FLAG) != (uint16_t)RESET)
	  {
            /* 清除已处理的定时器事件标志；具体清除位由当前写入值决定。 */
			TMR3->ists = (uint16_t)~TMR_C1_FLAG;
	  }
        /* 确认对应定时器事件已经发生，再处理并清除标志。 */
		if((TMR3->ists & TMR_OVF_FLAG) != (uint16_t)RESET)
	  {
            /* 清除已处理的定时器事件标志；具体清除位由当前写入值决定。 */
			TMR3->ists = (uint16_t)~TMR_OVF_FLAG;
	  }  
}



/**
 * @brief 保留的外部中断入口；当前没有启用有效的输入边沿处理代码。
 */
void EXTI4_15_IRQHandler(void){

}


/******************************************************************************/
/*                 AT32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_at32f413_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */

/**
  * @}
  */
