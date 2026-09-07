/*
 * F415 外设中断适配；这是另一套 MCU 实现，当前 F421 目标不编译此源文件。
 * 中文说明按当前实现编写；保留原代码、历史注释和编译条件。
 */
/* 处理信号 DMA 一帧完成事件：自动识别协议、解码油门、切换双向遥测收发并检查解锁条件。 */
extern void transfercomplete();
/* 换相延时到期回调：关闭本次定时中断，执行换相，并按模式重新允许过零中断。 */
extern void PeriodElapsedCallback();
/* 处理过零中断：拒绝过早事件并过滤错误电平，计算提前角和延时后安排下一次换相。 */
extern void interruptRoutine();
/* 遗留 PWM 更新回调声明；当前工程中未找到该名称的实现。 */
extern void doPWMChanges();
/* 周期性控制任务：解锁、油门到占空比映射、电流/低速控制、斜率限制、遥测及失联处理。 */
extern void tenKhzRoutine();
/* 把 gcr 波形数组通过 DMA 写入定时器比较寄存器，从信号线上发出双向 DShot 遥测。 */
extern void sendDshotDma();
/* 重建输入捕获 DMA 传输，将边沿时间戳写入 dma_buffer 并启动下一次接收。 */
extern void receiveDshotDma();
/* 遗留输入边沿回调声明；当前工程中未找到该名称的实现。 */
extern void signalEdgeRoutine();

/* 引用其他模块定义的待发送串口遥测标志。 */
extern char send_telemetry;
/* 引用其他模块定义的保留的遥测完成状态。 */
extern char telemetry_done;
/* 引用其他模块定义的当前输入识别为舵机 PWM 的标志。 */
extern char servoPwm;

/* 保存预留的接收中断计数。 */
int recieved_ints = 0;



/* Includes ------------------------------------------------------------------*/
/* 引入 at32f415_it.h：F415 外设中断适配；这是另一套 MCU 实现，当前 F421 目标不编译此源文件。 */
#include "at32f415_it.h"
/* 引入 main.h：电调主控制：配置加载、启动、六步换相、过零判断、保护、油门和遥测调度。 */
#include "main.h"
/* 引入 targets.h：硬件目标配置：选择 MCU、输入捕获资源、三相驱动引脚、比较器通道和默认参数。 */
#include "targets.h"
/* 引入 ADC.h：ADC 采样及 DMA 搬运接口，包含正常采样与 BEMF 诊断采样两套配置。 */
#include "ADC.h"
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
 * @brief 处理 F415 串口发送 DMA 通道 4 的状态；代码中错误分支检测的标志编号仍按现有实现保留。
 */
void DMA1_Channel4_IRQHandler(void)
{
      /* 检查指定 DMA 事件标志，区分半传输、整帧完成或传输错误。 */
	  if(dma_flag_get(DMA1_FDT4_FLAG) == SET)
	  {
        /* 清除对应 DMA 通道的已处理状态标志。 */
	    DMA1->clr = DMA1_GL4_FLAG;
            /* 将DMA1_CHANNEL4 的DMA 通道使能清零。 */
			DMA1_CHANNEL4->ctrl_bit.chen = FALSE;   
		//	USART1->ctrl1_bit.ren = TRUE;
		//	USART1->ctrl1_bit.ten = FALSE;
	  }
      /* 检查指定 DMA 事件标志，区分半传输、整帧完成或传输错误。 */
	  if(dma_flag_get(DMA1_DTERR2_FLAG) == SET)
	  {
          /* 清除对应 DMA 通道的已处理状态标志。 */
		  DMA1->clr = DMA1_GL4_FLAG;
          /* 将DMA1_CHANNEL4 的DMA 通道使能清零。 */
		  DMA1_CHANNEL4->ctrl_bit.chen = FALSE;
	  }
}



/**
 * @brief 处理 F415 输入 DMA 的半传输、整帧完成和错误事件，并调用协议处理回调。
 */
void DMA1_Channel6_IRQHandler(void)
{
	
	
	
/* 仅在定义 USE_TIMER_15_CHANNEL_1 时编译以下代码。 */
#ifdef USE_TIMER_15_CHANNEL_1
    /* 检查指定 DMA 事件标志，区分半传输、整帧完成或传输错误。 */
	if(DMA_GetFlagStatus(DMA1_FLAG_HT5) == SET){
            /* 当当前输入识别为舵机 PWM 的标志非零时进入此分支。 */
			if(servoPwm){
            /* 置位IC_TIMER_REGISTER 的兼容定时器控制寄存器。 */
			IC_TIMER_REGISTER->CTRL2 |= TMR_ICPolarity_Rising;	
            /* 清除对应 DMA 通道的已处理状态标志。 */
			DMA1->ICLR = DMA1_FLAG_HT5;
			}
		}

    /* 检查指定 DMA 事件标志，区分半传输、整帧完成或传输错误。 */
    if(DMA_GetFlagStatus(DMA1_FLAG_TC5) == SET)		
		{
      /* 清除对应 DMA 通道的已处理状态标志。 */
      DMA1->ICLR = DMA1_FLAG_GL5;
        /* 按掩码更新INPUT_DMA_CHANNEL 的兼容 DMA 通道控制寄存器。 */
   		INPUT_DMA_CHANNEL->CHCTRL &= (uint16_t)(~DMA_CHCTRL1_CHEN);
          /* 处理信号 DMA 一帧完成事件：自动识别协议、解码油门、切换双向遥测收发并检查解锁条件。 */
		  transfercomplete();

		  }
          /* 检查指定 DMA 事件标志，区分半传输、整帧完成或传输错误。 */
		  else if(DMA_GetFlagStatus(DMA1_FLAG_ERR5) == SET)
		  {
                /* 清除对应 DMA 通道的已处理状态标志。 */
				DMA1->ICLR = DMA1_FLAG_GL5;
			}
/* 结束当前条件编译分支。 */
#endif
/* 仅在定义 USE_TIMER_3_CHANNEL_1 时编译以下代码。 */
#ifdef USE_TIMER_3_CHANNEL_1
	//if(DMA_GetFlagStatus(DMA1_FLAG_HT4) == SET){
        /* 检查指定 DMA 事件标志，区分半传输、整帧完成或传输错误。 */
		if(dma_flag_get(DMA1_HDT6_FLAG) == SET){ 
            /* 当当前输入识别为舵机 PWM 的标志非零时进入此分支。 */
			if(servoPwm){
            /* 更新IC_TIMER_REGISTER 的通道 1 主输出/输入捕获极性。 */
			IC_TIMER_REGISTER->cctrl_bit.c1p = TMR_INPUT_FALLING_EDGE;
		//	IC_TIMER_REGISTER->CTRL2 |= TMR_ICPolarity_Rising;	
            /* 清除对应 DMA 通道的已处理状态标志。 */
			DMA1->clr = DMA1_HDT6_FLAG;
			}
		}

    /* 检查指定 DMA 事件标志，区分半传输、整帧完成或传输错误。 */
    if(dma_flag_get(DMA1_FDT6_FLAG) == SET)		
		{
	//		dma_reset(INPUT_DMA_CHANNEL);
			
      /* 清除对应 DMA 通道的已处理状态标志。 */
      DMA1->clr = DMA1_GL6_FLAG;
            /* 将INPUT_DMA_CHANNEL 的DMA 通道使能清零。 */
			INPUT_DMA_CHANNEL->ctrl_bit.chen = FALSE;
          /* 处理信号 DMA 一帧完成事件：自动识别协议、解码油门、切换双向遥测收发并检查解锁条件。 */
		  transfercomplete();

		  }
           /* 检查指定 DMA 事件标志，区分半传输、整帧完成或传输错误。 */
		   if(dma_flag_get(DMA1_DTERR6_FLAG) == SET)
		  {
			//dma_reset(INPUT_DMA_CHANNEL);	
            /* 清除对应 DMA 通道的已处理状态标志。 */
			DMA1->clr = DMA1_GL6_FLAG;
            /* 将INPUT_DMA_CHANNEL 的DMA 通道使能清零。 */
			INPUT_DMA_CHANNEL->ctrl_bit.chen = FALSE;
          /* 处理信号 DMA 一帧完成事件：自动识别协议、解码油门、切换双向遥测收发并检查解锁条件。 */
		  transfercomplete();
			}
/* 结束当前条件编译分支。 */
#endif
}

/**
  * @brief This function handles ADC and COMP interrupts (COMP interrupts through EXTI lines 21 and 22).
  */
/**
 * @brief F415 比较器中断入口：清除过零 EXINT 标志后进入过零过滤和换相调度。
 */
void CMP1_IRQHandler(void)
{


      /* 确认挂起的是本目标的比较器过零 EXINT 事件。 */
	  if((EXINT->intsts & EXTI_LINE) != (uint32_t)RESET)
	  {
			
		//	EXTI->PND = EXTI_LINE;
            /* 向目标 EXINT 挂起位写 1 清除，避免重复进入同一次过零中断。 */
			EXINT->intsts = EXTI_LINE;
        /* 处理过零中断：拒绝过早事件并过滤错误电平，计算提前角和延时后安排下一次换相。 */
	    interruptRoutine();
	  }
}

/**
  * @brief This function handles TIM6 global and DAC underrun error interrupts.
  */
/**
 * @brief F415 周期控制任务共用中断入口，清除状态后执行 tenKhzRoutine。
 */
void TMR1_BRK_TMR9_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */
	//TIM6->DIER &= ~(0x1UL << (0U));
  /* 清除已处理的定时器事件标志；具体清除位由当前写入值决定。 */
  TMR9->ists = (uint16_t)~TMR_OVF_FLAG;
    /* 清除已处理的定时器事件标志；具体清除位由当前写入值决定。 */
	TMR1->ists = 0x00;
	//		timer_interrupt_flag_clear(TIMER13, TIMER_INT_FLAG_UP);
    /* 周期性控制任务：解锁、油门到占空比映射、电流/低速控制、斜率限制、遥测及失联处理。 */
	tenKhzRoutine();

	  

  /* USER CODE END TIM6_DAC_IRQn 0 */
  
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles TIM14 global interrupt.
  */
/**
 * @brief F415 延迟换相共用中断入口，清除状态后执行 PeriodElapsedCallback。
 */
void TMR1_TRG_HALL_TMR11_IRQHandler(void)
{
  /* USER CODE BEGIN TIM14_IRQn 0 */
//	  if(LL_TIM_IsActiveFlag_UPDATE(TIM14) == 1)
//	  {
	//  timer_interrupt_flag_clear(TIMER15, TIMER_INT_FLAG_UP);
    
	  
      /* 清除已处理的定时器事件标志；具体清除位由当前写入值决定。 */
	  TMR11->ists = 0x00;
      /* 清除已处理的定时器事件标志；具体清除位由当前写入值决定。 */
	  TMR1->ists = 0x00;
        /* 换相延时到期回调：关闭本次定时中断，执行换相，并按模式重新允许过零中断。 */
		PeriodElapsedCallback();

//	  }

  /* USER CODE END TIM14_IRQn 0 */
  /* USER CODE BEGIN TIM14_IRQn 1 */

  /* USER CODE END TIM14_IRQn 1 */
}

/**
 * @brief 清除 F415 辅助计时器 TMR10 的溢出和通道 1 标志。
 */
void TMR1_OVF_TMR10_IRQHandler(void)
{
    /* 清除已处理的定时器事件标志；具体清除位由当前写入值决定。 */
	TMR10->ists = (uint16_t)~TMR_OVF_FLAG;
    /* 清除已处理的定时器事件标志；具体清除位由当前写入值决定。 */
	TMR10->ists = (uint16_t)~TMR_C1_FLAG;

}



/**
  * @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
  */
/**
 * @brief 保留的串口中断入口；当前函数没有业务处理，遥测发送使用 DMA。
 */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */



  /* USER CODE END USART1_IRQn 0 */
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
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

//void DMA_Channel0_IRQHandler(void)         // ADC
//{
//	  if(LL_DMA_IsActiveFlag_TC1(DMA1) == 1)
//	  {
//	    /* Clear flag DMA global interrupt */
//	    /* (global interrupt flag: half transfer and transfer complete flags) */
//	    LL_DMA_ClearFlag_GI1(DMA1);
//	    ADC_DMA_Callback();
//	    /* Call interruption treatment function */
//	 //   AdcDmaTransferComplete_Callback();
//	  }

//	  /* Check whether DMA transfer error caused the DMA interruption */
//	  if(LL_DMA_IsActiveFlag_TE1(DMA1) == 1)
//	  {
//	    /* Clear flag DMA transfer error */
//	    LL_DMA_ClearFlag_TE1(DMA1);

//	    /* Call interruption treatment function */
//	  }
//}


/**
 * @brief 保留的外部中断入口；当前没有启用有效的输入边沿处理代码。
 */
void EXTI4_15_IRQHandler(void){
//	  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_6) != RESET) {
//	    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_6);

//	    signalEdgeRoutine();
//	  }

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
