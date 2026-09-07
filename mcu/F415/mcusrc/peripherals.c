/*
 * 本 MCU 的时钟和外设适配初始化；属于工程代码，底层标准库保持原样。
 * 中文说明按当前实现编写；保留原代码、历史注释和编译条件。
 */
/*
 * peripherals.c
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */


// PERIPHERAL SETUP


/* 看门狗重载命令常量。 */
#define KR_KEY_Reload           ((uint16_t)0xAAAA)
/* 看门狗使能命令常量。 */
#define KR_KEY_Enable           ((uint16_t)0xCCCC)


/* 引入 peripherals.h：本 MCU 的时钟和外设适配初始化；属于工程代码，底层标准库保持原样。 */
#include "peripherals.h"



/* 引入 targets.h：硬件目标配置：选择 MCU、输入捕获资源、三相驱动引脚、比较器通道和默认参数。 */
#include "targets.h"
/* 引入 serial_telemetry.h：串口遥测组帧、CRC8 校验和 USART1 DMA 发送配置。 */
#include "serial_telemetry.h"
/* 引入 common.h：模块间共享变量及整数 PID 状态结构；变量实体主要在 main.c 和协议模块中定义。 */
#include "common.h"
/* 引入 functions.h：GPIO 模式切换、限幅线性映射、绝对差和阻塞延时等工程辅助函数。 */
#include "functions.h"


/**
 * @brief 按依赖顺序初始化系统时钟、GPIO、DMA、PWM、各用途定时器、比较器和可选串口遥测。
 */
void initCorePeripherals(void){
  /* 选择内部高速时钟作为 PLL 来源，设置总线分频并等待时钟稳定，最后更新系统时钟变量。 */
  system_clock_config();
    /* 预留通用 GPIO 初始化入口；当前实际引脚配置分散在具体外设初始化函数中。 */
	MX_GPIO_Init();
  /* 开启 DMA 控制器时钟，并设置输入捕获 DMA 中断优先级。 */
  MX_DMA_Init();
    /* 配置三相 PWM 的周期、输出模式、预装载、死区和引脚复用；极性按目标宏选择。 */
	TIM1_Init();
  /* 配置 F415 的换相间隔计时器 TMR4，供电周期测量和超时判断使用。 */
  TIM4_Init();
  /* 配置 F415 周期控制任务的 TMR9 时基和中断；实际周期由当前时钟和分频决定。 */
  TIM9_Init();
  /* 配置本 MCU 的比较器输入和中断入口并启用比较器；浮空相选择由 changeCompInput 完成。 */
  AT_COMP_Init();
  /* 配置 F415 延迟换相定时器 TMR11，过零后用它安排下一次换相。 */
  TIM11_Init();
  /* 配置 F415 通用辅助计时器 TMR10，供延时等工具函数使用。 */
  TIM10_Init();
  	 
  /* 配置输入信号定时器及其 DMA 请求，具体定时器和引脚由 targets.h 选择。 */
  UN_TIM_Init();
  /* 仅在定义 USE_SERIAL_TELEMETRY 时编译以下代码。 */
  #ifdef USE_SERIAL_TELEMETRY
    /* 初始化 USART1 遥测发送和 DMA；发送引脚为 PB6，具体 DMA 通道随 MCU 适配代码变化。 */
    telem_UART_Init();
  /* 结束当前条件编译分支。 */
  #endif
}


/**
 * @brief 预留的跳转后初始化钩子；当前实现为空。
 */
void initAfterJump(void){

}


/**
 * @brief 选择内部高速时钟作为 PLL 来源，设置总线分频并等待时钟稳定，最后更新系统时钟变量。
 */
void system_clock_config(void)
{

  /* config flash psr register */
 

/// 144 mhz hick setup

/* 根据运行时钟设置 Flash 访问等待周期。 */
flash_psr_set(FLASH_WAIT_CYCLE_4);
  /* 复位时钟控制配置，为重新设置系统时钟做准备。 */
  crm_reset();

  /* 开启指定振荡器或 PLL 时钟源。 */
  crm_clock_source_enable(CRM_CLOCK_SOURCE_HICK, TRUE);
  /* 重复检查等待条件：检查时钟源稳定/切换状态，未就绪前继续等待。 */
  while(crm_flag_get(CRM_HICK_STABLE_FLAG) != SET)
  {
  }
  /* 选择 PLL 输入源及倍频系数。 */
  crm_pll_config(CRM_PLL_SOURCE_HICK, CRM_PLL_MULT_36);
  /* 开启指定振荡器或 PLL 时钟源。 */
  crm_clock_source_enable(CRM_CLOCK_SOURCE_PLL, TRUE);
  /* 重复检查等待条件：检查时钟源稳定/切换状态，未就绪前继续等待。 */
  while(crm_flag_get(CRM_PLL_STABLE_FLAG) != SET)
  {
  }
  /* 设置 AHB 总线分频。 */
  crm_ahb_div_set(CRM_AHB_DIV_1);
  /* 设置 APB2 总线分频。 */
  crm_apb2_div_set(CRM_APB2_DIV_2);
  /* 设置 APB1 总线分频。 */
  crm_apb1_div_set(CRM_APB1_DIV_2);
  /* 配置时钟切换时的自动分步功能。 */
  crm_auto_step_mode_enable(TRUE);
  /* 请求切换系统主时钟源。 */
  crm_sysclk_switch(CRM_SCLK_PLL);
  /* 重复检查等待条件：检查时钟源稳定/切换状态，未就绪前继续等待。 */
  while(crm_sysclk_switch_status_get() != CRM_SCLK_PLL)
  {
  }
  /* 配置时钟切换时的自动分步功能。 */
  crm_auto_step_mode_enable(FALSE);
  system_core_clock_update();
}


/**
 * @brief 配置本 MCU 的比较器输入和中断入口并启用比较器；浮空相选择由 changeCompInput 完成。
 */
void AT_COMP_Init(void)
{
	
  /* 配置对应外设时钟使能，保证后续寄存器访问有效。 */
  crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    /* 配置对应外设时钟使能，保证后续寄存器访问有效。 */
	crm_periph_clock_enable(CRM_CMP_PERIPH_CLOCK, TRUE);
	  /* configure PA1 as comparator input */
     /* 切换选定 GPIO 的工作模式；具体驱动电平由随后的置位/清零或 PWM 决定。 */
     gpio_mode_QUICK(GPIOA, GPIO_MODE_ANALOG, GPIO_PULL_NONE, GPIO_PINS_1);

     /* 切换选定 GPIO 的工作模式；具体驱动电平由随后的置位/清零或 PWM 决定。 */
     gpio_mode_QUICK(GPIOA, GPIO_MODE_ANALOG, GPIO_PULL_NONE, GPIO_PINS_5);

		
		NVIC_SetPriority(CMP1_IRQn, 0);
    NVIC_EnableIRQ(CMP1_IRQn);
        /* 配置比较器使能状态。 */
		cmp_enable(CMP1_SELECTION, TRUE);
}


/**
 * @brief 配置并启动独立看门狗，之后运行流程需定期写重载命令。
 */
void MX_IWDG_Init(void)
{

    /* 更新WDT 的看门狗命令寄存器。 */
	WDT->cmd = WDT_CMD_UNLOCK;
    /* 更新WDT 的看门狗命令寄存器。 */
	WDT->cmd = WDT_CMD_ENABLE;
  /* 更新WDT 的时钟预分频寄存器。 */
  WDT->div = WDT_CLK_DIV_64;
    /* 更新WDT 的看门狗重装载值。 */
	WDT->rld = 4000;
    /* 重载看门狗，表明控制流程仍在正常执行。 */
	WDT->cmd = WDT_CMD_RELOAD;
	
}

/**
 * @brief 配置三相 PWM 的周期、输出模式、预装载、死区和引脚复用；极性按目标宏选择。
 */
void TIM1_Init(void){

    /* 配置对应外设时钟使能，保证后续寄存器访问有效。 */
	crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, TRUE);
    /* 配置对应外设时钟使能，保证后续寄存器访问有效。 */
	crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, TRUE);

    /* 配置 F415 引脚重映射，释放或连接目标外设信号。 */
	gpio_pin_remap_config(TMR1_GMUX_0001, TRUE);
	
	
    /* 更新TMR1 的周期/自动重装载寄存器。 */
	TMR1->pr = 3000;
    /* 将TMR1 的时钟预分频寄存器清零。 */
	TMR1->div = 0;
	
    /* 将 PWM 通道 1、2 配置为 PWM 模式并启用比较值预装载。 */
	TMR1->cm1 = 0x6868;   // Channel 1 and 2 in PWM output mode
    /* 将 PWM 通道 3 配置为 PWM 模式并启用比较值预装载。 */
	TMR1->cm2 = 0x68;     // channel 3 in PWM output mode

    /* 配置通道比较值预装载，在更新事件时生效。 */
	tmr_output_channel_buffer_enable(TMR1,TMR_SELECT_CHANNEL_1, TRUE);
    /* 配置通道比较值预装载，在更新事件时生效。 */
	tmr_output_channel_buffer_enable(TMR1,TMR_SELECT_CHANNEL_2, TRUE);
    /* 配置通道比较值预装载，在更新事件时生效。 */
	tmr_output_channel_buffer_enable(TMR1,TMR_SELECT_CHANNEL_3, TRUE);
	
    /* 配置周期值预装载，避免直接改变正在进行的周期。 */
	tmr_period_buffer_enable(TMR1, TRUE);
	
    /* 更新TMR1 的高低侧驱动死区配置。 */
	TMR1->brk_bit.dtc = DEAD_TIME;
    /* 配置对应外设时钟使能，保证后续寄存器访问有效。 */
	crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    /* 配置对应外设时钟使能，保证后续寄存器访问有效。 */
	crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
	
	    /*configure PA8/PA9/PA10(TIMER0/CH0/CH1/CH2) as alternate function*/
    /* 切换选定 GPIO 的工作模式；具体驱动电平由随后的置位/清零或 PWM 决定。 */
    gpio_mode_QUICK(PHASE_A_GPIO_PORT_LOW, GPIO_MODE_MUX, GPIO_PULL_NONE, PHASE_A_GPIO_LOW);


    /* 切换选定 GPIO 的工作模式；具体驱动电平由随后的置位/清零或 PWM 决定。 */
    gpio_mode_QUICK(PHASE_B_GPIO_PORT_LOW, GPIO_MODE_MUX, GPIO_PULL_NONE, PHASE_B_GPIO_LOW);


    /* 切换选定 GPIO 的工作模式；具体驱动电平由随后的置位/清零或 PWM 决定。 */
    gpio_mode_QUICK(PHASE_C_GPIO_PORT_LOW, GPIO_MODE_MUX, GPIO_PULL_NONE, PHASE_C_GPIO_LOW);

    /*configure PB13/PB14/PB15(TIMER0/CH0N/CH1N/CH2N) as alternate function*/
    /* 切换选定 GPIO 的工作模式；具体驱动电平由随后的置位/清零或 PWM 决定。 */
    gpio_mode_QUICK(PHASE_A_GPIO_PORT_HIGH, GPIO_MODE_MUX, GPIO_PULL_NONE, PHASE_A_GPIO_HIGH);


    /* 切换选定 GPIO 的工作模式；具体驱动电平由随后的置位/清零或 PWM 决定。 */
    gpio_mode_QUICK(PHASE_B_GPIO_PORT_HIGH, GPIO_MODE_MUX, GPIO_PULL_NONE, PHASE_B_GPIO_HIGH);


    /* 切换选定 GPIO 的工作模式；具体驱动电平由随后的置位/清零或 PWM 决定。 */
    gpio_mode_QUICK(PHASE_C_GPIO_PORT_HIGH, GPIO_MODE_MUX, GPIO_PULL_NONE, PHASE_C_GPIO_HIGH);

}



/**
 * @brief 配置 F415 的换相间隔计时器 TMR4，供电周期测量和超时判断使用。
 */
void TIM4_Init(void)
{
    /* 配置对应外设时钟使能，保证后续寄存器访问有效。 */
	crm_periph_clock_enable(CRM_TMR4_PERIPH_CLOCK, TRUE);
	
    /* 更新TMR4 的周期/自动重装载寄存器。 */
	TMR4->pr = 0xFFFF;
    /* 更新TMR4 的时钟预分频寄存器。 */
	TMR4->div = 74;

}



/**
 * @brief 配置 F415 周期控制任务的 TMR9 时基和中断；实际周期由当前时钟和分频决定。
 */
void TIM9_Init(void)
{

    /* 配置对应外设时钟使能，保证后续寄存器访问有效。 */
	crm_periph_clock_enable(CRM_TMR9_PERIPH_CLOCK, TRUE);

    /* 更新TMR9 的周期/自动重装载寄存器。 */
	TMR9->pr = 100;
    /* 更新TMR9 的时钟预分频寄存器。 */
	TMR9->div = 149;
	
	
	NVIC_SetPriority(TMR1_BRK_TMR9_IRQn, 2);
  NVIC_EnableIRQ(TMR1_BRK_TMR9_IRQn);
	
	//TMR_Cmd(TMR14, ENABLE);
}



/**
 * @brief 配置 F415 延迟换相定时器 TMR11，过零后用它安排下一次换相。
 */
void TIM11_Init(void)
{
    /* 配置对应外设时钟使能，保证后续寄存器访问有效。 */
	crm_periph_clock_enable(CRM_TMR11_PERIPH_CLOCK, TRUE);

    /* 更新TMR11 的周期/自动重装载寄存器。 */
	TMR11->pr = 500;
    /* 更新TMR11 的时钟预分频寄存器。 */
	TMR11->div = 75;
    /* 将TMR11 的定时器周期预装载使能设为 1。 */
	TMR11->ctrl1_bit.prben = TRUE;
	
	NVIC_SetPriority(TMR1_TRG_HALL_TMR11_IRQn, 0);
  NVIC_EnableIRQ(TMR1_TRG_HALL_TMR11_IRQn);
	
}



/**
 * @brief 配置 F415 通用辅助计时器 TMR10，供延时等工具函数使用。
 */
void TIM10_Init(void)
{

    /* 配置对应外设时钟使能，保证后续寄存器访问有效。 */
	crm_periph_clock_enable(CRM_TMR10_PERIPH_CLOCK, TRUE);
    /* 更新TMR10 的周期/自动重装载寄存器。 */
	TMR10->pr = 0xFFFF;
    /* 更新TMR10 的时钟预分频寄存器。 */
	TMR10->div = 75;
/* 将TMR10 的定时器周期预装载使能设为 1。 */
TMR10->ctrl1_bit.prben = TRUE;
	
}


/**
 * @brief 开启 DMA 控制器时钟，并设置输入捕获 DMA 中断优先级。
 */
void MX_DMA_Init(void)
{

/* 配置对应外设时钟使能，保证后续寄存器访问有效。 */
crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);

  NVIC_SetPriority(DMA1_Channel6_IRQn, 1);
  NVIC_EnableIRQ(DMA1_Channel6_IRQn);

}

/**
 * @brief 预留通用 GPIO 初始化入口；当前实际引脚配置分散在具体外设初始化函数中。
 */
void MX_GPIO_Init(void)
{

}


/**
 * @brief 配置输入信号定时器及其 DMA 请求，具体定时器和引脚由 targets.h 选择。
 */
void UN_TIM_Init(void)
{
/* 仅在定义 USE_TIMER_3_CHANNEL_1 时编译以下代码。 */
#ifdef USE_TIMER_3_CHANNEL_1
  /* 配置对应外设时钟使能，保证后续寄存器访问有效。 */
  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    /* 配置对应外设时钟使能，保证后续寄存器访问有效。 */
	crm_periph_clock_enable(CRM_TMR3_PERIPH_CLOCK, TRUE);
/* 结束当前条件编译分支。 */
#endif
/* 仅在定义 USE_TIMER_15_CHANNEL_1 时编译以下代码。 */
#ifdef USE_TIMER_15_CHANNEL_1
    /* 配置对应外设时钟使能，保证后续寄存器访问有效。 */
	crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    /* 配置对应外设时钟使能，保证后续寄存器访问有效。 */
	crm_periph_clock_enable(CRM_TMR15_PERIPH_CLOCK, TRUE);
/* 结束当前条件编译分支。 */
#endif
	
        /* 配置对应外设时钟使能，保证后续寄存器访问有效。 */
		crm_periph_clock_enable(CRM_IOMUX_PERIPH_CLOCK, TRUE);
      /* 配置 F415 引脚重映射，释放或连接目标外设信号。 */
	  gpio_pin_remap_config(SWJTAG_MUX_010, TRUE);
        /* 配置 F415 引脚重映射，释放或连接目标外设信号。 */
		gpio_pin_remap_config(TMR3_MUX_10, TRUE);
 
  /* 配置对应外设时钟使能，保证后续寄存器访问有效。 */
  crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
    /* 切换选定 GPIO 的工作模式；具体驱动电平由随后的置位/清零或 PWM 决定。 */
	gpio_mode_QUICK(INPUT_PIN_PORT, GPIO_MODE_INPUT, GPIO_PULL_NONE, INPUT_PIN);
	
	
	//GPIO_PinAFConfig(INPUT_PIN_PORT, GPIO_PinsSource4, GPIO_AF_1);
//	gpio_pin_mux_config(INPUT_PIN_PORT, INPUT_PIN, GPIO_MUX_1);

//	 dma_periph_address_config(INPUT_DMA_CHANNEL, (uint32_t)&TIMER_CH0CV(IC_TIMER_REGISTER));
//   dma_memory_address_config(INPUT_DMA_CHANNEL, (uint32_t)&dma_buffer);
//	 INPUT_DMA_CHANNEL->CPBA = (uint32_t)&IC_TIMER_REGISTER->CC1;
//	 INPUT_DMA_CHANNEL->CMBA = (uint32_t)&dma_buffer;
//   INPUT_DMA_CHANNEL->CHCTRL |= DMA_DIR_PERIPHERALSRC;

 //   DMA_Reset(INPUT_DMA_CHANNEL);
//  DMA_DefaultInitParaConfig(&DMA_InitStructure);

//  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&IC_TIMER_REGISTER->CC1;
//  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&dma_buffer;
//  DMA_InitStructure.DMA_Direction = DMA_DIR_PERIPHERALSRC;
//  DMA_InitStructure.DMA_BufferSize = 32;
//  DMA_InitStructure.DMA_PeripheralInc = DMA_PERIPHERALINC_DISABLE;
//  DMA_InitStructure.DMA_MemoryInc = DMA_MEMORYINC_ENABLE;
//  DMA_InitStructure.DMA_PeripheralDataWidth = DMA_PERIPHERALDATAWIDTH_HALFWORD;
//  DMA_InitStructure.DMA_MemoryDataWidth = DMA_MEMORYDATAWIDTH_WORD;
//  DMA_InitStructure.DMA_Mode = DMA_MODE_NORMAL;
//  DMA_InitStructure.DMA_Priority = DMA_PRIORITY_LOW;
//  DMA_InitStructure.DMA_MTOM = DMA_MEMTOMEM_DISABLE;
//  DMA_Init(INPUT_DMA_CHANNEL, &DMA_InitStructure);

  /* 配置外设半字、内存字、内存地址递增及完成/错误中断；当前尚未置位通道使能。 */
  INPUT_DMA_CHANNEL->ctrl = 0X98a; //  PERIPHERAL HALF WORD, MEMROY WORD , MEMORY INC ENABLE , TC AND ERROR INTS


   NVIC_SetPriority(IC_DMA_IRQ_NAME, 1);
   NVIC_EnableIRQ(IC_DMA_IRQ_NAME);

	

	
    /* 更新IC_TIMER_REGISTER 的周期/自动重装载寄存器。 */
	IC_TIMER_REGISTER->pr = 0xFFFF;
    /* 更新IC_TIMER_REGISTER 的时钟预分频寄存器。 */
	IC_TIMER_REGISTER->div = 16;
    /* 将IC_TIMER_REGISTER 的定时器周期预装载使能设为 1。 */
	IC_TIMER_REGISTER->ctrl1_bit.prben = TRUE;
  /* 切换选定 GPIO 的工作模式；具体驱动电平由随后的置位/清零或 PWM 决定。 */
  gpio_mode_QUICK(GPIOB, GPIO_MODE_INPUT, GPIO_PULL_NONE , INPUT_PIN);
    /* 将IC_TIMER_REGISTER 的定时器计数使能设为 1。 */
	IC_TIMER_REGISTER->ctrl1_bit.tmren = TRUE;
		
}

/* 仅在定义 USE_RGB_LED 时编译以下代码。 */
#ifdef USE_RGB_LED              // has 3 color led
/**
 * @brief 配置可选 RGB 状态灯引脚；仅在 USE_RGB_LED 编译分支中使用。
 */
void LED_GPIO_init(){
      /* 保存兼容分支的 GPIO 初始化结构。 */
	  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	  /* GPIO Ports Clock Enable */
      /* 兼容分支中开启 GPIO 端口时钟。 */
	  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

      /* 兼容分支中将状态灯引脚输出为低电平。 */
	  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_8);
      /* 兼容分支中将状态灯引脚输出为低电平。 */
      LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_5);
      /* 兼容分支中将状态灯引脚输出为低电平。 */
	  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_3);

      /* 更新兼容 GPIO 引脚选择。 */
	  GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
      /* 更新兼容 GPIO 工作模式。 */
	  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
      /* 更新兼容 GPIO 输出速度。 */
	  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
      /* 更新兼容 GPIO 输出类型。 */
	  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
      /* 更新兼容 GPIO 上下拉设置。 */
	  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
      /* 兼容分支中应用 GPIO 初始化结构。 */
	  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

      /* 更新兼容 GPIO 引脚选择。 */
	  GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
      /* 更新兼容 GPIO 工作模式。 */
	  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
      /* 更新兼容 GPIO 输出速度。 */
	  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
      /* 更新兼容 GPIO 输出类型。 */
	  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
      /* 更新兼容 GPIO 上下拉设置。 */
	  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
      /* 兼容分支中应用 GPIO 初始化结构。 */
	  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

      /* 更新兼容 GPIO 引脚选择。 */
	  GPIO_InitStruct.Pin = LL_GPIO_PIN_3;
      /* 更新兼容 GPIO 工作模式。 */
	  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
      /* 更新兼容 GPIO 输出速度。 */
	  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
      /* 更新兼容 GPIO 输出类型。 */
	  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
      /* 更新兼容 GPIO 上下拉设置。 */
	  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
      /* 兼容分支中应用 GPIO 初始化结构。 */
	  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* 结束当前条件编译分支。 */
#endif



