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
  /* 配置 F421 的换相间隔计时器 TMR6；120 MHz 下分频 60，计数单位为 0.5 微秒。 */
  TIM6_Init();
  /* 配置 F421 周期控制任务的 TMR14 时基和中断，供 tenKhzRoutine 调度使用。 */
  TIM14_Init();
  /* 配置本 MCU 的比较器输入和中断入口并启用比较器；浮空相选择由 changeCompInput 完成。 */
  AT_COMP_Init();
  /* 配置 F421 通用辅助计时器 TMR17，初始化为微秒级计数，供延时和耗时测量使用。 */
  TIM17_Init();
  /* 配置 F421 延迟换相定时器 TMR16；120 MHz 下计数单位为 0.5 微秒。 */
  TIM16_Init();
  	 
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
  /* 根据运行时钟设置 Flash 访问等待周期。 */
  flash_psr_set(FLASH_WAIT_CYCLE_3);
  /* 复位时钟控制配置，为重新设置系统时钟做准备。 */
  crm_reset();
  /* 开启指定振荡器或 PLL 时钟源。 */
  crm_clock_source_enable(CRM_CLOCK_SOURCE_HICK, TRUE);
  /* 重复检查等待条件：检查时钟源稳定/切换状态，未就绪前继续等待。 */
  while(crm_flag_get(CRM_HICK_STABLE_FLAG) != SET)
  {
  }
  /* 选择 PLL 输入源及倍频系数。 */
  crm_pll_config(CRM_PLL_SOURCE_HICK, CRM_PLL_MULT_30);
  /* 开启指定振荡器或 PLL 时钟源。 */
  crm_clock_source_enable(CRM_CLOCK_SOURCE_PLL, TRUE);
  /* 重复检查等待条件：检查时钟源稳定/切换状态，未就绪前继续等待。 */
  while(crm_flag_get(CRM_PLL_STABLE_FLAG) != SET)
  {
  }
  /* 设置 AHB 总线分频。 */
  crm_ahb_div_set(CRM_AHB_DIV_1);
  /* 设置 APB2 总线分频。 */
  crm_apb2_div_set(CRM_APB2_DIV_1);
  /* 设置 APB1 总线分频。 */
  crm_apb1_div_set(CRM_APB1_DIV_1);
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

    /* configure comparator inputs: PA1 MID, PA0/PA4/PA5 BEMF */
     /* 切换选定 GPIO 的工作模式；具体驱动电平由随后的置位/清零或 PWM 决定。 */
     gpio_mode_QUICK(GPIOA, GPIO_MODE_ANALOG, GPIO_PULL_NONE, GPIO_PINS_1);
     /* 切换选定 GPIO 的工作模式；具体驱动电平由随后的置位/清零或 PWM 决定。 */
     gpio_mode_QUICK(GPIOA, GPIO_MODE_ANALOG, GPIO_PULL_NONE, GPIO_PINS_0);
     /* 切换选定 GPIO 的工作模式；具体驱动电平由随后的置位/清零或 PWM 决定。 */
     gpio_mode_QUICK(GPIOA, GPIO_MODE_ANALOG, GPIO_PULL_NONE, GPIO_PINS_4);
     /* 切换选定 GPIO 的工作模式；具体驱动电平由随后的置位/清零或 PWM 决定。 */
     gpio_mode_QUICK(GPIOA, GPIO_MODE_ANALOG, GPIO_PULL_NONE, GPIO_PINS_5);
   //  rcu_periph_clock_enable(RCU_CFGCMP);

    /* configure comparator channel0 */
		
		NVIC_SetPriority(ADC1_CMP_IRQn, 0);
    NVIC_EnableIRQ(ADC1_CMP_IRQn);
		
	//	COMP_Cmd(COMP1_Selection, ENABLE);
        /* 配置比较器使能状态。 */
		cmp_enable(CMP1_SELECTION, TRUE);
}


/**
 * @brief 配置并启动独立看门狗，之后运行流程需定期写重载命令。
 */
void MX_IWDG_Init(void)
{
/* 条件编译：!defined USE_DEBUG；条件满足时采用以下实现。 */
#if !defined USE_DEBUG
    /* 更新WDT 的看门狗命令寄存器。 */
	WDT->cmd = WDT_CMD_UNLOCK;
    /* 更新WDT 的看门狗命令寄存器。 */
	WDT->cmd = WDT_CMD_ENABLE;
  /* 更新WDT 的时钟预分频寄存器。 */
  WDT->div = WDT_CLK_DIV_32;
    /* 更新WDT 的看门狗重装载值。 */
	WDT->rld = 4000;
    /* 重载看门狗，表明控制流程仍在正常执行。 */
	WDT->cmd = WDT_CMD_RELOAD;
/* 结束当前条件编译分支。 */
#endif	
}

/**
 * @brief 配置三相 PWM 的周期、输出模式、预装载、死区和引脚复用；极性按目标宏选择。
 */
void TIM1_Init(void){
    /* 配置对应外设时钟使能，保证后续寄存器访问有效。 */
	crm_periph_clock_enable(CRM_TMR1_PERIPH_CLOCK, TRUE);
    /* 更新TMR1 的周期/自动重装载寄存器。 */
	TMR1->pr = 3000;
    /* 将TMR1 的时钟预分频寄存器清零。 */
	TMR1->div = 0;
	
    /* 将 PWM 通道 1、2 配置为 PWM 模式并启用比较值预装载。 */
	TMR1->cm1 = 0x6868;   // Channel 1 and 2 in PWM output mode
    /* 将 PWM 通道 3 配置为 PWM 模式并启用比较值预装载。 */
	TMR1->cm2 = 0x68;     // channel 3 in PWM output mode

	/*tmr_output_config_type tmr_oc_init_structure;
	tmr_output_default_para_init(&tmr_oc_init_structure);
	
	tmr_output_struct->oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
  tmr_output_struct->oc_idle_state = FALSE;
  tmr_output_struct->occ_idle_state = FALSE;
  tmr_output_struct->oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  tmr_output_struct->occ_polarity = TMR_OUTPUT_ACTIVE_HIGH;
  tmr_output_struct->oc_output_state = TRUE;
  tmr_output_struct->occ_output_state = FALSE;
	
	tmr_oc_init_structure.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A;
	tmr_oc_init_structure.oc_idle_state = FALSE;
	tmr_oc_init_structure.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH;
	tmr_oc_init_structure.oc_output_state = TRUE;
	tmr_output_channel_config(TMR1, TMR_SELECT_CHANNEL_1, &tmr_oc_init_structure);*/
	
    /* 仅在定义 USE_INVERTED_HIGH 时编译以下代码。 */
	#ifdef USE_INVERTED_HIGH
        /* 更新TMR1 的通道 1 主输出/输入捕获极性。 */
		TMR1->cctrl_bit.c1p=TMR_OUTPUT_ACTIVE_LOW;
        /* 更新TMR1 的通道 2 主输出极性。 */
		TMR1->cctrl_bit.c2p=TMR_OUTPUT_ACTIVE_LOW;
        /* 更新TMR1 的通道 3 主输出极性。 */
		TMR1->cctrl_bit.c3p=TMR_OUTPUT_ACTIVE_LOW;
        /* 设置输出空闲电平，使关闭/空闲状态符合驱动有效极性。 */
		TMR1->ctrl2_bit.c1ios=TRUE;
        /* 设置输出空闲电平，使关闭/空闲状态符合驱动有效极性。 */
		TMR1->ctrl2_bit.c2ios=TRUE;
        /* 设置输出空闲电平，使关闭/空闲状态符合驱动有效极性。 */
		TMR1->ctrl2_bit.c3ios=TRUE;
  /* 采用上一编译条件不成立时的备选实现。 */
  #else
        /* 更新TMR1 的通道 1 主输出/输入捕获极性。 */
		TMR1->cctrl_bit.c1p=TMR_OUTPUT_ACTIVE_HIGH;
        /* 更新TMR1 的通道 2 主输出极性。 */
		TMR1->cctrl_bit.c2p=TMR_OUTPUT_ACTIVE_HIGH;
        /* 更新TMR1 的通道 3 主输出极性。 */
		TMR1->cctrl_bit.c3p=TMR_OUTPUT_ACTIVE_HIGH;
        /* 设置输出空闲电平，使关闭/空闲状态符合驱动有效极性。 */
		TMR1->ctrl2_bit.c1ios=FALSE;
        /* 设置输出空闲电平，使关闭/空闲状态符合驱动有效极性。 */
		TMR1->ctrl2_bit.c2ios=FALSE;
        /* 设置输出空闲电平，使关闭/空闲状态符合驱动有效极性。 */
		TMR1->ctrl2_bit.c3ios=FALSE;
  /* 结束当前条件编译分支。 */
  #endif
  /* 仅在定义 USE_INVERTED_LOW 时编译以下代码。 */
  #ifdef USE_INVERTED_LOW
        /* 更新TMR1 的通道 1 互补输出极性。 */
		TMR1->cctrl_bit.c1cp=TMR_OUTPUT_ACTIVE_LOW;
        /* 更新TMR1 的通道 2 互补输出极性。 */
		TMR1->cctrl_bit.c2cp=TMR_OUTPUT_ACTIVE_LOW;
        /* 更新TMR1 的通道 3 互补输出极性。 */
		TMR1->cctrl_bit.c3cp=TMR_OUTPUT_ACTIVE_LOW;
        /* 设置输出空闲电平，使关闭/空闲状态符合驱动有效极性。 */
		TMR1->ctrl2_bit.c1cios=TRUE;
        /* 设置输出空闲电平，使关闭/空闲状态符合驱动有效极性。 */
		TMR1->ctrl2_bit.c2cios=TRUE;
        /* 设置输出空闲电平，使关闭/空闲状态符合驱动有效极性。 */
		TMR1->ctrl2_bit.c3cios=TRUE;
  /* 采用上一编译条件不成立时的备选实现。 */
  #else
        /* 更新TMR1 的通道 1 互补输出极性。 */
		TMR1->cctrl_bit.c1cp=TMR_OUTPUT_ACTIVE_HIGH;
        /* 更新TMR1 的通道 2 互补输出极性。 */
		TMR1->cctrl_bit.c2cp=TMR_OUTPUT_ACTIVE_HIGH;
        /* 更新TMR1 的通道 3 互补输出极性。 */
		TMR1->cctrl_bit.c3cp=TMR_OUTPUT_ACTIVE_HIGH;
        /* 设置输出空闲电平，使关闭/空闲状态符合驱动有效极性。 */
		TMR1->ctrl2_bit.c1cios=FALSE;
        /* 设置输出空闲电平，使关闭/空闲状态符合驱动有效极性。 */
		TMR1->ctrl2_bit.c2cios=FALSE;
        /* 设置输出空闲电平，使关闭/空闲状态符合驱动有效极性。 */
		TMR1->ctrl2_bit.c3cios=FALSE;
  /* 结束当前条件编译分支。 */
  #endif
	
	
	
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

/* 把选定引脚连接到指定外设复用功能。 */
gpio_pin_mux_config(PHASE_A_GPIO_PORT_LOW, PHASE_A_PIN_SOURCE_LOW, GPIO_MUX_2);
/* 把选定引脚连接到指定外设复用功能。 */
gpio_pin_mux_config(PHASE_B_GPIO_PORT_LOW, PHASE_B_PIN_SOURCE_LOW, GPIO_MUX_2);
/* 把选定引脚连接到指定外设复用功能。 */
gpio_pin_mux_config(PHASE_C_GPIO_PORT_LOW, PHASE_C_PIN_SOURCE_LOW, GPIO_MUX_2);


    /*configure PB13/PB14/PB15(TIMER0/CH0N/CH1N/CH2N) as alternate function*/
    /* 切换选定 GPIO 的工作模式；具体驱动电平由随后的置位/清零或 PWM 决定。 */
    gpio_mode_QUICK(PHASE_A_GPIO_PORT_HIGH, GPIO_MODE_MUX, GPIO_PULL_NONE, PHASE_A_GPIO_HIGH);

    /* 切换选定 GPIO 的工作模式；具体驱动电平由随后的置位/清零或 PWM 决定。 */
    gpio_mode_QUICK(PHASE_B_GPIO_PORT_HIGH, GPIO_MODE_MUX, GPIO_PULL_NONE, PHASE_B_GPIO_HIGH);

    /* 切换选定 GPIO 的工作模式；具体驱动电平由随后的置位/清零或 PWM 决定。 */
    gpio_mode_QUICK(PHASE_C_GPIO_PORT_HIGH, GPIO_MODE_MUX, GPIO_PULL_NONE, PHASE_C_GPIO_HIGH);

/* 把选定引脚连接到指定外设复用功能。 */
gpio_pin_mux_config(PHASE_A_GPIO_PORT_HIGH, PHASE_A_PIN_SOURCE_HIGH, GPIO_MUX_2);
/* 把选定引脚连接到指定外设复用功能。 */
gpio_pin_mux_config(PHASE_B_GPIO_PORT_HIGH, PHASE_B_PIN_SOURCE_HIGH, GPIO_MUX_2);
/* 把选定引脚连接到指定外设复用功能。 */
gpio_pin_mux_config(PHASE_C_GPIO_PORT_HIGH, PHASE_C_PIN_SOURCE_HIGH, GPIO_MUX_2);

/* 条件编译：defined(USE_INNER_STEP)；条件满足时采用以下实现。 */
#if defined(USE_INNER_STEP)
  /* 配置定时器事件中断使能。 */
  tmr_interrupt_enable(TMR1, TMR_OVF_INT, TRUE);
	//TMR_C1_FLAG
  /* tmr1 overflow interrupt nvic init */
  //nvic_priority_group_config(NVIC_PRIORITY_GROUP_0);
  /* 设置工程外设中断的优先级并允许该中断。 */
  nvic_irq_enable(TMR1_BRK_OVF_TRG_HALL_IRQn, 0, 0);
/* 结束当前条件编译分支。 */
#endif

}



/**
 * @brief 配置 F421 的换相间隔计时器 TMR6；120 MHz 下分频 60，计数单位为 0.5 微秒。
 */
void TIM6_Init(void)
{
    /* 配置对应外设时钟使能，保证后续寄存器访问有效。 */
	crm_periph_clock_enable(CRM_TMR6_PERIPH_CLOCK, TRUE);
    /* 更新TMR6 的周期/自动重装载寄存器。 */
	TMR6->pr = 0xFFFF;
    /* 更新TMR6 的时钟预分频寄存器。 */
	TMR6->div = 59;
	NVIC_SetPriority(TMR6_GLOBAL_IRQn, 0);
  NVIC_EnableIRQ(TMR6_GLOBAL_IRQn);
}


/**
 * @brief 配置 F421 周期控制任务的 TMR14 时基和中断，供 tenKhzRoutine 调度使用。
 */
void TIM14_Init(void)
{
    /* 配置对应外设时钟使能，保证后续寄存器访问有效。 */
	crm_periph_clock_enable(CRM_TMR14_PERIPH_CLOCK, TRUE);

    /* 更新TMR14 的周期/自动重装载寄存器。 */
	TMR14->pr = 100;
    /* 更新TMR14 的时钟预分频寄存器。 */
	TMR14->div = 119;
	
	
	NVIC_SetPriority(TMR14_GLOBAL_IRQn, 2);
  NVIC_EnableIRQ(TMR14_GLOBAL_IRQn);
	
}


/**
 * @brief 配置 F421 延迟换相定时器 TMR16；120 MHz 下计数单位为 0.5 微秒。
 */
void TIM16_Init(void)
{
    /* 配置对应外设时钟使能，保证后续寄存器访问有效。 */
	crm_periph_clock_enable(CRM_TMR16_PERIPH_CLOCK, TRUE);
    /* 更新TMR16 的周期/自动重装载寄存器。 */
	TMR16->pr = 500;
    /* 更新TMR16 的时钟预分频寄存器。 */
	TMR16->div = 59;
    /* 将TMR16 的定时器周期预装载使能设为 1。 */
	TMR16->ctrl1_bit.prben = TRUE;
	
	NVIC_SetPriority(TMR16_GLOBAL_IRQn, 0);
  NVIC_EnableIRQ(TMR16_GLOBAL_IRQn);
}



/**
 * @brief 配置 F421 通用辅助计时器 TMR17，初始化为微秒级计数，供延时和耗时测量使用。
 */
void TIM17_Init(void)
{
    /* 配置对应外设时钟使能，保证后续寄存器访问有效。 */
	crm_periph_clock_enable(CRM_TMR17_PERIPH_CLOCK, TRUE);
    /* 更新TMR17 的周期/自动重装载寄存器。 */
	TMR17->pr = 0xFFFF;
	//TMR17->div = 59;
    /* 更新TMR17 的时钟预分频寄存器。 */
	TMR17->div = CPU_FREQUENCY_MHZ-1;
/* 将TMR17 的定时器周期预装载使能设为 1。 */
TMR17->ctrl1_bit.prben = TRUE;

}


/**
 * @brief 开启 DMA 控制器时钟，并设置输入捕获 DMA 中断优先级。
 */
void MX_DMA_Init(void)
{
/* 配置对应外设时钟使能，保证后续寄存器访问有效。 */
crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
	
  NVIC_SetPriority(DMA1_Channel5_4_IRQn, 1);
  NVIC_EnableIRQ(DMA1_Channel5_4_IRQn);

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
    /* 切换选定 GPIO 的工作模式；具体驱动电平由随后的置位/清零或 PWM 决定。 */
	gpio_mode_QUICK(INPUT_PIN_PORT, GPIO_MODE_MUX, GPIO_PULL_NONE, INPUT_PIN);
    /* 把选定引脚连接到指定外设复用功能。 */
	gpio_pin_mux_config(INPUT_PIN_PORT, INPUT_PIN_SOURCE, GPIO_MUX_1);
/* 结束当前条件编译分支。 */
#endif
/* 仅在定义 USE_TIMER_15_CHANNEL_1 时编译以下代码。 */
#ifdef USE_TIMER_15_CHANNEL_1
	
    /* 配置对应外设时钟使能，保证后续寄存器访问有效。 */
	crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    /* 配置对应外设时钟使能，保证后续寄存器访问有效。 */
	crm_periph_clock_enable(CRM_TMR15_PERIPH_CLOCK, TRUE);
  /* 切换选定 GPIO 的工作模式；具体驱动电平由随后的置位/清零或 PWM 决定。 */
  gpio_mode_QUICK(INPUT_PIN_PORT, GPIO_MODE_MUX, GPIO_PULL_NONE, INPUT_PIN);
/* 结束当前条件编译分支。 */
#endif
 
/* 配置对应外设时钟使能，保证后续寄存器访问有效。 */
crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
	
	
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



