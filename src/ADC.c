/*
 * ADC 采样及 DMA 搬运接口，包含正常采样与 BEMF 诊断采样两套配置。
 * 中文说明按当前实现编写；保留原代码、历史注释和编译条件。
 */
/*
 * ADC.c
 *
 *  Created on: May 20, 2020
 *      Author: Alka
 */
/* 引入 ADC.h：ADC 采样及 DMA 搬运接口，包含正常采样与 BEMF 诊断采样两套配置。 */
#include "ADC.h"
/* 引入 functions.h：GPIO 模式切换、限幅线性映射、绝对差和阻塞延时等工程辅助函数。 */
#include "functions.h"
/* 仅在定义 USE_ADC 时编译以下代码。 */
#ifdef USE_ADC
/* 仅在定义 USE_ADC_INPUT 时编译以下代码。 */
#ifdef USE_ADC_INPUT
/* 保存普通 ADC 扫描 DMA 接收缓冲区。 */
uint16_t ADCDataDMA[4];
/* 采用上一编译条件不成立时的备选实现。 */
#else
/* 保存普通 ADC 扫描 DMA 接收缓冲区。 */
uint16_t ADCDataDMA[3];
/* 结束当前条件编译分支。 */
#endif


/* 引用其他模块定义的温度通道 ADC 原始值。 */
extern uint16_t ADC_raw_temp;
/* 引用其他模块定义的电压通道 ADC 原始值。 */
extern uint16_t ADC_raw_volts;
/* 引用其他模块定义的电流通道 ADC 原始值。 */
extern uint16_t ADC_raw_current;
/* 引用其他模块定义的可选模拟油门 ADC 原始值。 */
extern uint16_t ADC_raw_input;

/* 保存BEMF 诊断 ADC 的四路采样缓冲区。 */
uint16_t adc_diagnose_data[4];

/**
 * @brief 按 ADC 扫描顺序取出 DMA 数据，更新温度、电压、电流及可选模拟油门的原始值。
 */
void ADC_DMA_Callback(){  // read dma buffer and set extern variables

/* 仅在定义 USE_ADC_INPUT 时编译以下代码。 */
#ifdef USE_ADC_INPUT
    /* 更新温度通道 ADC 原始值。 */
	ADC_raw_temp =    ADCDataDMA[3];
    /* 更新电压通道 ADC 原始值。 */
	ADC_raw_volts  = ADCDataDMA[1]/2;
    /* 更新电流通道 ADC 原始值。 */
	ADC_raw_current =ADCDataDMA[2];
    /* 更新可选模拟油门 ADC 原始值。 */
	ADC_raw_input = ADCDataDMA[0];


/* 采用上一编译条件不成立时的备选实现。 */
#else
/* 更新温度通道 ADC 原始值。 */
ADC_raw_temp =    ADCDataDMA[2];
/* 仅在定义 PA6_VOLTAGE 时编译以下代码。 */
#ifdef PA6_VOLTAGE
/* 更新电压通道 ADC 原始值。 */
ADC_raw_volts  = ADCDataDMA[1];
/* 更新电流通道 ADC 原始值。 */
ADC_raw_current =ADCDataDMA[0];
/* 采用上一编译条件不成立时的备选实现。 */
#else
/* 更新电压通道 ADC 原始值。 */
ADC_raw_volts  = ADCDataDMA[0];
/* 更新电流通道 ADC 原始值。 */
ADC_raw_current =ADCDataDMA[1];
/* 结束当前条件编译分支。 */
#endif
/* 结束当前条件编译分支。 */
#endif
}
/* 保存ADC 校准已完成标志，避免重复校准。 */
char calibration_done=0;

/**
 * @brief 配置普通 ADC 扫描和 DMA 循环搬运；当前序列依次为 PA3、PA6、内部温度通道。
 *
 * DMA[0]=ADC3/PA3，DMA[1]=ADC6/PA6，DMA[2]=ADC16/内部温度。
 * PA6_VOLTAGE 决定前两项解释为电压还是电流；本板 PA3 接 CURRENT、PA6 接 VOLTSENSE。
 * 当前 _540 目标未启用 PA6_VOLTAGE；这里只解释现有配置，不修改通道或系数。
 */
void ADC_Init(void)
{

  /* 保存DMA 源地址、目标地址和搬运格式配置结构。 */
  dma_init_type dma_init_struct;
  /* 配置对应外设时钟使能，保证后续寄存器访问有效。 */
  crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
  /* 设置工程外设中断的优先级并允许该中断。 */
  nvic_irq_enable(DMA1_Channel1_IRQn, 2, 0);
  /* 复位 DMA 通道，清除上次搬运配置。 */
  dma_reset(DMA1_CHANNEL1);
  /* 填充 DMA 配置结构的默认值。 */
  dma_default_para_init(&dma_init_struct);
  /* 更新DMA 搬运项数。 */
  dma_init_struct.buffer_size = 3;
  /* 更新DMA 数据传输方向。 */
  dma_init_struct.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
  /* 更新DMA 内存缓冲区首地址。 */
  dma_init_struct.memory_base_addr = (uint32_t)&ADCDataDMA;
  /* 更新DMA 内存侧单项数据宽度。 */
  dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
  /* 将DMA 内存地址递增使能设为 1。 */
  dma_init_struct.memory_inc_enable = TRUE;
  /* 更新DMA 外设寄存器地址。 */
  dma_init_struct.peripheral_base_addr = (uint32_t)&(ADC1->odt);
  /* 更新DMA 外设侧单项数据宽度。 */
  dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
  /* 将DMA 外设地址递增使能清零。 */
  dma_init_struct.peripheral_inc_enable = FALSE;
  /* 更新DMA 仲裁优先级。 */
  dma_init_struct.priority = DMA_PRIORITY_HIGH;
  /* 将DMA 循环搬运使能设为 1。 */
  dma_init_struct.loop_mode_enable = TRUE;
  /* 将地址、长度、宽度和方向等配置写入 DMA 通道。 */
  dma_init(DMA1_CHANNEL1, &dma_init_struct);

  /* 配置 DMA 完成或错误事件的中断使能。 */
  dma_interrupt_enable(DMA1_CHANNEL1, DMA_FDT_INT, TRUE);
  /* 配置 DMA 通道使能，允许硬件开始搬运。 */
  dma_channel_enable(DMA1_CHANNEL1, TRUE);

  /* 保存ADC 扫描序列和数据对齐配置结构。 */
  adc_base_config_type adc_base_struct;
  /* 配置对应外设时钟使能，保证后续寄存器访问有效。 */
  crm_periph_clock_enable(CRM_ADC1_PERIPH_CLOCK, TRUE);
  /* 设置 ADC 工作时钟分频。 */
  crm_adc_clock_div_set(CRM_ADC_DIV_6);

  /* 填充 ADC 基本配置结构的默认值。 */
  adc_base_default_para_init(&adc_base_struct);
  /* 将ADC 序列扫描使能设为 1。 */
  adc_base_struct.sequence_mode = TRUE;
  /* 将ADC 重复转换使能设为 1。 */
  adc_base_struct.repeat_mode = TRUE;
  /* 更新ADC 转换结果的数据对齐方式。 */
  adc_base_struct.data_align = ADC_RIGHT_ALIGNMENT;
  /* 更新ADC 普通序列的通道数量。 */
  adc_base_struct.ordinary_channel_length = 3;
  /* 应用 ADC 序列扫描及数据格式配置。 */
  adc_base_config(ADC1, &adc_base_struct);
  
	
    /* 指定普通扫描序列中的通道、顺序和采样时间。 */
	adc_ordinary_channel_set(ADC1, ADC_CHANNEL_3, 1, ADC_SAMPLETIME_28_5);
  /* 指定普通扫描序列中的通道、顺序和采样时间。 */
  adc_ordinary_channel_set(ADC1, ADC_CHANNEL_6, 2, ADC_SAMPLETIME_28_5);
  /* 指定普通扫描序列中的通道、顺序和采样时间。 */
  adc_ordinary_channel_set(ADC1, ADC_CHANNEL_16, 3, ADC_SAMPLETIME_28_5);
  
    /* 使能芯片内部温度/参考电压采样电路。 */
	adc_tempersensor_vintrv_enable(TRUE);
    /* 配置普通 ADC 序列的转换触发来源。 */
	adc_ordinary_conversion_trigger_set(ADC1, ADC12_ORDINARY_TRIG_SOFTWARE, TRUE);
  
    /* 使能 ADC 转换结果的 DMA 请求。 */
	adc_dma_mode_enable(ADC1, TRUE);

  /* 配置 ADC 使能状态。 */
  adc_enable(ADC1, TRUE);
    /* 当ADC 校准已完成标志为零时进入此分支。 */
	if(!calibration_done){
        /* 启动 ADC 校准初始化。 */
		adc_calibration_init(ADC1);
        /* 重复检查等待条件：等待 ADC 校准相关硬件状态结束。 */
		while(adc_calibration_init_status_get(ADC1));
        /* 开始 ADC 校准。 */
		adc_calibration_start(ADC1);
        /* 重复检查等待条件：等待 ADC 校准相关硬件状态结束。 */
		while(adc_calibration_status_get(ADC1));
        /* 将ADC 校准已完成标志设为 1。 */
		calibration_done=1;
	}
}



/**
 * @brief 切换为四路 BEMF 诊断采样，DMA 写入 adc_diagnose_data；会重新配置 ADC 和 DMA1 通道 1。
 *
 * 诊断序列来自 PHASE_*_COMP_CHANNEL；与比较器输入选择常量是两套独立定义。
 * 当前 A/C 诊断 ADC 通道与本板实际 A/C 接线相反；CHANNEL_TO_PIN 还存在加一偏移。
 */
void ADC_Init_Detector(void)
{
    /* 切换选定 GPIO 的工作模式；具体驱动电平由随后的置位/清零或 PWM 决定。 */
	gpio_mode_QUICK(GPIOA, GPIO_MODE_ANALOG, GPIO_PULL_NONE, CHANNEL_TO_PIN(PHASE_A_COMP_CHANNEL) );
    /* 切换选定 GPIO 的工作模式；具体驱动电平由随后的置位/清零或 PWM 决定。 */
	gpio_mode_QUICK(GPIOA, GPIO_MODE_ANALOG, GPIO_PULL_NONE, CHANNEL_TO_PIN(PHASE_B_COMP_CHANNEL) );
    /* 切换选定 GPIO 的工作模式；具体驱动电平由随后的置位/清零或 PWM 决定。 */
	gpio_mode_QUICK(GPIOA, GPIO_MODE_ANALOG, GPIO_PULL_NONE, CHANNEL_TO_PIN(PHASE_C_COMP_CHANNEL) );
    /* 切换选定 GPIO 的工作模式；具体驱动电平由随后的置位/清零或 PWM 决定。 */
	gpio_mode_QUICK(GPIOA, GPIO_MODE_ANALOG, GPIO_PULL_NONE, CHANNEL_TO_PIN(PHASE_CC_COMP_CHANNEL) );
    /* 切换选定 GPIO 的工作模式；具体驱动电平由随后的置位/清零或 PWM 决定。 */
	gpio_mode_QUICK(GPIOA, GPIO_MODE_ANALOG, GPIO_PULL_NONE, CHANNEL_TO_PIN(ADC_CHANNEL_3) );
	
  /* 保存DMA 源地址、目标地址和搬运格式配置结构。 */
  dma_init_type dma_init_struct;
  /* 配置对应外设时钟使能，保证后续寄存器访问有效。 */
  crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
  //nvic_irq_enable(DMA1_Channel1_IRQn, 2, 0);
  /* 复位 DMA 通道，清除上次搬运配置。 */
  dma_reset(DMA1_CHANNEL1);
  /* 填充 DMA 配置结构的默认值。 */
  dma_default_para_init(&dma_init_struct);
  /* 更新DMA 搬运项数。 */
  dma_init_struct.buffer_size = 4;
  /* 更新DMA 数据传输方向。 */
  dma_init_struct.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
  /* 更新DMA 内存缓冲区首地址。 */
  dma_init_struct.memory_base_addr = (uint32_t)&adc_diagnose_data;
  /* 更新DMA 内存侧单项数据宽度。 */
  dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
  /* 将DMA 内存地址递增使能设为 1。 */
  dma_init_struct.memory_inc_enable = TRUE;
  /* 更新DMA 外设寄存器地址。 */
  dma_init_struct.peripheral_base_addr = (uint32_t)&(ADC1->odt);
  /* 更新DMA 外设侧单项数据宽度。 */
  dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
  /* 将DMA 外设地址递增使能清零。 */
  dma_init_struct.peripheral_inc_enable = FALSE;
  /* 更新DMA 仲裁优先级。 */
  dma_init_struct.priority = DMA_PRIORITY_HIGH;
  /* 将DMA 循环搬运使能设为 1。 */
  dma_init_struct.loop_mode_enable = TRUE;
  /* 将地址、长度、宽度和方向等配置写入 DMA 通道。 */
  dma_init(DMA1_CHANNEL1, &dma_init_struct);

  //dma_interrupt_enable(DMA1_CHANNEL1, DMA_FDT_INT, TRUE);
  /* 配置 DMA 通道使能，允许硬件开始搬运。 */
  dma_channel_enable(DMA1_CHANNEL1, TRUE);

  /* 保存ADC 扫描序列和数据对齐配置结构。 */
  adc_base_config_type adc_base_struct;
  /* 配置对应外设时钟使能，保证后续寄存器访问有效。 */
  crm_periph_clock_enable(CRM_ADC1_PERIPH_CLOCK, TRUE);
  /* 设置 ADC 工作时钟分频。 */
  crm_adc_clock_div_set(CRM_ADC_DIV_6);

  /* 填充 ADC 基本配置结构的默认值。 */
  adc_base_default_para_init(&adc_base_struct);
  /* 将ADC 序列扫描使能设为 1。 */
  adc_base_struct.sequence_mode = TRUE;
  /* 将ADC 重复转换使能清零。 */
  adc_base_struct.repeat_mode = FALSE;
  /* 更新ADC 转换结果的数据对齐方式。 */
  adc_base_struct.data_align = ADC_RIGHT_ALIGNMENT;
  /* 更新ADC 普通序列的通道数量。 */
  adc_base_struct.ordinary_channel_length = 4;
  /* 应用 ADC 序列扫描及数据格式配置。 */
  adc_base_config(ADC1, &adc_base_struct);
  
	
    /* 指定普通扫描序列中的通道、顺序和采样时间。 */
	adc_ordinary_channel_set(ADC1, PHASE_A_COMP_CHANNEL, 1, ADC_SAMPLETIME_28_5);
  /* 指定普通扫描序列中的通道、顺序和采样时间。 */
  adc_ordinary_channel_set(ADC1, PHASE_B_COMP_CHANNEL, 2, ADC_SAMPLETIME_28_5);
  /* 指定普通扫描序列中的通道、顺序和采样时间。 */
  adc_ordinary_channel_set(ADC1, PHASE_C_COMP_CHANNEL, 3, ADC_SAMPLETIME_28_5);
    /* 指定普通扫描序列中的通道、顺序和采样时间。 */
	adc_ordinary_channel_set(ADC1, PHASE_CC_COMP_CHANNEL, 4, ADC_SAMPLETIME_28_5);
	
    /* 配置普通 ADC 序列的转换触发来源。 */
	adc_ordinary_conversion_trigger_set(ADC1, ADC12_ORDINARY_TRIG_SOFTWARE, TRUE);
  
    /* 使能 ADC 转换结果的 DMA 请求。 */
	adc_dma_mode_enable(ADC1, TRUE);

  /* 配置 ADC 使能状态。 */
  adc_enable(ADC1, TRUE);
	
    /* 当ADC 校准已完成标志为零时进入此分支。 */
	if(!calibration_done){
        /* 启动 ADC 校准初始化。 */
		adc_calibration_init(ADC1);
        /* 重复检查等待条件：等待 ADC 校准相关硬件状态结束。 */
		while(adc_calibration_init_status_get(ADC1));
        /* 开始 ADC 校准。 */
		adc_calibration_start(ADC1);
        /* 重复检查等待条件：等待 ADC 校准相关硬件状态结束。 */
		while(adc_calibration_status_get(ADC1));
        /* 将ADC 校准已完成标志设为 1。 */
		calibration_done=1;
	}
}



/* 结束当前条件编译分支。 */
#endif   // USE_ADC
