/*
 * 串口遥测组帧、CRC8 校验和 USART1 DMA 发送配置。
 * 中文说明按当前实现编写；保留原代码、历史注释和编译条件。
 */
/*
 * serial_telemetry.c
 *
 *  Created on: May 13, 2020
 *      Author: Alka
 */


/* 引入 serial_telemetry.h：串口遥测组帧、CRC8 校验和 USART1 DMA 发送配置。 */
#include "serial_telemetry.h"


/* 保存10 字节串口遥测发送缓冲区。 */
uint8_t aTxBuffer[10];
/* 保存串口遥测 DMA 发送字节数。 */
uint8_t nbDataToTransmit = sizeof(aTxBuffer);

/**
 * @brief 装载遥测帧长度并启动串口发送 DMA，完成后的通道清理由中断处理。
 */
void send_telem_DMA(){   // set data length and enable channel to start transfer
      /* 更新DMA1_CHANNEL4 的DMA 剩余传输项数。 */
      DMA1_CHANNEL4->dtcnt = nbDataToTransmit;
        /* 将DMA1_CHANNEL4 的DMA 通道使能设为 1。 */
	    DMA1_CHANNEL4->ctrl_bit.chen = TRUE;
}



/**
 * @brief 用多项式 0x07 对一个数据字节更新 CRC8，返回新的校验值。
 * @param crc 本次参与 CRC 计算的数据字节。
 * @param crc_seed 此前累计得到的 CRC8。
 * @return 更新后的 CRC8。
 */
uint8_t update_crc8(uint8_t crc, uint8_t crc_seed){
/* 保存CRC8 按位迭代的临时结果。 */
uint8_t crc_u, i;
/* 更新CRC8 按位迭代的临时结果。 */
crc_u = crc;
/* 异或更新CRC8 按位迭代的临时结果。 */
crc_u ^= crc_seed;
/* 按字节或按位遍历待处理数据，累计 CRC8。 */
for ( i=0; i<8; i++) crc_u = ( crc_u & 0x80 ) ? 0x7 ^ ( crc_u << 1 ) : ( crc_u << 1 );
/* 返回(crc_u)。 */
return (crc_u);
}

/**
 * @brief 从零初值遍历字节缓冲区，计算并返回整个缓冲区的 CRC8。
 * @param Buf 待校验的字节缓冲区。
 * @param BufLen 参与 CRC8 计算的字节数。
 * @return 整个缓冲区的 CRC8。
 */
uint8_t get_crc8(uint8_t *Buf, uint8_t BufLen){
/* 保存CRC8 累计校验值或当前参与计算的字节。 */
uint8_t crc = 0, i;
/* 按字节或按位遍历待处理数据，累计 CRC8。 */
for( i=0; i<BufLen; i++) crc = update_crc8(Buf[i], crc);
/* 返回(crc)。 */
return (crc);
}


/**
 * @brief 按温度、电压、电流、耗电量和电转速顺序组装 10 字节遥测帧，多字节字段高字节先发。
 * @param temp 温度字段，单位摄氏度。
 * @param voltage 电压字段，当前主程序以 0.01 V 为单位传入。
 * @param current 电流字段，当前主程序以约 0.01 A 为单位传入。
 * @param consumption 累计耗电量字段，单位 mAh。
 * @param e_rpm 电转速字段，当前主程序按实际电转速的 1/100 传入。
 */
void makeTelemPackage(uint8_t temp, uint16_t voltage, uint16_t current, uint16_t consumption, uint16_t e_rpm){

  /* 填入遥测帧的温度。 */
  aTxBuffer[0] = temp; // temperature

  /* 填入遥测帧的电压高字节。 */
  aTxBuffer[1] = (voltage >> 8) & 0xFF; // voltage hB
  /* 填入遥测帧的电压低字节。 */
  aTxBuffer[2] = voltage & 0xFF; // voltage   lowB

  /* 填入遥测帧的电流高字节。 */
  aTxBuffer[3] = (current >> 8) & 0xFF; // current
  /* 填入遥测帧的电流低字节。 */
  aTxBuffer[4] = current & 0xFF; // divide by 10 for Amps

  /* 填入遥测帧的耗电量高字节。 */
  aTxBuffer[5] = (consumption >> 8) & 0xFF; // consumption
  /* 填入遥测帧的耗电量低字节。 */
  aTxBuffer[6] = consumption & 0xFF; //  in mah

  /* 填入遥测帧的电转速高字节。 */
  aTxBuffer[7] = (e_rpm >> 8) & 0xFF; //
  /* 填入遥测帧的电转速低字节。 */
  aTxBuffer[8] = e_rpm & 0xFF; // eRpM *100

  /* 填入遥测帧的前九字节的 CRC8。 */
  aTxBuffer[9] = get_crc8(aTxBuffer,9);
}


/**
 * @brief 初始化 USART1 遥测发送和 DMA；发送引脚为 PB6，具体 DMA 通道随 MCU 适配代码变化。
 */
void telem_UART_Init(void)
{
  /* 保存GPIO 模式、引脚和上下拉配置结构。 */
  gpio_init_type gpio_init_struct;


  /* 配置对应外设时钟使能，保证后续寄存器访问有效。 */
  crm_periph_clock_enable(CRM_USART1_PERIPH_CLOCK, TRUE);
  /* 配置对应外设时钟使能，保证后续寄存器访问有效。 */
  crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    /* 配置对应外设时钟使能，保证后续寄存器访问有效。 */
	crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
  
	  /* configure the usart2 tx pin */
  /* 更新GPIO 输出驱动强度。 */
  gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  /* 更新GPIO 推挽/开漏输出类型。 */
  gpio_init_struct.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
  /* 更新GPIO 工作模式。 */
  gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
  /* 更新待配置的 GPIO 引脚掩码。 */
  gpio_init_struct.gpio_pins = GPIO_PINS_6;
  /* 更新GPIO 上下拉方式。 */
  gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
  /* 把 GPIO 配置结构应用到指定端口。 */
  gpio_init(GPIOB, &gpio_init_struct);
  //gpio_pin_mux_config(GPIOB, GPIO_PINS_SOURCE6, GPIO_MUX_0);

		  
  /* 复位 DMA 通道，清除上次搬运配置。 */
  dma_reset(DMA1_CHANNEL4);
	
    /* 保存DMA 源地址、目标地址和搬运格式配置结构。 */
	dma_init_type dma_init_struct;
  /* 填充 DMA 配置结构的默认值。 */
  dma_default_para_init(&dma_init_struct);
  /* 更新DMA 搬运项数。 */
  dma_init_struct.buffer_size = nbDataToTransmit;
  /* 更新DMA 数据传输方向。 */
  dma_init_struct.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
  /* 更新DMA 内存缓冲区首地址。 */
  dma_init_struct.memory_base_addr = (uint32_t)&aTxBuffer;
  /* 更新DMA 内存侧单项数据宽度。 */
  dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE;
  /* 将DMA 内存地址递增使能设为 1。 */
  dma_init_struct.memory_inc_enable = TRUE;
  /* 更新DMA 外设寄存器地址。 */
  dma_init_struct.peripheral_base_addr = (uint32_t)&USART1->dt;
  /* 更新DMA 外设侧单项数据宽度。 */
  dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
  /* 将DMA 外设地址递增使能清零。 */
  dma_init_struct.peripheral_inc_enable = FALSE;
  /* 更新DMA 仲裁优先级。 */
  dma_init_struct.priority = DMA_PRIORITY_LOW;
  /* 将DMA 循环搬运使能清零。 */
  dma_init_struct.loop_mode_enable = FALSE;
  /* 将地址、长度、宽度和方向等配置写入 DMA 通道。 */
  dma_init(DMA1_CHANNEL4, &dma_init_struct);
		
    /* 置位DMA1_CHANNEL4 的DMA 控制寄存器。 */
	DMA1_CHANNEL4->ctrl |= DMA_FDT_INT;
    /* 置位DMA1_CHANNEL4 的DMA 控制寄存器。 */
	DMA1_CHANNEL4->ctrl |= DMA_DTERR_INT;
	
	
  /* configure usart1 param */
	
    /* 配置 F415 引脚重映射，释放或连接目标外设信号。 */
	gpio_pin_remap_config(USART1_MUX, TRUE);
  /* 设置 USART1 为 115200 波特、8 数据位、1 停止位。 */
  usart_init(USART1, 115200, USART_DATA_8BITS, USART_STOP_1_BIT);
  /* 配置串口发送器使能。 */
  usart_transmitter_enable(USART1, TRUE);
  /* 配置串口接收器使能。 */
  usart_receiver_enable(USART1, TRUE);
  /* 启用串口单线半双工模式，共用发送引脚。 */
  usart_single_line_halfduplex_select(USART1, TRUE);
    /* 允许串口发送空闲事件请求 DMA 搬运。 */
	usart_dma_transmitter_enable(USART1, TRUE);
  /* 配置串口外设使能。 */
  usart_enable(USART1, TRUE);
	
  /* 设置工程外设中断的优先级并允许该中断。 */
  nvic_irq_enable(DMA1_Channel4_IRQn, 3, 0);
	
	
}