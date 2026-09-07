/*
 * 输入信号定时器/DMA 的收发切换，以及输入协议自动识别。
 * 中文说明按当前实现编写；保留原代码、历史注释和编译条件。
 */
/*
 * IO.c
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */

/* 引入 targets.h：硬件目标配置：选择 MCU、输入捕获资源、三相驱动引脚、比较器通道和默认参数。 */
#include "targets.h"
/* 引入 IO.h：输入信号定时器/DMA 的收发切换，以及输入协议自动识别。 */
#include "IO.h"
/* 引入 dshot.h：DShot 油门帧解码、命令执行及双向遥测的校验与 GCR 波形编码。 */
#include "dshot.h"
/* 引入 serial_telemetry.h：串口遥测组帧、CRC8 校验和 USART1 DMA 发送配置。 */
#include "serial_telemetry.h"
/* 引入 functions.h：GPIO 模式切换、限幅线性映射、绝对差和阻塞延时等工程辅助函数。 */
#include "functions.h"
/* 引入 common.h：模块间共享变量及整数 PID 状态结构；变量实体主要在 main.c 和协议模块中定义。 */
#include "common.h"

/* 保存输入捕获定时器分频值。 */
char ic_timer_prescaler = 16;
/* 保存双向遥测输出定时器分频值。 */
char output_timer_prescaler;
/* 保存当前输入 DMA 一帧搬运长度。 */
int buffersize = 32;

/* 保存输入信号边沿捕获时间戳缓冲区。 */
uint32_t dma_buffer[64] = {0};
/* 保存信号线当前方向，1 为发送遥测，0 为接收输入。 */
char out_put = 0;
/* 保存保留的输入缓冲区解码尺度参数。 */
char buffer_divider = 44;
/* 保存保留的 DShot 帧超时参数。 */
int dshot_runout_timer = 62500;

/**
 * @brief 把信号定时器切换为遥测 PWM 输出模式，设置输出时基并标记当前处于发送阶段。
 */
void changeToOutput(){
    /* 置位INPUT_DMA_CHANNEL 的DMA 控制寄存器。 */
	INPUT_DMA_CHANNEL->ctrl |= DMA_DIR_MEMORY_TO_PERIPHERAL;
       /* 复位信号定时器，重新建立捕获或输出配置。 */
	   tmr_reset(IC_TIMER_REGISTER);
      /* 更新IC_TIMER_REGISTER 的通道 1/2 模式寄存器。 */
	  IC_TIMER_REGISTER->cm1 = 0x60;            // oc mode pwm
    /* 更新IC_TIMER_REGISTER 的通道使能和极性寄存器。 */
	IC_TIMER_REGISTER->cctrl = 0x3;         // 
    /* 更新IC_TIMER_REGISTER 的时钟预分频寄存器。 */
	IC_TIMER_REGISTER->div = output_timer_prescaler;
    /* 更新IC_TIMER_REGISTER 的周期/自动重装载寄存器。 */
	IC_TIMER_REGISTER->pr = DSHOT_PRE;
	
      /* 将信号线当前方向设为 1。 */
	  out_put = 1;
    /* 将IC_TIMER_REGISTER 的软件触发定时器更新事件设为 1。 */
	IC_TIMER_REGISTER->swevt_bit.ovfswtr = TRUE;
}

/**
 * @brief 把信号定时器切换为输入捕获模式，恢复接收时基并标记当前处于接收阶段。
 */
void changeToInput(){
/* 仅在定义 MCU_AT415 时编译以下代码。 */
#ifdef MCU_AT415
/* 切换选定 GPIO 的工作模式；具体驱动电平由随后的置位/清零或 PWM 决定。 */
gpio_mode_QUICK(GPIOB, GPIO_MODE_INPUT, GPIO_PULL_NONE, INPUT_PIN);
/* 结束当前条件编译分支。 */
#endif
    /* 置位INPUT_DMA_CHANNEL 的DMA 控制寄存器。 */
	INPUT_DMA_CHANNEL->ctrl |= DMA_DIR_PERIPHERAL_TO_MEMORY;

    /* 复位信号定时器，重新建立捕获或输出配置。 */
	tmr_reset(IC_TIMER_REGISTER);
    /* 更新IC_TIMER_REGISTER 的通道 1/2 模式寄存器。 */
	IC_TIMER_REGISTER->cm1 = 0x1;
    /* 更新IC_TIMER_REGISTER 的通道使能和极性寄存器。 */
	IC_TIMER_REGISTER->cctrl = 0xB;
    /* 更新IC_TIMER_REGISTER 的时钟预分频寄存器。 */
	IC_TIMER_REGISTER->div = ic_timer_prescaler;
    /* 更新IC_TIMER_REGISTER 的周期/自动重装载寄存器。 */
	IC_TIMER_REGISTER->pr = 0xFFFF;
	
      /* 将IC_TIMER_REGISTER 的软件触发定时器更新事件设为 1。 */
	  IC_TIMER_REGISTER->swevt_bit.ovfswtr = TRUE;
      /* 将信号线当前方向清零。 */
	  out_put = 0;

}
/**
 * @brief 重建输入捕获 DMA 传输，将边沿时间戳写入 dma_buffer 并启动下一次接收。
 */
void receiveDshotDma(){

    /* 把信号定时器切换为输入捕获模式，恢复接收时基并标记当前处于接收阶段。 */
	changeToInput();
    /* 将IC_TIMER_REGISTER 的当前计数值清零。 */
	IC_TIMER_REGISTER->cval = 0;
    /* 更新INPUT_DMA_CHANNEL 的DMA 外设地址。 */
	INPUT_DMA_CHANNEL->paddr = (uint32_t)&IC_TIMER_REGISTER->c1dt;
    /* 更新INPUT_DMA_CHANNEL 的DMA 内存地址。 */
	INPUT_DMA_CHANNEL->maddr = (uint32_t)&dma_buffer;
    /* 更新INPUT_DMA_CHANNEL 的DMA 剩余传输项数。 */
	INPUT_DMA_CHANNEL->dtcnt = buffersize;
    /* 置位IC_TIMER_REGISTER 的中断/DMA 请求使能寄存器。 */
	IC_TIMER_REGISTER->iden |= TMR_C1_DMA_REQUEST;
    /* 将IC_TIMER_REGISTER 的定时器计数使能设为 1。 */
	IC_TIMER_REGISTER->ctrl1_bit.tmren = TRUE;
    /* 应用捕获 DMA 配置并置位通道使能，开始把捕获值搬到 RAM。 */
	INPUT_DMA_CHANNEL->ctrl = 0x0000098b;

}

/**
 * @brief 把 gcr 波形数组通过 DMA 写入定时器比较寄存器，从信号线上发出双向 DShot 遥测。
 */
void sendDshotDma(){

    /* 把信号定时器切换为遥测 PWM 输出模式，设置输出时基并标记当前处于发送阶段。 */
	changeToOutput();
	
    /* 更新INPUT_DMA_CHANNEL 的DMA 外设地址。 */
	INPUT_DMA_CHANNEL->paddr = (uint32_t)&IC_TIMER_REGISTER->c1dt;
    /* 更新INPUT_DMA_CHANNEL 的DMA 内存地址。 */
	INPUT_DMA_CHANNEL->maddr = (uint32_t)&gcr;
    /* 更新INPUT_DMA_CHANNEL 的DMA 剩余传输项数。 */
	INPUT_DMA_CHANNEL->dtcnt = 26;
    /* 置位INPUT_DMA_CHANNEL 的DMA 控制寄存器。 */
	INPUT_DMA_CHANNEL->ctrl |= DMA_FDT_INT;
    /* 置位INPUT_DMA_CHANNEL 的DMA 控制寄存器。 */
	INPUT_DMA_CHANNEL->ctrl |= DMA_DTERR_INT;
    /* 将INPUT_DMA_CHANNEL 的DMA 通道使能设为 1。 */
	INPUT_DMA_CHANNEL->ctrl_bit.chen = TRUE;
    /* 置位IC_TIMER_REGISTER 的中断/DMA 请求使能寄存器。 */
	IC_TIMER_REGISTER->iden |= TMR_C1_DMA_REQUEST;
    /* 将IC_TIMER_REGISTER 的高级定时器主输出使能设为 1。 */
	IC_TIMER_REGISTER->brk_bit.oen = TRUE;
    /* 将IC_TIMER_REGISTER 的定时器计数使能设为 1。 */
	IC_TIMER_REGISTER->ctrl1_bit.tmren = TRUE;
/* 仅在定义 MCU_AT415 时编译以下代码。 */
#ifdef MCU_AT415
/* 切换选定 GPIO 的工作模式；具体驱动电平由随后的置位/清零或 PWM 决定。 */
gpio_mode_QUICK(INPUT_PIN_PORT, GPIO_MODE_MUX, GPIO_PULL_NONE, INPUT_PIN);
/* 结束当前条件编译分支。 */
#endif
}


/**
 * @brief 统计捕获脉宽的最小值和平均值，识别 DShot 或舵机 PWM，并选择相应时基及 DMA 长度。
 */
void detectInput(){
    /* 保存有效输入脉宽中的最小值。 */
	int smallestnumber = 0;
    /* 保存有效输入脉宽的平均值。 */
	int average_signal_pulse;

    /* 更新有效输入脉宽中的最小值。 */
	smallestnumber = 20000;
    /* 将有效输入脉宽的平均值清零。 */
	average_signal_pulse = 0;
    /* 将当前输入识别为 DShot 的标志清零。 */
	dshot = 0;
    /* 将当前输入识别为舵机 PWM 的标志清零。 */
	servoPwm = 0;


    /* 保存参与脉宽平均的有效样本数。 */
	int sigcount=0;
    /* 逐对统计 16 组输入边沿的有效脉宽。 */
	for ( int j = 0 ; j < 16; j++){
        /* 当输入信号边沿捕获时间戳缓冲区中的当前元素 大于 输入信号边沿捕获时间戳缓冲区中的当前元素 时进入此分支。 */
		if(dma_buffer[j*2+1] > dma_buffer[j*2]  ){
            /* 保存相邻上升/下降沿间的捕获差值。 */
			int difft=dma_buffer[j*2+1] - dma_buffer[j*2];
            /* 当相邻上升/下降沿间的捕获差值 小于 有效输入脉宽中的最小值 时进入此分支。 */
			if(difft < smallestnumber){
                    /* 更新有效输入脉宽中的最小值。 */
					smallestnumber = dma_buffer[j*2+1] - dma_buffer[j*2];
			}
            /* 累计增加有效输入脉宽的平均值。 */
			average_signal_pulse += difft;
            /* 递增参与脉宽平均的有效样本数。 */
			++sigcount;
		}
	}
    /* 当参与脉宽平均的有效样本数 不大于 0 时进入此分支。 */
	if(sigcount<=0)sigcount=1;
    /* 更新有效输入脉宽的平均值。 */
	average_signal_pulse = average_signal_pulse/sigcount ;
/* 仅在定义 MCU_AT415 时编译以下代码。 */
#ifdef MCU_AT415
    /* 按条件 (smallestnumber > 1)&&(smallestnumber <= 5)&& (average_signal_pulse < 70) 选择当前处理路径。 */
	if ((smallestnumber > 1)&&(smallestnumber <= 5)&& (average_signal_pulse < 70)) {
/* 结束当前条件编译分支。 */
#endif			
/* 仅在定义 MCU_AT421 时编译以下代码。 */
#ifdef MCU_AT421		
    /* 按条件 (smallestnumber > 1)&&(smallestnumber <= 4)&& (average_signal_pulse < 60) 选择当前处理路径。 */
	if ((smallestnumber > 1)&&(smallestnumber <= 4)&& (average_signal_pulse < 60)) {
/* 结束当前条件编译分支。 */
#endif		
        /* 将输入捕获定时器分频值清零。 */
		ic_timer_prescaler= 0;
        /* 将双向遥测输出定时器分频值设为 1。 */
		output_timer_prescaler=1;
        /* 将当前输入识别为 DShot 的标志设为 1。 */
		dshot = 1;
        /* 更新保留的输入缓冲区解码尺度参数。 */
		buffer_divider = 44;
        /* 更新保留的 DShot 帧超时参数。 */
		dshot_runout_timer = 65000;
        /* 更新输入协议初始化时设置的解锁计数参数。 */
		armed_count_threshold = 10000;
        /* 更新当前输入 DMA 一帧搬运长度。 */
		buffersize = 32;
	}
/* 仅在定义 MCU_AT415 时编译以下代码。 */
#ifdef MCU_AT415
    /* 按条件 (smallestnumber > 5 )&&(smallestnumber <= 10)&& (average_signal_pulse < 100) 选择当前处理路径。 */
    if ((smallestnumber > 5 )&&(smallestnumber <= 10)&& (average_signal_pulse < 100)){
/* 结束当前条件编译分支。 */
#endif	
/* 仅在定义 MCU_AT421 时编译以下代码。 */
#ifdef MCU_AT421
    /* 按条件 (smallestnumber > 4 )&&(smallestnumber <= 8)&& (average_signal_pulse < 100) 选择当前处理路径。 */
	if ((smallestnumber > 4 )&&(smallestnumber <= 8)&& (average_signal_pulse < 100)){
/* 结束当前条件编译分支。 */
#endif		
        /* 将当前输入识别为 DShot 的标志设为 1。 */
		dshot = 1;
        /* 将输入捕获定时器分频值设为 1。 */
		ic_timer_prescaler=1;
        /* 更新双向遥测输出定时器分频值。 */
		output_timer_prescaler=3;
        /* 更新IC_TIMER_REGISTER 的当前计数值。 */
		IC_TIMER_REGISTER->cval = 0xffff;
        /* 更新保留的输入缓冲区解码尺度参数。 */
		buffer_divider = 44;
        /* 更新保留的 DShot 帧超时参数。 */
		dshot_runout_timer = 65000;
        /* 更新输入协议初始化时设置的解锁计数参数。 */
		armed_count_threshold = 10000;
        /* 更新当前输入 DMA 一帧搬运长度。 */
		buffersize = 32;
	}
//	if ((smallestnumber > 100 )&&(smallestnumber < 400)){
//		multishot = 1;
//		armed_count_threshold = 1000;
//		buffersize = 4;
//	}
//	if ((smallestnumber > 2000 )&&(smallestnumber < 3000)){
//		oneshot42 = 1;
//	}
        /* 按条件 smallestnumber > 30 && smallestnumber < 20000 选择当前处理路径。 */
		if (smallestnumber > 30 && smallestnumber < 20000){
            /* 按条件 (average_signal_pulse-smallestnumber)<smallestnumber/10 选择当前处理路径。 */
			if((average_signal_pulse-smallestnumber)<smallestnumber/10){
                /* 将当前输入识别为舵机 PWM 的标志设为 1。 */
				servoPwm = 1;
                /* 更新输入捕获定时器分频值。 */
				ic_timer_prescaler=119;
                /* 更新输入协议初始化时设置的解锁计数参数。 */
				armed_count_threshold = 35;
                /* 更新当前输入 DMA 一帧搬运长度。 */
				buffersize = 2;
			}
		}

    /* 按条件 dshot||servoPwm 选择当前处理路径。 */
	if(dshot||servoPwm){
        /* 将已识别输入信号类型的标志设为 1。 */
		inputSet = 1;
    /* 当前条件不满足时进入备选处理。 */
	}else{
        /* 将已识别输入信号类型的标志清零。 */
		inputSet = 0;
	}
	/*if (smallestnumber == 0 || smallestnumber == 20000){
		inputSet = 0;
	}else{
		inputSet = 1;
	}*/
    /* 更新UTILITY_TIMER 的通道 1 捕获/比较寄存器。 */
	UTILITY_TIMER->c1dt = smallestnumber;
}
























