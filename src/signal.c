/*
 * 舵机 PWM 解析、油门端点校准和输入 DMA 完成后的协议分发。
 * 中文说明按当前实现编写；保留原代码、历史注释和编译条件。
 */
/*
 * IO.c
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */
/* 引入 main.h：电调主控制：配置加载、启动、六步换相、过零判断、保护、油门和遥测调度。 */
#include "main.h"
/* 引入 targets.h：硬件目标配置：选择 MCU、输入捕获资源、三相驱动引脚、比较器通道和默认参数。 */
#include "targets.h"
/* 引入 signal.h：舵机 PWM 解析、油门端点校准和输入 DMA 完成后的协议分发。 */
#include "signal.h"
/* 引入 IO.h：输入信号定时器/DMA 的收发切换，以及输入协议自动识别。 */
#include  "IO.h"
/* 引入 dshot.h：DShot 油门帧解码、命令执行及双向遥测的校验与 GCR 波形编码。 */
#include "dshot.h"
/* 引入 serial_telemetry.h：串口遥测组帧、CRC8 校验和 USART1 DMA 发送配置。 */
#include "serial_telemetry.h"
/* 引入 functions.h：GPIO 模式切换、限幅线性映射、绝对差和阻塞延时等工程辅助函数。 */
#include "functions.h"
/* 引入 sounds.h：利用电机绕组和 PWM 播放提示音及 Bluejay 格式旋律；播放会暂时占用驱动定时器。 */
#include "sounds.h"
/* 引入 common.h：模块间共享变量及整数 PID 状态结构；变量实体主要在 main.c 和协议模块中定义。 */
#include "common.h"

/* 保存舵机油门每次更新的最大变化量。 */
int max_servo_deviation = 250;
/* 保存舵机脉宽映射后、尚未限斜率的油门。 */
int servorawinput;

/* 保存稳定高油门触发校准的计数。 */
uint8_t enter_calibration_count = 0;
/* 保存当前正在进行舵机端点校准的标志。 */
uint8_t calibration_required = 0;
/* 保存高油门端点的稳定采样次数。 */
uint8_t high_calibration_counts = 0;
/* 保存高油门端点已校准标志。 */
uint8_t high_calibration_set = 0;
/* 保存用于校准稳定性比较的上一次高端脉宽。 */
uint16_t last_high_threshold = 0;
/* 保存低油门端点的稳定采样次数。 */
uint8_t low_calibration_counts = 0;
/* 保存上一次用于判断校准条件的油门。 */
uint16_t last_input = 0;



/**
 * @brief 保留的短脉宽输入解析：把相邻捕获值之差映射为 0～2000 油门。
 */
void computeMSInput(){

    /* 保存用于计算脉宽的上一个捕获值。 */
	int lastnumber = dma_buffer[0];
    /* 按循环下标遍历本组数据或重复执行指定次数的处理。 */
	for ( int j = 1 ; j < 2; j++){

        /* 检查捕获边沿时间差，只有符合当前协议脉宽或校准区间才继续。 */
		if(((dma_buffer[j] - lastnumber) < 1500) && ((dma_buffer[j] - lastnumber) > 0)){ // blank space

            /* 按当前输入区间限幅并线性映射，得到输入协议刚解码得到的油门或命令数值。 */
			newinput = map((dma_buffer[j] - lastnumber),243,1200, 0, 2000);
            /* 结束当前分支或循环，避免继续处理后续候选项。 */
			break;
		}
        /* 更新用于计算脉宽的上一个捕获值。 */
		lastnumber = dma_buffer[j];
	}
}


/**
 * @brief 解析舵机 PWM 脉宽，处理油门端点校准、单双向映射和每次输入变化限幅。
 */
void computeServoInput(){

        /* 检查捕获边沿时间差，只有符合当前协议脉宽或校准区间才继续。 */
		if(((dma_buffer[1] - dma_buffer[0]) >800 ) && ((dma_buffer[1] - dma_buffer[0]) < 2200)){
        /* 当当前正在进行舵机端点校准的标志非零时进入此分支。 */
		if(calibration_required){
            /* 当高油门端点已校准标志为零时进入此分支。 */
			if(!high_calibration_set){
            /* 当高油门端点的稳定采样次数 等于 0 时进入此分支。 */
			if(high_calibration_counts == 0){
                /* 更新用于校准稳定性比较的上一次高端脉宽。 */
				last_high_threshold = dma_buffer[1] - dma_buffer[0];
			}
            /* 递增高油门端点的稳定采样次数。 */
			high_calibration_counts ++;
            /* 按条件 getAbsDif(last_high_threshold, servo_high_threshold) > 50 选择当前处理路径。 */
			if(getAbsDif(last_high_threshold, servo_high_threshold) > 50){
                /* 将当前正在进行舵机端点校准的标志清零。 */
				calibration_required = 0;
            /* 当前条件不满足时进入备选处理。 */
			}else{
            /* 用旧值 7/8、新值 1/8 做平滑更新，得到舵机 PWM 满油门脉宽端点。 */
			servo_high_threshold = ((7* servo_high_threshold + (dma_buffer[1] - dma_buffer[0])) >> 3);
            /* 当高油门端点的稳定采样次数 大于 50 时进入此分支。 */
			if(high_calibration_counts > 50){
                /* 更新舵机 PWM 满油门脉宽端点。 */
				servo_high_threshold =  servo_high_threshold - 25;
                /* 写入配置区中的舵机高端点编码，稍后由 Flash 保存流程持久化。 */
				eepromBuffer[33] = (servo_high_threshold - 1750)/2;
                /* 将高油门端点已校准标志设为 1。 */
				high_calibration_set = 1;
                /* 播放默认设置提示音，随后恢复电机 PWM 周期。 */
				playDefaultTone();
			}
			}
            /* 更新用于校准稳定性比较的上一次高端脉宽。 */
			last_high_threshold = servo_high_threshold;
			}
            /* 当高油门端点已校准标志非零时进入此分支。 */
			if(high_calibration_set){
                /* 当输入信号边沿捕获时间戳缓冲区中的当前元素 小于 1250 时进入此分支。 */
				if(dma_buffer[1] - dma_buffer[0] < 1250){
                /* 递增低油门端点的稳定采样次数。 */
				low_calibration_counts ++;
                /* 用旧值 7/8、新值 1/8 做平滑更新，得到舵机 PWM 零油门脉宽端点。 */
				servo_low_threshold = ((7*servo_low_threshold + (dma_buffer[1] - dma_buffer[0])) >> 3);
				}
                /* 当低油门端点的稳定采样次数 大于 75 时进入此分支。 */
				if(low_calibration_counts > 75){
                    /* 更新舵机 PWM 零油门脉宽端点。 */
					servo_low_threshold = servo_low_threshold + 25;
                    /* 写入配置区中的舵机低端点编码，稍后由 Flash 保存流程持久化。 */
					eepromBuffer[32] = (servo_low_threshold - 750)/2;
                /* 将当前正在进行舵机端点校准的标志清零。 */
				calibration_required = 0;
                /* 把运行中的方向及模式等字段写回配置缓冲区，再保存整个 176 字节配置到 Flash。 */
				saveEEpromSettings();
                /* 将低油门端点的稳定采样次数清零。 */
				low_calibration_counts = 0;
                /* 播放设置变更提示音，随后恢复电机 PWM 周期。 */
				playChangedTone();
				}
			}
            /* 将自最近有效输入以来的周期计数清零。 */
			signaltimeout = 0;
        /* 当前条件不满足时进入备选处理。 */
		}else{
            /* 当双向油门模式标志非零时进入此分支。 */
			if(bi_direction){
                /* 当输入信号边沿捕获时间戳缓冲区中的当前元素 不大于 双向舵机 PWM 中位脉宽 时进入此分支。 */
				if(dma_buffer[1] - dma_buffer[0] <= servo_neutral){
                /* 按当前输入区间限幅并线性映射，得到舵机脉宽映射后、尚未限斜率的油门。 */
				servorawinput = map((dma_buffer[1] - dma_buffer[0]), servo_low_threshold, servo_neutral, 0, 1000);
                /* 当前条件不满足时进入备选处理。 */
				}else{
                /* 按当前输入区间限幅并线性映射，得到舵机脉宽映射后、尚未限斜率的油门。 */
				servorawinput = map((dma_buffer[1] - dma_buffer[0]), servo_neutral+1, servo_high_threshold, 1001, 2000);
				}
            /* 当前条件不满足时进入备选处理。 */
			}else{
            /* 按当前输入区间限幅并线性映射，得到舵机脉宽映射后、尚未限斜率的油门。 */
			servorawinput = map((dma_buffer[1] - dma_buffer[0]), servo_low_threshold, servo_high_threshold, 0, 2000);
			}
            /* 将自最近有效输入以来的周期计数清零。 */
			signaltimeout = 0;
		}
        /* 当前条件不满足时进入备选处理。 */
		}else{
            /* 将连续检测到零油门的次数清零。 */
			zero_input_count = 0;      // reset if out of range
		}

    /* 当servorawinput - newinput 大于 舵机油门每次更新的最大变化量 时进入此分支。 */
	if (servorawinput - newinput > max_servo_deviation){
        /* 累计增加输入协议刚解码得到的油门或命令数值。 */
		newinput += max_servo_deviation;
    /* 当newinput - servorawinput 大于 舵机油门每次更新的最大变化量 时进入此分支。 */
	}else if(newinput - servorawinput > max_servo_deviation){
        /* 减去本次修正并更新输入协议刚解码得到的油门或命令数值。 */
		newinput -= max_servo_deviation;
    /* 当前条件不满足时进入备选处理。 */
	}else{
        /* 更新输入协议刚解码得到的油门或命令数值。 */
		newinput = servorawinput;
	}

}

/* 以下为保留的旧实现，当前预处理条件为 0，不参与编译。 */
#if 0
/**
 * @brief 处理信号 DMA 一帧完成事件：自动识别协议、解码油门、切换双向遥测收发并检查解锁条件。
 */
void transfercomplete(){
    /* 结合解锁、运行和输入状态判断是否允许执行驱动处理。 */
	if(armed && dshot_telemetry){
        /* 当信号线当前方向非零时进入此分支。 */
	    if(out_put){


        /* 重建输入捕获 DMA 传输，将边沿时间戳写入 dma_buffer 并启动下一次接收。 */
	  	receiveDshotDma();
        /* 结束当前函数，返回调用方。 */
	   	return;
        /* 当前条件不满足时进入备选处理。 */
	    }else{

            /* 把 gcr 波形数组通过 DMA 写入定时器比较寄存器，从信号线上发出双向 DShot 遥测。 */
			sendDshotDma();
            /* 把电周期或扩展遥测值编码成 12 位数据，追加校验、进行 GCR 编码并生成 DMA 输出波形。 */
			make_dshot_package();
            /* 从 32 个边沿时间戳还原 16 位 DShot 帧，验证脉宽和校验码，再更新油门或执行命令。 */
			computeDshotDMA();
        /* 结束当前函数，返回调用方。 */
	    return;
	    }
	}

      /* 当已识别输入信号类型的标志 等于 0 时进入此分支。 */
	  if (inputSet == 0){
         /* 统计捕获脉宽的最小值和平均值，识别 DShot 或舵机 PWM，并选择相应时基及 DMA 长度。 */
	 	 detectInput();
        /* 重建输入捕获 DMA 传输，将边沿时间戳写入 dma_buffer 并启动下一次接收。 */
	 	receiveDshotDma();
     /* 结束当前函数，返回调用方。 */
	 return;
	  }

    /* 当已识别输入信号类型的标志 等于 1 时进入此分支。 */
	if (inputSet == 1){



/* 当双向 DShot 遥测模式标志非零时进入此分支。 */
if(dshot_telemetry){
    /* 当信号线当前方向非零时进入此分支。 */
    if(out_put){
//    	TIM17->CNT = 0;
        /* 把电周期或扩展遥测值编码成 12 位数据，追加校验、进行 GCR 编码并生成 DMA 输出波形。 */
    	make_dshot_package();          // this takes around 10us !!
    /* 从 32 个边沿时间戳还原 16 位 DShot 帧，验证脉宽和校验码，再更新油门或执行命令。 */
  	computeDshotDMA();             //this is slow too..
    /* 重建输入捕获 DMA 传输，将边沿时间戳写入 dma_buffer 并启动下一次接收。 */
  	receiveDshotDma();             //holy smokes.. reverse the line and set up dma again
    /* 结束当前函数，返回调用方。 */
   	return;
    /* 当前条件不满足时进入备选处理。 */
    }else{
        /* 把 gcr 波形数组通过 DMA 写入定时器比较寄存器，从信号线上发出双向 DShot 遥测。 */
		sendDshotDma();
    /* 结束当前函数，返回调用方。 */
    return;
    }
/* 当前条件不满足时进入备选处理。 */
}else{

        /* 当当前输入识别为 DShot 的标志 等于 1 时进入此分支。 */
		if (dshot == 1){
            /* 从 32 个边沿时间戳还原 16 位 DShot 帧，验证脉宽和校验码，再更新油门或执行命令。 */
			computeDshotDMA();
            /* 当待发送串口遥测标志非零时进入此分支。 */
			if(send_telemetry){
            // done in 10khz routine
			}
            /* 重建输入捕获 DMA 传输，将边沿时间戳写入 dma_buffer 并启动下一次接收。 */
			receiveDshotDma();
		}
        /* 当当前输入识别为舵机 PWM 的标志 等于 1 时进入此分支。 */
		if  (servoPwm == 1){
            /* 解析舵机 PWM 脉宽，处理油门端点校准、单双向映射和每次输入变化限幅。 */
			computeServoInput();
		//	LL_TIM_IC_SetPolarity(IC_TIMER_REGISTER, IC_TIMER_CHANNEL, LL_TIM_IC_POLARITY_RISING); // setup rising pin trigger.
      //TIMER_CHCTL2(IC_TIMER_REGISTER) |= (uint32_t)(TIMER_IC_POLARITY_RISING);
		//	IC_TIMER_REGISTER->CTRL2 |= TMR_ICPolarity_Rising;
			  
        /* 更新IC_TIMER_REGISTER 的通道 1 主输出/输入捕获极性。 */
        IC_TIMER_REGISTER->cctrl_bit.c1p = TMR_INPUT_RISING_EDGE;
			
			
        /* 重建输入捕获 DMA 传输，将边沿时间戳写入 dma_buffer 并启动下一次接收。 */
   		receiveDshotDma();
    // 	LL_DMA_EnableIT_HT(DMA1, INPUT_DMA_CHANNEL);
	//		DMA_CHCTL(INPUT_DMA_CHANNEL) |= DMA_INT_HTF;
		//	INPUT_DMA_CHANNEL->CHCTRL |= DMA_INT_HT;
            /* 置位INPUT_DMA_CHANNEL 的DMA 控制寄存器。 */
			INPUT_DMA_CHANNEL->ctrl |= DMA_HDT_INT;
		}

	}
/* 当电调已解锁并允许驱动的标志为零时进入此分支。 */
if(!armed){
     /* 按条件 adjusted_input == 0 && calibration_required == 0 选择当前处理路径。 */
	 if (adjusted_input == 0 && calibration_required == 0){                       // note this in input..not newinput so it will be adjusted be main loop
        /* 递增连续检测到零油门的次数。 */
	 	zero_input_count++;
            /* 当前条件不满足时进入备选处理。 */
	 		}else{
        /* 将连续检测到零油门的次数清零。 */
	 	zero_input_count = 0;
        /* 当经过方向和死区处理的油门值 大于 1500 时进入此分支。 */
	 	if(adjusted_input > 1500){
            /* 按条件 getAbsDif(adjusted_input, last_input) > 50 选择当前处理路径。 */
	 		if(getAbsDif(adjusted_input, last_input) > 50){
                /* 将稳定高油门触发校准的计数清零。 */
	 			enter_calibration_count = 0;
            /* 当前条件不满足时进入备选处理。 */
	 		}else{
                /* 递增稳定高油门触发校准的计数。 */
	 			enter_calibration_count++;
	 		}

            /* 按条件 enter_calibration_count >50 && (!high_calibration_set) 选择当前处理路径。 */
	 		if(enter_calibration_count >50 && (!high_calibration_set)){
                /* 通过扫频鸣音帮助定位电调；循环切换换相步骤并周期性喂狗。 */
				playBeaconTune3();
                /* 将当前正在进行舵机端点校准的标志设为 1。 */
	 			calibration_required = 1;
                /* 将稳定高油门触发校准的计数清零。 */
	 			enter_calibration_count = 0;
	 		}
            /* 更新上一次用于判断校准条件的油门。 */
	 		last_input = adjusted_input;
	 	}
	 	}
	}
	}
}
/* 采用上一编译条件不成立时的备选实现。 */
#else 

/**
 * @brief 处理信号 DMA 一帧完成事件：自动识别协议、解码油门、切换双向遥测收发并检查解锁条件。
 */
void transfercomplete(){
    /* 结合解锁、运行和输入状态判断是否允许执行驱动处理。 */
	if(armed && dshot_telemetry){
        /* 当信号线当前方向非零时进入此分支。 */
	    if(out_put){
                    /* 重建输入捕获 DMA 传输，将边沿时间戳写入 dma_buffer 并启动下一次接收。 */
					receiveDshotDma();
                    /* 结束当前函数，返回调用方。 */
					return;
            /* 当前条件不满足时进入备选处理。 */
			}else{
                    /* 从 32 个边沿时间戳还原 16 位 DShot 帧，验证脉宽和校验码，再更新油门或执行命令。 */
					computeDshotDMA();
                    /* 把电周期或扩展遥测值编码成 12 位数据，追加校验、进行 GCR 编码并生成 DMA 输出波形。 */
					make_dshot_package();
                    /* 把 gcr 波形数组通过 DMA 写入定时器比较寄存器，从信号线上发出双向 DShot 遥测。 */
					sendDshotDma();
                    /* 结束当前函数，返回调用方。 */
					return;
			}
	}

    /* 当已识别输入信号类型的标志 等于 0 时进入此分支。 */
	if (inputSet == 0){
         /* 统计捕获脉宽的最小值和平均值，识别 DShot 或舵机 PWM，并选择相应时基及 DMA 长度。 */
	 	 detectInput();
         /* 重建输入捕获 DMA 传输，将边沿时间戳写入 dma_buffer 并启动下一次接收。 */
	 	 receiveDshotDma();
       /* 结束当前函数，返回调用方。 */
	   return;
	}

    /* 当已识别输入信号类型的标志 等于 1 时进入此分支。 */
	if (inputSet == 1){



/* 当双向 DShot 遥测模式标志非零时进入此分支。 */
if(dshot_telemetry){
    /* 当信号线当前方向非零时进入此分支。 */
    if(out_put){
//    	TIM17->CNT = 0;
       /* 重建输入捕获 DMA 传输，将边沿时间戳写入 dma_buffer 并启动下一次接收。 */
  	   receiveDshotDma();             //holy smokes.. reverse the line and set up dma again
    /* 当前条件不满足时进入备选处理。 */
    }else{
             /* 把电周期或扩展遥测值编码成 12 位数据，追加校验、进行 GCR 编码并生成 DMA 输出波形。 */
			 make_dshot_package();          // this takes around 10us !!
           /* 把 gcr 波形数组通过 DMA 写入定时器比较寄存器，从信号线上发出双向 DShot 遥测。 */
		   sendDshotDma();
             /* 从 32 个边沿时间戳还原 16 位 DShot 帧，验证脉宽和校验码，再更新油门或执行命令。 */
			 computeDshotDMA();             //this is slow too..
    }
/* 当前条件不满足时进入备选处理。 */
}else{
        /* 当当前输入识别为 DShot 的标志 等于 1 时进入此分支。 */
		if (dshot == 1){
            /* 从 32 个边沿时间戳还原 16 位 DShot 帧，验证脉宽和校验码，再更新油门或执行命令。 */
			computeDshotDMA();
            /* 当待发送串口遥测标志非零时进入此分支。 */
			if(send_telemetry){
            // done in 10khz routine
			}
            /* 重建输入捕获 DMA 传输，将边沿时间戳写入 dma_buffer 并启动下一次接收。 */
			receiveDshotDma();
		}
        /* 当当前输入识别为舵机 PWM 的标志 等于 1 时进入此分支。 */
		if  (servoPwm == 1){
            /* 解析舵机 PWM 脉宽，处理油门端点校准、单双向映射和每次输入变化限幅。 */
			computeServoInput();
		//	LL_TIM_IC_SetPolarity(IC_TIMER_REGISTER, IC_TIMER_CHANNEL, LL_TIM_IC_POLARITY_RISING); // setup rising pin trigger.
      //TIMER_CHCTL2(IC_TIMER_REGISTER) |= (uint32_t)(TIMER_IC_POLARITY_RISING);
		//	IC_TIMER_REGISTER->CTRL2 |= TMR_ICPolarity_Rising;
			  
        /* 更新IC_TIMER_REGISTER 的通道 1 主输出/输入捕获极性。 */
        IC_TIMER_REGISTER->cctrl_bit.c1p = TMR_INPUT_RISING_EDGE;
			
			
        /* 重建输入捕获 DMA 传输，将边沿时间戳写入 dma_buffer 并启动下一次接收。 */
   		receiveDshotDma();
    // 	LL_DMA_EnableIT_HT(DMA1, INPUT_DMA_CHANNEL);
	//		DMA_CHCTL(INPUT_DMA_CHANNEL) |= DMA_INT_HTF;
		//	INPUT_DMA_CHANNEL->CHCTRL |= DMA_INT_HT;
            /* 置位INPUT_DMA_CHANNEL 的DMA 控制寄存器。 */
			INPUT_DMA_CHANNEL->ctrl |= DMA_HDT_INT;
		}

	}
/* 当电调已解锁并允许驱动的标志为零时进入此分支。 */
if(!armed){
    /* 当经过方向和死区处理的油门值 小于 0 时进入此分支。 */
	if (adjusted_input < 0){
        /* 将经过方向和死区处理的油门值清零。 */
		adjusted_input = 0;
		}
     /* 按条件 adjusted_input == 0 && calibration_required == 0 选择当前处理路径。 */
	 if (adjusted_input == 0 && calibration_required == 0){                       // note this in input..not newinput so it will be adjusted be main loop
        /* 递增连续检测到零油门的次数。 */
	 	zero_input_count++;
            /* 当前条件不满足时进入备选处理。 */
	 		}else{
        /* 将连续检测到零油门的次数清零。 */
	 	zero_input_count = 0;
        /* 当经过方向和死区处理的油门值 大于 1500 时进入此分支。 */
	 	if(adjusted_input > 1500){
            /* 按条件 getAbsDif(adjusted_input, last_input) > 50 选择当前处理路径。 */
	 		if(getAbsDif(adjusted_input, last_input) > 50){
                /* 将稳定高油门触发校准的计数清零。 */
	 			enter_calibration_count = 0;
            /* 当前条件不满足时进入备选处理。 */
	 		}else{
                /* 递增稳定高油门触发校准的计数。 */
	 			enter_calibration_count++;
	 		}

            /* 按条件 enter_calibration_count >50 && (!high_calibration_set) 选择当前处理路径。 */
	 		if(enter_calibration_count >50 && (!high_calibration_set)){
                /* 通过扫频鸣音帮助定位电调；循环切换换相步骤并周期性喂狗。 */
				playBeaconTune3();
                /* 将当前正在进行舵机端点校准的标志设为 1。 */
	 			calibration_required = 1;
                /* 将稳定高油门触发校准的计数清零。 */
	 			enter_calibration_count = 0;
	 		}
            /* 更新上一次用于判断校准条件的油门。 */
	 		last_input = adjusted_input;
	 	}
	 	}
	}
	}
}

/* 结束当前条件编译分支。 */
#endif
