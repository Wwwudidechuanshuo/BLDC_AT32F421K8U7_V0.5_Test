/*
 * DShot 油门帧解码、命令执行及双向遥测的校验与 GCR 波形编码。
 * 中文说明按当前实现编写；保留原代码、历史注释和编译条件。
 */
/*
 * dshot.c
 *
 *  Created on: Apr. 22, 2020
 *      Author: Alka
 */

/* 引入 functions.h：GPIO 模式切换、限幅线性映射、绝对差和阻塞延时等工程辅助函数。 */
#include "functions.h"
/* 引入 dshot.h：DShot 油门帧解码、命令执行及双向遥测的校验与 GCR 波形编码。 */
#include "dshot.h"
/* 引入 targets.h：硬件目标配置：选择 MCU、输入捕获资源、三相驱动引脚、比较器通道和默认参数。 */
#include "targets.h"
/* 引入 common.h：模块间共享变量及整数 PID 状态结构；变量实体主要在 main.c 和协议模块中定义。 */
#include "common.h"
/* 引入 main.h：电调主控制：配置加载、启动、六步换相、过零判断、保护、油门和遥测调度。 */
#include "main.h"

/* 保存保留的 16 位 DShot 单位脉冲缓冲区。 */
int dpulse[16] = {0} ;

/* 保存4 位原始数据到 5 位 GCR 符号的查表。 */
const char gcr_encode_table[16] = { 0x19,
		0x1B,
		0x12,
		0x13,
		0x1D,
		0x15,
		0x16,
		0x17,
		0x1A,
		0x09,
		0x0A,
		0x0B,
		0x1E,
		0x0D,
		0x0E,
		0x0F
};

/* 保存遥测电周期压缩编码的指数位移量。 */
int shift_amount = 0;
/* 保存四个 GCR 符号拼成的 20 位编码。 */
uint32_t gcrnumber;
/* 引用其他模块定义的电气旋转一周的估计时间，单位微秒。 */
extern int e_com_time;
/* 引用其他模块定义的过零/换相累计计数；中断模式的换相回调也会递增。 */
extern int zero_crosses;
/* 引用其他模块定义的待发送串口遥测标志。 */
extern char send_telemetry;
/* 引用其他模块定义的换相间隔的滑动平均值，名称沿用旧代码。 */
extern int smoothedinput;
/* 引用其他模块定义的本次允许的占空比最大增加量。 */
extern uint8_t max_duty_cycle_change;
/* 保存当前遥测的压缩电周期或扩展数据。 */
int dshot_full_number;
/* 引用其他模块定义的待播放的设置提示音标志。 */
extern char play_tone_flag;
/* 保存相同 DShot 命令的重复接收次数。 */
uint8_t command_count = 0;
/* 保存用于重复命令确认的上一命令编号。 */
uint8_t last_command = 0;
/* 保存保留的输入空闲高电平计数。 */
uint8_t high_pin_count = 0;
/* 保存双向 DShot 遥测的 26 项定时器比较值序列。 */
uint32_t gcr[26] =  {0};
/* 保存当前 DShot 帧首尾边沿的捕获时间差。 */
uint16_t dshot_frametime;
/* 保存通过校验的 DShot 帧计数。 */
uint16_t dshot_goodcounts;
/* 保存脉宽或校验码不合法的 DShot 帧计数。 */
uint16_t dshot_badcounts;
/* 保存扩展遥测使能及温度/电流/电压轮换状态。 */
char dshot_extended_telemetry = 0;
/* 保存待发送的扩展遥测编码，非零时优先于转速数据。 */
uint16_t send_extended_dshot = 0;

/*inline int inrange(int val,int target,int margin){
	return val>=target-margin && val<=target+margin;
}*/

/**
 * @brief 从 32 个边沿时间戳还原 16 位 DShot 帧，验证脉宽和校验码，再更新油门或执行命令。
 */
void computeDshotDMA(){


    /* 保存捕获缓冲区解析的起始偏移。 */
	const int j = 0;
    /* 更新当前 DShot 帧首尾边沿的捕获时间差。 */
	dshot_frametime = dma_buffer[31]- dma_buffer[0];	
    /* 保存由帧内 15 个间隔估计的单比特时间。 */
	uint32_t pulstime=(dma_buffer[30]- dma_buffer[0])/15;
	/*//uint32_t midtime=pulstime*9/16;
	uint32_t lowtarget=pulstime*6/16;
	uint32_t hightarget=pulstime*12/16;	
	uint32_t margin=pulstime*2/16;*/
	
    /* 使用估算比特周期的 52.5% 作为长短脉冲判定门限。 */
	uint32_t midtime=pulstime*21/40;
    /* 保存DShot 短脉冲的目标宽度。 */
	uint32_t lowtarget=pulstime*7/20;
    /* 保存DShot 长脉冲的目标宽度。 */
	uint32_t hightarget=pulstime*14/20;	
    /* 保存DShot 脉宽允许偏差。 */
	const uint32_t margin=pulstime*3/20;
    /* 保存首位短脉冲兼容处理的放宽偏差。 */
	const uint32_t margin_loose=pulstime*5/20;
	
    /* 保存当前帧脉宽检查是否有效。 */
	int isValid=0;
		
    /* 保存逐位拼接得到的 16 位 DShot 原始帧。 */
	uint32_t dpulsbits=0;
	
/* 条件编译：defined MCU_AT421；条件满足时采用以下实现。 */
#if defined MCU_AT421
            /* F421 捕获时基下，仅接受帧总时间在 2800～3500 计数之间的帧。 */
			if((dshot_frametime < 3500)&&(dshot_frametime > 2800)){
/* 上一编译条件不满足时，检查此替代条件。 */
#elif defined MCU_AT415
            /* F415 捕获时基下，仅接受帧总时间在 3000～5000 计数之间的帧。 */
			if((dshot_frametime < 5000)&&(dshot_frametime > 3000)){
/* 结束当前条件编译分支。 */
#endif
                /* 将当前帧脉宽检查是否有效设为 1。 */
				isValid=1;
                /* 按顺序处理帧内 16 个数据位，恢复接收到的 DShot 数据。 */
				for (int i = 0; i < 16; i++){
                    /* 左移更新逐位拼接得到的 16 位 DShot 原始帧。 */
					dpulsbits<<=1;
                    /* 保存当前数据位的高低边沿时间差。 */
					int timediff=dma_buffer[j + (i<<1) +1] - dma_buffer[j + (i<<1)];
                    /* 验证单比特脉宽是否落入容差范围；下一行对首位短脉冲放宽下限。 */
					if((timediff>=lowtarget - margin && timediff<=hightarget + margin)
                        /* 续接上一行的位组合或条件判断。 */
						||(i==0 && timediff>=lowtarget - margin_loose && timediff<=hightarget + margin)//special process for at32f435 fc which might send shorter signal for the 1 bit
					){
                        /* 当当前数据位的高低边沿时间差 大于 判定 DShot 数据位 0/1 的脉宽门限 时进入此分支。 */
						if(timediff>midtime){
                            /* 置位逐位拼接得到的 16 位 DShot 原始帧。 */
							dpulsbits|=1;
						}
                    /* 当前条件不满足时进入备选处理。 */
					}else{
                        /* 将当前帧脉宽检查是否有效清零。 */
						isValid=0;
                        /* 结束当前分支或循环，避免继续处理后续候选项。 */
						break;
					}
				}
			}


			
            /* 当当前帧脉宽检查是否有效非零时进入此分支。 */
			if(isValid){
                /* 将高 12 位数据的三个半字节异或，得到普通 DShot 的四位校验值。 */
				int calcCRC = ((dpulsbits>>12 )^ (dpulsbits>>8) ^ (dpulsbits>>4))&0xf;
                /* 保存从接收帧末尾取出的四位校验值。 */
				int checkCRC = dpulsbits &0xf;		
				

                /* 当电调已解锁并允许驱动的标志为零时进入此分支。 */
				if(!armed){
                    /* 当双向 DShot 遥测模式标志 等于 0 时进入此分支。 */
					if (dshot_telemetry == 0){
                         /* 以下为保留的旧实现，当前预处理条件为 0，不参与编译。 */
						 #if 0
                         /* 按条件 (INPUT_PIN_PORT->idt & INPUT_PIN) 选择当前处理路径。 */
						 if((INPUT_PIN_PORT->idt & INPUT_PIN)){  // if the pin is high for 100 checks between signal pulses its inverted
                             /* 递增保留的输入空闲高电平计数。 */
							 high_pin_count++;
                             /* 当保留的输入空闲高电平计数 大于 100 时进入此分支。 */
							 if(high_pin_count > 100){
                                 /* 将双向 DShot 遥测模式标志设为 1。 */
								 dshot_telemetry = 1;
							 }
						 }
                         /* 采用上一编译条件不成立时的备选实现。 */
						 #else
                         /* 按四位取反校验关系识别双向 DShot 帧。 */
						 if(calcCRC == ~checkCRC+16){
                            /* 递增保留的输入空闲高电平计数。 */
							high_pin_count++;
                             /* 当保留的输入空闲高电平计数 大于 4 时进入此分支。 */
							 if(high_pin_count > 4){
                                 /* 将双向 DShot 遥测模式标志设为 1。 */
								 dshot_telemetry = 1;
							 }
                         /* 当前条件不满足时进入备选处理。 */
						 }else{
                             /* 将保留的输入空闲高电平计数清零。 */
							 high_pin_count=0;
						 }
                         /* 结束当前条件编译分支。 */
						 #endif
					}
				}
                /* 当双向 DShot 遥测模式标志非零时进入此分支。 */
				if(dshot_telemetry){
                    /* 更新从接收帧末尾取出的四位校验值。 */
					checkCRC= ~checkCRC+16;
				}
                /* 保存从 DShot 帧取出的 11 位油门/命令字段。 */
				int tocheck =	dpulsbits>>5;

                /* 计算校验值与接收帧校验值一致，接受该帧。 */
				if(calcCRC == checkCRC){
                    /* 将自最近有效输入以来的周期计数清零。 */
					signaltimeout = 0;
                    /* 递增通过校验的 DShot 帧计数。 */
					dshot_goodcounts++;
                    /* 接收帧的遥测请求位置位，准备发送遥测。 */
					if(dpulsbits&(1<<4)){
                    /* 将待发送串口遥测标志设为 1。 */
                    send_telemetry=1;
					}
                    /* 当从 DShot 帧取出的 11 位油门/命令字段 大于 47 时进入此分支。 */
					if (tocheck > 47){


                        /* 更新输入协议刚解码得到的油门或命令数值。 */
						newinput = tocheck;
                        /* 将当前待处理的 DShot 命令编号清零。 */
	                    dshotcommand = 0;
                        /* 将相同 DShot 命令的重复接收次数清零。 */
	                    command_count = 0;
                        /* 结束当前函数，返回调用方。 */
	                    return;
					}

                /* 数值 1～47 属于 DShot 命令区，不作为驱动油门。 */
				if ((tocheck <= 47)&& (tocheck > 0)){
                    /* 将输入协议刚解码得到的油门或命令数值清零。 */
					newinput = 0;
                    /* 更新当前待处理的 DShot 命令编号。 */
					dshotcommand = tocheck;    //  todo
				}
                /* 当从 DShot 帧取出的 11 位油门/命令字段 等于 0 时进入此分支。 */
				if (tocheck == 0){
                    /* 将输入协议刚解码得到的油门或命令数值清零。 */
					newinput = 0;
                    /* 将当前待处理的 DShot 命令编号清零。 */
					dshotcommand = 0;
                    /* 将相同 DShot 命令的重复接收次数清零。 */
					command_count = 0;
				}

                /* 只有电调已解锁且电机未运行时，才执行非零 DShot 命令。 */
				if ((dshotcommand > 0) && (running == 0) && armed) {
                    /* 当当前待处理的 DShot 命令编号 不等于 用于重复命令确认的上一命令编号 时进入此分支。 */
					if(dshotcommand != last_command){
                        /* 更新用于重复命令确认的上一命令编号。 */
						last_command = dshotcommand;
                        /* 将相同 DShot 命令的重复接收次数清零。 */
						command_count = 0;
					}
                    /* 当当前待处理的 DShot 命令编号 小于 5 时进入此分支。 */
					if(dshotcommand < 5){    // beacons
                        /* 更新相同 DShot 命令的重复接收次数。 */
						command_count = 6;      //go on right away
					}
                    /* 递增相同 DShot 命令的重复接收次数。 */
					command_count++;
                    /* 当相同 DShot 命令的重复接收次数 不小于 6 时进入此分支。 */
					if(command_count >= 6){
                        /* 将相同 DShot 命令的重复接收次数清零。 */
						command_count = 0;
                        /* 按命令编号、遥测状态或换相步骤分派到对应处理分支。 */
						switch (dshotcommand){                   // todo

                        /* 命令 1：播放第一组提示音。 */
						case 1:
                            /* 播放输入识别提示旋律；临时关闭中断以保持鸣音时序。 */
							playInputTune();
                        /* 结束当前分支或循环，避免继续处理后续候选项。 */
						break;
                        /* 命令 2：播放第二组提示音。 */
						case 2:
                            /* 播放第二组输入提示音，依次改变鸣音分频并恢复桥臂关闭状态。 */
							playInputTune2();
                                /* 结束当前分支或循环，避免继续处理后续候选项。 */
								break;
                        /* 命令 3：播放定位鸣音。 */
						case 3:
                            /* 通过扫频鸣音帮助定位电调；循环切换换相步骤并周期性喂狗。 */
							playBeaconTune3();
                        /* 结束当前分支或循环，避免继续处理后续候选项。 */
						break;
                        /* 命令 7：恢复默认正向设置。 */
						case 7:
                            /* 将保存的电机默认方向反转设置清零。 */
							dir_reversed = 0;
                            /* 按顺序处理帧内 16 个数据位，恢复接收到的 DShot 数据。 */
							forward = 1 - dir_reversed;
                            /* 将待播放的设置提示音标志设为 1。 */
							play_tone_flag = 1;
                            /* 结束当前分支或循环，避免继续处理后续候选项。 */
							break;
                        /* 命令 8：设置默认反向。 */
						case 8:
                                /* 将保存的电机默认方向反转设置设为 1。 */
								dir_reversed = 1;
                                /* 按顺序处理帧内 16 个数据位，恢复接收到的 DShot 数据。 */
								forward = 1 - dir_reversed;
                                /* 更新待播放的设置提示音标志。 */
								play_tone_flag = 2;
                            /* 结束当前分支或循环，避免继续处理后续候选项。 */
							break;
                        /* 命令 9：关闭双向油门并重新等待解锁。 */
						case 9:
                            /* 将双向油门模式标志清零。 */
							bi_direction = 0;
                            /* 将电调已解锁并允许驱动的标志清零。 */
							armed = 0;
                            /* 将连续检测到零油门的次数清零。 */
							zero_input_count = 0;
                            /* 结束当前分支或循环，避免继续处理后续候选项。 */
							break;
                        /* 命令 10：开启双向油门并重新等待解锁。 */
						case 10:
                            /* 将双向油门模式标志设为 1。 */
							bi_direction = 1;
                            /* 将连续检测到零油门的次数清零。 */
							zero_input_count = 0;
                            /* 将电调已解锁并允许驱动的标志清零。 */
							armed = 0;
                            /* 结束当前分支或循环，避免继续处理后续候选项。 */
							break;
                        /* 命令 12：保存当前设置到 Flash。 */
						case 12:
                            /* 把运行中的方向及模式等字段写回配置缓冲区，再保存整个 176 字节配置到 Flash。 */
							saveEEpromSettings();
							//delayMillis(100);
						//	NVIC_SystemReset();
                            /* 结束当前分支或循环，避免继续处理后续候选项。 */
							break;
                        /* 命令 13：启用扩展遥测并准备确认数据。 */
						case 13:
                            /* 将扩展遥测使能及温度/电流/电压轮换状态设为 1。 */
							dshot_extended_telemetry = 1;
                            /* 更新待发送的扩展遥测编码。 */
							send_extended_dshot = 0xE00;
							//make_dshot_package();
                            /* 结束当前分支或循环，避免继续处理后续候选项。 */
							break;
                        /* 命令 14：关闭扩展遥测并准备确认数据。 */
						case 14:
                            /* 将扩展遥测使能及温度/电流/电压轮换状态清零。 */
							dshot_extended_telemetry = 0;
                            /* 更新待发送的扩展遥测编码。 */
							send_extended_dshot = 0xEFF;
							//make_dshot_package();
                            /* 结束当前分支或循环，避免继续处理后续候选项。 */
							break;
                        /* 命令 20：按保存的正常方向运行。 */
						case 20:
                            /* 按顺序处理帧内 16 个数据位，恢复接收到的 DShot 数据。 */
							forward = 1 - dir_reversed;
                            /* 结束当前分支或循环，避免继续处理后续候选项。 */
							break;
                        /* 命令 21：按保存的相反方向运行。 */
						case 21:
                            /* 按顺序处理帧内 16 个数据位，恢复接收到的 DShot 数据。 */
							forward = dir_reversed;
                        /* 结束当前分支或循环，避免继续处理后续候选项。 */
						break;

						}
                        /* 更新最近一次执行的 DShot 命令编号。 */
						last_dshot_command = dshotcommand;
                        /* 将当前待处理的 DShot 命令编号清零。 */
						dshotcommand = 0;
					}
				}
            /* 当前条件不满足时进入备选处理。 */
			}else{
                /* 递增脉宽或校验码不合法的 DShot 帧计数。 */
				dshot_badcounts++;
			}
        /* 当前条件不满足时进入备选处理。 */
		}else{
            /* 递增脉宽或校验码不合法的 DShot 帧计数。 */
			dshot_badcounts++;
		}
}



/* 以下为保留的旧实现，当前预处理条件为 0，不参与编译。 */
#if 0
/**
 * @brief 把电周期或扩展遥测值编码成 12 位数据，追加校验、进行 GCR 编码并生成 DMA 输出波形。
 *
 * 电周期压缩为三位指数加九位尾数；扩展遥测使用相同的数据长度。
 * 每四位查表编码为五位 GCR，20 位 GCR 再通过相邻电平异或生成实际输出序列。
 */
void make_dshot_package(){
/* 当待发送的扩展遥测编码 大于 0 时进入此分支。 */
if(send_extended_dshot > 0){
  /* 更新当前遥测的压缩电周期或扩展数据。 */
  dshot_full_number = send_extended_dshot;
  /* 将待发送的扩展遥测编码清零。 */
  send_extended_dshot = 0;
/* 当前条件不满足时进入备选处理。 */
}else{
  /* 当电机控制流程正在运行的标志为零时进入此分支。 */
  if (!running){
      /* 更新电气旋转一周的估计时间。 */
	  e_com_time = 65535;
  }
//	calculate shift amount for data in format eee mmm mmm mmm, first 1 found in first seven bits of data determines shift amount
// this allows for a range of up to 65408 microseconds which would be shifted 0b111 (eee) or 7 times.
/* 从高位到低位生成遥测编码或输出电平序列。 */
for (int i = 15; i >= 9 ; i--){
    /* 当e_com_time >> i 等于 1 时进入此分支。 */
	if(e_com_time >> i == 1){
        /* 更新遥测电周期压缩编码的指数位移量。 */
		shift_amount = i+1 - 9;
        /* 结束当前分支或循环，避免继续处理后续候选项。 */
		break;
    /* 当前条件不满足时进入备选处理。 */
	}else{
        /* 将遥测电周期压缩编码的指数位移量清零。 */
		shift_amount = 0;
	}
}
// shift the commutation time to allow for expanded range and put shift amount in first three bits
    /* 更新当前遥测的压缩电周期或扩展数据。 */
	dshot_full_number = ((shift_amount << 9) | (e_com_time >> shift_amount));

}
//calculate checksum
    /* 保存发送遥测使用的四位校验值。 */
	uint16_t  csum = 0;
    /* 保存待折叠计算校验的数据副本。 */
	uint16_t csum_data = dshot_full_number;
          /* 从高位到低位生成遥测编码或输出电平序列。 */
		  for (int i = 0; i < 3; i++) {
              /* 异或更新发送遥测使用的四位校验值。 */
		      csum ^=  csum_data;   // xor data by nibbles
              /* 右移更新待折叠计算校验的数据副本。 */
		      csum_data >>= 4;
		  }
          /* 更新发送遥测使用的四位校验值。 */
		  csum = ~csum;       // invert it
          /* 按掩码更新发送遥测使用的四位校验值。 */
		  csum &= 0xf;

          /* 更新当前遥测的压缩电周期或扩展数据。 */
		  dshot_full_number = (dshot_full_number << 4)  | csum; // put checksum at the end of 12 bit dshot number

// GCR RLL encode 16 to 20 bit

          /* 更新四个 GCR 符号拼成的 20 位编码。 */
		  gcrnumber = gcr_encode_table[(dshot_full_number >> 12)] << 15  // first set of four digits
          /* 续接上一行的位组合或条件判断。 */
		  | gcr_encode_table[(((1 << 4) - 1) & (dshot_full_number >> 8))] << 10  // 2nd set of 4 digits
          /* 续接上一行的位组合或条件判断。 */
		  | gcr_encode_table[(((1 << 4) - 1) & (dshot_full_number >> 4))] << 5  //3rd set of four digits
          /* 续接上一行的位组合或条件判断。 */
		  | gcr_encode_table[(((1 << 4) - 1) & (dshot_full_number >> 0))];  //last four digits
//GCR RLL encode 20 to 21bit output

/* 仅在定义 MCU_AT421 时编译以下代码。 */
#ifdef MCU_AT421
          /* 更新双向 DShot 遥测的 26 项定时器比较值序列中的当前元素。 */
		  gcr[1+3] = 78;
          /* 从高位到低位生成遥测编码或输出电平序列。 */
		  for( int i= 19; i >= 0; i--){              // each digit in gcrnumber
              /* 更新双向 DShot 遥测的 26 项定时器比较值序列中的当前元素。 */
			  gcr[3+20-i+1] = ((((gcrnumber &  1 << i )) >> i) ^ (gcr[3+20-i]>>6)) *78;        // exclusive ored with number before it multiplied by 64 to match output timer.
		  }
          /* 将双向 DShot 遥测的 26 项定时器比较值序列中的当前元素清零。 */
          gcr[3] = 0;
/* 结束当前条件编译分支。 */
#endif
/* 仅在定义 MCU_AT415 时编译以下代码。 */
#ifdef MCU_AT415
          /* 更新双向 DShot 遥测的 26 项定时器比较值序列中的当前元素。 */
		  gcr[1+3] = 97;
          /* 从高位到低位生成遥测编码或输出电平序列。 */
		  for( int i= 19; i >= 0; i--){              // each digit in gcrnumber
              /* 更新双向 DShot 遥测的 26 项定时器比较值序列中的当前元素。 */
			  gcr[3+20-i+1] = ((((gcrnumber &  1 << i )) >> i) ^ (gcr[3+20-i]>>6)) *97;        // exclusive ored with number before it multiplied by 64 to match output timer.
		  }
          /* 将双向 DShot 遥测的 26 项定时器比较值序列中的当前元素清零。 */
          gcr[3] = 0;
/* 结束当前条件编译分支。 */
#endif

}
/* 采用上一编译条件不成立时的备选实现。 */
#else




/**
 * @brief 把电周期或扩展遥测值编码成 12 位数据，追加校验、进行 GCR 编码并生成 DMA 输出波形。
 *
 * 电周期压缩为三位指数加九位尾数；扩展遥测使用相同的数据长度。
 * 每四位查表编码为五位 GCR，20 位 GCR 再通过相邻电平异或生成实际输出序列。
 */
void make_dshot_package(){
/* 当待发送的扩展遥测编码 大于 0 时进入此分支。 */
if(send_extended_dshot > 0){
  /* 更新当前遥测的压缩电周期或扩展数据。 */
  dshot_full_number = send_extended_dshot;
  /* 将待发送的扩展遥测编码清零。 */
  send_extended_dshot = 0;
/* 当前条件不满足时进入备选处理。 */
}else{
  /* 当电机控制流程正在运行的标志为零时进入此分支。 */
  if (!running){
      /* 更新电气旋转一周的估计时间。 */
	  e_com_time = 65535;
  }
    /* 当电气旋转一周的估计时间 大于 65535 时进入此分支。 */
	if(e_com_time > 65535){
        /* 更新电气旋转一周的估计时间。 */
		e_com_time = 65535;
	}
//	calculate shift amount for data in format eee mmm mmm mmm, first 1 found in first seven bits of data determines shift amount
// this allows for a range of up to 65408 microseconds which would be shifted 0b111 (eee) or 7 times.


/* 检查遥测电周期的指定位并设置压缩指数，命中后退出外层 do 循环。 */
#define checkbit(i) if(e_com_time & (1<<i)){shift_amount = i+1 - 9;break;}
    /* 建立单次执行块，便于查到首个有效位后用 break 提前结束。 */
	do{
        /* 检查此高位是否为 1，确定压缩指数并在命中时退出查找。 */
		checkbit(15);
        /* 检查此高位是否为 1，确定压缩指数并在命中时退出查找。 */
		checkbit(14);
        /* 检查此高位是否为 1，确定压缩指数并在命中时退出查找。 */
		checkbit(13);
        /* 检查此高位是否为 1，确定压缩指数并在命中时退出查找。 */
		checkbit(12);
        /* 检查此高位是否为 1，确定压缩指数并在命中时退出查找。 */
		checkbit(11);
        /* 检查此高位是否为 1，确定压缩指数并在命中时退出查找。 */
		checkbit(10);
        /* 检查此高位是否为 1，确定压缩指数并在命中时退出查找。 */
		checkbit(9);
        /* 将遥测电周期压缩编码的指数位移量清零。 */
		shift_amount = 0;
    /* 重复检查等待条件：按条件 0 选择当前处理路径。 */
	}while(0);
	
	/*shift_amount = 0;
	for(int i=15;i>=9;--i){
		checkbit(i);
	}*/
	
// shift the commutation time to allow for expanded range and put shift amount in first three bits
    /* 更新当前遥测的压缩电周期或扩展数据。 */
	dshot_full_number = ((shift_amount << 9) | (e_com_time >> shift_amount));

}
//calculate checksum
	
    /* 保存发送遥测使用的四位校验值。 */
	int csum = dshot_full_number ^ (dshot_full_number>>4) ^(dshot_full_number>>8) ^(dshot_full_number>>16);
    /* 更新发送遥测使用的四位校验值。 */
	csum = ~csum;       // invert it
    /* 按掩码更新发送遥测使用的四位校验值。 */
	csum &= 0xf;

// GCR RLL encode 16 to 20 bit
          /* 更新四个 GCR 符号拼成的 20 位编码。 */
		  gcrnumber = gcr_encode_table[(dshot_full_number >> 8)] << 15  // first set of four digits
          /* 续接上一行的位组合或条件判断。 */
		  | gcr_encode_table[(((1 << 4) - 1) & (dshot_full_number >> 4))] << 10  // 2nd set of 4 digits
          /* 续接上一行的位组合或条件判断。 */
		  | gcr_encode_table[(((1 << 4) - 1) & (dshot_full_number >> 0))] << 5  //3rd set of four digits
          /* 续接上一行的位组合或条件判断。 */
		  | gcr_encode_table[csum];  //last four digits
//GCR RLL encode 20 to 21bit output

/* 仅在定义 MCU_AT421 时编译以下代码。 */
#ifdef MCU_AT421
          /* 保存遥测输出的高电平比较值，按 MCU 时基选择。 */
		  const int timerdt=78;
/* 结束当前条件编译分支。 */
#endif
/* 仅在定义 MCU_AT415 时编译以下代码。 */
#ifdef MCU_AT415
        /* 保存遥测输出的高电平比较值，按 MCU 时基选择。 */
		const int timerdt=97;
/* 结束当前条件编译分支。 */
#endif			
            /* 更新双向 DShot 遥测的 26 项定时器比较值序列中的当前元素。 */
			gcr[1+3] = timerdt;
            /* 保存GCR 波形编码的前一个输出电平。 */
			int prev=1;
          /* 从高位到低位生成遥测编码或输出电平序列。 */
		  for(int i= 19; i >= 0; i--){ // each digit in gcrnumber
                /* 更新GCR 波形编码的前一个输出电平。 */
				prev=(int)((gcrnumber>> i) &  1) ^ prev;
              /* 更新双向 DShot 遥测的 26 项定时器比较值序列中的当前元素。 */
			  gcr[3+20-i+1] = prev ? timerdt:0;
		  }
      /* 将双向 DShot 遥测的 26 项定时器比较值序列中的当前元素清零。 */
      gcr[3] = 0;	

}
/* 结束当前条件编译分支。 */
#endif
