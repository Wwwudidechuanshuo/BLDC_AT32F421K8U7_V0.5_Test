/*
 * 利用电机绕组和 PWM 播放提示音及 Bluejay 格式旋律；播放会暂时占用驱动定时器。
 * 中文说明按当前实现编写；保留原代码、历史注释和编译条件。
 */
/*
 * sounds.c
 *
 *  Created on: May 13, 2020
 *      Author: Alka
 */

/* 引入 sounds.h：利用电机绕组和 PWM 播放提示音及 Bluejay 格式旋律；播放会暂时占用驱动定时器。 */
#include "sounds.h"
/* 引入 phaseouts.h：三相桥臂状态控制，包含六步换相、浮空、低侧导通、PWM 和制动组合。 */
#include "phaseouts.h"
/* 引入 functions.h：GPIO 模式切换、限幅线性映射、绝对差和阻塞延时等工程辅助函数。 */
#include "functions.h"
/* 引入 eeprom.h：使用片内 Flash 保存和读取配置；此处是软件模拟 EEPROM，并非外接 EEPROM。 */
#include "eeprom.h"
/* 引入 targets.h：硬件目标配置：选择 MCU、输入捕获资源、三相驱动引脚、比较器通道和默认参数。 */
#include "targets.h"
/* 引入 common.h：模块间共享变量及整数 PID 状态结构；变量实体主要在 main.c 和协议模块中定义。 */
#include "common.h"


/* 保存提示音 PWM 比较值。 */
uint8_t beep_volume;

//uint8_t blueJayTuneBuffer[128] = {};

/**
 * @brief 暂停电机鸣音指定毫秒数，期间把三个比较值清零，随后恢复当前音量。
 * @param ms 休止时间，单位毫秒。
 */
void pause(uint16_t ms){
    /* 将TMR1 的通道 1 捕获/比较寄存器清零。 */
	TMR1->c1dt = 0; // volume of the beep, (duty cycle) don't go above 25 out of 2000
    /* 将TMR1 的通道 2 捕获/比较寄存器清零。 */
	TMR1->c2dt = 0;
    /* 将TMR1 的通道 3 捕获/比较寄存器清零。 */
	TMR1->c3dt = 0;

    /* 临时修改 UTILITY_TIMER 分频进行毫秒级忙等，等待期间喂狗，结束后恢复代码指定的分频。 */
	delayMillis(ms);
    /* 更新TMR1 的通道 1 捕获/比较寄存器。 */
	TMR1->c1dt = beep_volume; // volume of the beep, (duty cycle) don't go above 25 out of 2000
    /* 更新TMR1 的通道 2 捕获/比较寄存器。 */
	TMR1->c2dt = beep_volume;
    /* 更新TMR1 的通道 3 捕获/比较寄存器。 */
	TMR1->c3dt = beep_volume;
}


/**
 * @brief 将音量限制到代码规定范围并换算为鸣音 PWM 比较值。
 * @param volume 音量设置值，代码上限为 11。
 */
void setVolume(uint8_t volume){
    /* 当鸣音音量设置值 大于 11 时进入此分支。 */
	if(volume > 11){
        /* 更新鸣音音量设置值。 */
		volume = 11;
	}
    /* 当鸣音音量设置值 小于 0 时进入此分支。 */
	if(volume < 0){
        /* 将鸣音音量设置值清零。 */
		volume = 0;
	}
    /* 更新提示音 PWM 比较值。 */
	beep_volume = volume * 2;           // volume variable from 0 - 11 equates to CCR value of 0-22
}

/**
 * @brief 把当前鸣音音量写入 TMR1 三个比较通道。
 */
void setCaptureCompare(){
    /* 更新TMR1 的通道 1 捕获/比较寄存器。 */
	TMR1->c1dt = beep_volume; // volume of the beep, (duty cycle) don't go above 25 out of 2000
    /* 更新TMR1 的通道 2 捕获/比较寄存器。 */
	TMR1->c2dt = beep_volume;
    /* 更新TMR1 的通道 3 捕获/比较寄存器。 */
	TMR1->c3dt = beep_volume;
}

/**
 * @brief 按音符频率设置 PWM 周期和音量，保持指定持续时间；会改变电机 PWM 定时器配置。
 * @param freq 音符频率，单位 Hz，必须非零。
 * @param bduration 音符持续时间，单位毫秒。
 */
void playBJNote(uint16_t freq, uint16_t bduration){        // hz and ms
    /* 保存鸣音所需的 PWM 周期值。 */
	uint16_t timerOne_reload = TIM1_AUTORELOAD;
 
    /* 更新TMR1 的时钟预分频寄存器。 */
	TMR1->div = 10;
    /* 更新鸣音所需的 PWM 周期值。 */
	timerOne_reload = CPU_FREQUENCY_MHZ*100000 / freq-1;
	
    /* 更新TMR1 的周期/自动重装载寄存器。 */
	TMR1->pr = timerOne_reload;
    /* 更新TMR1 的通道 1 捕获/比较寄存器。 */
	TMR1->c1dt = beep_volume * timerOne_reload /TIM1_AUTORELOAD ; // volume of the beep, (duty cycle) don't go above 25 out of 2000
    /* 更新TMR1 的通道 2 捕获/比较寄存器。 */
	TMR1->c2dt = beep_volume * timerOne_reload /TIM1_AUTORELOAD;
    /* 更新TMR1 的通道 3 捕获/比较寄存器。 */
	TMR1->c3dt = beep_volume * timerOne_reload /TIM1_AUTORELOAD;

    /* 临时修改 UTILITY_TIMER 分频进行毫秒级忙等，等待期间喂狗，结束后恢复代码指定的分频。 */
	delayMillis(bduration);
}


/**
 * @brief 将 Bluejay 音符编码换算为频率并返回，使用当前固件的整数换算公式。
 * @param bjarrayfreq Bluejay 音符频率编码。
 * @return 换算后的音符频率，单位 Hz。
 */
uint16_t getBlueJayNoteFrequency(uint32_t bjarrayfreq){
    /* 返回10000000/(bjarrayfreq * 247 + 4000)。 */
	return 10000000/(bjarrayfreq * 247 + 4000);
}

/**
 * @brief 解析成对的时长/音符编码，播放休止符和音符；播放完成后关闭桥臂并恢复 PWM 时基。
 * @param buffer 旋律字节数组，按时长和音符成对读取。
 * @param start 开始读取的字节下标。
 * @param end 结束字节下标，不包含该位置；需保证每对音符数据完整。
 * @param changeComStep 非零时在音符间切换六步换相位置。
 */
void playBlueJayTune(const uint8_t *buffer, int start,int end,int changeComStep){
    /* 保存Bluejay 旋律中累计的延长时长标记数。 */
	uint8_t full_time_count = 0;
    /* 保存当前音符或休止符持续时间，单位毫秒。 */
	uint16_t duration;
    /* 保存由旋律编码换算出的音符频率。 */
	float frequency;
    /* 保存鸣音时当前使用的六步换相位置。 */
	int currentcomstep=1;
    /* 按 1～6 的步骤号执行六步换相，每步包含一相 PWM、一相低侧导通和一相浮空。 */
	comStep(currentcomstep);
	//read_flash_bin(blueJayTuneBuffer , EEPROM_START_ADD + 48 , 128);
    /* 每次读取一对时长/音符数据，直到指定旋律范围结束。 */
	for(int i = start ; i < end ; i+=2){
    /* 重载看门狗，表明控制流程仍在正常执行。 */
	WDT->cmd = WDT_CMD_RELOAD;
        /* 将自最近有效输入以来的周期计数清零。 */
		signaltimeout = 0;

        /* 当buffer中的当前元素 等于 255 时进入此分支。 */
		if(buffer[i] == 255){
            /* 递增Bluejay 旋律中累计的延长时长标记数。 */
			full_time_count++;

        /* 当前条件不满足时进入备选处理。 */
		}else{
            /* 当buffer中的当前元素 等于 0 时进入此分支。 */
			if(buffer[i+1] == 0){
                /* 更新当前音符或休止符持续时间。 */
				duration = full_time_count * 254 + buffer[i];
                /* 将TMR1 的通道 1 捕获/比较寄存器清零。 */
				TMR1->c1dt = 0 ; //
                /* 将TMR1 的通道 2 捕获/比较寄存器清零。 */
				TMR1->c2dt = 0;
                /* 将TMR1 的通道 3 捕获/比较寄存器清零。 */
				TMR1->c3dt = 0;
                /* 临时修改 UTILITY_TIMER 分频进行毫秒级忙等，等待期间喂狗，结束后恢复代码指定的分频。 */
				delayMillis(duration);
            /* 当前条件不满足时进入备选处理。 */
			}else{
                /* 更新由旋律编码换算出的音符频率。 */
				frequency = getBlueJayNoteFrequency(buffer[i+1]);
                /* 更新当前音符或休止符持续时间。 */
				duration= (full_time_count * 254 + buffer[i])  * (float)(1000.f / frequency);
                /* 按音符频率设置 PWM 周期和音量，保持指定持续时间；会改变电机 PWM 定时器配置。 */
				playBJNote(frequency, duration);
                /* 当changeComStep非零时进入此分支。 */
				if(changeComStep){
                    /* 递增鸣音时当前使用的六步换相位置。 */
					currentcomstep++;
                    /* 当鸣音时当前使用的六步换相位置 大于 6 时进入此分支。 */
					if(currentcomstep>6){
                        /* 将鸣音时当前使用的六步换相位置设为 1。 */
						currentcomstep=1;
					}
                    /* 按 1～6 的步骤号执行六步换相，每步包含一相 PWM、一相低侧导通和一相浮空。 */
					comStep(currentcomstep);
				}
			}
            /* 将Bluejay 旋律中累计的延长时长标记数清零。 */
			full_time_count = 0;
		}
	}
    /* 将 A、B、C 三相全部置为浮空，关闭各相高低侧驱动。 */
	allOff();                // turn all channels low again
    /* 将TMR1 的时钟预分频寄存器清零。 */
	TMR1->div = 0;           // set prescaler back to 0.
    /* 更新TMR1 的周期/自动重装载寄存器。 */
	TMR1->pr = TIMER1_MAX_ARR;
    /* 将自最近有效输入以来的周期计数清零。 */
	signaltimeout = 0;
/* 重载看门狗，表明控制流程仍在正常执行。 */
WDT->cmd = WDT_CMD_RELOAD;
}

/**
 * @brief 播放启动提示音；有自定义旋律时读取配置缓冲区，否则播放默认三音提示。
 */
void playStartupTune(){
	__disable_irq();

    /* 保存Flash 中自定义旋律有效性标记。 */
	uint8_t value = *(uint8_t*)(EEPROM_START_ADD+48);
        /* 当Flash 中自定义旋律有效性标记 不等于 0xFF 时进入此分支。 */
		if(value != 0xFF){
            /* 解析成对的时长/音符编码，播放休止符和音符；播放完成后关闭桥臂并恢复 PWM 时基。 */
			playBlueJayTune(eepromBuffer,52,167,0);
        /* 当前条件不满足时进入备选处理。 */
		}else{
    /* 更新TMR1 的周期/自动重装载寄存器。 */
	TMR1->pr = TIM1_AUTORELOAD;
    /* 把当前鸣音音量写入 TMR1 三个比较通道。 */
	setCaptureCompare();
    /* 按 1～6 的步骤号执行六步换相，每步包含一相 PWM、一相低侧导通和一相浮空。 */
	comStep(3);       // activate a pwm channel

    /* 更新TMR1 的时钟预分频寄存器。 */
	TMR1->div = 55;        // frequency of beep
  /* 临时修改 UTILITY_TIMER 分频进行毫秒级忙等，等待期间喂狗，结束后恢复代码指定的分频。 */
  delayMillis(200);         // duration of beep
  /* 按 1～6 的步骤号执行六步换相，每步包含一相 PWM、一相低侧导通和一相浮空。 */
  comStep(5);
  /* 重载看门狗，表明控制流程仍在正常执行。 */
  WDT->cmd = WDT_CMD_RELOAD;
  /* 更新TMR1 的时钟预分频寄存器。 */
  TMR1->div = 40;            // next beep is higher frequency
    /* 临时修改 UTILITY_TIMER 分频进行毫秒级忙等，等待期间喂狗，结束后恢复代码指定的分频。 */
	delayMillis(200);

    /* 按 1～6 的步骤号执行六步换相，每步包含一相 PWM、一相低侧导通和一相浮空。 */
	comStep(6);
    /* 更新TMR1 的时钟预分频寄存器。 */
	TMR1->div = 25;         // higher again..
    /* 临时修改 UTILITY_TIMER 分频进行毫秒级忙等，等待期间喂狗，结束后恢复代码指定的分频。 */
	delayMillis(200);
    /* 将 A、B、C 三相全部置为浮空，关闭各相高低侧驱动。 */
	allOff();                // turn all channels low again
    /* 将TMR1 的时钟预分频寄存器清零。 */
	TMR1->div = 0;           // set prescaler back to 0.
    /* 将自最近有效输入以来的周期计数清零。 */
	signaltimeout = 0;
	}
	
    /* 更新TMR1 的周期/自动重装载寄存器。 */
	TMR1->pr = TIMER1_MAX_ARR;
    /* 重载看门狗，表明控制流程仍在正常执行。 */
	WDT->cmd = WDT_CMD_RELOAD;
	__enable_irq();
}

/**
 * @brief 播放有刷模式启动提示音，结束时关闭桥臂并恢复正常 PWM 周期。
 */
void playBrushedStartupTune(){
	__disable_irq();
    /* 更新TMR1 的周期/自动重装载寄存器。 */
	TMR1->pr = TIM1_AUTORELOAD;
    /* 把当前鸣音音量写入 TMR1 三个比较通道。 */
	setCaptureCompare();
    /* 按 1～6 的步骤号执行六步换相，每步包含一相 PWM、一相低侧导通和一相浮空。 */
	comStep(1);       // activate a pwm channel
    /* 更新TMR1 的时钟预分频寄存器。 */
	TMR1->div = 40;        // frequency of beep
    /* 临时修改 UTILITY_TIMER 分频进行毫秒级忙等，等待期间喂狗，结束后恢复代码指定的分频。 */
    delayMillis(300);         // duration of beep
    /* 按 1～6 的步骤号执行六步换相，每步包含一相 PWM、一相低侧导通和一相浮空。 */
	comStep(2);       // activate a pwm channel
    /* 更新TMR1 的时钟预分频寄存器。 */
	TMR1->div = 30;        // frequency of beep
    /* 临时修改 UTILITY_TIMER 分频进行毫秒级忙等，等待期间喂狗，结束后恢复代码指定的分频。 */
    delayMillis(300);         // duration of beep
    /* 按 1～6 的步骤号执行六步换相，每步包含一相 PWM、一相低侧导通和一相浮空。 */
	comStep(3);       // activate a pwm channel
    /* 更新TMR1 的时钟预分频寄存器。 */
	TMR1->div = 25;        // frequency of beep
    /* 临时修改 UTILITY_TIMER 分频进行毫秒级忙等，等待期间喂狗，结束后恢复代码指定的分频。 */
    delayMillis(300);         // duration of beep
    /* 按 1～6 的步骤号执行六步换相，每步包含一相 PWM、一相低侧导通和一相浮空。 */
    comStep(4);
    /* 更新TMR1 的时钟预分频寄存器。 */
	TMR1->div = 20;         // higher again..
    /* 临时修改 UTILITY_TIMER 分频进行毫秒级忙等，等待期间喂狗，结束后恢复代码指定的分频。 */
	delayMillis(300);
    /* 将 A、B、C 三相全部置为浮空，关闭各相高低侧驱动。 */
	allOff();                // turn all channels low again
    /* 将TMR1 的时钟预分频寄存器清零。 */
	TMR1->div = 0;           // set prescaler back to 0.
    /* 将自最近有效输入以来的周期计数清零。 */
	signaltimeout = 0;
    /* 更新TMR1 的周期/自动重装载寄存器。 */
	TMR1->pr = TIMER1_MAX_ARR;
	__enable_irq();
}

/**
 * @brief 播放预设升降调提示旋律，结束后恢复 PWM 定时器配置。
 */
void playDuskingTune(){
    /* 把当前鸣音音量写入 TMR1 三个比较通道。 */
	setCaptureCompare();
    /* 更新TMR1 的周期/自动重装载寄存器。 */
	TMR1->pr = TIM1_AUTORELOAD;
    /* 按 1～6 的步骤号执行六步换相，每步包含一相 PWM、一相低侧导通和一相浮空。 */
	comStep(2);       // activate a pwm channel
    /* 更新TMR1 的时钟预分频寄存器。 */
	TMR1->div = 60;        // frequency of beep
    /* 临时修改 UTILITY_TIMER 分频进行毫秒级忙等，等待期间喂狗，结束后恢复代码指定的分频。 */
    delayMillis(200);         // duration of beep
    /* 更新TMR1 的时钟预分频寄存器。 */
	TMR1->div = 55;            // next beep is higher frequency
    /* 临时修改 UTILITY_TIMER 分频进行毫秒级忙等，等待期间喂狗，结束后恢复代码指定的分频。 */
	delayMillis(150);
    /* 更新TMR1 的时钟预分频寄存器。 */
	TMR1->div = 50;         // higher again..
    /* 临时修改 UTILITY_TIMER 分频进行毫秒级忙等，等待期间喂狗，结束后恢复代码指定的分频。 */
	delayMillis(150);
    /* 更新TMR1 的时钟预分频寄存器。 */
	TMR1->div = 45;        // frequency of beep
    /* 临时修改 UTILITY_TIMER 分频进行毫秒级忙等，等待期间喂狗，结束后恢复代码指定的分频。 */
    delayMillis(100);         // duration of beep
    /* 更新TMR1 的时钟预分频寄存器。 */
	TMR1->div = 50;            // next beep is higher frequency
    /* 临时修改 UTILITY_TIMER 分频进行毫秒级忙等，等待期间喂狗，结束后恢复代码指定的分频。 */
	delayMillis(100);
    /* 更新TMR1 的时钟预分频寄存器。 */
	TMR1->div = 55;         // higher again..
    /* 临时修改 UTILITY_TIMER 分频进行毫秒级忙等，等待期间喂狗，结束后恢复代码指定的分频。 */
	delayMillis(100);
    /* 更新TMR1 的时钟预分频寄存器。 */
	TMR1->div = 25;         // higher again..
    /* 临时修改 UTILITY_TIMER 分频进行毫秒级忙等，等待期间喂狗，结束后恢复代码指定的分频。 */
	delayMillis(200);
    /* 更新TMR1 的时钟预分频寄存器。 */
	TMR1->div = 55;         // higher again..
    /* 临时修改 UTILITY_TIMER 分频进行毫秒级忙等，等待期间喂狗，结束后恢复代码指定的分频。 */
	delayMillis(150);
    /* 将 A、B、C 三相全部置为浮空，关闭各相高低侧驱动。 */
	allOff();                // turn all channels low again
    /* 将TMR1 的时钟预分频寄存器清零。 */
	TMR1->div = 0;           // set prescaler back to 0.
    /* 更新TMR1 的周期/自动重装载寄存器。 */
	TMR1->pr = TIMER1_MAX_ARR;
}


/**
 * @brief 播放第二组输入提示音，依次改变鸣音分频并恢复桥臂关闭状态。
 */
void playInputTune2(){
    /* 更新TMR1 的周期/自动重装载寄存器。 */
    TMR1->pr = TIM1_AUTORELOAD;
	__disable_irq();
/* 重载看门狗，表明控制流程仍在正常执行。 */
WDT->cmd = WDT_CMD_RELOAD;
    /* 更新TMR1 的时钟预分频寄存器。 */
	TMR1->div = 60;
    /* 把当前鸣音音量写入 TMR1 三个比较通道。 */
	setCaptureCompare();
    /* 按 1～6 的步骤号执行六步换相，每步包含一相 PWM、一相低侧导通和一相浮空。 */
	comStep(1);
    /* 临时修改 UTILITY_TIMER 分频进行毫秒级忙等，等待期间喂狗，结束后恢复代码指定的分频。 */
	delayMillis(75);
    /* 更新TMR1 的时钟预分频寄存器。 */
	TMR1->div = 80;
    /* 临时修改 UTILITY_TIMER 分频进行毫秒级忙等，等待期间喂狗，结束后恢复代码指定的分频。 */
	delayMillis(75);
    /* 更新TMR1 的时钟预分频寄存器。 */
	TMR1->div = 90;
/* 重载看门狗，表明控制流程仍在正常执行。 */
WDT->cmd = WDT_CMD_RELOAD;
    /* 临时修改 UTILITY_TIMER 分频进行毫秒级忙等，等待期间喂狗，结束后恢复代码指定的分频。 */
	delayMillis(75);
    /* 将 A、B、C 三相全部置为浮空，关闭各相高低侧驱动。 */
	allOff();
    /* 将TMR1 的时钟预分频寄存器清零。 */
	TMR1->div = 0;
    /* 将自最近有效输入以来的周期计数清零。 */
	signaltimeout = 0;
    /* 更新TMR1 的周期/自动重装载寄存器。 */
	TMR1->pr = TIMER1_MAX_ARR;
	__enable_irq();
}




/**
 * @brief 播放输入识别提示旋律；临时关闭中断以保持鸣音时序。
 */
void playInputTune(){
	__disable_irq();
    /* 保存当前提示旋律的时长/音符编码数组。 */
	const uint8_t buf[]={0x45,0x2d,0xe0,0,0xa1,0x19};
    /* 解析成对的时长/音符编码，播放休止符和音符；播放完成后关闭桥臂并恢复 PWM 时基。 */
	playBlueJayTune(buf,0,sizeof(buf),0);
		
	/*TMR1->pr = TIM1_AUTORELOAD;
WDT->cmd = WDT_CMD_RELOAD;
	TMR1->div = 80;
	setCaptureCompare();
	comStep(3);
	delayMillis(100);
	TMR1->div = 70;
	delayMillis(100);
	TMR1->div = 40;
	delayMillis(100);
	allOff();
	TMR1->div = 0;
	signaltimeout = 0;
	TMR1->pr = TIMER1_MAX_ARR;*/
	__enable_irq();
}

/**
 * @brief 播放默认设置提示音，随后恢复电机 PWM 周期。
 */
void playDefaultTone(){
    /* 更新TMR1 的周期/自动重装载寄存器。 */
	TMR1->pr = TIM1_AUTORELOAD;
    /* 更新TMR1 的时钟预分频寄存器。 */
	TMR1->div = 50;
    /* 把当前鸣音音量写入 TMR1 三个比较通道。 */
	setCaptureCompare();
    /* 按 1～6 的步骤号执行六步换相，每步包含一相 PWM、一相低侧导通和一相浮空。 */
	comStep(2);
    /* 临时修改 UTILITY_TIMER 分频进行毫秒级忙等，等待期间喂狗，结束后恢复代码指定的分频。 */
	delayMillis(150);
/* 重载看门狗，表明控制流程仍在正常执行。 */
WDT->cmd = WDT_CMD_RELOAD;
    /* 更新TMR1 的时钟预分频寄存器。 */
	TMR1->div = 30;
    /* 临时修改 UTILITY_TIMER 分频进行毫秒级忙等，等待期间喂狗，结束后恢复代码指定的分频。 */
	delayMillis(150);
    /* 将 A、B、C 三相全部置为浮空，关闭各相高低侧驱动。 */
	allOff();
    /* 将TMR1 的时钟预分频寄存器清零。 */
	TMR1->div = 0;
    /* 将自最近有效输入以来的周期计数清零。 */
	signaltimeout = 0;
    /* 更新TMR1 的周期/自动重装载寄存器。 */
	TMR1->pr = TIMER1_MAX_ARR;

}

/**
 * @brief 播放设置变更提示音，随后恢复电机 PWM 周期。
 */
void playChangedTone(){
    /* 更新TMR1 的周期/自动重装载寄存器。 */
	TMR1->pr = TIM1_AUTORELOAD;
    /* 更新TMR1 的时钟预分频寄存器。 */
	TMR1->div = 40;
    /* 把当前鸣音音量写入 TMR1 三个比较通道。 */
	setCaptureCompare();
    /* 按 1～6 的步骤号执行六步换相，每步包含一相 PWM、一相低侧导通和一相浮空。 */
	comStep(2);
    /* 临时修改 UTILITY_TIMER 分频进行毫秒级忙等，等待期间喂狗，结束后恢复代码指定的分频。 */
	delayMillis(150);
/* 重载看门狗，表明控制流程仍在正常执行。 */
WDT->cmd = WDT_CMD_RELOAD;
    /* 更新TMR1 的时钟预分频寄存器。 */
	TMR1->div = 80;
    /* 临时修改 UTILITY_TIMER 分频进行毫秒级忙等，等待期间喂狗，结束后恢复代码指定的分频。 */
	delayMillis(150);
    /* 将 A、B、C 三相全部置为浮空，关闭各相高低侧驱动。 */
	allOff();
    /* 将TMR1 的时钟预分频寄存器清零。 */
	TMR1->div = 0;
    /* 将自最近有效输入以来的周期计数清零。 */
	signaltimeout = 0;
    /* 更新TMR1 的周期/自动重装载寄存器。 */
	TMR1->pr = TIMER1_MAX_ARR;

}


/**
 * @brief 通过扫频鸣音帮助定位电调；循环切换换相步骤并周期性喂狗。
 */
void playBeaconTune3(){
    /* 更新TMR1 的周期/自动重装载寄存器。 */
	TMR1->pr = TIM1_AUTORELOAD;
	__disable_irq();
    /* 把当前鸣音音量写入 TMR1 三个比较通道。 */
	setCaptureCompare();
    /* 逐级改变鸣音分频并切换激励步骤，形成扫频定位音。 */
	for(int i = 119 ; i > 0 ; i = i- 2){
/* 重载看门狗，表明控制流程仍在正常执行。 */
WDT->cmd = WDT_CMD_RELOAD;
        /* 按 1～6 的步骤号执行六步换相，每步包含一相 PWM、一相低侧导通和一相浮空。 */
		comStep(i/20);
        /* 更新TMR1 的时钟预分频寄存器。 */
		TMR1->div = 10+(i / 2);
        /* 临时修改 UTILITY_TIMER 分频进行毫秒级忙等，等待期间喂狗，结束后恢复代码指定的分频。 */
		delayMillis(10);
	}
    /* 将 A、B、C 三相全部置为浮空，关闭各相高低侧驱动。 */
	allOff();
    /* 将TMR1 的时钟预分频寄存器清零。 */
	TMR1->div = 0;
    /* 将自最近有效输入以来的周期计数清零。 */
	signaltimeout = 0;
    /* 更新TMR1 的周期/自动重装载寄存器。 */
	TMR1->pr = TIMER1_MAX_ARR;
	__enable_irq();
}
