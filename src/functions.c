/*
 * GPIO 模式切换、限幅线性映射、绝对差和阻塞延时等工程辅助函数。
 * 中文说明按当前实现编写；保留原代码、历史注释和编译条件。
 */
/*
 * functions.c
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */

/* 引入 functions.h：GPIO 模式切换、限幅线性映射、绝对差和阻塞延时等工程辅助函数。 */
#include "functions.h"
/* 引入 targets.h：硬件目标配置：选择 MCU、输入捕获资源、三相驱动引脚、比较器通道和默认参数。 */
#include "targets.h"

/* 仅在定义 MCU_AT421 时编译以下代码。 */
#ifdef MCU_AT421
/**
 * @brief 设置输入信号引脚的模式和上下拉；F421 直接操作寄存器，F415 使用 GPIO 初始化接口。
 * @param mode 输入、输出、模拟或复用模式枚举。
 * @param pull_up_down 上下拉配置；F421 的 QUICK 实现不使用此参数。
 * @param pin 单个 GPIO 引脚位掩码；F421 快速模式表达式按单引脚设计。
 */
void gpio_mode_set(uint32_t mode, uint32_t pull_up_down, uint32_t pin)
{
 /* 更新INPUT_PIN_PORT 的GPIO 模式配置寄存器。 */
 INPUT_PIN_PORT->cfgr = (((((INPUT_PIN_PORT->cfgr))) & (~(((pin * pin) * (0x3UL << (0U)))))) | (((pin * pin) * mode)));
 /* 更新INPUT_PIN_PORT 的GPIO 上下拉配置寄存器。 */
 INPUT_PIN_PORT->pull = ((((((INPUT_PIN_PORT->pull))) & (~(((pin * pin) * (0x3UL << (0U)))))) | (((pin * pin) * pull_up_down))));
}


/**
 * @brief 快速切换桥臂或信号引脚模式；F421 实现只更新模式寄存器，不处理 pull_up_down 参数。
 *
 * F421 中 pin*pin 将单个引脚掩码变为双位模式字段的偏移；不适用于任意多引脚组合。
 * F415 路径会调用标准库 GPIO 配置；此处只注释工程封装，不修改底层库。
 * @param gpio_periph GPIO 外设寄存器基址。
 * @param mode 输入、输出、模拟或复用模式枚举。
 * @param pull_up_down 上下拉配置；F421 的 QUICK 实现不使用此参数。
 * @param pin 单个 GPIO 引脚位掩码；F421 快速模式表达式按单引脚设计。
 */
void gpio_mode_QUICK(gpio_type* gpio_periph, uint32_t mode, uint32_t pull_up_down, uint32_t pin){
/* 单引脚掩码平方用于定位双位模式字段：先清原模式，再写入新模式；此处不更新上下拉。 */
gpio_periph->cfgr = (((((gpio_periph->cfgr))) & (~(((pin * pin) * (0x3UL << (0U)))))) | (((pin * pin) * mode)));
}
/* 结束当前条件编译分支。 */
#endif

/* 仅在定义 MCU_AT415 时编译以下代码。 */
#ifdef MCU_AT415
/**
 * @brief 设置输入信号引脚的模式和上下拉；F421 直接操作寄存器，F415 使用 GPIO 初始化接口。
 * @param mode 输入、输出、模拟或复用模式枚举。
 * @param pull_up_down 上下拉配置；F421 的 QUICK 实现不使用此参数。
 * @param pin 单个 GPIO 引脚位掩码；F421 快速模式表达式按单引脚设计。
 */
void gpio_mode_set(uint32_t mode, uint32_t pull_up_down, uint32_t pin)
{
	__disable_irq();
	
    /* 保存GPIO 模式、引脚和上下拉配置结构。 */
 	gpio_init_type gpio_init_struct;
    /* 填充 GPIO 配置结构的默认值。 */
	gpio_default_para_init(&gpio_init_struct);
    /* 更新GPIO 工作模式。 */
	gpio_init_struct.gpio_mode = mode;
    /* 更新待配置的 GPIO 引脚掩码。 */
    gpio_init_struct.gpio_pins = pin;
    /* 更新GPIO 上下拉方式。 */
	gpio_init_struct.gpio_pull = pull_up_down;
    /* 把 GPIO 配置结构应用到指定端口。 */
	gpio_init(GPIOB, &gpio_init_struct);
	
	__enable_irq();
}

/**
 * @brief 快速切换桥臂或信号引脚模式；F421 实现只更新模式寄存器，不处理 pull_up_down 参数。
 *
 * F421 中 pin*pin 将单个引脚掩码变为双位模式字段的偏移；不适用于任意多引脚组合。
 * F415 路径会调用标准库 GPIO 配置；此处只注释工程封装，不修改底层库。
 * @param gpio_periph GPIO 外设寄存器基址。
 * @param mode 输入、输出、模拟或复用模式枚举。
 * @param pull_up_down 上下拉配置；F421 的 QUICK 实现不使用此参数。
 * @param pin 单个 GPIO 引脚位掩码；F421 快速模式表达式按单引脚设计。
 */
void gpio_mode_QUICK(gpio_type* gpio_periph, uint32_t mode, uint32_t pull_up_down, uint32_t pin)
{
	__disable_irq();
    /* 保存GPIO 模式、引脚和上下拉配置结构。 */
	gpio_init_type gpio_init_struct;
    /* 填充 GPIO 配置结构的默认值。 */
	gpio_default_para_init(&gpio_init_struct);
	
/* 保留的空判断：当前检查的是模式常量，分支体为空，不改变 GPIO 配置。 */
if (GPIO_MODE_MUX){
}
	
    /* 更新GPIO 工作模式。 */
	gpio_init_struct.gpio_mode = mode;
    /* 更新待配置的 GPIO 引脚掩码。 */
    gpio_init_struct.gpio_pins = pin;
    /* 更新GPIO 上下拉方式。 */
	gpio_init_struct.gpio_pull = pull_up_down;

  /* 把 GPIO 配置结构应用到指定端口。 */
  gpio_init(gpio_periph, &gpio_init_struct);
	
	__enable_irq();
	
}
/* 结束当前条件编译分支。 */
#endif

/**
 * @brief 先将输入限制在给定范围，再作整数线性映射；返回映射结果，要求 in_max 与 in_min 不相等。
 * @param x 待映射输入值。
 * @param in_min 输入范围下限。
 * @param in_max 输入范围上限，必须不同于下限。
 * @param out_min 输出范围起点。
 * @param out_max 输出范围终点，可小于起点以实现反向映射。
 * @return 限幅后的整数映射值。
 */
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
    /* 当待线性映射的输入值 小于 in_min 时进入此分支。 */
	if (x < in_min){
        /* 更新待线性映射的输入值。 */
		x = in_min;
	}
    /* 当待线性映射的输入值 大于 in_max 时进入此分支。 */
	if (x > in_max){
        /* 更新待线性映射的输入值。 */
		x = in_max;
	}
    /* 返回(x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min。 */
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

}


/**
 * @brief 计算两个整数之差的绝对值，返回比较差值；用于输入稳定性和失步检测。
 * @param number1 参与绝对差计算的第一个数。
 * @param number2 参与绝对差计算的第二个数。
 * @return 两数差的绝对值。
 */
int getAbsDif(int number1, int number2){
    /* 保存两个整数的差值及其绝对值结果。 */
	int result = number1 - number2;
    /* 当两个整数的差值及其绝对值结果 小于 0 时进入此分支。 */
	if (result < 0) {
        /* 更新两个整数的差值及其绝对值结果。 */
	    result = -result;
	}
    /* 返回两个整数的差值及其绝对值结果。 */
	return result;
}


/**
 * @brief 使用 UTILITY_TIMER 忙等延时；会清零共享计数器，时间基准取决于当前定时器分频。
 * @param micros 期望延时，单位微秒，依赖辅助计时器当前时基。
 */
void delayMicros(uint32_t micros){
/* 将UTILITY_TIMER 的当前计数值清零。 */
UTILITY_TIMER->cval = 0;

    /* 重复检查等待条件：当UTILITY_TIMER 的当前计数值 小于 micros 时继续循环。 */
	while (UTILITY_TIMER->cval < micros){

	}
}
/**
 * @brief 临时修改 UTILITY_TIMER 分频进行毫秒级忙等，等待期间喂狗，结束后恢复代码指定的分频。
 * @param millis 期望延时，单位毫秒。
 */
void delayMillis(uint32_t millis){
    /* 将UTILITY_TIMER 的当前计数值清零。 */
	UTILITY_TIMER->cval = 0; 
    /* 更新UTILITY_TIMER 的时钟预分频寄存器。 */
	UTILITY_TIMER->div = (CPU_FREQUENCY_MHZ * 100);
    /* 置位UTILITY_TIMER 的软件事件寄存器。 */
	UTILITY_TIMER->swevt |= TMR_OVERFLOW_SWTRIG;
    /* 重复检查等待条件：按条件 UTILITY_TIMER->cval < (millis*10) 选择当前处理路径。 */
	while (UTILITY_TIMER->cval < (millis*10)){
        /* 重载看门狗，表明控制流程仍在正常执行。 */
		WDT->cmd = WDT_CMD_RELOAD;
	}
    /* 更新UTILITY_TIMER 的时钟预分频寄存器。 */
	UTILITY_TIMER->div = CPU_FREQUENCY_MHZ;
    /* 置位UTILITY_TIMER 的软件事件寄存器。 */
	UTILITY_TIMER->swevt |= TMR_OVERFLOW_SWTRIG;
}
