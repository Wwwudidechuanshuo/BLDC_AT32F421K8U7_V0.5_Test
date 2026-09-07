/*
 * 内置比较器的浮空相选择、过零 EXINT 边沿配置及中断屏蔽控制。
 * 中文说明按当前实现编写；保留原代码、历史注释和编译条件。
 */
/*
 * comparator.c
 *
 *  Created on: Sep. 26, 2020
 *      Author: Alka
 */

/* 引入 comparator.h：内置比较器的浮空相选择、过零 EXINT 边沿配置及中断屏蔽控制。 */
#include "comparator.h"
/* 引入 targets.h：硬件目标配置：选择 MCU、输入捕获资源、三相驱动引脚、比较器通道和默认参数。 */
#include "targets.h"


/**
 * @brief 屏蔽过零 EXINT 中断并清除挂起标志；比较器本身继续工作，仍可轮询输出。
 */
void maskPhaseInterrupts(){
     /* 按掩码更新EXINT 的外部中断使能寄存器。 */
	 EXINT->inten &= ~EXTI_LINE;
    /* 向目标 EXINT 挂起位写 1 清除，避免重复进入同一次过零中断。 */
	EXINT->intsts = EXTI_LINE;
}

/**
 * @brief 允许过零 EXINT 中断；本函数只置位中断使能，不清除之前的挂起标志。
 */
void enableCompInterrupts(){
    /* 置位EXINT 的外部中断使能寄存器。 */
	EXINT->inten |= EXTI_LINE;
}


/**
 * @brief 按当前换相步骤选择浮空相，并根据相电压预期变化方向设置比较器中断边沿。
 *
 * 对应关系：步骤 1/4 采 C 相，2/5 采 A 相，3/6 采 B 相。
 * 本板 PA1 为中性点正输入；相电压上升时比较器输出下降，下降时输出上升。
 * 切换输入使用整寄存器赋值；若日后增加迟滞或消隐，需要避免在此被覆盖。
 */
void changeCompInput()
{
/* 仅在定义 MCU_AT415 时编译以下代码。 */
#ifdef MCU_AT415
    /* 第 1、4 步 C 相浮空，因此检测 C 相反电动势。 */
	if (step == 1 || step == 4)
	{   // c floating
    /* 切换到当前浮空相的比较器配置；整字赋值会同时覆盖迟滞、消隐和其他控制位。 */
	CMP->ctrlsts1 = PHASE_C_COMP;
	}
    /* 第 2、5 步 A 相浮空，因此检测 A 相反电动势。 */
	if (step == 2 || step == 5)
	{   // a floating	
        /* 切换到当前浮空相的比较器配置；整字赋值会同时覆盖迟滞、消隐和其他控制位。 */
		CMP->ctrlsts1 = PHASE_A_COMP;
	}
    /* 第 3、6 步 B 相浮空，因此检测 B 相反电动势。 */
	if (step == 3 || step == 6)
	{   // b floating
        /* 切换到当前浮空相的比较器配置；整字赋值会同时覆盖迟滞、消隐和其他控制位。 */
		CMP->ctrlsts1 = PHASE_B_COMP;
	}
/* 结束当前条件编译分支。 */
#endif
/* 仅在定义 MCU_AT421 时编译以下代码。 */
#ifdef MCU_AT421	
/* 第 1、4 步 C 相浮空，因此检测 C 相反电动势。 */
if (step == 1 || step == 4)
	{   // c floating
    /* 切换到当前浮空相的比较器配置；整字赋值会同时覆盖迟滞、消隐和其他控制位。 */
	CMP->ctrlsts = PHASE_C_COMP;
	}
    /* 第 2、5 步 A 相浮空，因此检测 A 相反电动势。 */
	if (step == 2 || step == 5)
	{   // a floating	
        /* 切换到当前浮空相的比较器配置；整字赋值会同时覆盖迟滞、消隐和其他控制位。 */
		CMP->ctrlsts = PHASE_A_COMP;
	}
    /* 第 3、6 步 B 相浮空，因此检测 B 相反电动势。 */
	if (step == 3 || step == 6)
	{   // b floating
        /* 切换到当前浮空相的比较器配置；整字赋值会同时覆盖迟滞、消隐和其他控制位。 */
		CMP->ctrlsts = PHASE_B_COMP;
	}
/* 结束当前条件编译分支。 */
#endif	
    /* 当前扇区等待浮空相电压上升过零。 */
	if (rising)
	{
     /* 浮空相电压上升对应比较器输出下降，因此关闭比较器输出上升沿触发。 */
     EXINT->polcfg1 &= ~(uint32_t)EXTI_LINE;
     /* 允许比较器输出下降沿，检测浮空相电压上升穿过中性点。 */
     EXINT->polcfg2 |= (uint32_t)EXTI_LINE;
		
    /* 当前条件不满足时进入备选处理。 */
	}else{
         /* 允许比较器输出上升沿，检测浮空相电压下降穿过中性点。 */
		 EXINT->polcfg1 |= (uint32_t)EXTI_LINE;
     /* 关闭另一方向的边沿触发，避免一次扇区接受两种过零方向。 */
     EXINT->polcfg2 &= ~(uint32_t)EXTI_LINE;
	}


}
