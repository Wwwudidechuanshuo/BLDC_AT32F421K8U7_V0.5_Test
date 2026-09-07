/*
 * 三相桥臂状态控制，包含六步换相、浮空、低侧导通、PWM 和制动组合。
 * 中文说明按当前实现编写；保留原代码、历史注释和编译条件。
 */
/*
 * phaseouts.c
 *
 *  Created on: Apr 22, 2020
 *      Author: Alka
 */
/* 引入 phaseouts.h：三相桥臂状态控制，包含六步换相、浮空、低侧导通、PWM 和制动组合。 */
#include "phaseouts.h"
/* 引入 targets.h：硬件目标配置：选择 MCU、输入捕获资源、三相驱动引脚、比较器通道和默认参数。 */
#include "targets.h"
/* 引入 functions.h：GPIO 模式切换、限幅线性映射、绝对差和阻塞延时等工程辅助函数。 */
#include "functions.h"

/* 引用其他模块定义的互补 PWM 使能标志。 */
extern char comp_pwm;
/* 引用其他模块定义的比例制动当前生效标志。 */
extern char prop_brake_active;

/* 仅在未定义 PWM_ENABLE_BRIDGE 时编译以下代码。 */
#ifndef PWM_ENABLE_BRIDGE

/* 仅在定义 USE_INVERTED_LOW 时编译以下代码。 */
#ifdef USE_INVERTED_LOW
    /* 编译时输出提示，便于核对所选驱动极性。 */
	#pragma message ("using inverted low side output")
    /* 按驱动有效极性选择 GPIO 置位/清零寄存器别名。 */
	#define LOW_BITREG_ON  clr
    /* 按驱动有效极性选择 GPIO 置位/清零寄存器别名。 */
	#define LOW_BITREG_OFF scr
/* 采用上一编译条件不成立时的备选实现。 */
#else
    /* 按驱动有效极性选择 GPIO 置位/清零寄存器别名。 */
	#define LOW_BITREG_ON  scr
    /* 按驱动有效极性选择 GPIO 置位/清零寄存器别名。 */
	#define LOW_BITREG_OFF clr
/* 结束当前条件编译分支。 */
#endif

/* 仅在定义 USE_INVERTED_HIGH 时编译以下代码。 */
#ifdef USE_INVERTED_HIGH
    /* 编译时输出提示，便于核对所选驱动极性。 */
	#pragma message ("using inverted high side output")
    /* 按驱动有效极性选择 GPIO 置位/清零寄存器别名。 */
	#define HIGH_BITREG_ON  clr
    /* 按驱动有效极性选择 GPIO 置位/清零寄存器别名。 */
	#define HIGH_BITREG_OFF scr
/* 采用上一编译条件不成立时的备选实现。 */
#else
    /* 按驱动有效极性选择 GPIO 置位/清零寄存器别名。 */
	#define HIGH_BITREG_ON  scr
    /* 按驱动有效极性选择 GPIO 置位/清零寄存器别名。 */
	#define HIGH_BITREG_OFF clr
/* 结束当前条件编译分支。 */
#endif

//void gpio_mode_QUICK(GPIO_Type* gpio_periph, uint32_t mode, uint32_t pull_up_down, uint32_t pin){
//gpio_periph->MODER = (((((gpio_periph->MODER))) & (~(((pin * pin) * (0x3UL << (0U)))))) | (((pin * pin) * mode)));
//}


/**
 * @brief 关闭高侧驱动并把低侧切到 PWM 复用模式，用占空比控制制动力。
 */
void proportionalBrake(){  // alternate all channels between braking (ABC LOW) and coasting (ABC float)
	                        // put lower channel into alternate mode and turn upper OFF for each channel
	// turn all HIGH channels off for ABC

//	gpio_mode_QUICK(PHASE_A_GPIO_PORT_HIGH, GPIO_MODE_OUTPUT, GPIO_PULL_NONE, PHASE_A_GPIO_HIGH);
 /* 切换选定 GPIO 的工作模式；具体驱动电平由随后的置位/清零或 PWM 决定。 */
 gpio_mode_QUICK(PHASE_A_GPIO_PORT_HIGH, GPIO_MODE_OUTPUT, GPIO_PULL_NONE, PHASE_A_GPIO_HIGH);
 /* 按目标硬件的有效极性关闭当前高侧或低侧驱动。 */
 PHASE_A_GPIO_PORT_HIGH->HIGH_BITREG_OFF = PHASE_A_GPIO_HIGH;

    /* 切换选定 GPIO 的工作模式；具体驱动电平由随后的置位/清零或 PWM 决定。 */
	gpio_mode_QUICK(PHASE_B_GPIO_PORT_HIGH, GPIO_MODE_OUTPUT, GPIO_PULL_NONE, PHASE_B_GPIO_HIGH);
    /* 按目标硬件的有效极性关闭当前高侧或低侧驱动。 */
	PHASE_B_GPIO_PORT_HIGH->HIGH_BITREG_OFF = PHASE_B_GPIO_HIGH;

    /* 切换选定 GPIO 的工作模式；具体驱动电平由随后的置位/清零或 PWM 决定。 */
	gpio_mode_QUICK(PHASE_C_GPIO_PORT_HIGH, GPIO_MODE_OUTPUT, GPIO_PULL_NONE, PHASE_C_GPIO_HIGH);
    /* 按目标硬件的有效极性关闭当前高侧或低侧驱动。 */
	PHASE_C_GPIO_PORT_HIGH->HIGH_BITREG_OFF = PHASE_C_GPIO_HIGH;


	// set low channel to PWM, duty cycle will now control braking
    /* 切换选定 GPIO 的工作模式；具体驱动电平由随后的置位/清零或 PWM 决定。 */
	gpio_mode_QUICK(PHASE_A_GPIO_PORT_LOW, GPIO_MODE_MUX, GPIO_PULL_NONE, PHASE_A_GPIO_LOW);
    /* 切换选定 GPIO 的工作模式；具体驱动电平由随后的置位/清零或 PWM 决定。 */
	gpio_mode_QUICK(PHASE_A_GPIO_PORT_LOW, GPIO_MODE_MUX, GPIO_PULL_NONE, PHASE_B_GPIO_LOW);
    /* 切换选定 GPIO 的工作模式；具体驱动电平由随后的置位/清零或 PWM 决定。 */
	gpio_mode_QUICK(PHASE_C_GPIO_PORT_LOW, GPIO_MODE_MUX, GPIO_PULL_NONE, PHASE_C_GPIO_LOW);
}


/**
 * @brief 将 B 相切为 PWM 驱动；是否启用互补低侧由 comp_pwm 决定，另一种桥型使用使能脚。
 */
void phaseBPWM() {
        /* 当互补 PWM 使能标志为零时进入此分支。 */
		if(!comp_pwm){          
            /* 切换选定 GPIO 的工作模式；具体驱动电平由随后的置位/清零或 PWM 决定。 */
			gpio_mode_QUICK(PHASE_B_GPIO_PORT_LOW, GPIO_MODE_OUTPUT, GPIO_PULL_NONE, PHASE_B_GPIO_LOW);
            /* 按目标硬件的有效极性关闭当前高侧或低侧驱动。 */
			PHASE_B_GPIO_PORT_LOW->LOW_BITREG_OFF = PHASE_B_GPIO_LOW;
        /* 当前条件不满足时进入备选处理。 */
		}else{
            /* 切换选定 GPIO 的工作模式；具体驱动电平由随后的置位/清零或 PWM 决定。 */
			gpio_mode_QUICK(PHASE_B_GPIO_PORT_LOW, GPIO_MODE_MUX, GPIO_PULL_NONE, PHASE_B_GPIO_LOW); // low
		}
        /* 切换选定 GPIO 的工作模式；具体驱动电平由随后的置位/清零或 PWM 决定。 */
		gpio_mode_QUICK(PHASE_B_GPIO_PORT_HIGH, GPIO_MODE_MUX, GPIO_PULL_NONE, PHASE_B_GPIO_HIGH);  // high

}
/**
 * @brief 关闭 B 相的驱动，使该相浮空，供反电动势检测或停机使用。
 */
void phaseBFLOAT() {
        /* 切换选定 GPIO 的工作模式；具体驱动电平由随后的置位/清零或 PWM 决定。 */
		gpio_mode_QUICK(PHASE_B_GPIO_PORT_LOW, GPIO_MODE_OUTPUT, GPIO_PULL_NONE, PHASE_B_GPIO_LOW);
        /* 按目标硬件的有效极性关闭当前高侧或低侧驱动。 */
		PHASE_B_GPIO_PORT_LOW->LOW_BITREG_OFF = PHASE_B_GPIO_LOW;
        /* 切换选定 GPIO 的工作模式；具体驱动电平由随后的置位/清零或 PWM 决定。 */
		gpio_mode_QUICK(PHASE_B_GPIO_PORT_HIGH, GPIO_MODE_OUTPUT, GPIO_PULL_NONE, PHASE_B_GPIO_HIGH);
        /* 按目标硬件的有效极性关闭当前高侧或低侧驱动。 */
		PHASE_B_GPIO_PORT_HIGH->HIGH_BITREG_OFF = PHASE_B_GPIO_HIGH;
	}
/**
 * @brief 将 B 相设为低侧导通状态；普通桥型同时保持高侧关闭。
 */
void phaseBLOW() {
	        // low mosfet on
        /* 切换选定 GPIO 的工作模式；具体驱动电平由随后的置位/清零或 PWM 决定。 */
		gpio_mode_QUICK(PHASE_B_GPIO_PORT_LOW, GPIO_MODE_OUTPUT, GPIO_PULL_NONE, PHASE_B_GPIO_LOW);
        /* 按目标硬件的有效极性导通当前高侧或低侧驱动。 */
		PHASE_B_GPIO_PORT_LOW->LOW_BITREG_ON = PHASE_B_GPIO_LOW;
        /* 切换选定 GPIO 的工作模式；具体驱动电平由随后的置位/清零或 PWM 决定。 */
		gpio_mode_QUICK(PHASE_B_GPIO_PORT_HIGH, GPIO_MODE_OUTPUT, GPIO_PULL_NONE, PHASE_B_GPIO_HIGH);
        /* 按目标硬件的有效极性关闭当前高侧或低侧驱动。 */
		PHASE_B_GPIO_PORT_HIGH->HIGH_BITREG_OFF = PHASE_B_GPIO_HIGH;
}

//////////////////////////////PHASE 2//////////////////////////////////////////////////


/**
 * @brief 将 C 相切为 PWM 驱动；是否启用互补低侧由 comp_pwm 决定，另一种桥型使用使能脚。
 */
void phaseCPWM() {
    /* 当互补 PWM 使能标志为零时进入此分支。 */
	if (!comp_pwm){
            /* 切换选定 GPIO 的工作模式；具体驱动电平由随后的置位/清零或 PWM 决定。 */
			gpio_mode_QUICK(PHASE_C_GPIO_PORT_LOW, GPIO_MODE_OUTPUT, GPIO_PULL_NONE, PHASE_C_GPIO_LOW);
            /* 按目标硬件的有效极性关闭当前高侧或低侧驱动。 */
			PHASE_C_GPIO_PORT_LOW->LOW_BITREG_OFF = PHASE_C_GPIO_LOW;
        /* 当前条件不满足时进入备选处理。 */
		}else{
            /* 切换选定 GPIO 的工作模式；具体驱动电平由随后的置位/清零或 PWM 决定。 */
			gpio_mode_QUICK(PHASE_C_GPIO_PORT_LOW, GPIO_MODE_MUX, GPIO_PULL_NONE, PHASE_C_GPIO_LOW);;
		}
        /* 切换选定 GPIO 的工作模式；具体驱动电平由随后的置位/清零或 PWM 决定。 */
		gpio_mode_QUICK(PHASE_C_GPIO_PORT_HIGH, GPIO_MODE_MUX, GPIO_PULL_NONE, PHASE_C_GPIO_HIGH);

	}


/**
 * @brief 关闭 C 相的驱动，使该相浮空，供反电动势检测或停机使用。
 */
void phaseCFLOAT() {
	  // floating
        /* 切换选定 GPIO 的工作模式；具体驱动电平由随后的置位/清零或 PWM 决定。 */
		gpio_mode_QUICK(PHASE_C_GPIO_PORT_LOW, GPIO_MODE_OUTPUT, GPIO_PULL_NONE, PHASE_C_GPIO_LOW);
        /* 按目标硬件的有效极性关闭当前高侧或低侧驱动。 */
		PHASE_C_GPIO_PORT_LOW->LOW_BITREG_OFF = PHASE_C_GPIO_LOW;
        /* 切换选定 GPIO 的工作模式；具体驱动电平由随后的置位/清零或 PWM 决定。 */
		gpio_mode_QUICK(PHASE_C_GPIO_PORT_HIGH, GPIO_MODE_OUTPUT, GPIO_PULL_NONE, PHASE_C_GPIO_HIGH);
        /* 按目标硬件的有效极性关闭当前高侧或低侧驱动。 */
		PHASE_C_GPIO_PORT_HIGH->HIGH_BITREG_OFF = PHASE_C_GPIO_HIGH;
	}



/**
 * @brief 将 C 相设为低侧导通状态；普通桥型同时保持高侧关闭。
 */
void phaseCLOW() {
        /* 切换选定 GPIO 的工作模式；具体驱动电平由随后的置位/清零或 PWM 决定。 */
		gpio_mode_QUICK(PHASE_C_GPIO_PORT_LOW, GPIO_MODE_OUTPUT, GPIO_PULL_NONE, PHASE_C_GPIO_LOW);
        /* 按目标硬件的有效极性导通当前高侧或低侧驱动。 */
		PHASE_C_GPIO_PORT_LOW->LOW_BITREG_ON = PHASE_C_GPIO_LOW;
        /* 切换选定 GPIO 的工作模式；具体驱动电平由随后的置位/清零或 PWM 决定。 */
		gpio_mode_QUICK(PHASE_C_GPIO_PORT_HIGH, GPIO_MODE_OUTPUT, GPIO_PULL_NONE, PHASE_C_GPIO_HIGH);
        /* 按目标硬件的有效极性关闭当前高侧或低侧驱动。 */
		PHASE_C_GPIO_PORT_HIGH->HIGH_BITREG_OFF = PHASE_C_GPIO_HIGH;
	}



///////////////////////////////////////////////PHASE 3 /////////////////////////////////////////////////

/**
 * @brief 将 A 相切为 PWM 驱动；是否启用互补低侧由 comp_pwm 决定，另一种桥型使用使能脚。
 */
void phaseAPWM() {
        /* 当互补 PWM 使能标志为零时进入此分支。 */
		if (!comp_pwm){
            /* 切换选定 GPIO 的工作模式；具体驱动电平由随后的置位/清零或 PWM 决定。 */
			gpio_mode_QUICK(PHASE_A_GPIO_PORT_LOW, GPIO_MODE_OUTPUT, GPIO_PULL_NONE, PHASE_A_GPIO_LOW);
            /* 按目标硬件的有效极性关闭当前高侧或低侧驱动。 */
			PHASE_A_GPIO_PORT_LOW->LOW_BITREG_OFF = PHASE_A_GPIO_LOW;
            /* 当前条件不满足时进入备选处理。 */
			}else{
        /* 切换选定 GPIO 的工作模式；具体驱动电平由随后的置位/清零或 PWM 决定。 */
		gpio_mode_QUICK(PHASE_A_GPIO_PORT_LOW, GPIO_MODE_MUX, GPIO_PULL_NONE, PHASE_A_GPIO_LOW);
			}
        /* 切换选定 GPIO 的工作模式；具体驱动电平由随后的置位/清零或 PWM 决定。 */
		gpio_mode_QUICK(PHASE_A_GPIO_PORT_HIGH, GPIO_MODE_MUX, GPIO_PULL_NONE, PHASE_A_GPIO_HIGH);
	}

/**
 * @brief 关闭 A 相的驱动，使该相浮空，供反电动势检测或停机使用。
 */
void phaseAFLOAT() {
        /* 切换选定 GPIO 的工作模式；具体驱动电平由随后的置位/清零或 PWM 决定。 */
		gpio_mode_QUICK(PHASE_A_GPIO_PORT_LOW, GPIO_MODE_OUTPUT, GPIO_PULL_NONE, PHASE_A_GPIO_LOW);
        /* 按目标硬件的有效极性关闭当前高侧或低侧驱动。 */
		PHASE_A_GPIO_PORT_LOW->LOW_BITREG_OFF = PHASE_A_GPIO_LOW;
        /* 切换选定 GPIO 的工作模式；具体驱动电平由随后的置位/清零或 PWM 决定。 */
		gpio_mode_QUICK(PHASE_A_GPIO_PORT_HIGH, GPIO_MODE_OUTPUT, GPIO_PULL_NONE, PHASE_A_GPIO_HIGH);
        /* 按目标硬件的有效极性关闭当前高侧或低侧驱动。 */
		PHASE_A_GPIO_PORT_HIGH->HIGH_BITREG_OFF = PHASE_A_GPIO_HIGH;
	}

/**
 * @brief 将 A 相设为低侧导通状态；普通桥型同时保持高侧关闭。
 */
void phaseALOW() {
        /* 切换选定 GPIO 的工作模式；具体驱动电平由随后的置位/清零或 PWM 决定。 */
		gpio_mode_QUICK(PHASE_A_GPIO_PORT_LOW, GPIO_MODE_OUTPUT, GPIO_PULL_NONE, PHASE_A_GPIO_LOW);
        /* 按目标硬件的有效极性导通当前高侧或低侧驱动。 */
		PHASE_A_GPIO_PORT_LOW->LOW_BITREG_ON = PHASE_A_GPIO_LOW;
        /* 切换选定 GPIO 的工作模式；具体驱动电平由随后的置位/清零或 PWM 决定。 */
		gpio_mode_QUICK(PHASE_A_GPIO_PORT_HIGH, GPIO_MODE_OUTPUT, GPIO_PULL_NONE, PHASE_A_GPIO_HIGH);
        /* 按目标硬件的有效极性关闭当前高侧或低侧驱动。 */
		PHASE_A_GPIO_PORT_HIGH->HIGH_BITREG_OFF = PHASE_A_GPIO_HIGH;
	}

/* 采用上一编译条件不成立时的备选实现。 */
#else

//////////////////////////////////PHASE 1//////////////////////
/**
 * @brief 将 B 相切为 PWM 驱动；是否启用互补低侧由 comp_pwm 决定，另一种桥型使用使能脚。
 */
void phaseBPWM() {
        /* 当互补 PWM 使能标志为零时进入此分支。 */
		if(!comp_pwm){            // for future
		//gpio_mode_QUICK(PHASE_B_GPIO_PORT_LOW, GPIO_MODE_OUTPUT, GPIO_PULL_NONE, PHASE_B_GPIO_LOW);
		//PHASE_B_GPIO_PORT_LOW->clr = PHASE_B_GPIO_LOW;
        /* 当前条件不满足时进入备选处理。 */
		}else{
        /* 兼容桥型中切换引脚为 GPIO 或外设复用模式。 */
		LL_GPIO_SetPinMode(PHASE_B_GPIO_PORT_ENABLE, PHASE_B_GPIO_ENABLE, LL_GPIO_MODE_OUTPUT); // enable on
        /* 更新PHASE_B_GPIO_PORT_ENABLE 的兼容 GPIO 引脚置位/复位寄存器。 */
		PHASE_B_GPIO_PORT_ENABLE->BSRR = PHASE_B_GPIO_ENABLE;
		}
        /* 兼容桥型中切换引脚为 GPIO 或外设复用模式。 */
		LL_GPIO_SetPinMode(PHASE_B_GPIO_PORT_PWM, PHASE_B_GPIO_PWM, LL_GPIO_MODE_ALTERNATE);  // high pwm

	}


/**
 * @brief 关闭 B 相的驱动，使该相浮空，供反电动势检测或停机使用。
 */
void phaseBFLOAT() {
    /* 兼容桥型中切换引脚为 GPIO 或外设复用模式。 */
	LL_GPIO_SetPinMode(PHASE_B_GPIO_PORT_ENABLE, PHASE_B_GPIO_ENABLE, LL_GPIO_MODE_OUTPUT); // enable off
    /* 更新PHASE_B_GPIO_PORT_ENABLE 的兼容 GPIO 引脚清零寄存器。 */
	PHASE_B_GPIO_PORT_ENABLE->BRR = PHASE_B_GPIO_ENABLE;
        /* 兼容桥型中切换引脚为 GPIO 或外设复用模式。 */
		LL_GPIO_SetPinMode(PHASE_B_GPIO_PORT_PWM, PHASE_B_GPIO_PWM, LL_GPIO_MODE_OUTPUT);  // pwm off
        /* 更新PHASE_B_GPIO_PORT_PWM 的兼容 GPIO 引脚清零寄存器。 */
		PHASE_B_GPIO_PORT_PWM->BRR = PHASE_B_GPIO_PWM;
	}


/**
 * @brief 将 B 相设为低侧导通状态；普通桥型同时保持高侧关闭。
 */
void phaseBLOW() {
	        // low mosfet on
    /* 兼容桥型中切换引脚为 GPIO 或外设复用模式。 */
	LL_GPIO_SetPinMode(PHASE_B_GPIO_PORT_ENABLE, PHASE_B_GPIO_ENABLE, LL_GPIO_MODE_OUTPUT);  // enable on
    /* 更新PHASE_B_GPIO_PORT_ENABLE 的兼容 GPIO 引脚置位/复位寄存器。 */
	PHASE_B_GPIO_PORT_ENABLE->BSRR = PHASE_B_GPIO_ENABLE;
        /* 兼容桥型中切换引脚为 GPIO 或外设复用模式。 */
		LL_GPIO_SetPinMode(PHASE_B_GPIO_PORT_PWM, PHASE_B_GPIO_PWM, LL_GPIO_MODE_OUTPUT);  // pwm off
        /* 更新PHASE_B_GPIO_PORT_PWM 的兼容 GPIO 引脚清零寄存器。 */
		PHASE_B_GPIO_PORT_PWM->BRR = PHASE_B_GPIO_PWM;
}



//////////////////////////////PHASE 2//////////////////////////////////////////////////


/**
 * @brief 将 C 相切为 PWM 驱动；是否启用互补低侧由 comp_pwm 决定，另一种桥型使用使能脚。
 */
void phaseCPWM() {
    /* 当互补 PWM 使能标志为零时进入此分支。 */
	if (!comp_pwm){
		//	gpio_mode_QUICK(PHASE_C_GPIO_PORT_LOW, GPIO_MODE_OUTPUT, GPIO_PULL_NONE, PHASE_C_GPIO_LOW);
			//PHASE_C_GPIO_PORT_LOW->clr = PHASE_C_GPIO_LOW;
        /* 当前条件不满足时进入备选处理。 */
		}else{
            /* 兼容桥型中切换引脚为 GPIO 或外设复用模式。 */
			LL_GPIO_SetPinMode(PHASE_C_GPIO_PORT_ENABLE, PHASE_C_GPIO_ENABLE, LL_GPIO_MODE_OUTPUT);  // enable on
            /* 更新PHASE_C_GPIO_PORT_ENABLE 的兼容 GPIO 引脚置位/复位寄存器。 */
			PHASE_C_GPIO_PORT_ENABLE->BSRR = PHASE_C_GPIO_ENABLE;
		}
        /* 兼容桥型中切换引脚为 GPIO 或外设复用模式。 */
		LL_GPIO_SetPinMode(PHASE_C_GPIO_PORT_PWM, PHASE_C_GPIO_PWM, LL_GPIO_MODE_ALTERNATE);

	}


/**
 * @brief 关闭 C 相的驱动，使该相浮空，供反电动势检测或停机使用。
 */
void phaseCFLOAT() {
	         // floating
    /* 兼容桥型中切换引脚为 GPIO 或外设复用模式。 */
	LL_GPIO_SetPinMode(PHASE_C_GPIO_PORT_ENABLE, PHASE_C_GPIO_ENABLE, LL_GPIO_MODE_OUTPUT);  // enable off
                /* 更新PHASE_C_GPIO_PORT_ENABLE 的兼容 GPIO 引脚清零寄存器。 */
				PHASE_C_GPIO_PORT_ENABLE->BRR = PHASE_C_GPIO_ENABLE;
        /* 兼容桥型中切换引脚为 GPIO 或外设复用模式。 */
		LL_GPIO_SetPinMode(PHASE_C_GPIO_PORT_PWM, PHASE_C_GPIO_PWM, LL_GPIO_MODE_OUTPUT);
        /* 更新PHASE_C_GPIO_PORT_PWM 的兼容 GPIO 引脚清零寄存器。 */
		PHASE_C_GPIO_PORT_PWM->BRR = PHASE_C_GPIO_PWM;
	}



/**
 * @brief 将 C 相设为低侧导通状态；普通桥型同时保持高侧关闭。
 */
void phaseCLOW() {
    /* 兼容桥型中切换引脚为 GPIO 或外设复用模式。 */
	LL_GPIO_SetPinMode(PHASE_C_GPIO_PORT_ENABLE, PHASE_C_GPIO_ENABLE, LL_GPIO_MODE_OUTPUT);  // enable on
        /* 更新PHASE_C_GPIO_PORT_ENABLE 的兼容 GPIO 引脚置位/复位寄存器。 */
		PHASE_C_GPIO_PORT_ENABLE->BSRR = PHASE_C_GPIO_ENABLE;
        /* 兼容桥型中切换引脚为 GPIO 或外设复用模式。 */
		LL_GPIO_SetPinMode(PHASE_C_GPIO_PORT_PWM, PHASE_C_GPIO_PWM, LL_GPIO_MODE_OUTPUT);
        /* 更新PHASE_C_GPIO_PORT_PWM 的兼容 GPIO 引脚清零寄存器。 */
		PHASE_C_GPIO_PORT_PWM->BRR = PHASE_C_GPIO_PWM;
	}



///////////////////////////////////////////////PHASE 3 /////////////////////////////////////////////////

/**
 * @brief 将 A 相切为 PWM 驱动；是否启用互补低侧由 comp_pwm 决定，另一种桥型使用使能脚。
 */
void phaseAPWM() {
        /* 当互补 PWM 使能标志为零时进入此分支。 */
		if (!comp_pwm){
		//	gpio_mode_QUICK(PHASE_A_GPIO_PORT_LOW, GPIO_MODE_OUTPUT, GPIO_PULL_NONE, PHASE_A_GPIO_LOW);
			//PHASE_A_GPIO_PORT_LOW->clr = PHASE_A_GPIO_LOW;
            /* 当前条件不满足时进入备选处理。 */
			}else{
                /* 兼容桥型中切换引脚为 GPIO 或外设复用模式。 */
				LL_GPIO_SetPinMode(PHASE_A_GPIO_PORT_ENABLE, PHASE_A_GPIO_ENABLE, LL_GPIO_MODE_OUTPUT);  // enable on
                /* 更新PHASE_A_GPIO_PORT_ENABLE 的兼容 GPIO 引脚置位/复位寄存器。 */
				PHASE_A_GPIO_PORT_ENABLE->BSRR = PHASE_A_GPIO_ENABLE;
			}
        /* 兼容桥型中切换引脚为 GPIO 或外设复用模式。 */
		LL_GPIO_SetPinMode(PHASE_A_GPIO_PORT_PWM, PHASE_A_GPIO_PWM, LL_GPIO_MODE_ALTERNATE);
	}

/**
 * @brief 关闭 A 相的驱动，使该相浮空，供反电动势检测或停机使用。
 */
void phaseAFLOAT() {
        /* 兼容桥型中切换引脚为 GPIO 或外设复用模式。 */
		LL_GPIO_SetPinMode(PHASE_A_GPIO_PORT_ENABLE, PHASE_A_GPIO_ENABLE, LL_GPIO_MODE_OUTPUT);  // enable on
        /* 更新PHASE_A_GPIO_PORT_ENABLE 的兼容 GPIO 引脚清零寄存器。 */
		PHASE_A_GPIO_PORT_ENABLE->BRR = PHASE_A_GPIO_ENABLE;
        /* 兼容桥型中切换引脚为 GPIO 或外设复用模式。 */
		LL_GPIO_SetPinMode(PHASE_A_GPIO_PORT_PWM, PHASE_A_GPIO_PWM, LL_GPIO_MODE_OUTPUT);
        /* 更新PHASE_A_GPIO_PORT_PWM 的兼容 GPIO 引脚清零寄存器。 */
		PHASE_A_GPIO_PORT_PWM->BRR = PHASE_A_GPIO_PWM;
	}

/**
 * @brief 将 A 相设为低侧导通状态；普通桥型同时保持高侧关闭。
 */
void phaseALOW() {
        /* 兼容桥型中切换引脚为 GPIO 或外设复用模式。 */
		LL_GPIO_SetPinMode(PHASE_A_GPIO_PORT_ENABLE, PHASE_A_GPIO_ENABLE, LL_GPIO_MODE_OUTPUT);  // enable on
        /* 更新PHASE_A_GPIO_PORT_ENABLE 的兼容 GPIO 引脚置位/复位寄存器。 */
		PHASE_A_GPIO_PORT_ENABLE->BSRR = PHASE_A_GPIO_ENABLE;
        /* 兼容桥型中切换引脚为 GPIO 或外设复用模式。 */
		LL_GPIO_SetPinMode(PHASE_A_GPIO_PORT_PWM, PHASE_A_GPIO_PWM, LL_GPIO_MODE_OUTPUT);
        /* 更新PHASE_A_GPIO_PORT_PWM 的兼容 GPIO 引脚清零寄存器。 */
		PHASE_A_GPIO_PORT_PWM->BRR = PHASE_A_GPIO_PWM;
	}



/* 结束当前条件编译分支。 */
#endif

/**
 * @brief 将 A、B、C 三相全部置为浮空，关闭各相高低侧驱动。
 */
void allOff(){
    /* 关闭 A 相的驱动，使该相浮空，供反电动势检测或停机使用。 */
	phaseAFLOAT();
    /* 关闭 B 相的驱动，使该相浮空，供反电动势检测或停机使用。 */
	phaseBFLOAT();
    /* 关闭 C 相的驱动，使该相浮空，供反电动势检测或停机使用。 */
	phaseCFLOAT();
}


/**
 * @brief 按 1～6 的步骤号执行六步换相，每步包含一相 PWM、一相低侧导通和一相浮空。
 *
 * 此函数只改变各相工作状态，PWM 比较值由主控制/PWM 更新路径写入。
 * newStep 超出 1～6 时 switch 没有匹配分支，桥臂保持此前状态。
 * @param newStep 六步换相步骤，范围为 1～6。
 */
void  comStep (int newStep){
//TIM14->CNT = 0;
/* 按命令编号、遥测状态或换相步骤分派到对应处理分支。 */
switch(newStep)
{

        /* 第 1 步：A 相 PWM、B 相低侧导通、C 相浮空。 */
        case 1:			//A-B
            /* 将 A 相切为 PWM 驱动；是否启用互补低侧由 comp_pwm 决定，另一种桥型使用使能脚。 */
        	phaseAPWM();
            /* 将 B 相设为低侧导通状态；普通桥型同时保持高侧关闭。 */
        	phaseBLOW();
            /* 关闭 C 相的驱动，使该相浮空，供反电动势检测或停机使用。 */
        	phaseCFLOAT();
            /* 结束当前分支或循环，避免继续处理后续候选项。 */
        	break;


        /* 第 2 步：C 相 PWM、B 相低侧导通、A 相浮空。 */
        case 2:		// C-B
            /* 关闭 A 相的驱动，使该相浮空，供反电动势检测或停机使用。 */
        	phaseAFLOAT();
            /* 将 B 相设为低侧导通状态；普通桥型同时保持高侧关闭。 */
        	phaseBLOW();
            /* 将 C 相切为 PWM 驱动；是否启用互补低侧由 comp_pwm 决定，另一种桥型使用使能脚。 */
        	phaseCPWM();
            /* 结束当前分支或循环，避免继续处理后续候选项。 */
        	break;



        /* 第 3 步：C 相 PWM、A 相低侧导通、B 相浮空。 */
        case 3:	// C-A
            /* 将 A 相设为低侧导通状态；普通桥型同时保持高侧关闭。 */
        	phaseALOW();
            /* 关闭 B 相的驱动，使该相浮空，供反电动势检测或停机使用。 */
        	phaseBFLOAT();
            /* 将 C 相切为 PWM 驱动；是否启用互补低侧由 comp_pwm 决定，另一种桥型使用使能脚。 */
        	phaseCPWM();
            /* 结束当前分支或循环，避免继续处理后续候选项。 */
        	break;


        /* 第 4 步：B 相 PWM、A 相低侧导通、C 相浮空。 */
        case 4:// B-A
            /* 将 A 相设为低侧导通状态；普通桥型同时保持高侧关闭。 */
        	phaseALOW();
            /* 将 B 相切为 PWM 驱动；是否启用互补低侧由 comp_pwm 决定，另一种桥型使用使能脚。 */
        	phaseBPWM();
            /* 关闭 C 相的驱动，使该相浮空，供反电动势检测或停机使用。 */
        	phaseCFLOAT();
            /* 结束当前分支或循环，避免继续处理后续候选项。 */
        	break;


        /* 第 5 步：B 相 PWM、C 相低侧导通、A 相浮空。 */
        case 5:    // B-C
            /* 关闭 A 相的驱动，使该相浮空，供反电动势检测或停机使用。 */
        	phaseAFLOAT();
            /* 将 B 相切为 PWM 驱动；是否启用互补低侧由 comp_pwm 决定，另一种桥型使用使能脚。 */
        	phaseBPWM();
            /* 将 C 相设为低侧导通状态；普通桥型同时保持高侧关闭。 */
        	phaseCLOW();
            /* 结束当前分支或循环，避免继续处理后续候选项。 */
        	break;


        /* 第 6 步：A 相 PWM、C 相低侧导通、B 相浮空。 */
        case 6:      // A-C
            /* 将 A 相切为 PWM 驱动；是否启用互补低侧由 comp_pwm 决定，另一种桥型使用使能脚。 */
        	phaseAPWM();
            /* 关闭 B 相的驱动，使该相浮空，供反电动势检测或停机使用。 */
        	phaseBFLOAT();
            /* 将 C 相设为低侧导通状态；普通桥型同时保持高侧关闭。 */
        	phaseCLOW();
            /* 结束当前分支或循环，避免继续处理后续候选项。 */
        	break;
	}

//stop_time = TIM14->CNT;

}

/**
 * @brief 把三相全部切为低侧导通，实现三相短接制动。
 */
void fullBrake(){                     // full braking shorting all low sides
    /* 将 A 相设为低侧导通状态；普通桥型同时保持高侧关闭。 */
	phaseALOW();
    /* 将 B 相设为低侧导通状态；普通桥型同时保持高侧关闭。 */
	phaseBLOW();
    /* 将 C 相设为低侧导通状态；普通桥型同时保持高侧关闭。 */
	phaseCLOW();
}


/**
 * @brief 把三相切换到 PWM 模式，供正弦启动等三通道调制逻辑使用。
 */
void allpwm(){                        // for stepper_sine
    /* 将 A 相切为 PWM 驱动；是否启用互补低侧由 comp_pwm 决定，另一种桥型使用使能脚。 */
	phaseAPWM();
    /* 将 B 相切为 PWM 驱动；是否启用互补低侧由 comp_pwm 决定，另一种桥型使用使能脚。 */
	phaseBPWM();
    /* 将 C 相切为 PWM 驱动；是否启用互补低侧由 comp_pwm 决定，另一种桥型使用使能脚。 */
	phaseCPWM();
}

/* 仅在定义 BRUSHED_MODE 时编译以下代码。 */
#ifdef BRUSHED_MODE
/**
 * @brief 设置有刷电机正向桥臂组合：A/C 相 PWM，B 相低侧导通。
 */
void twoChannelForward(){
/* 将 A 相切为 PWM 驱动；是否启用互补低侧由 comp_pwm 决定，另一种桥型使用使能脚。 */
phaseAPWM();
/* 将 B 相设为低侧导通状态；普通桥型同时保持高侧关闭。 */
phaseBLOW();
/* 将 C 相切为 PWM 驱动；是否启用互补低侧由 comp_pwm 决定，另一种桥型使用使能脚。 */
phaseCPWM();
}

/**
 * @brief 设置有刷电机反向桥臂组合：A/C 相低侧导通，B 相 PWM。
 */
void twoChannelReverse(){
/* 将 A 相设为低侧导通状态；普通桥型同时保持高侧关闭。 */
phaseALOW();
/* 将 B 相切为 PWM 驱动；是否启用互补低侧由 comp_pwm 决定，另一种桥型使用使能脚。 */
phaseBPWM();
/* 将 C 相设为低侧导通状态；普通桥型同时保持高侧关闭。 */
phaseCLOW();
}
/* 结束当前条件编译分支。 */
#endif
