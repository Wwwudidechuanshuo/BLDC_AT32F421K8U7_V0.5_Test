/*
 * 模块间共享变量及整数 PID 状态结构；变量实体主要在 main.c 和协议模块中定义。
 * 中文说明按当前实现编写；保留原代码、历史注释和编译条件。
 */
/* 保证本头文件在同一编译单元内只包含一次。 */
#pragma once
/* 引入 main.h：电调主控制：配置加载、启动、六步换相、过零判断、保护、油门和遥测调度。 */
#include "main.h"

/* 引用其他模块定义的176 字节 Flash 参数及旋律数据的 RAM 镜像。 */
extern uint8_t eepromBuffer[176];
/* 引用其他模块定义的占空比换算所依赖的基准 PWM 周期值。 */
extern uint16_t TIMER1_MAX_ARR;

/* 引用其他模块定义的双向 DShot 遥测的 26 项定时器比较值序列。 */
extern uint32_t gcr[26];
/* 引用其他模块定义的经过方向和死区处理的油门值。 */
extern uint16_t adjusted_input;
/* 引用其他模块定义的输入信号边沿捕获时间戳缓冲区。 */
extern uint32_t dma_buffer[64];
/* 引用其他模块定义的当前待处理的 DShot 命令编号。 */
extern uint8_t dshotcommand;
/* 引用其他模块定义的输入协议初始化时设置的解锁计数参数。 */
extern uint16_t armed_count_threshold;
/* 引用其他模块定义的当前六步旋转方向，1 递增步骤，0 递减步骤。 */
extern char forward;
/* 引用其他模块定义的电机控制流程正在运行的标志。 */
extern uint8_t running;
/* 引用其他模块定义的连续检测到零油门的次数。 */
extern uint16_t zero_input_count;
/* 引用其他模块定义的自最近有效输入以来的周期计数。 */
extern uint16_t signaltimeout;
/* 引用其他模块定义的交给电机控制状态机的最终油门值。 */
extern uint16_t input;
/* 引用其他模块定义的输入协议刚解码得到的油门或命令数值。 */
extern uint16_t newinput;
/* 引用其他模块定义的待播放的设置提示音标志。 */
extern char play_tone_flag;
/* 引用其他模块定义的保留的当前 GPIO 引脚配置。 */
extern uint32_t current_GPIO_PIN;
/* 引用其他模块定义的保留的当前 GPIO 端口配置。 */
extern uint32_t current_GPIO_PORT;
/* 引用其他模块定义的保留的当前外部中断线路。 */
extern uint32_t current_EXTI_LINE;
/* 引用其他模块定义的扩展遥测使能及温度/电流/电压轮换状态。 */
extern char dshot_extended_telemetry;
/* 引用其他模块定义的待发送的扩展遥测编码，非零时优先于转速数据。 */
extern uint16_t send_extended_dshot;

/* 引用其他模块定义的桥臂换相调用后的辅助计时器读数。 */
extern uint16_t comp_change_time;
/* 引用其他模块定义的外部声明的中断耗时变量。 */
extern uint16_t interrupt_time;

//typedef struct PID{
//	float error;
//	float Kp;
//	float Ki;
//	float Kd;
//	float integral;
//	float derivative;
//	float last_error;
//	float pid_output;
//	int16_t integral_limit;
//	int16_t output_limit;
//}PID;

/* 定义跨调用保存参数和历史状态的 PID 结构类型。 */
typedef struct fastPID{
    /* 保存PID 当前误差，定义为测量值减目标值。 */
	int32_t error;
    /* 保存PID 比例增益，按调用方约定的整数尺度使用。 */
	uint32_t Kp;
    /* 保存PID 积分增益，按调用方约定的整数尺度使用。 */
	uint32_t Ki;
    /* 保存PID 微分增益，按调用方约定的整数尺度使用。 */
	uint32_t Kd;
    /* 保存PID 积分累计量。 */
	int32_t integral;
    /* 保存PID 相邻误差变化形成的微分项。 */
	int32_t derivative;
    /* 保存PID 上一次误差。 */
	int32_t last_error;
    /* 保存PID 合成输出，尚未按调用方比例缩放。 */
	int32_t pid_output;
    /* 保存PID 积分项的对称限幅值。 */
	int32_t integral_limit;
    /* 保存PID 输出的对称限幅值。 */
	int32_t output_limit;
/* 结束类型定义，并提供可直接使用的类型别名。 */
}fastPID;


