/*
 * 电调主控制：配置加载、启动、六步换相、过零判断、保护、油门和遥测调度。
 * 中文说明按当前实现编写；保留原代码、历史注释和编译条件。
 * 阅读顺序：main 初始化 -> tenKhzRoutine 控制 -> 轮询/中断过零 -> 延迟换相。
 * F421 的 INTERVAL_TIMER/COM_TIMER 每计数 0.5 us；e_com_time 为一电周期的 us。
 * input 通常沿用 DShot 的 0～2047 尺度；duty_cycle 是计数值而非百分比。
 * POWER_ON_TEST_MODE 当前已启用，上电后使用试转流程和专用故障锁存。
 */
/* AM32- multi-purpose brushless controller firmware for the stm32f051 */

//===========================================================================
//=============================== Changelog =================================
//===========================================================================
/*
 * 1.54 Changelog;
 * --Added firmware name to targets and firmware version to main
 * --added two more dshot to beacons 1-3 currently working
 * --added KV option to firmware, low rpm power protection is based on KV
 * --start power now controls minimum idle power as well as startup strength.
 * --change default timing to 22.5
 * --Lowered default minimum idle setting to 1.5 percent duty cycle, slider range from 1-2.
 * --Added dshot commands to save settings and reset ESC.
 *
 *1.56 Changelog.
 * -- added check to stall protection to wait until after 40 zero crosses to fix high startup throttle hiccup.
 * -- added TIMER 1 update interrupt and PWM changes are done once per pwm period
 * -- reduce commutation interval averaging length
 * -- reduce false positive filter level to 2 and eliminate threshold where filter is stopped.
 * -- disable interrupt before sounds
 * -- disable TIM1 interrupt during stepper sinusoidal mode
 * -- add 28us delay for dshot300
 * -- report 0 rpm until the first 10 successful steps.
 * -- move serial ADC telemetry calculations and desync check to 10Khz interrupt.
 *
 * 1.57
 * -- remove spurious commutations and rpm data at startup by polling for longer interval on startup
 *
 * 1.58
 * -- move signal timeout to 10khz routine and set armed timeout to one quarter second 2500 / 10000
 * 1.59
 * -- moved comp order definitions to target.h
 * -- fixed update version number if older than new version
 * -- cleanup, moved all input and output to IO.c
 * -- moved comparator functions to comparator.c
 * -- removed ALOT of useless variables
 * -- added siskin target
 * -- moved pwm changes to 10khz routine
 * -- moved basic functions to functions.c
 * -- moved peripherals setup to periherals.c
 * -- added crawler mode settings
 *
 * 1.60
 * -- added sine mode hysteresis
 * -- increased power in stall protection and lowered start rpm for crawlers
 * -- removed onehot125 from crawler mode
 * -- reduced maximum startup power from 400 to 350
 * -- change minimum duty cycle to DEAD_TIME
 * -- version and name moved to permanent spot in FLASH memory, thanks mikeller
 *
 * 1.61
 * -- moved duty cycle calculation to 10khz and added max change option.
 * -- decreased maximum interval change to 25%
 * -- reduce wait time on fast acceleration (fast_accel)
 * -- added check in interrupt for early zero cross
 *
 * 1.62
 * --moved control to 10khz loop
 * --changed condition for low rpm filter for duty cycle from || to &&
 * --introduced max deceleration and set it to 20ms to go from 100 to 0
 * --added configurable servo throttle ranges
 *
 *
 *1.63
 *-- increase time for zero cross error detection below 250us commutation interval
 *-- increase max change a low rpm x10
 *-- set low limit of throttle ramp to a lower point and increase upper range
 *-- change desync event from full restart to just lower throttle.

 *1.64
 * --added startup check for continuous high signal, reboot to enter bootloader.
 *-- added brake on stop from eeprom
 *-- added stall protection from eeprom
 *-- added motor pole divider for sinusoidal and low rpm power protection
 *-- fixed dshot commands, added confirmation beeps and removed blocking behavior
 *--
 *1.65
 *-- Added 32 millisecond telemetry output
 *-- added low voltage cutoff , divider value and cutoff voltage needs to be added to eeprom
 *-- added beep to indicate cell count if low voltage active
 *-- added current reading on pa3 , conversion factor needs to be added to eeprom
 *-- fixed servo input capture to only read positive pulse to handle higher refresh rates.
 *-- disabled oneshot 125.
 *-- extended servo range to match full output range of receivers
 *-- added RC CAR style reverse, proportional brake on first reverse , double tap to change direction
 *-- added brushed motor control mode
 *-- added settings to EEPROM version 1
 *-- add gimbal control option.
 *--
 *1.66
 *-- move idwg init to after input tune
 *-- remove reset after save command -- dshot
 *-- added wraith32 target
 *-- added average pulse check for signal detection
 *--
 *1.67
 *-- Rework file structure for multiple MCU support
 *-- Add g071 mcu
 *--
 *1.68
 *--increased allowed average pulse length to avoid double startup
 *1.69
 *--removed line re-enabling comparator after disabling.
 *1.70 fix dshot for Kiss FC
 *1.71 fix dshot for Ardupilot / Px4 FC
 *1.72 Fix telemetry output and add 1 second arming.
 *1.73 Fix false arming if no signal. Remove low rpm throttle protection below 300kv
 *1.74 Add Sine Mode range and drake brake strength adjustment
 *1.75 Disable brake on stop for PWM_ENABLE_BRIDGE 
	   Removed automatic brake on stop on neutral for RC car proportional brake.
	   Adjust sine speed and stall protection speed to more closely match
	   makefile fixes from Cruwaller 
	   Removed gd32 build, until firmware is functional
 *1.76 Adjust g071 PWM frequency, and startup power to be same frequency as f051. 
       Reduce number of polling back emf checks for g071
 *1.77 increase PWM frequency range to 8-48khz
 *1.78 Fix bluejay tunes frequency and speed. 
	   Fix g071 Dead time 	
	   Increment eeprom version
 *1.79 Add stick throttle calibration routine  
	   Add variable for telemetry interval
 *1.80 -Enable Comparator blanking for g071 on timer 1 channel 4
	   -add hardware group F for Iflight Blitz
	   -adjust parameters for pwm frequency
	   -add sine mode power variable and eeprom setting
	   -fix telemetry rpm during sine mode
	   -fix sounds for extended pwm range
	   -Add adjustable braking strength when driving
 *1.81 -Add current limiting PID loop
 	   -fix current sense scale
 	   -Increase brake power on maximum reverse ( car mode only)
	   -Add HK and Blpwr targets
	   -Change low kv motor throttle limit
	   -add reverse speed threshold changeover based on motor kv
	   -doubled filter length for motors under 900kv
*1.82  -Add speed control pid loop.
*1.83  -Add stall protection pid loop.
  	   -Improve sine mode transition.
  	   -decrease speed step re-entering sine mode
  	   -added fixed duty cycle and speed mode build option
  	   -added rpm_controlled by input signal ( to be added to config tool )
*1.84  -Change PID value to int for faster calculations
	   -Enable two channel brushed motor control for dual motors
	   -Add current limit max duty cycle
*1.85  -fix current limit not allowing full rpm on g071 or low pwm frequency
		-remove unused brake on stop conditional 
*1.90 -Fix double beep on startup
			-fix brushed mode comparator
			-add extended dshot
			-fix drive by rpm mode
*1.91 -reset average interval on any desync after 100 zero crosses 
*1.92 - increase ADC read rate and filtering
*1.93 - Add filename to fixed location in flash memory
 */	     


/* 仅在定义 AT32F421K8U7 时编译以下代码。 */
#ifdef AT32F421K8U7
/* 引入 at32f421.h：芯片或运行库接口。 */
#include "at32f421.h"
/* 结束当前条件编译分支。 */
#endif
/* 仅在定义 AT32F415K8U7 时编译以下代码。 */
#ifdef AT32F415K8U7
/* 引入 at32f415.h：芯片或运行库接口。 */
#include "at32f415.h"
/* 结束当前条件编译分支。 */
#endif


//#include "systick.h"
/* 引入 stdio.h：芯片或运行库接口。 */
#include <stdio.h>
/* 引入 main.h：电调主控制：配置加载、启动、六步换相、过零判断、保护、油门和遥测调度。 */
#include "main.h"

/* 引入 targets.h：硬件目标配置：选择 MCU、输入捕获资源、三相驱动引脚、比较器通道和默认参数。 */
#include "targets.h"
/* 引入 signal.h：舵机 PWM 解析、油门端点校准和输入 DMA 完成后的协议分发。 */
#include "signal.h"
/* 引入 dshot.h：DShot 油门帧解码、命令执行及双向遥测的校验与 GCR 波形编码。 */
#include "dshot.h"
/* 引入 phaseouts.h：三相桥臂状态控制，包含六步换相、浮空、低侧导通、PWM 和制动组合。 */
#include "phaseouts.h"
/* 引入 eeprom.h：使用片内 Flash 保存和读取配置；此处是软件模拟 EEPROM，并非外接 EEPROM。 */
#include "eeprom.h"
/* 引入 sounds.h：利用电机绕组和 PWM 播放提示音及 Bluejay 格式旋律；播放会暂时占用驱动定时器。 */
#include "sounds.h"
/* 引入 ADC.h：ADC 采样及 DMA 搬运接口，包含正常采样与 BEMF 诊断采样两套配置。 */
#include "ADC.h"
/* 仅在定义 USE_SERIAL_TELEMETRY 时编译以下代码。 */
#ifdef USE_SERIAL_TELEMETRY
/* 引入 serial_telemetry.h：串口遥测组帧、CRC8 校验和 USART1 DMA 发送配置。 */
#include "serial_telemetry.h"
/* 结束当前条件编译分支。 */
#endif
/* 引入 IO.h：输入信号定时器/DMA 的收发切换，以及输入协议自动识别。 */
#include "IO.h"
/* 引入 comparator.h：内置比较器的浮空相选择、过零 EXINT 边沿配置及中断屏蔽控制。 */
#include "comparator.h"
/* 引入 functions.h：GPIO 模式切换、限幅线性映射、绝对差和阻塞延时等工程辅助函数。 */
#include "functions.h"
/* 引入 peripherals.h：本 MCU 的时钟和外设适配初始化；属于工程代码，底层标准库保持原样。 */
#include "peripherals.h"
/* 引入 common.h：模块间共享变量及整数 PID 状态结构；变量实体主要在 main.c 和协议模块中定义。 */
#include "common.h"
/* 引入 firmwareversion.h：固件识别字符串的导出接口；当前 test 数组保存 FILE_NAME，不是数值版本数组。 */
#include "firmwareversion.h"



/* 保存桥臂换相调用后的辅助计时器读数。 */
uint16_t comp_change_time = 0;


//firmware build options !! fixed speed and duty cycle modes are not to be used with sinusoidal startup !!

/* 启用上电自动试转，绕过正常信号输入和解锁流程。 */
#define POWER_ON_TEST_MODE       // bypasses signal input and arming for bench motor testing
/* 闭环试转的固定油门百分比。 */
#define POWER_ON_TEST_POWER 10   // 0-100 percent, bounded by the startup fault latch below
/* 试转使用的电机 KV 参数。 */
#define POWER_ON_TEST_MOTOR_KV 1400
/* 试转使用的电机磁极总数。 */
#define POWER_ON_TEST_MOTOR_POLES 14
/* 试转最小启动占空比，按 0～2000 尺度表示。 */
#define POWER_ON_TEST_MIN_STARTUP_DUTY 160
/* 试转捕获阶段的启动占空比，按 0～2000 尺度表示。 */
#define POWER_ON_TEST_STARTUP_MAX_DUTY 480
/* 试转换相提前角档位。 */
#define POWER_ON_TEST_ADVANCE_LEVEL 0
/* 试转时强制采用的旋转方向。 */
#define POWER_ON_TEST_FORWARD 1
// Ramp forced commutation only long enough to acquire BEMF. If acquisition
// fails, latch the bridge off instead of driving a stalled motor indefinitely.
/* 强制换相初始间隔，F421 每计数 0.5 微秒。 */
#define POWER_ON_TEST_COMM_TIMEOUT_START 20000
/* 强制换相允许的最短间隔，F421 每计数 0.5 微秒。 */
#define POWER_ON_TEST_COMM_TIMEOUT_MIN 3000
/* 每次强制换相后保留的间隔百分比，用于开环加速。 */
#define POWER_ON_TEST_COMM_TIMEOUT_RAMP_PERCENT 96
/* 开始接受 BEMF 捕获的间隔门限。 */
#define POWER_ON_TEST_BEMF_ACQUIRE_INTERVAL 3500
/* 试转最多允许的强制换相次数。 */
#define POWER_ON_TEST_MAX_FORCED_STEPS 90
/* 进入捕获速度范围后允许的连续超时次数。 */
#define POWER_ON_TEST_MAX_ACQUIRE_TIMEOUTS 18
/* 认定捕获闭环所需的连续有效过零次数。 */
#define POWER_ON_TEST_REQUIRED_VALID_ZC 12
/* 启动转子对齐保持时间，单位毫秒。 */
#define POWER_ON_TEST_ALIGNMENT_MS 200
/* 试转轮询过零的有效读数门限。 */
#define POWER_ON_TEST_MIN_BEMF_COUNTS 4
/* 试转允许切入比较器中断的最大扇区间隔。 */
#define POWER_ON_TEST_INTERRUPT_COMM_INTERVAL 2000

/* 仅在定义 POWER_ON_TEST_MODE 时编译以下代码。 */
#ifdef POWER_ON_TEST_MODE
/* 启用固定油门模式，绕过外部油门输入。 */
#define FIXED_DUTY_MODE
/* 固定油门百分比。 */
#define FIXED_DUTY_MODE_POWER POWER_ON_TEST_POWER
/* 结束当前条件编译分支。 */
#endif

//#define FIXED_DUTY_MODE  // bypasses signal input and arming, uses a set duty cycle. For pumps, slot cars etc
//#define FIXED_DUTY_MODE_POWER 10     // 0-100 percent not used in fixed speed mode

//#define FIXED_SPEED_MODE  // bypasses input signal and runs at a fixed rpm using the speed control loop PID
//#define FIXED_SPEED_MODE_RPM  1000  // intended final rpm , ensure pole pair numbers are entered correctly in config tool.

//#define BRUSHED_MODE         // overrides all brushless config settings, enables two channels for brushed control
//#define GIMBAL_MODE     // also sinusoidal_startup needs to be on, maps input to sinusoidal angle.

//===========================================================================
//=============================  Defaults =============================
//===========================================================================

/* 保存按目标转速映射输入的使能标志。 */
uint8_t drive_by_rpm = 0;
/* 保存转速控制输入映射的最高机械转速。 */
uint32_t MAXIMUM_RPM_SPEED_CONTROL = 12000;
/* 保存转速控制输入映射的最低机械转速。 */
uint32_t MINIMUM_RPM_SPEED_CONTROL = 2000;

 //assign speed control PID values values are x10000
 /* 保存转速环 PID 参数及历史状态。 */
 fastPID speedPid = {      //commutation speed loop time
        /* 更新PID 比例增益。 */
 		.Kp = 10,
        /* 更新PID 积分增益。 */
 		.Ki = 0,
        /* 更新PID 微分增益。 */
 		.Kd = 100,
        /* 更新PID 积分项的对称限幅值。 */
 		.integral_limit = 10000,
        /* 更新PID 输出的对称限幅值。 */
 		.output_limit = 50000
 };

 /* 保存电流限制环 PID 参数及历史状态。 */
 fastPID currentPid = {   // 1khz loop time
        /* 更新PID 比例增益。 */
 		.Kp = 800,
        /* 更新PID 积分增益。 */
 		.Ki = 0,
        /* 更新PID 微分增益。 */
 		.Kd = 1000,
        /* 更新PID 积分项的对称限幅值。 */
 		.integral_limit = 20000,
        /* 更新PID 输出的对称限幅值。 */
 		.output_limit = 100000
 };

 /* 保存低速补偿环 PID 参数及历史状态。 */
 fastPID stallPid = {          //1khz loop time
        /* 更新PID 比例增益。 */
 		.Kp = 2,
        /* 更新PID 积分增益。 */
 		.Ki = 0,
        /* 更新PID 微分增益。 */
 		.Kd = 50,
        /* 更新PID 积分项的对称限幅值。 */
 		.integral_limit = 10000,
        /* 更新PID 输出的对称限幅值。 */
 		.output_limit = 50000
 };

/* 保存预留的目标电周期上界。 */
uint16_t target_e_com_time_high;
/* 保存预留的目标电周期下界。 */
uint16_t target_e_com_time_low;

/* 保存Flash 参数布局版本。 */
char eeprom_layout_version = 2;
/* 保存保存的电机默认方向反转设置。 */
char dir_reversed = 0;
/* 保存互补 PWM 使能标志。 */
char comp_pwm = 1;
/* 保存随转速调整 PWM 周期的使能标志。 */
char VARIABLE_PWM = 1;
/* 保存双向油门模式标志。 */
char bi_direction = 0;
/* 保存堵转超时停机保护使能。 */
char stuck_rotor_protection = 1;	// Turn off for Crawlers
/* 保存零油门时制动的设置。 */
char brake_on_stop = 0;
/* 保存停止后延迟关闭桥臂的周期计数。 */
uint16_t stop_counter_for_alloff=0; //the counter for pwm off for 0 input
/* 保存低速防停转补偿使能，区别于堵转停机保护。 */
char stall_protection = 0;
/* 保存正弦启动使能。 */
char use_sin_start = 0;
/* 保存固定间隔串口遥测使能。 */
char TLM_ON_INTERVAL = 0;
/* 保存周期遥测间隔，单位毫秒。 */
uint8_t telemetry_interval_ms = 30;
/* 保存温度降额门限，单位摄氏度，255 为默认不触发值。 */
uint8_t TEMPERATURE_LIMIT = 255;  // degrees 255 to disable
/* 保存换相提前角档位，中断路径每档约 7.5 电角度。 */
char advance_level = 2;			// 7.5 degree increments 0 , 7.5, 15, 22.5)
/* 保存电机 KV 参数，用于计算启动和斜率限制。 */
uint16_t motor_kv = 2000;
/* 保存电机磁极总数，极对数为此值的一半。 */
char motor_poles = 14;
/* 保存电流限制设定，单位 A。 */
uint16_t CURRENT_LIMIT = 202;
/* 保存正弦模式输出幅度的缩放参数。 */
uint8_t sine_mode_power = 5;
/* 保存停止时拖曳制动强度。 */
char drag_brake_strength = 10;		// Drag Brake Power when brake on stop is enabled
/* 保存运行制动强度，配置加载时影响死区补偿。 */
uint8_t driving_brake_strength = 10;
/* 保存运行参数中的死区补偿值。 */
uint8_t dead_time_override = DEAD_TIME;
/* 保存正弦启动到六步运行的油门切换百分比。 */
char sine_mode_changeover_thottle_level = 5;	// Sine Startup Range
/* 保存低速补偿目标换相间隔。 */
uint16_t stall_protect_target_interval = TARGET_STALL_PROTECTION_INTERVAL;
/* 保存可选霍尔传感器模式标志。 */
char USE_HALL_SENSOR = 0;
/* 保存从六步状态切入正弦表时的角度偏移。 */
uint16_t enter_sine_angle = 180;

//============================= Servo Settings ==============================
/* 保存舵机 PWM 零油门脉宽端点。 */
uint16_t servo_low_threshold = 1100;	// anything below this point considered 0
/* 保存舵机 PWM 满油门脉宽端点。 */
uint16_t servo_high_threshold = 1900;	// anything above this point considered 2000 (max)
/* 保存双向舵机 PWM 中位脉宽。 */
uint16_t servo_neutral = 1500;
/* 保存双向油门中位死区参数。 */
uint8_t servo_dead_band = 100;

//========================= Battery Cuttoff Settings ========================
/* 保存低电压停机保护使能。 */
char LOW_VOLTAGE_CUTOFF = 0;		// Turn Low Voltage CUTOFF on or off
/* 保存单节低压门限，单位 0.01 V。 */
uint16_t low_cell_volt_cutoff = 330;	// 3.3volts per cell

//=========================== END EEPROM Defaults ===========================

//typedef struct __attribute__((packed)) {
//  uint8_t version_major;
//  uint8_t version_minor;
//  char device_name[12];
//} firmware_info_s;

//firmware_info_s __attribute__ ((section(".firmware_info"))) firmware_info = {
//  version_major: VERSION_MAJOR,
//  version_minor: VERSION_MINOR,
//  device_name: FIRMWARE_NAME
//};

/* 保存预留的配置版本变量。 */
uint8_t EEPROM_VERSION;
//move these to targets folder or peripherals for each mcu
/* 保存车模先制动再反转模式标志。 */
char RC_CAR_REVERSE = 0;   // have to set bidirectional, comp_pwm off and stall protection off, no sinusoidal startup
/* 保存预留的 ADC 触发比较值。 */
uint16_t ADC_CCR = 30;
/* 保存云台模式当前角度索引。 */
uint16_t current_angle = 90;
/* 保存云台模式目标角度索引。 */
uint16_t desired_angle = 90;


//assign current control PID values

//PID speedPid = {
//		.Kp = 0.001,
//		.Ki = 0,
//		.Kd = 0.010,
//		.integral_limit = 1,
//		.output_limit = 5
//};
//
//PID currentPid = {
//		.Kp = 0.04,
//		.Ki = 0,
//		.Kd = 0.05,
//		.integral_limit = 2,
//		.output_limit = 10
//};
//
//PID stallPid = {
//		.Kp = 0.0002,
//		.Ki = 0.00000001,
//		.Kd = 0.05,
//		.integral_limit = 1,
//		.output_limit = 5
//};
/* 保存启动音已播放标志。 */
char boot_up_tune_played = 0;
/* 保存转速环目标电周期，单位微秒。 */
uint16_t target_e_com_time = 0;
/* 保存预留的转速 PID 输出变量。 */
int16_t Speed_pid_output;
/* 保存转速闭环使能。 */
char use_speed_control_loop = 0;
/* 保存转速 PID 生成的油门替代值。 */
float input_override = 0;
/* 保存电流环给出的占空比上限。 */
int16_t	use_current_limit_adjust = 2000;
/* 保存电流限制环使能。 */
char use_current_limit = 0;
/* 保存低速补偿追加的占空比。 */
float stall_protection_adjust = 0;

/* 保存预留的 MCU 标识。 */
uint32_t MCU_Id = 0;
/* 保存预留的芯片修订标识。 */
uint32_t REV_Id = 0;

/* 保存满足零油门解锁条件的时间计数。 */
uint16_t armed_timeout_count;
/* 保存允许改变旋转方向的换相间隔门限。 */
uint16_t reverse_speed_threshold = 1500;
/* 保存检测到失步的次数。 */
uint8_t desync_happened = 0;
/* 保存占空比变化斜率限制使能。 */
char maximum_throttle_change_ramp = 1;
/* 保存按电机和电池估算的斜率系数，放大 10 倍保存。 */
int target_rampratio_mul10=0;
/* 保存按 KV 和电池电压估计的机械转速上限。 */
int targetmax_rpm=0;

/* 保存保留的爬车模式参数，部分超时和占空比逻辑仍引用它。 */
char crawler_mode = 0;  // no longer used //
/* 保存预留的速度计数。 */
uint16_t velocity_count = 0;
/* 保存预留的速度计数门限。 */
uint16_t velocity_count_threshold = 75;

/* 保存低转速限制最大占空比的使能标志。 */
char low_rpm_throttle_limit = 1;

/* 保存低电压连续检测计数。 */
uint16_t low_voltage_count = 0;
/* 保存周期遥测调度计数，以周期控制回调为步长。 */
uint16_t telem_ms_count;

/* 保存电压分压倍数乘 10 的整数系数。 */
char VOLTAGE_DIVIDER = TARGET_VOLTAGE_DIVIDER;     // 100k upper and 10k lower resistor in divider
/* 保存滤波后的电池电压，单位 0.01 V。 */
uint16_t battery_voltage;  // scale in volts * 10.  1260 is a battery voltage of 12.60
/* 保存估计的串联电芯数量。 */
char cell_count = 0;
/* 保存有刷模式桥臂方向已配置标志。 */
char brushed_direction_set = 0;

/* 保存周期控制回调累计计数。 */
uint16_t tenkhzcounter = 0;
/* 保存累计耗电量，单位 mAh。 */
float consumed_current = 0;
/* 保存低通滤波后的电流 ADC 原始值。 */
uint16_t smoothed_raw_current = 0;
/* 保存换算后的电流，当前公式单位约为 0.01 A。 */
uint16_t actual_current = 0;

/* 保存保留的低 KV 标志。 */
char lowkv = 0;

/* 保存启动时最小 PWM 比较值。 */
uint16_t min_startup_duty = 120;
/* 保存正弦启动所用的最小占空比参数。 */
uint16_t sin_mode_min_s_d = 120;
/* 保存允许过零超时的次数门限。 */
char bemf_timeout = 10;

/* 保存保留的启动增强参数。 */
char startup_boost = 50;
/* 保存DShot 双向油门映射的死区修正量。 */
char reversing_dead_band = 1;

/* 保存启动输入检查中采到低电平的次数。 */
uint16_t low_pin_count = 0;


/* 保存快速加速时修正换相等待时间的标志。 */
char fast_accel = 1;
/* 保存上一次输出的基准占空比，用于斜率限制。 */
uint16_t last_duty_cycle = 0;
/* 保存待播放的设置提示音标志。 */
char play_tone_flag = 0;

/* 定义 GPIO 电平状态的枚举类型。 */
typedef enum
{
  /* GPIO 逻辑电平枚举：RESET 为低电平，SET 为高电平。 */
  GPIO_PIN_RESET = 0U,
  /* GPIO 逻辑电平枚举：RESET 为低电平，SET 为高电平。 */
  GPIO_PIN_SET
/* 结束类型定义，并提供可直接使用的类型别名。 */
}GPIO_PinState;

/* 保存启动阶段允许的最大 PWM 比较值。 */
uint16_t startup_max_duty_cycle = 300 + DEAD_TIME;
/* 保存正常驱动最小 PWM 比较值。 */
uint16_t minimum_duty_cycle = DEAD_TIME;
/* 保存低速补偿模式的最小 PWM 比较值。 */
uint16_t stall_protect_minimum_duty = DEAD_TIME;
/* 保存完成一轮换相后执行失步检查的标志。 */
char desync_check = 0;
/* 保存保留的低 KV 滤波参数。 */
char low_kv_filter_level = 20;

/* 保存当前实际使用的 PWM 周期值。 */
uint16_t tim1_arr = TIM1_AUTORELOAD;         // current auto reset value
/* 保存占空比换算所依赖的基准 PWM 周期值。 */
uint16_t TIMER1_MAX_ARR = TIM1_AUTORELOAD;      // maximum auto reset register value
/* 保存变频 PWM 允许的最小周期值。 */
int VARIABLE_PWM_MIN_ARR= TIM1_AUTORELOAD / 2;
/* 保存温度或低转速保护给出的占空比上限。 */
uint16_t duty_cycle_maximum = TIM1_AUTORELOAD;     //restricted by temperature or low rpm throttle protect
/* 保存低转速限幅区间的下端点。 */
uint16_t low_rpm_level  = 20;        // thousand erpm used to set range for throttle resrictions
/* 保存低转速限幅区间的上端点。 */
uint16_t high_rpm_level = 70;      //
/* 保存低转速端允许的最大占空比。 */
uint16_t throttle_max_at_low_rpm  = 400;
/* 保存高转速端允许的最大占空比。 */
uint16_t throttle_max_at_high_rpm = TIM1_AUTORELOAD;

/* 保存最近六个扇区的换相间隔记录。 */
uint16_t commutation_intervals[6] = {0};
/* 保存由电周期折算的平均扇区计数，F421 单位 0.5 微秒。 */
uint32_t average_interval = 0;
/* 保存上一轮用于失步比较的平均扇区计数。 */
uint32_t last_average_interval;
/* 保存电气旋转一周的估计时间，单位微秒。 */
int e_com_time;

/* 保存滤波后的模拟油门采样值。 */
uint16_t ADC_smoothed_input = 0;
/* 保存用于保护和遥测的滤波温度，单位摄氏度。 */
uint8_t degrees_celsius;
/* 保存内部温度 ADC 换算的中间结果。 */
uint16_t converted_degrees;
/* 保存温度 ADC 原始值偏移修正量。 */
uint8_t temperature_offset;
/* 保存温度通道 ADC 原始值。 */
uint16_t ADC_raw_temp;
/* 保存电压通道 ADC 原始值。 */
uint16_t ADC_raw_volts;
/* 保存电流通道 ADC 原始值。 */
uint16_t ADC_raw_current;
/* 保存可选模拟油门 ADC 原始值。 */
uint16_t ADC_raw_input;
/* 保存主循环 ADC 更新分频计数，启动时也作为流程标记。 */
uint8_t adc_counter = 0;
/* 保存待发送串口遥测标志。 */
char send_telemetry = 0;
/* 保存保留的遥测完成状态。 */
char telemetry_done = 0;
/* 保存比例制动当前生效标志。 */
char prop_brake_active = 0;

/* 保存176 字节 Flash 参数及旋律数据的 RAM 镜像。 */
uint8_t eepromBuffer[176] ={0};

/* 保存双向 DShot 遥测模式标志。 */
char dshot_telemetry = 0;

/* 保存最近一次执行的 DShot 命令编号。 */
uint8_t last_dshot_command = 0;

/* 保存过零检测路径选择，1 为主循环轮询，0 为中断路径。 */
char old_routine = 0;
/* 保存允许等待过零的最长间隔，F421 单位 0.5 微秒。 */
int comm_timeout=50000;
/* 仅在定义 POWER_ON_TEST_MODE 时编译以下代码。 */
#ifdef POWER_ON_TEST_MODE
/* 保存试转过程中强制换相累计次数。 */
volatile uint8_t power_on_test_forced_steps = 0;
/* 保存上电试转故障锁存标志，置位后阻止再次启动。 */
volatile uint8_t power_on_test_fault = 0;
/* 保存试转期间有效 BEMF 读数计数的峰值。 */
volatile uint8_t power_on_test_bemf_peak = 0;
/* 保存试转期间过零计数的峰值。 */
volatile uint32_t power_on_test_zero_cross_peak = 0;
/* 保存捕获阶段连续有效过零次数。 */
volatile uint8_t power_on_test_valid_zero_crosses = 0;
/* 保存进入捕获速度范围后的连续超时次数。 */
volatile uint8_t power_on_test_acquire_timeouts = 0;
/* 保存试转已达到有效过零锁定条件的标志。 */
volatile uint8_t power_on_test_closed_loop = 0;
/* 结束当前条件编译分支。 */
#endif


/* 保存经过方向和死区处理的油门值。 */
uint16_t adjusted_input = 0;

/* 保留的 30 摄氏度温度校准数据地址。 */
#define TEMP30_CAL_VALUE            ((uint16_t*)((uint32_t)0x1FFFF7B8))
/* 保留的 110 摄氏度温度校准数据地址。 */
#define TEMP110_CAL_VALUE           ((uint16_t*)((uint32_t)0x1FFFF7C2))

/* 保存换相间隔的滑动平均值，名称沿用旧代码。 */
uint16_t smoothedinput = 0;
/* 保存换相间隔滑动平均窗口长度。 */
const uint8_t numReadings = 30;     // the readings from the analog input
/* 保存滑动平均环形缓冲区写入位置。 */
uint8_t readIndex = 0;              // the index of the current reading
/* 保存滑动平均窗口内的数值总和。 */
int total = 0;
/* 保存换相间隔滑动平均的历史缓冲区。 */
uint16_t readings[30];

/* 保存累计过零超时次数，用于降占空比和停机判断。 */
uint8_t bemf_timeout_happened = 0;
/* 保存正弦启动切到六步模式时的衔接步骤。 */
uint8_t changeover_step = 5;
/* 保存比较器中断软件过滤的读数次数参数。 */
uint8_t filter_level = 5;
/* 保存电机控制流程正在运行的标志。 */
uint8_t running = 0;
/* 保存提前角对应的时间计数。 */
uint16_t advance = 0;
/* 保存轮询路径计算提前时间时使用的除数。 */
uint8_t advancedivisor = 6;
/* 保存预期浮空相电压上升标志；比较器输出方向与其相反。 */
char rising = 1;


/* 保存换相附近的 PWM 调整阶段，负值表示已检测过零等待换相。 */
int inner_step=1;
/* 保存最终交给 PWM 寄存器的比较值缓存。 */
uint16_t last_adjusted_duty_cycle=0;
/**
 * @brief 按过零超时情况限制占空比，再把调整后的值写入三个 PWM 通道；可选换相瞬间降占空比。
 */
void setPwmRatio(){
    /* 检查跨行列出的组合条件后选择处理分支。 */
	if(!crawler_mode
/* 仅在定义 POWER_ON_TEST_MODE 时编译以下代码。 */
#ifdef POWER_ON_TEST_MODE
        /* 续接上一行的位组合或条件判断。 */
		&& (power_on_test_closed_loop || power_on_test_fault)
/* 结束当前条件编译分支。 */
#endif
	){
        /* 保存本次 PWM 更新使用的过零超时次数快照。 */
		int bbemf_timeout_count=bemf_timeout_happened;
		//bbemf_timeout_count=8;
        /* 保存按过零超时次数得到的占空比上限。 */
		int maxduty;
        /* 当本次 PWM 更新使用的过零超时次数快照 不小于 4 时进入此分支。 */
		if(bbemf_timeout_count>=4){
            /* 更新按过零超时次数得到的占空比上限。 */
			maxduty=300;
        /* 当本次 PWM 更新使用的过零超时次数快照 不小于 2 时进入此分支。 */
		}else if(bbemf_timeout_count>=2){
            /* 更新按过零超时次数得到的占空比上限。 */
			maxduty=600;
        /* 当本次 PWM 更新使用的过零超时次数快照 不小于 1 时进入此分支。 */
		}else if(bbemf_timeout_count>=1){
            /* 更新按过零超时次数得到的占空比上限。 */
			maxduty=1200;
        /* 当前条件不满足时进入备选处理。 */
		}else{
            /* 更新按过零超时次数得到的占空比上限。 */
			maxduty=2000;
		}
        /* 更新按过零超时次数得到的占空比上限。 */
		maxduty=maxduty*tim1_arr/2000;
        /* 当最终交给 PWM 寄存器的比较值缓存 大于 按过零超时次数得到的占空比上限 时进入此分支。 */
		if(last_adjusted_duty_cycle>maxduty){
            /* 更新最终交给 PWM 寄存器的比较值缓存。 */
			last_adjusted_duty_cycle=maxduty;
		}
	}
	
/* 条件编译：defined(USE_INNER_STEP)；条件满足时采用以下实现。 */
#if defined(USE_INNER_STEP)
    /* 保存本次写入三相的 PWM 比较值。 */
	uint32_t targetduty;	
    /* 当换相附近的 PWM 调整阶段 小于 0 时进入此分支。 */
	if(inner_step<0){
        /* 更新本次写入三相的 PWM 比较值。 */
		targetduty = (uint32_t)last_adjusted_duty_cycle*95/100;
    /* 当换相附近的 PWM 调整阶段 不大于 1 时进入此分支。 */
	}else if(inner_step<=1){
        /* 更新本次写入三相的 PWM 比较值。 */
		targetduty = (uint32_t)last_adjusted_duty_cycle*90/100;
        /* 递增换相附近的 PWM 调整阶段，负值表示已检测过零等待换相。 */
		++inner_step;
    /* 当前条件不满足时进入备选处理。 */
	}else {
        /* 更新本次写入三相的 PWM 比较值。 */
		targetduty = (uint32_t)last_adjusted_duty_cycle;
	}
    /* 更新TMR1 的通道 1 捕获/比较寄存器。 */
	TMR1->c1dt = targetduty;
    /* 更新TMR1 的通道 2 捕获/比较寄存器。 */
	TMR1->c2dt = targetduty;
    /* 更新TMR1 的通道 3 捕获/比较寄存器。 */
	TMR1->c3dt = targetduty;
/* 采用上一编译条件不成立时的备选实现。 */
#else
    /* 更新TMR1 的通道 1 捕获/比较寄存器。 */
	TMR1->c1dt = last_adjusted_duty_cycle;
    /* 更新TMR1 的通道 2 捕获/比较寄存器。 */
	TMR1->c2dt = last_adjusted_duty_cycle;
    /* 更新TMR1 的通道 3 捕获/比较寄存器。 */
	TMR1->c3dt = last_adjusted_duty_cycle;
/* 结束当前条件编译分支。 */
#endif	
}

////Space Vector PWM ////////////////
//const int pwmSin[] ={128, 132, 136, 140, 143, 147, 151, 155, 159, 162, 166, 170, 174, 178, 181, 185, 189, 192, 196, 200, 203, 207, 211, 214, 218, 221, 225, 228, 232, 235, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 248, 249, 250, 250, 251, 252, 252, 253, 253, 253, 254, 254, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 254, 254, 253, 253, 253, 252, 252, 251, 250, 250, 249, 248, 248, 247, 246, 245, 244, 243, 242, 241, 240, 239, 238, 239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 248, 249, 250, 250, 251, 252, 252, 253, 253, 253, 254, 254, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 254, 254, 254, 253, 253, 253, 252, 252, 251, 250, 250, 249, 248, 248, 247, 246, 245, 244, 243, 242, 241, 240, 239, 238, 235, 232, 228, 225, 221, 218, 214, 211, 207, 203, 200, 196, 192, 189, 185, 181, 178, 174, 170, 166, 162, 159, 155, 151, 147, 143, 140, 136, 132, 128, 124, 120, 116, 113, 109, 105, 101, 97, 94, 90, 86, 82, 78, 75, 71, 67, 64, 60, 56, 53, 49, 45, 42, 38, 35, 31, 28, 24, 21, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 8, 7, 6, 6, 5, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 6, 6, 7, 8, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 8, 7, 6, 6, 5, 4, 4, 3, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 5, 6, 6, 7, 8, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 21, 24, 28, 31, 35, 38, 42, 45, 49, 53, 56, 60, 64, 67, 71, 75, 78, 82, 86, 90, 94, 97, 101, 105, 109, 113, 116, 120, 124};

////Sine Wave PWM ///////////////////
/* 保存一电周期的正弦 PWM 查表数据，索引范围 0～359。 */
int16_t pwmSin[] = {180,183,186,189,193,196,199,202,
		205,208,211,214,217,220,224,227,
		230,233,236,239,242,245,247,250,
		253,256,259,262,265,267,270,273,
		275,278,281,283,286,288,291,293,
		296,298,300,303,305,307,309,312,
		314,316,318,320,322,324,326,327,
		329,331,333,334,336,337,339,340,
		342,343,344,346,347,348,349,350,
		351,352,353,354,355,355,356,357,
		357,358,358,359,359,359,360,360,
		360,360,360,360,360,360,360,359,
		359,359,358,358,357,357,356,355,
		355,354,353,352,351,350,349,348,
		347,346,344,343,342,340,339,337,
		336,334,333,331,329,327,326,324,
		322,320,318,316,314,312,309,307,
		305,303,300,298,296,293,291,288,
		286,283,281,278,275,273,270,267,
		265,262,259,256,253,250,247,245,
		242,239,236,233,230,227,224,220,
		217,214,211,208,205,202,199,196,
		193,189,186,183,180,177,174,171,
		167,164,161,158,155,152,149,146,
		143,140,136,133,130,127,124,121,
		118,115,113,110,107,104,101,98,
		95,93,90,87,85,82,79,77,
		74,72,69,67,64,62,60,57,
		55,53,51,48,46,44,42,40,
		38,36,34,33,31,29,27,26,
		24,23,21,20,18,17,16,14,
		13,12,11,10,9,8,7,6,
		5,5,4,3,3,2,2,1,
		1,1,0,0,0,0,0,0,
		0,0,0,1,1,1,2,2,
		3,3,4,5,5,6,7,8,
		9,10,11,12,13,14,16,17,
		18,20,21,23,24,26,27,29,
		31,33,34,36,38,40,42,44,
		46,48,51,53,55,57,60,62,
		64,67,69,72,74,77,79,82,
		85,87,90,93,95,98,101,104,
		107,110,113,115,118,121,124,127,
		130,133,136,140,143,146,149,152,
		155,158,161,164,167,171,174,177};


//int sin_divider = 2;
/* 保存A 相正弦查表角度索引。 */
int16_t phase_A_position;
/* 保存B 相正弦查表角度索引。 */
int16_t phase_B_position;
/* 保存C 相正弦查表角度索引。 */
int16_t phase_C_position;
/* 保存正弦模式每次角度推进的延时，单位微秒。 */
uint16_t step_delay  = 100;
/* 保存当前处于正弦步进模式的标志。 */
char stepper_sine = 0;
/* 保存当前六步旋转方向，1 递增步骤，0 递减步骤。 */
char forward = 1;
/* 保存正弦 PWM 输出的驱动补偿量。 */
uint16_t gate_drive_offset = DEAD_TIME;

/* 保存主循环两次运行之间进入过零中断的次数。 */
uint8_t stuckcounter = 0;
/* 保存电转速的千转每分钟尺度值。 */
uint16_t k_erpm;
/* 保存电转速除以 100 后的遥测数值。 */
uint16_t e_rpm;      // electrical revolution /100 so,  123 is 12300 erpm

/* 保存按当前 PWM 周期缩放后的实际比较值。 */
uint16_t adjusted_duty_cycle;

/* 保存轮询检测到错误方向电平的累计次数。 */
uint8_t bad_count = 0;
/* 保存错误电平累计次数门限。 */
uint8_t bad_count_threshold = CPU_FREQUENCY_MHZ / 24;
/* 保存当前待处理的 DShot 命令编号。 */
uint8_t dshotcommand;
/* 保存输入协议初始化时设置的解锁计数参数。 */
uint16_t armed_count_threshold = 1000;

/* 保存电调已解锁并允许驱动的标志。 */
char armed = 0;
/* 保存连续检测到零油门的次数。 */
uint16_t zero_input_count = 0;

/* 保存交给电机控制状态机的最终油门值。 */
uint16_t input = 0;
/* 保存输入协议刚解码得到的油门或命令数值。 */
uint16_t newinput =0;
/* 保存已识别输入信号类型的标志。 */
char inputSet = 0;
/* 保存当前输入识别为 DShot 的标志。 */
char dshot = 0;
/* 保存当前输入识别为舵机 PWM 的标志。 */
char servoPwm = 0;
/* 保存过零/换相累计计数；中断模式的换相回调也会递增。 */
uint32_t zero_crosses;

/* 保存轮询路径本次过零已处理的标志。 */
uint8_t zcfound = 0;

/* 保存轮询中满足预期电平的读数累计次数。 */
uint8_t bemfcounter;
/* 保存上升方向过零的有效读数门限。 */
uint8_t min_bemf_counts_up = TARGET_MIN_BEMF_COUNTS;
/* 保存下降方向过零的有效读数门限。 */
uint8_t min_bemf_counts_down = TARGET_MIN_BEMF_COUNTS;

/* 保存保留的上一次过零时间记录。 */
uint16_t lastzctime;
/* 保存本次过零或超时发生时的间隔计数。 */
uint16_t thiszctime;

/* 保存以基准 PWM 周期表示的目标比较值。 */
uint16_t duty_cycle = 0;
/* 保存当前六步换相步骤，正常范围为 1～6。 */
char step = 1;
/* 保存单扇区的估计时间计数，F421 单位 0.5 微秒。 */
uint16_t commutation_interval = 12500;
/* 保存过零到下一次换相的等待计数。 */
uint16_t waitTime = 0;
/* 保存自最近有效输入以来的周期计数。 */
uint16_t signaltimeout = 0;
/* 保存预留的模拟看门狗状态。 */
uint8_t ubAnalogWatchdogStatus = RESET;


/**
 * @brief 采样输入引脚电平；检测到持续高电平时关闭桥臂并复位，否则返回继续启动流程。
 */
void checkForHighSignal(){
/* 把信号定时器切换为输入捕获模式，恢复接收时基并标记当前处于接收阶段。 */
changeToInput();
    /* 切换选定 GPIO 的工作模式；具体驱动电平由随后的置位/清零或 PWM 决定。 */
	gpio_mode_QUICK(INPUT_PIN_PORT, GPIO_MODE_INPUT, GPIO_PULL_DOWN, INPUT_PIN);
/* 使用 UTILITY_TIMER 忙等延时；会清零共享计数器，时间基准取决于当前定时器分频。 */
delayMicros(100);
/* 按固定采样间隔重复检查信号电平，区分持续高电平和有效输入。 */
for(int i = 0 ; i < 1000; i ++){
     /* 按条件 !((INPUT_PIN_PORT->idt & INPUT_PIN)) 选择当前处理路径。 */
	 if(!((INPUT_PIN_PORT->idt & INPUT_PIN))){  // if the pin is low for 5 checks out of 100 in  100ms or more its either no signal or signal. jump to application
         /* 递增启动输入检查中采到低电平的次数。 */
		 low_pin_count++;
	 }
     /* 使用 UTILITY_TIMER 忙等延时；会清零共享计数器，时间基准取决于当前定时器分频。 */
     delayMicros(10);
}
/* 切换选定 GPIO 的工作模式；具体驱动电平由随后的置位/清零或 PWM 决定。 */
gpio_mode_QUICK(INPUT_PIN_PORT, GPIO_MODE_MUX, GPIO_PULL_NONE, INPUT_PIN);
     /* 当启动输入检查中采到低电平的次数 大于 5 时进入此分支。 */
	 if(low_pin_count > 5){
         /* 结束当前函数，返回调用方。 */
		 return;      // its either a signal or a disconnected pin
     /* 当前条件不满足时进入备选处理。 */
	 }else{
        /* 将 A、B、C 三相全部置为浮空，关闭各相高低侧驱动。 */
		allOff();
		NVIC_SystemReset();
	 }
}



/**
 * @brief 计算带积分和输出限幅的整数 PID；误差定义为 actual-target，返回未除缩放系数的输出。
 *
 * 积分使用本次与上次误差之和乘 Ki；未显式乘采样周期，增益需与调用频率一起理解。
 * 控制环调用方通常再除以 10000；函数内部只负责累计状态和限幅。
 * @param pidnow PID 参数和历史状态，函数会原地更新。
 * @param actual 当前测量值，单位必须与目标值一致。
 * @param target 参考目标值。
 * @return 限幅后的 PID 输出，调用方根据控制环再除缩放系数。
 */
float doPidCalculations(struct fastPID *pidnow, int actual, int target){

    /* 更新PID 当前误差。 */
	pidnow->error = actual - target;
    /* 更新PID 积分累计量。 */
	pidnow->integral = pidnow->integral + pidnow->error*pidnow->Ki + pidnow->last_error*pidnow->Ki;
    /* 当integral > pidnow- 大于 PID 积分项的对称限幅值 时进入此分支。 */
	if(pidnow->integral > pidnow->integral_limit){
        /* 更新PID 积分累计量。 */
		pidnow->integral = pidnow->integral_limit;
	}
    /* 当integral < -pidnow- 大于 PID 积分项的对称限幅值 时进入此分支。 */
	if(pidnow->integral < -pidnow->integral_limit){
        /* 更新PID 积分累计量。 */
		pidnow->integral = -pidnow->integral_limit;
	}

    /* 更新PID 相邻误差变化形成的微分项。 */
	pidnow->derivative = pidnow->Kd * (pidnow->error - pidnow->last_error);
    /* 更新PID 上一次误差。 */
	pidnow->last_error = pidnow->error;

    /* 更新PID 合成输出。 */
	pidnow->pid_output = pidnow->error*pidnow->Kp + pidnow->integral + pidnow->derivative;


    /* 当pid_output>pidnow- 大于 PID 输出的对称限幅值 时进入此分支。 */
	if (pidnow->pid_output>pidnow->output_limit){
        /* 更新PID 合成输出。 */
		pidnow->pid_output = pidnow->output_limit;
    /* 当pid_output <-pidnow- 大于 PID 输出的对称限幅值 时进入此分支。 */
	}if(pidnow->pid_output <-pidnow->output_limit){
        /* 更新PID 合成输出。 */
		pidnow->pid_output = -pidnow->output_limit;
	}
    /* 返回PID 合成输出，尚未按调用方比例缩放。 */
	return pidnow->pid_output;

}


/**
 * @brief 从 Flash 读取 176 字节配置，按字段解码模式和参数，并为部分越界值保留或选择默认值。
 */
void loadEEpromSettings(){
       /* 从 Flash 映射地址逐字节读取到调用方缓冲区；调用方保证地址和容量有效。 */
	   read_flash_bin( eepromBuffer , EEPROM_START_ADD , 176);

       /* 当配置区中的默认方向反转 等于 0x01 时进入此分支。 */
	   if(eepromBuffer[17] == 0x01){
          /* 将保存的电机默认方向反转设置设为 1。 */
	 	  dir_reversed =  1;
       /* 当前条件不满足时进入备选处理。 */
	   }else{
           /* 将保存的电机默认方向反转设置清零。 */
		   dir_reversed = 0;
	   }
       /* 当配置区中的双向油门 等于 0x01 时进入此分支。 */
	   if(eepromBuffer[18] == 0x01){
          /* 将双向油门模式标志设为 1。 */
	 	  bi_direction = 1;
       /* 当前条件不满足时进入备选处理。 */
	   }else{
          /* 将双向油门模式标志清零。 */
		  bi_direction = 0;
	   }
       /* 当配置区中的正弦启动 等于 0x01 时进入此分支。 */
	   if(eepromBuffer[19] == 0x01){
          /* 将正弦启动使能设为 1。 */
	 	  use_sin_start = 1;
	 //	 min_startup_duty = sin_mode_min_s_d;
	   }
       /* 当配置区中的互补 PWM 等于 0x01 时进入此分支。 */
	   if(eepromBuffer[20] == 0x01){
          /* 将互补 PWM 使能标志设为 1。 */
	  	  comp_pwm = 1;
        /* 当前条件不满足时进入备选处理。 */
	    }else{
            /* 将互补 PWM 使能标志清零。 */
	    	comp_pwm = 0;
	    }
       /* 当配置区中的变频 PWM 等于 0x01 时进入此分支。 */
	   if(eepromBuffer[21] == 0x01){
           /* 将随转速调整 PWM 周期的使能标志设为 1。 */
		   VARIABLE_PWM = 1;
        /* 当前条件不满足时进入备选处理。 */
	    }else{
            /* 将随转速调整 PWM 周期的使能标志清零。 */
	    	VARIABLE_PWM = 0;
	    }
       /* 当配置区中的堵转停机保护 等于 0x01 时进入此分支。 */
	   if(eepromBuffer[22] == 0x01){
           /* 将堵转超时停机保护使能设为 1。 */
		   stuck_rotor_protection = 1;
        /* 当前条件不满足时进入备选处理。 */
	    }else{
            /* 将堵转超时停机保护使能清零。 */
	    	stuck_rotor_protection = 0;
	    }
       /* 当配置区中的提前角档位 小于 4 时进入此分支。 */
	   if(eepromBuffer[23] < 4){
           /* 更新换相提前角档位。 */
		   advance_level = eepromBuffer[23];
        /* 当前条件不满足时进入备选处理。 */
	    }else{
            /* 更新换相提前角档位。 */
	    	advance_level = 2;  // * 7.5 increments
	    }

       /* 检查PWM 频率参数的配置值是否满足本分支要求。 */
	   if(eepromBuffer[24] < 49 && eepromBuffer[24] > 7){
           /* 检查PWM 频率参数的配置值是否满足本分支要求。 */
		   if(eepromBuffer[24] < 49 && eepromBuffer[24] > 23){
               /* 更新占空比换算所依赖的基准 PWM 周期值。 */
			   TIMER1_MAX_ARR = map (eepromBuffer[24], 24, 144, TIM1_AUTORELOAD,TIM1_AUTORELOAD/6);
		   }
           /* 检查PWM 频率参数的配置值是否满足本分支要求。 */
		   if(eepromBuffer[24] < 24 && eepromBuffer[24] > 11){
               /* 更新占空比换算所依赖的基准 PWM 周期值。 */
			   TIMER1_MAX_ARR = map (eepromBuffer[24], 12, 24, TIM1_AUTORELOAD *2 ,TIM1_AUTORELOAD);
		   }
           /* 检查PWM 频率参数的配置值是否满足本分支要求。 */
		   if(eepromBuffer[24] < 12 && eepromBuffer[24] > 7){
               /* 更新占空比换算所依赖的基准 PWM 周期值。 */
			   TIMER1_MAX_ARR = map (eepromBuffer[24], 7, 16, TIM1_AUTORELOAD *3 ,TIM1_AUTORELOAD/2*3);
		   }
           /* 更新TMR1 的周期/自动重装载寄存器。 */
		   TMR1->pr  = TIMER1_MAX_ARR;
           /* 更新高转速端允许的最大占空比。 */
		   throttle_max_at_high_rpm = TIMER1_MAX_ARR;
           /* 更新温度或低转速保护给出的占空比上限。 */
		   duty_cycle_maximum = TIMER1_MAX_ARR;
        /* 当前条件不满足时进入备选处理。 */
	    }else{
            /* 更新当前实际使用的 PWM 周期值。 */
	    	tim1_arr = TIM1_AUTORELOAD;
            /* 更新TMR1 的周期/自动重装载寄存器。 */
	    	TMR1->pr = tim1_arr;
	    }

       /* 检查启动功率的配置值是否满足本分支要求。 */
	   if(eepromBuffer[25] < 151 && eepromBuffer[25] > 49){
       /* 更新启动时最小 PWM 比较值。 */
	   min_startup_duty = (eepromBuffer[25]/2 + DEAD_TIME) * TIMER1_MAX_ARR / 2000;
       /* 更新正常驱动最小 PWM 比较值。 */
	   minimum_duty_cycle = (eepromBuffer[25]/ 4 + DEAD_TIME/4) * TIMER1_MAX_ARR / 2000 ;
       /* 更新低速补偿模式的最小 PWM 比较值。 */
	   stall_protect_minimum_duty = minimum_duty_cycle+10;
        /* 当前条件不满足时进入备选处理。 */
	    }else{
            /* 更新启动时最小 PWM 比较值。 */
	    	min_startup_duty = 150;
            /* 更新正常驱动最小 PWM 比较值。 */
	    	minimum_duty_cycle = (min_startup_duty / 2) + 10;
	    }
      /* 更新电机 KV 参数。 */
      motor_kv = (eepromBuffer[26] * 40) + 20;
      /* 更新电机磁极总数。 */
      motor_poles = eepromBuffer[27];
       /* 当配置区中的停止制动 等于 0x01 时进入此分支。 */
	   if(eepromBuffer[28] == 0x01){
           /* 将零油门时制动的设置设为 1。 */
		   brake_on_stop = 1;
        /* 当前条件不满足时进入备选处理。 */
	    }else{
            /* 将零油门时制动的设置清零。 */
	    	brake_on_stop = 0;
	    }
       /* 当配置区中的低速补偿 等于 0x01 时进入此分支。 */
	   if(eepromBuffer[29] == 0x01){
           /* 将低速防停转补偿使能设为 1。 */
		   stall_protection = 1;
        /* 当前条件不满足时进入备选处理。 */
	    }else{
            /* 将低速防停转补偿使能清零。 */
	    	stall_protection = 0;
	    }
       /* 将音量限制到代码规定范围并换算为鸣音 PWM 比较值。 */
	   setVolume(5);
       /* 当配置区中的参数布局版本 大于 0 时进入此分支。 */
	   if(eepromBuffer[1] > 0){             // these commands weren't introduced until eeprom version 1.

           /* 当配置区中的鸣音音量 大于 11 时进入此分支。 */
		   if(eepromBuffer[30] > 11){
               /* 将音量限制到代码规定范围并换算为鸣音 PWM 比较值。 */
			   setVolume(5);
           /* 当前条件不满足时进入备选处理。 */
		   }else{
               /* 将音量限制到代码规定范围并换算为鸣音 PWM 比较值。 */
			   setVolume(eepromBuffer[30]);
		   }
           /* 当配置区中的遥测间隔 等于 0x01 时进入此分支。 */
		   if(eepromBuffer[31] == 0x01){
               /* 将固定间隔串口遥测使能设为 1。 */
			   TLM_ON_INTERVAL = 1;
           /* 当前条件不满足时进入备选处理。 */
		   }else{
               /* 将固定间隔串口遥测使能清零。 */
			   TLM_ON_INTERVAL = 0;
		   }
           /* 更新舵机 PWM 零油门脉宽端点。 */
		   servo_low_threshold = (eepromBuffer[32]*2) + 750; // anything below this point considered 0
           /* 更新舵机 PWM 满油门脉宽端点。 */
		   servo_high_threshold = (eepromBuffer[33]*2) + 1750;;  // anything above this point considered 2000 (max)
           /* 更新双向舵机 PWM 中位脉宽。 */
		   servo_neutral = (eepromBuffer[34]) + 1374;
           /* 更新双向油门中位死区参数。 */
		   servo_dead_band = eepromBuffer[35];

           /* 当配置区中的低压保护开关 等于 0x01 时进入此分支。 */
		   if(eepromBuffer[36] == 0x01){
               /* 将低电压停机保护使能设为 1。 */
			   LOW_VOLTAGE_CUTOFF = 1;
           /* 当前条件不满足时进入备选处理。 */
		   }else{
               /* 将低电压停机保护使能清零。 */
			   LOW_VOLTAGE_CUTOFF = 0;
		   }

           /* 更新单节低压门限。 */
		   low_cell_volt_cutoff = eepromBuffer[37] + 250; // 2.5 to 3.5 volts per cell range
           /* 当配置区中的车模反转 等于 0x01 时进入此分支。 */
		   if(eepromBuffer[38] == 0x01){
               /* 将车模先制动再反转模式标志设为 1。 */
			   RC_CAR_REVERSE = 1;
           /* 当前条件不满足时进入备选处理。 */
		   }else{
               /* 将车模先制动再反转模式标志清零。 */
			   RC_CAR_REVERSE = 0;
		   }
           /* 当配置区中的霍尔/诊断选项 等于 0x01 时进入此分支。 */
		   if(eepromBuffer[39] == 0x01){
/* 仅在定义 HAS_HALL_SENSORS 时编译以下代码。 */
#ifdef HAS_HALL_SENSORS
               /* 将可选霍尔传感器模式标志设为 1。 */
			   USE_HALL_SENSOR = 1;
/* 采用上一编译条件不成立时的备选实现。 */
#else
               /* 将可选霍尔传感器模式标志清零。 */
			   USE_HALL_SENSOR = 0;
/* 结束当前条件编译分支。 */
#endif
           /* 当前条件不满足时进入备选处理。 */
		   }else{
               /* 将可选霍尔传感器模式标志清零。 */
			   USE_HALL_SENSOR = 0;
		   }
       /* 检查正弦切换油门的配置值是否满足本分支要求。 */
	   if(eepromBuffer[40] > 4 && eepromBuffer[40] < 26){            // sine mode changeover 5-25 percent throttle
       /* 更新正弦启动到六步运行的油门切换百分比。 */
       sine_mode_changeover_thottle_level = eepromBuffer[40];
	   }
       /* 检查拖曳制动强度的配置值是否满足本分支要求。 */
	   if(eepromBuffer[41] > 0 && eepromBuffer[41] < 11){        // drag brake 1-10
       /* 更新停止时拖曳制动强度。 */
       drag_brake_strength = eepromBuffer[41];
	   }
	   
       /* 检查运行制动强度的配置值是否满足本分支要求。 */
	   if(eepromBuffer[42] > 0 && eepromBuffer[42] < 10){        // motor brake 1-9
       /* 更新运行制动强度。 */
       driving_brake_strength = eepromBuffer[42];
       /* 更新运行参数中的死区补偿值。 */
	   dead_time_override = DEAD_TIME + (150 - (driving_brake_strength * 10));
       /* 当运行参数中的死区补偿值 大于 200 时进入此分支。 */
	   if(dead_time_override > 200){
       /* 更新运行参数中的死区补偿值。 */
	   dead_time_override = 200;
	   }
       /* 更新启动时最小 PWM 比较值。 */
	   min_startup_duty = eepromBuffer[25] + dead_time_override;
       /* 更新正常驱动最小 PWM 比较值。 */
	   minimum_duty_cycle = eepromBuffer[25]/2 + dead_time_override;
       /* 更新低转速端允许的最大占空比。 */
	   throttle_max_at_low_rpm  = throttle_max_at_low_rpm + dead_time_override;
       /* 更新启动阶段允许的最大 PWM 比较值。 */
	   startup_max_duty_cycle = startup_max_duty_cycle  + dead_time_override;
	//   TIMER_CCHP(TIMER0)|= TIMER_CCHP_DTCFG & dead_time_override;
	   }
	   
       /* 检查温度门限的配置值是否满足本分支要求。 */
	   if(eepromBuffer[43] >= 70 && eepromBuffer[43] <= 140){ 
       /* 更新温度降额门限。 */
	   TEMPERATURE_LIMIT = eepromBuffer[43];
	   
	   }
	   
       /* 检查电流门限编码的配置值是否满足本分支要求。 */
	   if(eepromBuffer[44] > 0 && eepromBuffer[44] < 100){
       /* 更新电流限制设定。 */
	   CURRENT_LIMIT = eepromBuffer[44] * 2;
       /* 将电流限制环使能设为 1。 */
	   use_current_limit = 1;
	   
	   }
       /* 检查正弦输出功率的配置值是否满足本分支要求。 */
	   if(eepromBuffer[45] > 0 && eepromBuffer[45] < 11){ 
       /* 更新正弦模式输出幅度的缩放参数。 */
	   sine_mode_power = eepromBuffer[45];
	   }
	   /*if(eepromBuffer[46] >= 0 && eepromBuffer[46] < 10){
		   switch (eepromBuffer[46]){
		   case AUTO_IN:
			   dshot= 0;
			   servoPwm = 0;
			   EDT_ARMED = 1;
			   break;
		   case DSHOT_IN:
			   dshot = 1;
			   EDT_ARMED = 1;
			   break;
		   case SERVO_IN:
			   servoPwm = 1;
			   break;
		   case SERIAL_IN:
			   break;
		   case EDTARM:
			   EDT_ARM_ENABLE = 1;
			   EDT_ARMED = 0;
			   dshot = 1;
			   break;
		   };
	   }else{
		   dshot = 0;
		   servoPwm = 0;
		   EDT_ARMED = 1;
	   }*/
       /* 当电机 KV 参数 小于 300 时进入此分支。 */
       if(motor_kv < 300){
           /* 将低转速限制最大占空比的使能标志清零。 */
		   low_rpm_throttle_limit = 0;
	   }
        /* 按条件 (motor_kv < 1500)|| (stall_protection) 选择当前处理路径。 */
		if((motor_kv < 1500)|| (stall_protection)){
            /* 更新上升方向过零的有效读数门限。 */
			min_bemf_counts_up = TARGET_MIN_BEMF_COUNTS *2;
            /* 更新下降方向过零的有效读数门限。 */
			min_bemf_counts_down = TARGET_MIN_BEMF_COUNTS *2;
		}
       /* 更新低转速限幅区间的下端点。 */
	   low_rpm_level  = motor_kv / 100 / (32 / motor_poles);
       /* 更新低转速限幅区间的上端点。 */
	   high_rpm_level = motor_kv / 17 / (32/motor_poles);
	   }
       /* 按当前输入区间限幅并线性映射，得到允许改变旋转方向的换相间隔门限。 */
	   reverse_speed_threshold =  map(motor_kv, 300, 3000, 2500 , 1250);
    /* 当互补 PWM 使能标志为零时进入此分支。 */
	if(!comp_pwm){
        /* 将双向油门模式标志清零。 */
		bi_direction = 0;
	}
	
    /* 按当前输入区间限幅并线性映射，得到变频 PWM 允许的最小周期值。 */
	VARIABLE_PWM_MIN_ARR=map(motor_kv, 2800, 4800, TIMER1_MAX_ARR/2 , TIMER1_MAX_ARR/3);



}

/**
 * @brief 把运行中的方向及模式等字段写回配置缓冲区，再保存整个 176 字节配置到 Flash。
 */
void saveEEpromSettings(){    

   /* 写入配置区中的参数布局版本，稍后由 Flash 保存流程持久化。 */
   eepromBuffer[1] = eeprom_layout_version;
   /* 当保存的电机默认方向反转设置 等于 1 时进入此分支。 */
   if(dir_reversed == 1){
       /* 写入配置区中的默认方向反转，稍后由 Flash 保存流程持久化。 */
	   eepromBuffer[17] = 0x01;
   /* 当前条件不满足时进入备选处理。 */
   }else{
       /* 写入配置区中的默认方向反转，稍后由 Flash 保存流程持久化。 */
	   eepromBuffer[17] = 0x00;
   }
   /* 当双向油门模式标志 等于 1 时进入此分支。 */
   if(bi_direction == 1){
       /* 写入配置区中的双向油门，稍后由 Flash 保存流程持久化。 */
	   eepromBuffer[18] = 0x01;
      /* 当前条件不满足时进入备选处理。 */
      }else{
          /* 写入配置区中的双向油门，稍后由 Flash 保存流程持久化。 */
    	  eepromBuffer[18] = 0x00;
      }
   /* 当正弦启动使能 等于 1 时进入此分支。 */
   if(use_sin_start == 1){
       /* 写入配置区中的正弦启动，稍后由 Flash 保存流程持久化。 */
	   eepromBuffer[19] = 0x01;
      /* 当前条件不满足时进入备选处理。 */
      }else{
          /* 写入配置区中的正弦启动，稍后由 Flash 保存流程持久化。 */
    	  eepromBuffer[19] = 0x00;
      }

   /* 当互补 PWM 使能标志 等于 1 时进入此分支。 */
   if(comp_pwm == 1){
       /* 写入配置区中的互补 PWM，稍后由 Flash 保存流程持久化。 */
	   eepromBuffer[20] = 0x01;
      /* 当前条件不满足时进入备选处理。 */
      }else{
          /* 写入配置区中的互补 PWM，稍后由 Flash 保存流程持久化。 */
    	  eepromBuffer[20] = 0x00;
      }
   /* 当随转速调整 PWM 周期的使能标志 等于 1 时进入此分支。 */
   if(VARIABLE_PWM == 1){
       /* 写入配置区中的变频 PWM，稍后由 Flash 保存流程持久化。 */
	   eepromBuffer[21] = 0x01;
      /* 当前条件不满足时进入备选处理。 */
      }else{
          /* 写入配置区中的变频 PWM，稍后由 Flash 保存流程持久化。 */
    	  eepromBuffer[21] = 0x00;
      }
   /* 当堵转超时停机保护使能 等于 1 时进入此分支。 */
   if(stuck_rotor_protection == 1){
       /* 写入配置区中的堵转停机保护，稍后由 Flash 保存流程持久化。 */
	   eepromBuffer[22] = 0x01;
      /* 当前条件不满足时进入备选处理。 */
      }else{
          /* 写入配置区中的堵转停机保护，稍后由 Flash 保存流程持久化。 */
    	  eepromBuffer[22] = 0x00;
      }
   /* 写入配置区中的提前角档位，稍后由 Flash 保存流程持久化。 */
   eepromBuffer[23] = advance_level;
   /* 把字节缓冲区按小端顺序组装为 32 位字后写入 Flash；最多写一页，尾部不足 4 字节不写入。 */
   save_flash_nolib(eepromBuffer, 176, EEPROM_START_ADD);
}



/**
 * @brief 维护 30 点滑动平均；实际平均对象是 commutation_interval，并非油门输入。
 */
void getSmoothedInput() {

        /* 更新滑动平均窗口内的数值总和。 */
		total = total - readings[readIndex];
        /* 更新换相间隔滑动平均的历史缓冲区中的当前元素。 */
		readings[readIndex] = commutation_interval;
        /* 更新滑动平均窗口内的数值总和。 */
		total = total + readings[readIndex];
        /* 更新滑动平均环形缓冲区写入位置。 */
		readIndex = readIndex + 1;
        /* 当滑动平均环形缓冲区写入位置 不小于 换相间隔滑动平均窗口长度 时进入此分支。 */
		if (readIndex >= numReadings) {
            /* 将滑动平均环形缓冲区写入位置清零。 */
			readIndex = 0;
		}
        /* 更新换相间隔的滑动平均值。 */
		smoothedinput = total / numReadings;


}

/**
 * @brief 轮询浮空相比较器电平，按预期方向累计有效读数；错误读数达到门限后清除有效计数。
 *
 * bemfcounter 统计符合方向的采样读数，并不直接统计完整的过零边沿。
 * bad_count 在换相收尾处复位，当前实现并非每个正确采样都将它清零。
 */
void getBemfState(){
    /* 保存对比较器输出取反后的浮空相状态。 */
	uint8_t current_state = 0;
/* 仅在定义 MCU_AT415 时编译以下代码。 */
#ifdef MCU_AT415
     /* 更新对比较器输出取反后的浮空相状态。 */
	 current_state = !(CMP->ctrlsts1_bit.cmp1value);  // polarity reversed
/* 采用上一编译条件不成立时的备选实现。 */
#else
    /* 比较器正端为 PA1 中性点、负端为浮空相；取反后 1 表示相电压高于中性点。 */
    current_state = !(CMP->ctrlsts_bit.cmpvalue);  // polarity reversed
/* 结束当前条件编译分支。 */
#endif
    /* 当前扇区等待浮空相电压上升过零。 */
    if (rising){
        /* 当对比较器输出取反后的浮空相状态非零时进入此分支。 */
    	if (current_state){
            /* 递增轮询中满足预期电平的读数累计次数。 */
    		bemfcounter++;
            /* 当前条件不满足时进入备选处理。 */
    		}else{
            /* 递增轮询检测到错误方向电平的累计次数。 */
    		bad_count++;
            /* 当轮询检测到错误方向电平的累计次数 大于 错误电平累计次数门限 时进入此分支。 */
    		if(bad_count > bad_count_threshold){
            /* 将轮询中满足预期电平的读数累计次数清零。 */
    		bemfcounter = 0;
    		}
   	}
    /* 当前条件不满足时进入备选处理。 */
    }else{
        /* 当对比较器输出取反后的浮空相状态为零时进入此分支。 */
    	if(!current_state){
            /* 递增轮询中满足预期电平的读数累计次数。 */
    		bemfcounter++;
        /* 当前条件不满足时进入备选处理。 */
    	}else{
            /* 递增轮询检测到错误方向电平的累计次数。 */
    		bad_count++;
            /* 当轮询检测到错误方向电平的累计次数 大于 错误电平累计次数门限 时进入此分支。 */
    	    if(bad_count > bad_count_threshold){
            /* 将轮询中满足预期电平的读数累计次数清零。 */
    	    bemfcounter = 0;
    	  }
    	}
    }
/* 仅在定义 POWER_ON_TEST_MODE 时编译以下代码。 */
#ifdef POWER_ON_TEST_MODE
    /* 当轮询中满足预期电平的读数累计次数 大于 试转期间有效 BEMF 读数计数的峰值 时进入此分支。 */
	if(bemfcounter > power_on_test_bemf_peak){
        /* 更新试转期间有效 BEMF 读数计数的峰值。 */
		power_on_test_bemf_peak = bemfcounter;
	}
/* 结束当前条件编译分支。 */
#endif
}


/**
 * @brief 推进或回退六步相序，更新电周期估计和过零方向，执行桥臂换相并切换比较器输入。
 *
 * 先将旧步骤的间隔写入六项历史表，再求和并右移一位，把 F421 的半微秒计数换成微秒。
 * 正转步骤递增，反转递减；旋转方向和浮空相预期边沿必须保持一致。
 */
void commutate(){
    /* 将换相附近的 PWM 调整阶段清零。 */
	inner_step=0;
    /* 更新最近六个扇区的换相间隔记录中的当前元素。 */
	commutation_intervals[step-1] = commutation_interval;
    /* 更新电气旋转一周的估计时间。 */
	e_com_time = (commutation_intervals[0] + commutation_intervals[1] + commutation_intervals[2] + commutation_intervals[3] + commutation_intervals[4] +commutation_intervals[5]) >> 1;  // COMMUTATION INTERVAL IS 0.5US INCREMENTS

    /* 当当前六步旋转方向 等于 1 时进入此分支。 */
	if (forward == 1){
        /* 递增当前六步换相步骤。 */
		step++;
        /* 当当前六步换相步骤 大于 6 时进入此分支。 */
		if (step > 6) {
            /* 将当前六步换相步骤设为 1。 */
			step = 1;
            /* 将完成一轮换相后执行失步检查的标志设为 1。 */
			desync_check = 1;
		}
        /* 正向递增六步相序时，奇数步等待浮空相上升过零，偶数步等待下降过零。 */
		rising = step % 2;
    /* 当前条件不满足时进入备选处理。 */
	}else{
        /* 递减当前六步换相步骤。 */
		step--;
        /* 当当前六步换相步骤 小于 1 时进入此分支。 */
		if (step < 1) {
            /* 更新当前六步换相步骤。 */
			step = 6;
            /* 将完成一轮换相后执行失步检查的标志设为 1。 */
			desync_check = 1;
		}
        /* 反向递减相序时，预期过零方向相对正向反转。 */
		rising = !(step % 2);
	}
/***************************************************/
    /* 将UTILITY_TIMER 的当前计数值清零。 */
	UTILITY_TIMER->cval = 0;
    /* 当比例制动当前生效标志为零时进入此分支。 */
	if(!prop_brake_active){
    /* 按 1～6 的步骤号执行六步换相，每步包含一相 PWM、一相低侧导通和一相浮空。 */
	comStep(step);
	}
  /* 更新桥臂换相调用后的辅助计时器读数。 */
  comp_change_time = UTILITY_TIMER->cval;
/****************************************************/
    /* 按当前换相步骤选择浮空相，并根据相电压预期变化方向设置比较器中断边沿。 */
	changeCompInput();
  /* 按条件 average_interval > 2000 && (stall_protection || RC_CAR_REVERSE) 选择当前处理路径。 */
  if(average_interval > 2000 && (stall_protection || RC_CAR_REVERSE)){
    /* 将过零检测路径选择设为 1。 */
	old_routine = 1;
}
    /* 将轮询中满足预期电平的读数累计次数清零。 */
	bemfcounter = 0;
    /* 将轮询路径本次过零已处理的标志清零。 */
	zcfound = 0;
      /* 结合解锁、运行和输入状态判断是否允许执行驱动处理。 */
	  if(use_speed_control_loop && running){
      /* 累计增加转速 PID 生成的油门替代值。 */
	  input_override += doPidCalculations(&speedPid, e_com_time, target_e_com_time)/10000;
      /* 当转速 PID 生成的油门替代值 大于 2000 时进入此分支。 */
	  if(input_override > 2000){
          /* 更新转速 PID 生成的油门替代值。 */
		  input_override = 2000;
	  }
      /* 当转速 PID 生成的油门替代值 小于 0 时进入此分支。 */
	  if(input_override < 0){
          /* 将转速 PID 生成的油门替代值清零。 */
		  input_override = 0;
	  }
      /* 当过零/换相累计计数 小于 100 时进入此分支。 */
	  if(zero_crosses < 100){
          /* 将PID 积分累计量清零。 */
		  speedPid.integral = 0;
	  }
}		
}

/**
 * @brief 换相延时到期回调：关闭本次定时中断，执行换相，并按模式重新允许过零中断。
 */
void PeriodElapsedCallback(){
        /* 按掩码更新COM_TIMER 的中断/DMA 请求使能寄存器。 */
	    COM_TIMER->iden &= ~TMR_OVF_INT; // disable interrupt         

            /* 推进或回退六步相序，更新电周期估计和过零方向，执行桥臂换相并切换比较器输入。 */
			commutate();

            /* 当过零检测路径选择为零时进入此分支。 */
			if(!old_routine){
            /* 允许过零 EXINT 中断；本函数只置位中断使能，不清除之前的挂起标志。 */
			enableCompInterrupts();     // enable comp interrupt
			}
/* 仅在定义 POWER_ON_TEST_MODE 时编译以下代码。 */
#ifdef POWER_ON_TEST_MODE
            /* 将进入捕获速度范围后的连续超时次数清零。 */
			power_on_test_acquire_timeouts = 0;
/* 结束当前条件编译分支。 */
#endif
            /* 当过零/换相累计计数 小于 10000 时进入此分支。 */
			if(zero_crosses<10000){
            /* 递增过零/换相计数；具体是否代表真实过零由本调用路径决定。 */
			zero_crosses++;
			
			}

}
/* 条件编译：defined(USE_DEBUG)；条件满足时采用以下实现。 */
#if defined(USE_DEBUG)
/* 保存调试用比较器错误读数峰值。 */
int max_bad_count=0;
/* 保存调试用过零软件处理耗时。 */
int debug_passed_time=0;
/* 保存调试用最短换相间隔记录。 */
int min_commutation_interval=999999;
/* 保存调试用相邻周期变化比例峰值。 */
int debug_desync_max_diff_ratio=0;
/* 结束当前条件编译分支。 */
#endif

/**
 * @brief 处理过零中断：拒绝过早事件并过滤错误电平，计算提前角和延时后安排下一次换相。
 *
 * 本函数运行于比较器中断上下文；接受一次过零后屏蔽后续事件，等待 COM_TIMER 到期。
 * filter_level 表示反复读取寄存器的次数，不是固定的微秒滤波时长。
 * 当前实现有软件时间门限，但没有在此建立与 PWM 开关沿同步的硬件消隐窗口。
 */
void interruptRoutine(){
    /* 当由电周期折算的平均扇区计数 大于 125 时进入此分支。 */
	if (average_interval > 125){
        /* 检查当前间隔计数是否落在本路径允许的过零或换相时间范围。 */
		if ((INTERVAL_TIMER->cval < 125) && (duty_cycle < 600) && (zero_crosses < 500)){    //should be impossible, desync?exit anyway
            /* 结束当前函数，返回调用方。 */
			return;
		}
        /* 当单扇区的估计时间计数 大于 1000 时进入此分支。 */
		if(commutation_interval>1000){
            /* 检查当前间隔计数是否落在本路径允许的过零或换相时间范围。 */
			if ((INTERVAL_TIMER->cval < (commutation_interval / 2 ))){
                /* 结束当前函数，返回调用方。 */
				return;
			}
        /* 当前条件不满足时进入备选处理。 */
		}else{
            /* 检查当前间隔计数是否落在本路径允许的过零或换相时间范围。 */
			if ((INTERVAL_TIMER->cval < (commutation_interval / 4))){
                /* 结束当前函数，返回调用方。 */
				return;
			}
		}
        /* 递增主循环两次运行之间进入过零中断的次数。 */
		stuckcounter++;             // stuck at 100 interrupts before the main loop happens again.
        /* 当主循环两次运行之间进入过零中断的次数 大于 100 时进入此分支。 */
		if (stuckcounter > 100){
            /* 屏蔽过零 EXINT 中断并清除挂起标志；比较器本身继续工作，仍可轮询输出。 */
			maskPhaseInterrupts();
            /* 将过零/换相累计计数清零。 */
			zero_crosses = 0;
            /* 结束当前函数，返回调用方。 */
			return;
		}
	}
    /* 拒绝距离上次计时起点不足 10 个计数的事件，F421 约为 5 微秒。 */
	if(INTERVAL_TIMER->cval < 10){//about 285710 rpm for 14-poles motor
        /* 结束当前函数，返回调用方。 */
		return;
	}

		/*int prevtime=INTERVAL_TIMER->cval;
		for(int i=0;i<2;++i){
			while(prevtime==INTERVAL_TIMER->cval){
				if((rising && CMP_VALUE) || (!rising && !CMP_VALUE)){
					return;
				}
			}
			prevtime=INTERVAL_TIMER->cval;
		}*/
/* 条件编译：defined(FAST_INTERRUPT)；条件满足时采用以下实现。 */
#if defined(FAST_INTERRUPT)
        /* 更新本次过零或超时发生时的间隔计数。 */
		thiszctime = INTERVAL_TIMER->cval;
        /* 当前扇区等待浮空相电压上升过零。 */
		if(rising){
            /* 连续读取比较器多次，用读数一致性过滤短暂抖动。 */
			for(int i = 0; i < filter_level; i++){
                    /* 当当前 MCU 比较器输出状态位非零时进入此分支。 */
					if(CMP_VALUE){
                        /* 结束当前函数，返回调用方。 */
						return;
					}
			}
        /* 当前条件不满足时进入备选处理。 */
		}else{
            /* 连续读取比较器多次，用读数一致性过滤短暂抖动。 */
			for(int i = 0; i < filter_level; i++){
                    /* 当当前 MCU 比较器输出状态位为零时进入此分支。 */
					if(!CMP_VALUE){
                        /* 结束当前函数，返回调用方。 */
						return;
					}
			}
		}
/* 采用上一编译条件不成立时的备选实现。 */
#else
        /* 按循环下标遍历本组数据或重复执行指定次数的处理。 */
		for(int n=0;n<2;++n){
            /* 更新本次过零或超时发生时的间隔计数。 */
			thiszctime = INTERVAL_TIMER->cval;
            /* 保存本轮中断过滤中不满足预期方向的读数数量。 */
			int badcount=0;
            /* 当前扇区等待浮空相电压上升过零。 */
			if(rising){
                /* 连续读取比较器多次，用读数一致性过滤短暂抖动。 */
				for(int i = 0; i < filter_level*4; i++){
                        /* 当当前 MCU 比较器输出状态位非零时进入此分支。 */
						if(CMP_VALUE){
                            /* 递增本轮中断过滤中不满足预期方向的读数数量。 */
							++badcount;
						}
				}
            /* 当前条件不满足时进入备选处理。 */
			}else{
                /* 连续读取比较器多次，用读数一致性过滤短暂抖动。 */
				for(int i = 0; i < filter_level*4; i++){
                        /* 当当前 MCU 比较器输出状态位为零时进入此分支。 */
						if(!CMP_VALUE){
                            /* 递增本轮中断过滤中不满足预期方向的读数数量。 */
							++badcount;
						}
				}
			}
/* 条件编译：defined(USE_DEBUG)；条件满足时采用以下实现。 */
#if defined(USE_DEBUG)
            /* 当本轮中断过滤中不满足预期方向的读数数量 大于 调试用比较器错误读数峰值 时进入此分支。 */
			if(badcount>max_bad_count){
                /* 更新调试用比较器错误读数峰值。 */
				max_bad_count=badcount;
			}
/* 结束当前条件编译分支。 */
#endif

            /* 当本轮中断过滤中不满足预期方向的读数数量 大于 比较器中断软件过滤的读数次数参数 时进入此分支。 */
			if(badcount>filter_level){//if badcount > 1/4*total test
                /* 当n 等于 1 时进入此分支。 */
				if(n==1){
                    /* 结束当前函数，返回调用方。 */
					return;
				}
            /* 当前条件不满足时进入备选处理。 */
			}else{
                /* 结束当前分支或循环，避免继续处理后续候选项。 */
				break;
			}
		}
/* 结束当前条件编译分支。 */
#endif
		
            /* 保存记录过零时间后已经消耗的处理时间。 */
			int passed_time=INTERVAL_TIMER->cval - thiszctime;
            /* 更新INTERVAL_TIMER 的当前计数值。 */
			INTERVAL_TIMER->cval = passed_time;
/* 条件编译：defined(USE_DEBUG)；条件满足时采用以下实现。 */
#if defined(USE_DEBUG)
            /* 更新调试用过零软件处理耗时。 */
			debug_passed_time=passed_time;
/* 结束当前条件编译分支。 */
#endif

          /* 更新单扇区的估计时间计数。 */
		  commutation_interval = thiszctime;
		  //commutation_interval = (( 3*commutation_interval) + thiszctime)>>2;
		
            /* 保存本次实测换相间隔的快照。 */
			int c1=commutation_interval;

			
			//commutation_interval = (commutation_interval+thiszctime)>>1;
			//commutation_interval = (( 3*commutation_interval) + thiszctime)>>2;			
            /* 保存用于预测下一次换相时刻的间隔。 */
			int next_commutation_interval=commutation_interval;
		
/* 条件编译：defined(USE_DEBUG)；条件满足时采用以下实现。 */
#if defined(USE_DEBUG)
            /* 当调试用最短换相间隔记录 大于 单扇区的估计时间计数 时进入此分支。 */
			if(min_commutation_interval>commutation_interval){
                /* 更新调试用最短换相间隔记录。 */
				min_commutation_interval=commutation_interval;
			}
/* 结束当前条件编译分支。 */
#endif
			
            /* 当快速加速时修正换相等待时间的标志非零时进入此分支。 */
			if(fast_accel){
                    /* 保存快速加速路径采用的预测间隔。 */
					int c2;
                    /* 当由电周期折算的平均扇区计数 大于 800 时进入此分支。 */
					if(average_interval>800){
                        /* 更新快速加速路径采用的预测间隔。 */
						c2=c1;
                    /* 当由电周期折算的平均扇区计数 大于 300 时进入此分支。 */
					}else if(average_interval>300){
                        /* 更新快速加速路径采用的预测间隔。 */
						c2=c1/2;
                    /* 当前条件不满足时进入备选处理。 */
					}else{
                        /* 更新快速加速路径采用的预测间隔。 */
						c2=c1/3;
					}
                    /* 更新用于预测下一次换相时刻的间隔。 */
					next_commutation_interval=c2;
			}
            /* 按预测扇区间隔的 1/8 计算每档提前量，即每档约 7.5 电角度。 */
			advance = (next_commutation_interval>>3) * advance_level;   // 60 divde 8 7.5 degree increments
            /* 更新过零到下一次换相的等待计数。 */
			waitTime = (next_commutation_interval >>1)  - advance;
			
            /* 更新换相附近的 PWM 调整阶段。 */
			inner_step=-1;
            /* 屏蔽过零 EXINT 中断并清除挂起标志；比较器本身继续工作，仍可轮询输出。 */
			maskPhaseInterrupts();						
			//waitTime = waitTime >> fast_accel;
            /* 把中断处理已消耗的时间计入换相定时器，避免再次完整等待造成额外滞后。 */
			COM_TIMER->cval = INTERVAL_TIMER->cval;//already passed INTERVAL_TIMER->cval
            /* 换相截止时间至少比当前处理进度晚两个计数，避免设置已过期的周期。 */
			if(waitTime<INTERVAL_TIMER->cval+2){
                /* 更新过零到下一次换相的等待计数。 */
				waitTime=INTERVAL_TIMER->cval+2;
			}
            /* 更新COM_TIMER 的周期/自动重装载寄存器。 */
			COM_TIMER->pr = waitTime;
            /* 清除已处理的定时器事件标志；具体清除位由当前写入值决定。 */
			COM_TIMER->ists = 0x00;
            /* 置位COM_TIMER 的中断/DMA 请求使能寄存器。 */
			COM_TIMER->iden |= TMR_OVF_INT;
}

/**
 * @brief 初始化电机启动状态；试转模式下先对齐转子，再通过轮询和有限次数强制换相捕获 BEMF。
 */
void startMotor() {
    /* 当电机控制流程正在运行的标志 等于 0 时进入此分支。 */
	if (running == 0){
/* 仅在定义 POWER_ON_TEST_MODE 时编译以下代码。 */
#ifdef POWER_ON_TEST_MODE
    /* 试转故障已经锁存，进入停机或禁止重启路径。 */
	if (power_on_test_fault){
        /* 结束当前函数，返回调用方。 */
		return;
	}
    /* 将试转过程中强制换相累计次数清零。 */
	power_on_test_forced_steps = 0;
    /* 将试转期间有效 BEMF 读数计数的峰值清零。 */
	power_on_test_bemf_peak = 0;
    /* 将试转期间过零计数的峰值清零。 */
	power_on_test_zero_cross_peak = 0;
    /* 将捕获阶段连续有效过零次数清零。 */
	power_on_test_valid_zero_crosses = 0;
    /* 将进入捕获速度范围后的连续超时次数清零。 */
	power_on_test_acquire_timeouts = 0;
    /* 将试转已达到有效过零锁定条件的标志清零。 */
	power_on_test_closed_loop = 0;
    /* 更新允许等待过零的最长间隔。 */
	comm_timeout = POWER_ON_TEST_COMM_TIMEOUT_START;
	// startMotor() runs before the normal duty calculation in the main loop.
	// Preload the test duty here so the alignment hold produces real torque.
    /* 更新以基准 PWM 周期表示的目标比较值。 */
	duty_cycle = startup_max_duty_cycle;
    /* 更新按当前 PWM 周期缩放后的实际比较值。 */
	adjusted_duty_cycle = startup_max_duty_cycle;
    /* 更新最终交给 PWM 寄存器的比较值缓存。 */
	last_adjusted_duty_cycle = startup_max_duty_cycle;
    /* 按过零超时情况限制占空比，再把调整后的值写入三个 PWM 通道；可选换相瞬间降占空比。 */
	setPwmRatio();
/* 结束当前条件编译分支。 */
#endif
    /* 推进或回退六步相序，更新电周期估计和过零方向，执行桥臂换相并切换比较器输入。 */
	commutate();
/* 仅在定义 POWER_ON_TEST_MODE 时编译以下代码。 */
#ifdef POWER_ON_TEST_MODE
	// Hold one electrical sector so the rotor starts the acceleration ramp
	// from a known position. The interval ISR ignores overflow while running=0.
    /* 重载看门狗，表明控制流程仍在正常执行。 */
	WDT->cmd = WDT_CMD_RELOAD;
    /* 临时修改 UTILITY_TIMER 分频进行毫秒级忙等，等待期间喂狗，结束后恢复代码指定的分频。 */
	delayMillis(POWER_ON_TEST_ALIGNMENT_MS);
    /* 重载看门狗，表明控制流程仍在正常执行。 */
	WDT->cmd = WDT_CMD_RELOAD;
/* 结束当前条件编译分支。 */
#endif
    /* 更新单扇区的估计时间计数。 */
	commutation_interval = 10000;
    /* 更新INTERVAL_TIMER 的当前计数值。 */
	INTERVAL_TIMER->cval = 5000;
    /* 将电机控制流程正在运行的标志设为 1。 */
	running = 1;
	}
    /* 仅在定义 POWER_ON_TEST_MODE 时编译以下代码。 */
	#ifdef POWER_ON_TEST_MODE
    /* 将过零检测路径选择设为 1。 */
	old_routine = 1;
    /* 屏蔽过零 EXINT 中断并清除挂起标志；比较器本身继续工作，仍可轮询输出。 */
	maskPhaseInterrupts();
    /* 采用上一编译条件不成立时的备选实现。 */
	#else
    /* 允许过零 EXINT 中断；本函数只置位中断使能，不清除之前的挂起标志。 */
	enableCompInterrupts();
    /* 结束当前条件编译分支。 */
	#endif
}

/**
 * @brief 处理堵转停机：关闭桥臂、屏蔽过零并清零油门；试转模式同时取消待执行换相和 PWM。
 */
void stopStuckMotor(){
    /* 仅在定义 POWER_ON_TEST_MODE 时编译以下代码。 */
	#ifdef POWER_ON_TEST_MODE
        /* 将电机控制流程正在运行的标志清零。 */
		running = 0;
        /* 按掩码更新COM_TIMER 的中断/DMA 请求使能寄存器。 */
		COM_TIMER->iden &= ~TMR_OVF_INT;
        /* 将按当前 PWM 周期缩放后的实际比较值清零。 */
		adjusted_duty_cycle = 0;
        /* 将最终交给 PWM 寄存器的比较值缓存清零。 */
		last_adjusted_duty_cycle = 0;
        /* 按过零超时情况限制占空比，再把调整后的值写入三个 PWM 通道；可选换相瞬间降占空比。 */
		setPwmRatio();
    /* 结束当前条件编译分支。 */
	#endif
        /* 将 A、B、C 三相全部置为浮空，关闭各相高低侧驱动。 */
		allOff();
        /* 屏蔽过零 EXINT 中断并清除挂起标志；比较器本身继续工作，仍可轮询输出。 */
		maskPhaseInterrupts();
        /* 将交给电机控制状态机的最终油门值清零。 */
		input = 0;
        /* 更新累计过零超时次数。 */
		bemf_timeout_happened = 102;
/* 仅在定义 USE_RGB_LED 时编译以下代码。 */
#ifdef USE_RGB_LED
            /* 更新GPIOB 的兼容 GPIO 引脚清零寄存器。 */
			GPIOB->BRR = LL_GPIO_PIN_8; // on red
            /* 更新GPIOB 的兼容 GPIO 引脚置位/复位寄存器。 */
			GPIOB->BSRR = LL_GPIO_PIN_5;  //
            /* 更新GPIOB 的兼容 GPIO 引脚置位/复位寄存器。 */
			GPIOB->BSRR = LL_GPIO_PIN_3;
/* 结束当前条件编译分支。 */
#endif
}

/* 轮询路径的过零处理；区分真实过零和强制换相，等待半扇区减提前量后执行下一步。 */
void zcfoundroutine(uint8_t forced_commutation);
/* 轮询路径的换相收尾：执行换相并复位计数，满足已捕获过零等条件后切到中断检测。 */
void commutateProcess(uint8_t real_zero_cross);
/**
 * @brief 处理过零等待超时；试转模式限制强制换相次数并锁存故障，正常回退路径转入轮询。
 * @param commutateRightNow 非零立即执行回退换相；为零则走带等待的轮询换相流程。
 */
void IntervalTimerOverflowProcess(int commutateRightNow){
    /* 递增累计过零超时次数。 */
	bemf_timeout_happened++;
/* 仅在定义 POWER_ON_TEST_MODE 时编译以下代码。 */
#ifdef POWER_ON_TEST_MODE
    /* 当commutateRightNow为零时进入此分支。 */
	if(!commutateRightNow){
        /* 当试转过程中强制换相累计次数 小于 255 时进入此分支。 */
		if(power_on_test_forced_steps < 255){
            /* 递增试转过程中强制换相累计次数。 */
			power_on_test_forced_steps++;
		}
        /* 将捕获阶段连续有效过零次数清零。 */
		power_on_test_valid_zero_crosses = 0;
        /* 将试转已达到有效过零锁定条件的标志清零。 */
		power_on_test_closed_loop = 0;

        /* 当允许等待过零的最长间隔 不大于 开始接受 BEMF 捕获的间隔门限 时进入此分支。 */
		if(comm_timeout <= POWER_ON_TEST_BEMF_ACQUIRE_INTERVAL){
            /* 当进入捕获速度范围后的连续超时次数 小于 255 时进入此分支。 */
			if(power_on_test_acquire_timeouts < 255){
                /* 递增进入捕获速度范围后的连续超时次数。 */
				power_on_test_acquire_timeouts++;
			}
        /* 当前条件不满足时进入备选处理。 */
		}else{
            /* 将进入捕获速度范围后的连续超时次数清零。 */
			power_on_test_acquire_timeouts = 0;
		}
        /* 当试转过程中强制换相累计次数 不小于 试转最多允许的强制换相次数 时进入此分支。 */
		if(power_on_test_forced_steps >= POWER_ON_TEST_MAX_FORCED_STEPS){
            /* 锁存试转故障；后续固定油门路径和 startMotor 均检查该标志。 */
			power_on_test_fault = 1;
            /* 处理堵转停机：关闭桥臂、屏蔽过零并清零油门；试转模式同时取消待执行换相和 PWM。 */
			stopStuckMotor();
            /* 结束当前函数，返回调用方。 */
			return;
		}
        /* 当进入捕获速度范围后的连续超时次数 不小于 进入捕获速度范围后允许的连续超时次数 时进入此分支。 */
		if(power_on_test_acquire_timeouts >= POWER_ON_TEST_MAX_ACQUIRE_TIMEOUTS){
            /* 锁存试转故障；后续固定油门路径和 startMotor 均检查该标志。 */
			power_on_test_fault = 1;
            /* 处理堵转停机：关闭桥臂、屏蔽过零并清零油门；试转模式同时取消待执行换相和 PWM。 */
			stopStuckMotor();
            /* 结束当前函数，返回调用方。 */
			return;
		}

		// Accelerate the open-loop fallback so the motor can generate enough
		// BEMF for the polling detector to acquire a real zero crossing.
        /* 当允许等待过零的最长间隔 大于 强制换相允许的最短间隔 时进入此分支。 */
		if(comm_timeout > POWER_ON_TEST_COMM_TIMEOUT_MIN){
            /* 按固定比例缩短强制换相间隔，使转子加速到可以检测 BEMF 的范围。 */
			comm_timeout = comm_timeout * POWER_ON_TEST_COMM_TIMEOUT_RAMP_PERCENT / 100;
            /* 当允许等待过零的最长间隔 小于 强制换相允许的最短间隔 时进入此分支。 */
			if(comm_timeout < POWER_ON_TEST_COMM_TIMEOUT_MIN){
                /* 更新允许等待过零的最长间隔。 */
				comm_timeout = POWER_ON_TEST_COMM_TIMEOUT_MIN;
			}
		}
	}
/* 结束当前条件编译分支。 */
#endif
	//set pwm as bemf_timeout_happened changed
    /* 按过零超时情况限制占空比，再把调整后的值写入三个 PWM 通道；可选换相瞬间降占空比。 */
	setPwmRatio();
    /* 保存允许过零超时的次数门限。 */
	const int bemf_timeout=100;
    /* 按条件 bemf_timeout_happened > bemf_timeout * ( 1 + (crawler_mode*100)) && stuck_rotor_protection 选择当前处理路径。 */
	if(bemf_timeout_happened > bemf_timeout * ( 1 + (crawler_mode*100)) && stuck_rotor_protection){
        /* 处理堵转停机：关闭桥臂、屏蔽过零并清零油门；试转模式同时取消待执行换相和 PWM。 */
		stopStuckMotor();
        /* 结束当前函数，返回调用方。 */
		return;
	}

    /* 屏蔽过零 EXINT 中断并清除挂起标志；比较器本身继续工作，仍可轮询输出。 */
	maskPhaseInterrupts();
    /* 将过零检测路径选择设为 1。 */
	old_routine = 1;
    /* 当交给电机控制状态机的最终油门值 小于 48 时进入此分支。 */
	if(input < 48){
     /* 将电机控制流程正在运行的标志清零。 */
	 running = 0;
	}
    /* 将过零/换相累计计数清零。 */
	zero_crosses = 0;
    /* 当commutateRightNow非零时进入此分支。 */
	if(commutateRightNow){
        /* 轮询路径的换相收尾：执行换相并复位计数，满足已捕获过零等条件后切到中断检测。 */
		commutateProcess(0);
    /* 当前条件不满足时进入备选处理。 */
	}else{
        /* 轮询路径的过零处理；区分真实过零和强制换相，等待半扇区减提前量后执行下一步。 */
		zcfoundroutine(1);
	}
	
 // if(stall_protection){
	 // min_startup_duty = 130;
	 // minimum_duty_cycle = minimum_duty_cycle + 10;
	 // if(minimum_duty_cycle > 80){
		 // minimum_duty_cycle = 80;
	 // }
 // }

}

//interval timer overflow process, in case main loop is stucked and the zc timeout escaped
/* 保存调试用间隔定时器溢出次数。 */
int intervaloverflowcount=0;
/**
 * @brief 间隔定时器溢出的兜底处理，避免主循环阻塞后遗漏过零超时；仅在 running 时执行。
 */
void IntervalTimerOverflow(){
    /* 当电机控制流程正在运行的标志非零时进入此分支。 */
	if(running){
        /* 处理过零等待超时；试转模式限制强制换相次数并锁存故障，正常回退路径转入轮询。 */
		IntervalTimerOverflowProcess(1);
        /* 条件编译：defined(USE_DEBUG)；条件满足时采用以下实现。 */
		#if defined(USE_DEBUG)
            /* 递增调试用间隔定时器溢出次数。 */
			++intervaloverflowcount;
        /* 结束当前条件编译分支。 */
		#endif
	}
}


/**
 * @brief 周期性控制任务：解锁、油门到占空比映射、电流/低速控制、斜率限制、遥测及失联处理。
 *
 * 由外设定时中断调用，名称沿用约 10 kHz 设计；精确周期取决于定时器 PR、DIV 和时钟。
 * 每十次回调运行电流/低速控制计算；PWM 最终还要按当前 tim1_arr 重缩放。
 */
void tenKhzRoutine(){
    /* 递增周期控制回调累计计数。 */
	tenkhzcounter++;
	
          /* 当启动音已播放标志 等于 0 时进入此分支。 */
		  if(boot_up_tune_played == 0){
            /* 当周期控制回调累计计数 大于 1000 时进入此分支。 */
			if(tenkhzcounter > 1000){ 
            /* 播放启动提示音；有自定义旋律时读取配置缓冲区，否则播放默认三音提示。 */
			playStartupTune();
            /* 将启动音已播放标志设为 1。 */
			boot_up_tune_played = 1;
			}
	  }
	
    /* 当周期控制回调累计计数 大于 10000 时进入此分支。 */
	if(tenkhzcounter > 10000){      // 1s sample interval
        /* 按约一秒周期积分电流；actual_current 以 0.01 A 表示，累计结果为 mAh。 */
		consumed_current = (float)actual_current/360 + consumed_current;
                            /* 按命令编号、遥测状态或换相步骤分派到对应处理分支。 */
							switch (dshot_extended_telemetry){

                    /* 扩展遥测轮换：准备温度数据。 */
					case 1:
                        /* 更新待发送的扩展遥测编码。 */
					    send_extended_dshot = 0x0200 | degrees_celsius;
                        /* 更新扩展遥测使能及温度/电流/电压轮换状态。 */
					    dshot_extended_telemetry = 2;
                    /* 结束当前分支或循环，避免继续处理后续候选项。 */
					break;
                    /* 扩展遥测轮换：准备电流数据。 */
					case 2:
                        /* 更新待发送的扩展遥测编码。 */
					    send_extended_dshot = 0x0600 | ((uint8_t)actual_current / 50);
                        /* 更新扩展遥测使能及温度/电流/电压轮换状态。 */
					    dshot_extended_telemetry = 3;
                    /* 结束当前分支或循环，避免继续处理后续候选项。 */
					break;
                    /* 扩展遥测轮换：准备电压数据。 */
					case 3:
                        /* 更新待发送的扩展遥测编码。 */
					    send_extended_dshot = 0x0400 | (uint8_t)(battery_voltage / 25);
                        /* 将扩展遥测使能及温度/电流/电压轮换状态设为 1。 */
					    dshot_extended_telemetry = 1;
                    /* 结束当前分支或循环，避免继续处理后续候选项。 */
					break;

					}
        /* 将周期控制回调累计计数清零。 */
		tenkhzcounter = 0;
	}
/* 当电调已解锁并允许驱动的标志为零时进入此分支。 */
if(!armed){
    /* 当已识别输入信号类型的标志非零时进入此分支。 */
	if(inputSet){
        /* 当经过方向和死区处理的油门值 等于 0 时进入此分支。 */
		if(adjusted_input == 0){
            /* 递增满足零油门解锁条件的时间计数。 */
			armed_timeout_count++;
            /* 当满足零油门解锁条件的时间计数 大于 10000 时进入此分支。 */
			if(armed_timeout_count > 10000){    // one second
                /* 当连续检测到零油门的次数 大于 30 时进入此分支。 */
				if(zero_input_count > 30){
                    /* 将电调已解锁并允许驱动的标志设为 1。 */
					armed = 1;
		//			receiveDshotDma();
                    /* 仅在定义 USE_RGB_LED 时编译以下代码。 */
					#ifdef USE_RGB_LED
                                    /* 更新GPIOB 的兼容 GPIO 引脚清零寄存器。 */
									GPIOB->BRR = LL_GPIO_PIN_3;    // turn on green
                                    /* 更新GPIOB 的兼容 GPIO 引脚置位/复位寄存器。 */
									GPIOB->BSRR = LL_GPIO_PIN_8;   // turn on green
                                    /* 更新GPIOB 的兼容 GPIO 引脚置位/复位寄存器。 */
									GPIOB->BSRR = LL_GPIO_PIN_5;
                    /* 结束当前条件编译分支。 */
					#endif
                                    /* 按条件 cell_count == 0 && LOW_VOLTAGE_CUTOFF 选择当前处理路径。 */
									if(cell_count == 0 && LOW_VOLTAGE_CUTOFF){
                                        /* 更新估计的串联电芯数量。 */
										cell_count = battery_voltage / 370;
                                        /* 按循环下标遍历本组数据或重复执行指定次数的处理。 */
										for (int i = 0 ; i < cell_count; i++){
                                        /* 播放输入识别提示旋律；临时关闭中断以保持鸣音时序。 */
										playInputTune();
                                        /* 临时修改 UTILITY_TIMER 分频进行毫秒级忙等，等待期间喂狗，结束后恢复代码指定的分频。 */
										delayMillis(100);
		//			  			 IWDG_ReloadCounter(IWDG);
										}
                                        /* 当前条件不满足时进入备选处理。 */
										}else{
                                        /* 播放输入识别提示旋律；临时关闭中断以保持鸣音时序。 */
										playInputTune();
										}
                                    /* 当当前输入识别为舵机 PWM 的标志为零时进入此分支。 */
									if(!servoPwm){
                                        /* 将车模先制动再反转模式标志清零。 */
										RC_CAR_REVERSE = 0;
									}
                        /* 保存根据电压估计的串联电芯数。 */
						int batcellcount=battery_voltage / 370;
                        /* 当根据电压估计的串联电芯数 不大于 0 时进入此分支。 */
						if(batcellcount<=0){
                            /* 更新根据电压估计的串联电芯数。 */
							batcellcount=4;
						}
                        /* 更新按 KV 和电池电压估计的机械转速上限。 */
						targetmax_rpm=(int)motor_kv*batcellcount*42/10;
                        /* 保存加速斜率映射的低转速参考值。 */
						int targetrpmlow=2000*6*42/10;
                        /* 保存加速斜率映射的高转速参考值。 */
						int targetrpmhigh=3500*6*42/10;
                        /* 按当前输入区间限幅并线性映射，得到按电机和电池估算的斜率系数。 */
						target_rampratio_mul10=map(targetmax_rpm,targetrpmlow,targetrpmhigh,10,6);
                /* 当前条件不满足时进入备选处理。 */
				}else{
                    /* 将已识别输入信号类型的标志清零。 */
					inputSet = 0;
                    /* 将满足零油门解锁条件的时间计数清零。 */
					armed_timeout_count =0;
				}
			}
        /* 当前条件不满足时进入备选处理。 */
		}else{
            /* 将满足零油门解锁条件的时间计数清零。 */
			armed_timeout_count =0;
		}
	}
}

    /* 当固定间隔串口遥测使能非零时进入此分支。 */
	if(TLM_ON_INTERVAL){
        /* 递增周期遥测调度计数。 */
		telem_ms_count++;
        /* 当周期遥测调度计数 大于 telemetry_interval_ms*10 时进入此分支。 */
		if(telem_ms_count>telemetry_interval_ms*10){
            /* 将待发送串口遥测标志设为 1。 */
			send_telemetry = 1;
            /* 将周期遥测调度计数清零。 */
			telem_ms_count = 0;
		}
	}
/* 仅在未定义 BRUSHED_MODE 时编译以下代码。 */
#ifndef BRUSHED_MODE
    /* 当当前处于正弦步进模式的标志为零时进入此分支。 */
	if(!stepper_sine){
      /* 结合解锁、运行和输入状态判断是否允许执行驱动处理。 */
	  if (input >= 47 +(80*use_sin_start) && armed){
            /* 将停止后延迟关闭桥臂的周期计数清零。 */
			stop_counter_for_alloff=0;
          /* 当电机控制流程正在运行的标志 等于 0 时进入此分支。 */
		  if (running == 0){
              /* 将 A、B、C 三相全部置为浮空，关闭各相高低侧驱动。 */
			  allOff();
              /* 当过零检测路径选择为零时进入此分支。 */
			  if(!old_routine){
             /* 初始化电机启动状态；试转模式下先对齐转子，再通过轮询和有限次数强制换相捕获 BEMF。 */
			 startMotor();
			  }
              /* 将电机控制流程正在运行的标志设为 1。 */
			  running = 1;
              /* 更新上一次输出的基准占空比。 */
			  last_duty_cycle = min_startup_duty;

		  }
      /* 当正弦启动使能非零时进入此分支。 */
	  if(use_sin_start){
        /* 按当前输入区间限幅并线性映射，得到以基准 PWM 周期表示的目标比较值。 */
		duty_cycle = map(input, 137, 2047, minimum_duty_cycle, TIMER1_MAX_ARR);
      /* 当前条件不满足时进入备选处理。 */
  	  }else{
         /* 按当前输入区间限幅并线性映射，得到以基准 PWM 周期表示的目标比较值。 */
	 	 duty_cycle = map(input, 47, 2047, minimum_duty_cycle, TIMER1_MAX_ARR);
	  }
      /* 当tenkhzcounter%10 等于 0 时进入此分支。 */
	  if(tenkhzcounter%10 == 0){     // 1khz PID loop
          /* 结合解锁、运行和输入状态判断是否允许执行驱动处理。 */
		  if(use_current_limit && running){
            /* 减去本次修正并更新电流环给出的占空比上限。 */
			use_current_limit_adjust -= (int16_t)(doPidCalculations(&currentPid, actual_current, CURRENT_LIMIT*100)/10000);
            /* 当电流环给出的占空比上限 小于 正常驱动最小 PWM 比较值 时进入此分支。 */
			if(use_current_limit_adjust < minimum_duty_cycle){
                /* 更新电流环给出的占空比上限。 */
				use_current_limit_adjust = minimum_duty_cycle;
			}
            /* 当电流环给出的占空比上限 大于 以基准 PWM 周期表示的目标比较值 时进入此分支。 */
			if(use_current_limit_adjust > duty_cycle){
                /* 更新电流环给出的占空比上限。 */
				use_current_limit_adjust = duty_cycle;
			}

	  }

             /* 结合解锁、运行和输入状态判断是否允许执行驱动处理。 */
		  	 if(stall_protection && running ){  // this boosts throttle as the rpm gets lower, for crawlers and rc cars only, do not use for multirotors.
                 /* 累计增加低速补偿追加的占空比。 */
		  		 stall_protection_adjust += (doPidCalculations(&stallPid, commutation_interval, stall_protect_target_interval))/10000;
                             /* 当低速补偿追加的占空比 大于 150 时进入此分支。 */
		  					 if(stall_protection_adjust > 150){
                                /* 更新低速补偿追加的占空比。 */
		  						stall_protection_adjust = 150;
		  					 }
                             /* 当低速补偿追加的占空比 不大于 0 时进入此分支。 */
		  					 if(stall_protection_adjust <= 0){
                                /* 将低速补偿追加的占空比清零。 */
		  						stall_protection_adjust = 0;
		  					 }
		  	 }
	  }
      /* 当车模先制动再反转模式标志为零时进入此分支。 */
	  if(!RC_CAR_REVERSE){
          /* 将比例制动当前生效标志清零。 */
		  prop_brake_active = 0;
	  }
	  }
      /* 按条件 input < 47 + (80*use_sin_start) 选择当前处理路径。 */
	  if (input < 47 + (80*use_sin_start)){
        /* 当待播放的设置提示音标志 不等于 0 时进入此分支。 */
		if(play_tone_flag != 0){
            /* 当待播放的设置提示音标志 等于 1 时进入此分支。 */
			if(play_tone_flag == 1){
                /* 播放默认设置提示音，随后恢复电机 PWM 周期。 */
				playDefaultTone();

            /* 当待播放的设置提示音标志 等于 2 时进入此分支。 */
			}if(play_tone_flag == 2){
                /* 播放设置变更提示音，随后恢复电机 PWM 周期。 */
				playChangedTone();
			}
            /* 将待播放的设置提示音标志清零。 */
			play_tone_flag = 0;
		}

          /* 当互补 PWM 使能标志为零时进入此分支。 */
		  if(!comp_pwm){
            /* 将以基准 PWM 周期表示的目标比较值清零。 */
			duty_cycle = 0;
            /* 当电机控制流程正在运行的标志为零时进入此分支。 */
			if(!running){
                /* 将过零检测路径选择设为 1。 */
				old_routine = 1;
                /* 将过零/换相累计计数清零。 */
				zero_crosses = 0;
                  /* 当零油门时制动的设置非零时进入此分支。 */
				  if(brake_on_stop){
                      /* 把三相全部切为低侧导通，实现三相短接制动。 */
					  fullBrake();
                  /* 当前条件不满足时进入备选处理。 */
				  }else{
                      /* 当比例制动当前生效标志为零时进入此分支。 */
					  if(!prop_brake_active){
                      /* 将 A、B、C 三相全部置为浮空，关闭各相高低侧驱动。 */
					  allOff();
					  }
				  }
			}
            /* 按条件 RC_CAR_REVERSE && prop_brake_active 选择当前处理路径。 */
			if (RC_CAR_REVERSE && prop_brake_active) {
/* 仅在未定义 PWM_ENABLE_BRIDGE 时编译以下代码。 */
#ifndef PWM_ENABLE_BRIDGE
                    /* 更新以基准 PWM 周期表示的目标比较值。 */
					duty_cycle = getAbsDif(1000, newinput) + 1000;
                    /* 当以基准 PWM 周期表示的目标比较值 等于 2000 时进入此分支。 */
					if(duty_cycle == 2000){
                        /* 把三相全部切为低侧导通，实现三相短接制动。 */
						fullBrake();
                    /* 当前条件不满足时进入备选处理。 */
					}else{
                        /* 关闭高侧驱动并把低侧切到 PWM 复用模式，用占空比控制制动力。 */
						proportionalBrake();
					}
/* 结束当前条件编译分支。 */
#endif
					}
          /* 当前条件不满足时进入备选处理。 */
		  }else{
          /* 当电机控制流程正在运行的标志为零时进入此分支。 */
		  if (!running){
              /* 将以基准 PWM 周期表示的目标比较值清零。 */
			  duty_cycle = 0;
              /* 将过零检测路径选择设为 1。 */
			  old_routine = 1;
              /* 将过零/换相累计计数清零。 */
			  zero_crosses = 0;
              /* 将轮询检测到错误方向电平的累计次数清零。 */
			  bad_count = 0;
                  /* 当零油门时制动的设置非零时进入此分支。 */
			  	  if(brake_on_stop){
                      /* 当正弦启动使能为零时进入此分支。 */
			  		  if(!use_sin_start){
/* 仅在未定义 PWM_ENABLE_BRIDGE 时编译以下代码。 */
#ifndef PWM_ENABLE_BRIDGE				
                          /* 更新以基准 PWM 周期表示的目标比较值。 */
			  			  duty_cycle = (TIMER1_MAX_ARR-19) + drag_brake_strength*2;
                          /* 关闭高侧驱动并把低侧切到 PWM 复用模式，用占空比控制制动力。 */
			  			  proportionalBrake();
                          /* 将比例制动当前生效标志设为 1。 */
			  			  prop_brake_active = 1;
/* 采用上一编译条件不成立时的备选实现。 */
#else
	//todo add proportional braking for pwm/enable style bridge.
/* 结束当前条件编译分支。 */
#endif
			  		  }
                  /* 当前条件不满足时进入备选处理。 */
			  	  }else{							
                            /* 当停止后延迟关闭桥臂的周期计数 大于 1000 时进入此分支。 */
							if(stop_counter_for_alloff>1000){
                                /* 将 A、B、C 三相全部置为浮空，关闭各相高低侧驱动。 */
								allOff();
                            /* 当前条件不满足时进入备选处理。 */
							}else{
                                /* 递增停止后延迟关闭桥臂的周期计数。 */
								++stop_counter_for_alloff;
							}
                      /* 将以基准 PWM 周期表示的目标比较值清零。 */
			  		  duty_cycle = 0;
			  	  }
		  }

              /* 更新A 相正弦查表角度索引。 */
		  	  phase_A_position = ((step-1) * 60) + enter_sine_angle;
              /* 当A 相正弦查表角度索引 大于 359 时进入此分支。 */
		  	  if(phase_A_position > 359){
                  /* 减去本次修正并更新A 相正弦查表角度索引。 */
		  		  phase_A_position -= 360;
		  	  }
              /* 更新B 相正弦查表角度索引。 */
		  	  phase_B_position = phase_A_position +  119;
              /* 当B 相正弦查表角度索引 大于 359 时进入此分支。 */
		  	  if(phase_B_position > 359){
                  /* 减去本次修正并更新B 相正弦查表角度索引。 */
		  		  phase_B_position -= 360;
		  	  }
              /* 更新C 相正弦查表角度索引。 */
		  	  phase_C_position = phase_A_position + 239;
             /* 当C 相正弦查表角度索引 大于 359 时进入此分支。 */
		  	 if(phase_C_position > 359){
             /* 减去本次修正并更新C 相正弦查表角度索引。 */
		  	 phase_C_position -= 360;
		  	 }

              /* 当正弦启动使能 等于 1 时进入此分支。 */
		 	  if(use_sin_start == 1){
                 /* 将当前处于正弦步进模式的标志设为 1。 */
		    	 stepper_sine = 1;
		 	  }
		  }
		  }
/* 当比例制动当前生效标志为零时进入此分支。 */
if(!prop_brake_active){
/* 仅在定义 POWER_ON_TEST_MODE 时编译以下代码。 */
#ifdef POWER_ON_TEST_MODE
	// Hold a short, bounded 24% startup torque until BEMF is genuinely locked.
    /* 按试转捕获状态、连续过零和故障条件决定是否进入此路径。 */
	if(!power_on_test_closed_loop && !power_on_test_fault && running){
        /* 更新以基准 PWM 周期表示的目标比较值。 */
		duty_cycle = startup_max_duty_cycle;
	}
/* 结束当前条件编译分支。 */
#endif
 /* 结合换相速度和过零累计情况选择启动、过滤或正常运行路径。 */
 if (zero_crosses < (20 >> stall_protection)){
       /* 当以基准 PWM 周期表示的目标比较值 小于 启动时最小 PWM 比较值 时进入此分支。 */
	   if (duty_cycle < min_startup_duty){
       /* 更新以基准 PWM 周期表示的目标比较值。 */
	   duty_cycle = min_startup_duty;

	   }
       /* 当以基准 PWM 周期表示的目标比较值 大于 启动阶段允许的最大 PWM 比较值 时进入此分支。 */
	   if (duty_cycle > startup_max_duty_cycle){
           /* 更新以基准 PWM 周期表示的目标比较值。 */
		   duty_cycle = startup_max_duty_cycle;
	   }
 }

     /* 当以基准 PWM 周期表示的目标比较值 大于 温度或低转速保护给出的占空比上限 时进入此分支。 */
	 if (duty_cycle > duty_cycle_maximum){
         /* 更新以基准 PWM 周期表示的目标比较值。 */
		 duty_cycle = duty_cycle_maximum;
	 }
     /* 当电流限制环使能非零时进入此分支。 */
	 if(use_current_limit){
         /* 当以基准 PWM 周期表示的目标比较值 大于 电流环给出的占空比上限 时进入此分支。 */
		 if (duty_cycle > use_current_limit_adjust){
             /* 更新以基准 PWM 周期表示的目标比较值。 */
			 duty_cycle = use_current_limit_adjust;
		 }
	 }

     /* 当低速补偿追加的占空比 大于 0 时进入此分支。 */
	 if(stall_protection_adjust > 0){

         /* 更新以基准 PWM 周期表示的目标比较值。 */
		 duty_cycle = duty_cycle + (uint16_t)stall_protection_adjust;
	 }

     /* 当占空比变化斜率限制使能非零时进入此分支。 */
	 if(maximum_throttle_change_ramp){
            /* 保存当前转速和占空比区间采用的变化倍率。 */
		 	int ramp_ratio;
          /* 当以基准 PWM 周期表示的目标比较值 小于 TIMER1_MAX_ARR/10*1 时进入此分支。 */
		  if(duty_cycle<TIMER1_MAX_ARR/10*1){
                /* 将当前转速和占空比区间采用的变化倍率设为 1。 */
				ramp_ratio=1;
            /* 当前条件不满足时进入备选处理。 */
			}else {
                /* 当由电周期折算的平均扇区计数 大于 500 时进入此分支。 */
				if(average_interval>500){
                    /* 更新当前转速和占空比区间采用的变化倍率。 */
					ramp_ratio=2;
                /* 当前条件不满足时进入备选处理。 */
				}else{
                    /* 更新当前转速和占空比区间采用的变化倍率。 */
					ramp_ratio=4;
				}
			}
				
          /* 保存本次允许的占空比最大增加量。 */
		  int max_duty_cycle_change = 2;
	//	max_duty_cycle_change = map(k_erpm, low_rpm_level, high_rpm_level, 1, 40);
            /* 当由电周期折算的平均扇区计数 大于 500 时进入此分支。 */
			if(average_interval > 500){
                /* 更新本次允许的占空比最大增加量。 */
				max_duty_cycle_change = 15*ramp_ratio;
            /* 当前条件不满足时进入备选处理。 */
			}else{
                /* 更新本次允许的占空比最大增加量。 */
				max_duty_cycle_change = 45*ramp_ratio;
			}
/* 条件编译：defined(USE_KV_VBAT_RAMP)；条件满足时采用以下实现。 */
#if defined(USE_KV_VBAT_RAMP)
            /* 当按电机和电池估算的斜率系数 大于 0 时进入此分支。 */
			if(target_rampratio_mul10>0){
                /* 更新本次允许的占空比最大增加量。 */
				max_duty_cycle_change=max_duty_cycle_change*target_rampratio_mul10/10;
                /* 当本次允许的占空比最大增加量 小于 2 时进入此分支。 */
				if(max_duty_cycle_change<2)max_duty_cycle_change=2;
			}
/* 结束当前条件编译分支。 */
#endif
			
            /* 保存本次允许的占空比最大减少量。 */
			int max_duty_cycle_change_dec;
            /* 当由电周期折算的平均扇区计数 大于 500 时进入此分支。 */
			if(average_interval > 500){
                /* 更新本次允许的占空比最大减少量。 */
				max_duty_cycle_change_dec = 20*ramp_ratio;
            /* 当前条件不满足时进入备选处理。 */
			}else{
                /* 更新本次允许的占空比最大减少量。 */
				max_duty_cycle_change_dec = 60*ramp_ratio;
			}
			
             /* 检查占空比与当前限制值或变化量的关系。 */
			 if ((duty_cycle - last_duty_cycle) > max_duty_cycle_change){
                /* 更新以基准 PWM 周期表示的目标比较值。 */
				duty_cycle = last_duty_cycle + max_duty_cycle_change;
                /* 将快速加速时修正换相等待时间的标志设为 1。 */
				fast_accel = 1;
				/*if(commutation_interval > 500){
					fast_accel = 1;
				}else{
					fast_accel = 0;
				}*/
            /* 检查占空比与当前限制值或变化量的关系。 */
			}else if ((last_duty_cycle - duty_cycle) > max_duty_cycle_change_dec){
                /* 更新以基准 PWM 周期表示的目标比较值。 */
				duty_cycle = last_duty_cycle - max_duty_cycle_change_dec;
                /* 将快速加速时修正换相等待时间的标志清零。 */
				fast_accel = 0;
            /* 当前条件不满足时进入备选处理。 */
			}else{
                /* 将快速加速时修正换相等待时间的标志清零。 */
				fast_accel = 0;
			}
            /* 当由电周期折算的平均扇区计数 大于 1000 时进入此分支。 */
			if(average_interval>1000){
                /* 当以基准 PWM 周期表示的目标比较值 大于 TIMER1_MAX_ARR/2 时进入此分支。 */
				if(duty_cycle>TIMER1_MAX_ARR/2){
                    /* 更新以基准 PWM 周期表示的目标比较值。 */
					duty_cycle=TIMER1_MAX_ARR/2;
				}
			}
			/*if(e_com_time > 10 * 1000000/(targetmax_rpm/60*motor_poles/2)){// 1/10 max speed
				if(duty_cycle>TIMER1_MAX_ARR/2){//limit to half max duty
					duty_cycle=TIMER1_MAX_ARR/2;
				}
			}*/
		}
	}
        /* 结合解锁、运行和输入状态判断是否允许执行驱动处理。 */
		if ((armed && running) && input > 47){
            /* 当随转速调整 PWM 周期的使能标志非零时进入此分支。 */
			if(VARIABLE_PWM){
				//tim1_arr = map(commutation_interval, 96, 200, TIMER1_MAX_ARR/2, TIMER1_MAX_ARR);				
                /* 按当前输入区间限幅并线性映射，得到当前实际使用的 PWM 周期值。 */
				tim1_arr = map(commutation_interval, 150, 750, VARIABLE_PWM_MIN_ARR, TIMER1_MAX_ARR);
			}
            /* 更新按当前 PWM 周期缩放后的实际比较值。 */
			adjusted_duty_cycle = ((duty_cycle * tim1_arr)/TIMER1_MAX_ARR)+1;
        /* 当前条件不满足时进入备选处理。 */
		}else{
                /* 当比例制动当前生效标志非零时进入此分支。 */
				if(prop_brake_active){
                    /* 更新按当前 PWM 周期缩放后的实际比较值。 */
					adjusted_duty_cycle = TIMER1_MAX_ARR - ((duty_cycle * tim1_arr)/TIMER1_MAX_ARR)+1;
                /* 当前条件不满足时进入备选处理。 */
				}else{
                /* 更新按当前 PWM 周期缩放后的实际比较值。 */
				adjusted_duty_cycle = DEAD_TIME * running;
				}
	    }
        /* 更新上一次输出的基准占空比。 */
		last_duty_cycle = duty_cycle;
        /* 更新TMR1 的周期/自动重装载寄存器。 */
		TMR1->pr = tim1_arr;

    /* 更新最终交给 PWM 寄存器的比较值缓存。 */
	last_adjusted_duty_cycle=adjusted_duty_cycle;
    /* 按过零超时情况限制占空比，再把调整后的值写入三个 PWM 通道；可选换相瞬间降占空比。 */
	setPwmRatio();
	}
/* 电周期除以 6 得到微秒扇区时间，再乘 2 换成 F421 的 0.5 微秒计数。 */
average_interval = e_com_time / 3;
/* 结合换相速度和过零累计情况选择启动、过滤或正常运行路径。 */
if(desync_check && zero_crosses > 10){
//	if(average_interval < last_average_interval){
//
//	}
        /* 保存相邻平均换相间隔的绝对差。 */
		int diff=getAbsDif(last_average_interval,average_interval);
/* 条件编译：defined(USE_DEBUG)；条件满足时采用以下实现。 */
#if defined(USE_DEBUG)
        /* 保存调试记录的间隔变化比例，放大 10000 倍。 */
		int diffratio=diff*10000/average_interval;
        /* 按条件 average_interval < 2000 && debug_desync_max_diff_ratio<diffratio 选择当前处理路径。 */
		if( average_interval < 2000 && debug_desync_max_diff_ratio<diffratio){
            /* 更新调试用相邻周期变化比例峰值。 */
			debug_desync_max_diff_ratio=diffratio;
		}
/* 结束当前条件编译分支。 */
#endif
        /* 按条件 (diff > average_interval>>1) && (average_interval < 2000) 选择当前处理路径。 */
		if((diff > average_interval>>1) && (average_interval < 2000)){ //throttle resitricted before zc 20.
        /* 将过零/换相累计计数清零。 */
		zero_crosses = 0;
        /* 递增检测到失步的次数。 */
		desync_happened ++;
        /* 将电机控制流程正在运行的标志清零。 */
		running = 0;
        /* 将过零检测路径选择设为 1。 */
		old_routine = 1;
            /* 当过零/换相累计计数 大于 100 时进入此分支。 */
			if(zero_crosses > 100){
                /* 更新由电周期折算的平均扇区计数。 */
				average_interval = 5000;
			}
        /* 更新上一次输出的基准占空比。 */
		last_duty_cycle = min_startup_duty/2;
		}
        /* 将完成一轮换相后执行失步检查的标志清零。 */
		desync_check = 0;
//	}
    /* 更新上一轮用于失步比较的平均扇区计数。 */
	last_average_interval = average_interval;
	}
//#ifndef MCU_F031
//if(commutation_interval > 400){
//	   NVIC_SetPriority(IC_DMA_IRQ_NAME, 0);
//	   NVIC_SetPriority(ADC_CMP_IRQn, 1);
//}else{
//	NVIC_SetPriority(IC_DMA_IRQ_NAME, 1);
//	NVIC_SetPriority(ADC_CMP_IRQn, 0);
//}
//#endif   //mcu f031

/* 结束当前条件编译分支。 */
#endif // ndef brushed_mode

/* 当待发送串口遥测标志非零时进入此分支。 */
if(send_telemetry){
/* 仅在定义 USE_SERIAL_TELEMETRY 时编译以下代码。 */
#ifdef	USE_SERIAL_TELEMETRY
      /* 按温度、电压、电流、耗电量和电转速顺序组装 10 字节遥测帧，多字节字段高字节先发。 */
	  makeTelemPackage(degrees_celsius,
                       /* 遥测参数：电池电压，单位 0.01 V。 */
			           battery_voltage,
                       /* 遥测参数：电流，单位约 0.01 A。 */
					   actual_current,
                       /* 遥测参数：累计耗电量转为整数 mAh。 */
	  				   (uint16_t)consumed_current,
                        /* 遥测参数：电转速除以 100。 */
	  					e_rpm);
      /* 装载遥测帧长度并启动串口发送 DMA，完成后的通道清理由中断处理。 */
	  send_telem_DMA();
      /* 将待发送串口遥测标志清零。 */
	  send_telemetry = 0;
/* 结束当前条件编译分支。 */
#endif
	}
/* 条件编译：defined(FIXED_DUTY_MODE) || defined(FIXED_SPEED_MODE)；条件满足时采用以下实现。 */
#if defined(FIXED_DUTY_MODE) || defined(FIXED_SPEED_MODE)

//		if(INPUT_PIN_PORT->IDR & INPUT_PIN){
//			signaltimeout ++;
//			if(signaltimeout > 10000){
//				NVIC_SystemReset();
//			}
//		}else{
//			signaltimeout = 0;
//		}
/* 采用上一编译条件不成立时的备选实现。 */
#else
	  
        /* 递增自最近有效输入以来的周期计数。 */
		signaltimeout++;
	
        /* 当自最近有效输入以来的周期计数 大于 5000 时进入此分支。 */
		if(signaltimeout > 5000) { // quarter second timeout when armed half second for servo;
            /* 当电调已解锁并允许驱动的标志非零时进入此分支。 */
			if(armed){
                /* 将 A、B、C 三相全部置为浮空，关闭各相高低侧驱动。 */
				allOff();
                /* 将电调已解锁并允许驱动的标志清零。 */
				armed = 0;
                /* 将交给电机控制状态机的最终油门值清零。 */
				input = 0;
                /* 将已识别输入信号类型的标志清零。 */
				inputSet = 0;
                /* 将连续检测到零油门的次数清零。 */
				zero_input_count = 0;
				
                /* 将TMR1 的通道 1 捕获/比较寄存器清零。 */
				TMR1->c1dt = 0;
              /* 将TMR1 的通道 2 捕获/比较寄存器清零。 */
			  TMR1->c2dt = 0;
                /* 将TMR1 的通道 3 捕获/比较寄存器清零。 */
				TMR1->c3dt = 0;

                /* 将IC_TIMER_REGISTER 的时钟预分频寄存器清零。 */
				IC_TIMER_REGISTER->div = 0;
                /* 将IC_TIMER_REGISTER 的当前计数值清零。 */
				IC_TIMER_REGISTER->cval = 0;

                /* 按循环下标遍历本组数据或重复执行指定次数的处理。 */
				for(int i = 0; i < 64; i++){
                    /* 将输入信号边沿捕获时间戳缓冲区中的当前元素清零。 */
					dma_buffer[i] = 0;
				}
				NVIC_SystemReset();
			}
/* 条件编译：!defined USE_DEBUG；条件满足时采用以下实现。 */
#if !defined USE_DEBUG
        /* 当自最近有效输入以来的周期计数 大于 50000 时进入此分支。 */
		if ( signaltimeout > 50000){     // 5 second
            /* 将 A、B、C 三相全部置为浮空，关闭各相高低侧驱动。 */
			allOff();
            /* 将电调已解锁并允许驱动的标志清零。 */
			armed = 0;
            /* 将交给电机控制状态机的最终油门值清零。 */
			input = 0;
            /* 将已识别输入信号类型的标志清零。 */
			inputSet = 0;
            /* 将连续检测到零油门的次数清零。 */
			zero_input_count = 0;
				
                /* 将TMR1 的通道 1 捕获/比较寄存器清零。 */
				TMR1->c1dt = 0;
              /* 将TMR1 的通道 2 捕获/比较寄存器清零。 */
			  TMR1->c2dt = 0;
                /* 将TMR1 的通道 3 捕获/比较寄存器清零。 */
				TMR1->c3dt = 0;
                /* 将IC_TIMER_REGISTER 的时钟预分频寄存器清零。 */
				IC_TIMER_REGISTER->div = 0;
                /* 将IC_TIMER_REGISTER 的当前计数值清零。 */
				IC_TIMER_REGISTER->cval = 0;
            /* 按循环下标遍历本组数据或重复执行指定次数的处理。 */
			for(int i = 0; i < 64; i++){
                /* 将输入信号边沿捕获时间戳缓冲区中的当前元素清零。 */
				dma_buffer[i] = 0;
			}
			NVIC_SystemReset();
		}
/* 结束当前条件编译分支。 */
#endif

			}
/* 结束当前条件编译分支。 */
#endif
}


/**
 * @brief 按旋转方向推进三相正弦表索引，越界后回绕，并把查表结果缩放到 PWM 比较值。
 */
void advanceincrement(){
/* 当当前六步旋转方向为零时进入此分支。 */
if (!forward){
    /* 递增A 相正弦查表角度索引。 */
	phase_A_position ++;
    /* 当A 相正弦查表角度索引 大于 359 时进入此分支。 */
    if (phase_A_position > 359){
       /* 将A 相正弦查表角度索引清零。 */
	   phase_A_position = 0 ;
    }
        /* 递增B 相正弦查表角度索引。 */
	    phase_B_position ++;
         /* 当B 相正弦查表角度索引 大于 359 时进入此分支。 */
	     if (phase_B_position > 359){
        /* 将B 相正弦查表角度索引清零。 */
		phase_B_position = 0 ;
	}
        /* 递增C 相正弦查表角度索引。 */
	    phase_C_position ++;
         /* 当C 相正弦查表角度索引 大于 359 时进入此分支。 */
	     if (phase_C_position > 359){
        /* 将C 相正弦查表角度索引清零。 */
		phase_C_position = 0 ;
	}
/* 当前条件不满足时进入备选处理。 */
}else{
           /* 递减A 相正弦查表角度索引。 */
	       phase_A_position --;
        /* 当A 相正弦查表角度索引 小于 0 时进入此分支。 */
	    if (phase_A_position < 0){
           /* 更新A 相正弦查表角度索引。 */
		   phase_A_position = 359 ;
	    }
            /* 递减B 相正弦查表角度索引。 */
		    phase_B_position --;
             /* 当B 相正弦查表角度索引 小于 0 时进入此分支。 */
		     if (phase_B_position < 0){
            /* 更新B 相正弦查表角度索引。 */
			phase_B_position = 359;
		}
            /* 递减C 相正弦查表角度索引。 */
		    phase_C_position --;
             /* 当C 相正弦查表角度索引 小于 0 时进入此分支。 */
		     if (phase_C_position < 0){
            /* 更新C 相正弦查表角度索引。 */
			phase_C_position = 359 ;
		}
}
/* 仅在定义 GIMBAL_MODE 时编译以下代码。 */
#ifdef GIMBAL_MODE
    /* 更新TMR1 的通道 1 捕获/比较寄存器。 */
    TMR1->c1dt = ((2*pwmSin[phase_A_position])+gate_drive_offset)*TIMER1_MAX_ARR/2000;
    /* 更新TMR1 的通道 2 捕获/比较寄存器。 */
    TMR1->c2dt = ((2*pwmSin[phase_B_position])+gate_drive_offset)*TIMER1_MAX_ARR/2000;
    /* 更新TMR1 的通道 3 捕获/比较寄存器。 */
    TMR1->c3dt = ((2*pwmSin[phase_C_position])+gate_drive_offset)*TIMER1_MAX_ARR/2000;
/* 采用上一编译条件不成立时的备选实现。 */
#else
    /* 更新TMR1 的通道 1 捕获/比较寄存器。 */
    TMR1->c1dt = (((2*pwmSin[phase_A_position]/SINE_DIVIDER)+gate_drive_offset)*TIMER1_MAX_ARR/2000)*sine_mode_power / 10;
    /* 更新TMR1 的通道 2 捕获/比较寄存器。 */
    TMR1->c2dt = (((2*pwmSin[phase_B_position]/SINE_DIVIDER)+gate_drive_offset)*TIMER1_MAX_ARR/2000)*sine_mode_power / 10;
    /* 更新TMR1 的通道 3 捕获/比较寄存器。 */
    TMR1->c3dt = (((2*pwmSin[phase_C_position]/SINE_DIVIDER)+gate_drive_offset)*TIMER1_MAX_ARR/2000)*sine_mode_power / 10;
/* 结束当前条件编译分支。 */
#endif
}

/**
 * @brief 轮询路径的换相收尾：执行换相并复位计数，满足已捕获过零等条件后切到中断检测。
 * @param real_zero_cross 非零才把本次事件计入真实过零计数。
 */
void commutateProcess(uint8_t real_zero_cross){
        /* 推进或回退六步相序，更新电周期估计和过零方向，执行桥臂换相并切换比较器输入。 */
		commutate();
    /* 将轮询中满足预期电平的读数累计次数清零。 */
    bemfcounter = 0;
    /* 将轮询检测到错误方向电平的累计次数清零。 */
    bad_count = 0;

    /* 当real_zero_cross非零时进入此分支。 */
    if(real_zero_cross){
        /* 递增过零/换相计数；具体是否代表真实过零由本调用路径决定。 */
    	zero_crosses++;
    }
/* 仅在定义 POWER_ON_TEST_MODE 时编译以下代码。 */
#ifdef POWER_ON_TEST_MODE
    /* 当过零/换相累计计数 大于 试转期间过零计数的峰值 时进入此分支。 */
	if(zero_crosses > power_on_test_zero_cross_peak){
        /* 更新试转期间过零计数的峰值。 */
		power_on_test_zero_cross_peak = zero_crosses;
	}
/* 结束当前条件编译分支。 */
#endif
    /* 按条件 stall_protection || RC_CAR_REVERSE 选择当前处理路径。 */
    if(stall_protection || RC_CAR_REVERSE){
     /* 结合换相速度和过零累计情况选择启动、过滤或正常运行路径。 */
   	 if (zero_crosses >= 20 && commutation_interval <= 2000) {
            /* 将过零检测路径选择清零。 */
   	    	old_routine = 0;
            /* 允许过零 EXINT 中断；本函数只置位中断使能，不清除之前的挂起标志。 */
   	    	enableCompInterrupts();          // enable interrupt

    	 }
    /* 当前条件不满足时进入备选处理。 */
    }else{
            /* 检查跨行列出的组合条件后选择处理分支。 */
	    	if(zero_crosses > 30
/* 仅在定义 POWER_ON_TEST_MODE 时编译以下代码。 */
#ifdef POWER_ON_TEST_MODE
                /* 续接上一行的位组合或条件判断。 */
				&& power_on_test_closed_loop
                /* 续接上一行的位组合或条件判断。 */
				&& commutation_interval <= POWER_ON_TEST_INTERRUPT_COMM_INTERVAL
/* 结束当前条件编译分支。 */
#endif
			){
            /* 将过零检测路径选择清零。 */
	    	old_routine = 0;
        /* 允许过零 EXINT 中断；本函数只置位中断使能，不清除之前的挂起标志。 */
    	enableCompInterrupts();          // enable interrupt

    }
   }
}
/**
 * @brief 轮询路径的过零处理；区分真实过零和强制换相，等待半扇区减提前量后执行下一步。
 *
 * 试转捕获只接受设定时间窗口内的真实过零；强制换相不增加连续有效过零计数。
 * 这是主循环中的忙等路径，等待时间按 INTERVAL_TIMER 的当前计数尺度计算。
 * @param forced_commutation 1 表示超时强制换相，0 表示轮询检测到过零。
 */
void zcfoundroutine(uint8_t forced_commutation){   // only used in polling mode, blocking routine.
    /* 更新本次过零或超时发生时的间隔计数。 */
	thiszctime = INTERVAL_TIMER->cval;
/* 仅在定义 POWER_ON_TEST_MODE 时编译以下代码。 */
#ifdef POWER_ON_TEST_MODE
    /* 当forced_commutation为零时进入此分支。 */
	if(!forced_commutation){
		// Ignore commutation ringing and hand-generated transitions during the
		// slow forced ramp. A usable crossing must arrive near mid-sector after
		// the ramp has reached acquisition speed.
        /* 检查跨行列出的组合条件后选择处理分支。 */
		if(comm_timeout > POWER_ON_TEST_BEMF_ACQUIRE_INTERVAL ||
            /* 过早的跳变可能来自换相振铃；试转捕获窗口排除当前超时间隔的前四分之一。 */
			thiszctime < (comm_timeout / 4) ||
            /* 超出允许捕获窗口的事件不计入试转有效过零。 */
			thiszctime > comm_timeout){
            /* 将轮询中满足预期电平的读数累计次数清零。 */
			bemfcounter = 0;
            /* 将轮询检测到错误方向电平的累计次数清零。 */
			bad_count = 0;
            /* 将轮询路径本次过零已处理的标志清零。 */
			zcfound = 0;
            /* 结束当前函数，返回调用方。 */
			return;
		}
        /* 将进入捕获速度范围后的连续超时次数清零。 */
		power_on_test_acquire_timeouts = 0;
        /* 当捕获阶段连续有效过零次数 小于 255 时进入此分支。 */
		if(power_on_test_valid_zero_crosses < 255){
            /* 递增捕获阶段连续有效过零次数。 */
			power_on_test_valid_zero_crosses++;
		}
        /* 连续真实过零次数达到要求后，确认试转已捕获反电动势闭环。 */
		if(power_on_test_valid_zero_crosses >= POWER_ON_TEST_REQUIRED_VALID_ZC){
            /* 将试转已达到有效过零锁定条件的标志设为 1。 */
			power_on_test_closed_loop = 1;
		}
	}
/* 结束当前条件编译分支。 */
#endif
    /* 将INTERVAL_TIMER 的当前计数值清零。 */
	INTERVAL_TIMER->cval = 0;
    /* 更新单扇区的估计时间计数。 */
	commutation_interval = (thiszctime + (3*commutation_interval)) / 4;
    /* 更新提前角对应的时间计数。 */
	advance = commutation_interval / advancedivisor;
    /* 相邻过零相隔 60 电角度，过零后等待约半扇区（30 度）再扣除提前时间。 */
	waitTime = commutation_interval /2  - advance;
    /* 更新换相附近的 PWM 调整阶段。 */
	inner_step=-1;
    /* 重复检查等待条件：当INTERVAL_TIMER 的当前计数值 小于 过零到下一次换相的等待计数 时继续循环。 */
	while (INTERVAL_TIMER->cval < waitTime){
	}
    /* 轮询路径的换相收尾：执行换相并复位计数，满足已捕获过零等条件后切到中断检测。 */
	commutateProcess(!forced_commutation);
}

/**
 * @brief 判断 val 是否严格落在 target 的指定百分比范围内，边界值不包含在内。
 * @param target 参考目标值。
 * @param val 需要判断是否在范围内的值。
 * @param percent 目标值两侧的允许偏差百分比。
 * @return 范围内返回非零，否则返回 0。
 */
inline int inrange(int target,int val,int percent){
    /* 返回val < target*(100+percent)/100 && val > target*(100-percent)/100。 */
	return val < target*(100+percent)/100 && val > target*(100-percent)/100;
}
/* 保存诊断开始时播放的固定旋律序列。 */
const uint8_t detectorNotes[]={0xa0,0x19,0xe0,0,0xa0,0x19,0xe0,0,0xa0,0x19,0xe0,0,0xa0,0x19,0xe0,0,0xa0,0x19,0xe0,0,0xa0,0x19,0xe0,0};
/* 保存根据诊断结果动态生成的提示音序列。 */
uint8_t bjNotesTemp[4*8];
/* 保存三次相激励各采四路反馈的诊断矩阵。 */
uint16_t alldiagnose_data[4*3];
/* 保存三相激励结果对应的异常标志。 */
char phaseError[3];
/* 保存每次激励、每路反馈的临时偏差判定。 */
char compErrorTemp[12];
/* 保存四路 BEMF 采样通道对应的异常标志。 */
char compError[4];	
/**
 * @brief 逐相施加短时激励并采集四路反馈，通过数据一致性形成诊断提示音；此过程会驱动桥臂。
 *
 * 诊断会临时关闭中断并接管 ADC/DMA；不是仅仅读取变量的无激励检查。
 * 结束后 main 会再次调用 ADC_Init 恢复普通采样配置。
 */
void SystemDiagnose(){
	__disable_irq();	
    /* 切换为四路 BEMF 诊断采样，DMA 写入 adc_diagnose_data；会重新配置 ADC 和 DMA1 通道 1。 */
	ADC_Init_Detector();
    /* 临时修改 UTILITY_TIMER 分频进行毫秒级忙等，等待期间喂狗，结束后恢复代码指定的分频。 */
	delayMillis(100);
    /* 解析成对的时长/音符编码，播放休止符和音符；播放完成后关闭桥臂并恢复 PWM 时基。 */
	playBlueJayTune(detectorNotes,0,sizeof(detectorNotes),1);

    /* 保存诊断中有效反馈读数的平均值。 */
	int voltateAverage=0;
    /* 保存诊断中参与平均的有效读数个数。 */
	int effectiveVolateCount=0;
    /* 遍历诊断激励相或反馈通道，建立三相四路采样和判定结果。 */
	for(int i=0;i<3;++i){
        /* 将 A、B、C 三相全部置为浮空，关闭各相高低侧驱动。 */
		allOff();
        /* 将TMR1 的通道 1 捕获/比较寄存器清零。 */
		TMR1->c1dt=0;
        /* 将TMR1 的通道 2 捕获/比较寄存器清零。 */
		TMR1->c2dt=0;
        /* 将TMR1 的通道 3 捕获/比较寄存器清零。 */
		TMR1->c3dt=0;
        /* 按命令编号、遥测状态或换相步骤分派到对应处理分支。 */
		switch(i){//charging
            /* 诊断激励分支 0：选择下一相进行反馈采样。 */
			case 0:
                /* 将 A 相切为 PWM 驱动；是否启用互补低侧由 comp_pwm 决定，另一种桥型使用使能脚。 */
				phaseAPWM();
                /* 临时修改 UTILITY_TIMER 分频进行毫秒级忙等，等待期间喂狗，结束后恢复代码指定的分频。 */
				delayMillis(1);
            /* 结束当前分支或循环，避免继续处理后续候选项。 */
			break;
            /* 诊断激励分支 1：选择下一相进行反馈采样。 */
			case 1:
                /* 将 B 相切为 PWM 驱动；是否启用互补低侧由 comp_pwm 决定，另一种桥型使用使能脚。 */
				phaseBPWM();
                /* 临时修改 UTILITY_TIMER 分频进行毫秒级忙等，等待期间喂狗，结束后恢复代码指定的分频。 */
				delayMillis(1);
            /* 结束当前分支或循环，避免继续处理后续候选项。 */
			break;
            /* 诊断激励分支 2：选择下一相进行反馈采样。 */
			case 2:
                /* 将 C 相切为 PWM 驱动；是否启用互补低侧由 comp_pwm 决定，另一种桥型使用使能脚。 */
				phaseCPWM();
                /* 临时修改 UTILITY_TIMER 分频进行毫秒级忙等，等待期间喂狗，结束后恢复代码指定的分频。 */
				delayMillis(1);
            /* 结束当前分支或循环，避免继续处理后续候选项。 */
			break;
		}
        /* 更新TMR1 的通道 1 捕获/比较寄存器。 */
		TMR1->c1dt=TMR1->pr+1;
        /* 更新TMR1 的通道 2 捕获/比较寄存器。 */
		TMR1->c2dt=TMR1->pr+1;
        /* 更新TMR1 的通道 3 捕获/比较寄存器。 */
		TMR1->c3dt=TMR1->pr+1;
        /* 使用 UTILITY_TIMER 忙等延时；会清零共享计数器，时间基准取决于当前定时器分频。 */
		delayMicros(500);
        /* 通过软件启动一轮普通 ADC 转换。 */
		adc_ordinary_software_trigger_enable(ADC1, TRUE);
        /* 阻塞等待本轮四通道诊断 DMA 完成；现有实现没有额外超时退出。 */
		while(dma_flag_get(DMA1_FDT1_FLAG) == RESET);
        /* 清除已处理的 DMA 状态标志。 */
		dma_flag_clear(DMA1_FDT1_FLAG);
        /* 遍历诊断激励相或反馈通道，建立三相四路采样和判定结果。 */
		for(int j=0;j<4;++j){
            /* 更新三次相激励各采四路反馈的诊断矩阵中的当前元素。 */
			alldiagnose_data[i*4+j]=adc_diagnose_data[j];
            /* 当BEMF 诊断 ADC 的四路采样缓冲区中的当前元素 大于 100 时进入此分支。 */
			if(adc_diagnose_data[j]>100){				
                /* 累计增加诊断中有效反馈读数的平均值。 */
				voltateAverage+=adc_diagnose_data[j];
                /* 递增诊断中参与平均的有效读数个数。 */
				++effectiveVolateCount;
			}
		}
	}
    /* 当诊断中参与平均的有效读数个数 大于 0 时进入此分支。 */
	if(effectiveVolateCount>0){
        /* 更新诊断中有效反馈读数的平均值。 */
		voltateAverage=voltateAverage/effectiveVolateCount;
    /* 当前条件不满足时进入备选处理。 */
	}else{
        /* 更新诊断中有效反馈读数的平均值。 */
		voltateAverage=1000;
	}
    /* 将TMR1 的通道 1 捕获/比较寄存器清零。 */
	TMR1->c1dt=0;
    /* 将TMR1 的通道 2 捕获/比较寄存器清零。 */
	TMR1->c2dt=0;
    /* 将TMR1 的通道 3 捕获/比较寄存器清零。 */
	TMR1->c3dt=0;
    /* 将 A、B、C 三相全部置为浮空，关闭各相高低侧驱动。 */
	allOff();
    /* 临时修改 UTILITY_TIMER 分频进行毫秒级忙等，等待期间喂狗，结束后恢复代码指定的分频。 */
	delayMillis(100);
	

    /* 遍历诊断激励相或反馈通道，建立三相四路采样和判定结果。 */
	for(int i=0;i<3;++i){
        /* 遍历诊断激励相或反馈通道，建立三相四路采样和判定结果。 */
		for(int j=0;j<4;++j){
            /* 更新每次激励、每路反馈的临时偏差判定中的当前元素。 */
			compErrorTemp[i*4+j]=!inrange(voltateAverage,alldiagnose_data[i*4+j],5);
		}
        /* 更新三相激励结果对应的异常标志中的当前元素。 */
		phaseError[i]=compErrorTemp[i*4+0] & compErrorTemp[i*4+1] & compErrorTemp[i*4+2] & compErrorTemp[i*4+3];
	}
    /* 遍历诊断激励相或反馈通道，建立三相四路采样和判定结果。 */
	for(int j=0;j<4;++j){
        /* 更新四路 BEMF 采样通道对应的异常标志中的当前元素。 */
		compError[j]=compErrorTemp[0*4+j]&compErrorTemp[1*4+j]&compErrorTemp[2*4+j];
	}
	

    /* 遍历诊断激励相或反馈通道，建立三相四路采样和判定结果。 */
	for(int i=0;i<8;++i){
        /* 更新根据诊断结果动态生成的提示音序列中的当前元素。 */
		bjNotesTemp[i*4]=0xa0;
        /* 当i 小于 1 时进入此分支。 */
		if(i<1){
            /* 更新根据诊断结果动态生成的提示音序列中的当前元素。 */
			bjNotesTemp[i*4+1]=0x21;
        /* 当i 小于 4 时进入此分支。 */
		}else if(i<4){
            /* 更新根据诊断结果动态生成的提示音序列中的当前元素。 */
			bjNotesTemp[i*4+1]=phaseError[i-1]?0:0x19;
        /* 当前条件不满足时进入备选处理。 */
		}else{
            /* 更新根据诊断结果动态生成的提示音序列中的当前元素。 */
			bjNotesTemp[i*4+1]=compError[i-4]?0:0x19;
		}
        /* 更新根据诊断结果动态生成的提示音序列中的当前元素。 */
		bjNotesTemp[i*4+2]=0xe0;
        /* 将根据诊断结果动态生成的提示音序列中的当前元素清零。 */
		bjNotesTemp[i*4+3]=0;		
	}
    /* 解析成对的时长/音符编码，播放休止符和音符；播放完成后关闭桥臂并恢复 PWM 时基。 */
	playBlueJayTune(bjNotesTemp,0,sizeof(bjNotesTemp),0);			
	__enable_irq();
}

/**
 * @brief 程序入口：初始化外设和配置，再循环处理 ADC、输入方向、过零检测及正弦/六步运行状态。
 * @return 正常运行时不返回。
 */
int main(void)
{
	__enable_irq();

 /* 更新主循环 ADC 更新分频计数。 */
 adc_counter = 2;
 /* 按依赖顺序初始化系统时钟、GPIO、DMA、PWM、各用途定时器、比较器和可选串口遥测。 */
 initCorePeripherals();
	
    /* 配置指定主通道或互补通道的输出使能。 */
	tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_1, TRUE);
    /* 配置指定主通道或互补通道的输出使能。 */
	tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_2, TRUE);
    /* 配置指定主通道或互补通道的输出使能。 */
	tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_3, TRUE);
    /* 配置指定主通道或互补通道的输出使能。 */
	tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_1C, TRUE);
    /* 配置指定主通道或互补通道的输出使能。 */
	tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_2C, TRUE);
    /* 配置指定主通道或互补通道的输出使能。 */
	tmr_channel_enable(TMR1, TMR_SELECT_CHANNEL_3C, TRUE);


  /* Enable tim1 */
 /* 将TMR1 的定时器计数使能设为 1。 */
 TMR1->ctrl1_bit.tmren = TRUE;
 /* 将TMR1 的高级定时器主输出使能设为 1。 */
 TMR1->brk_bit.oen = TRUE;
 
 /* Force update generation */
/* 置位TMR1 的软件事件寄存器。 */
TMR1->swevt |= TMR_OVERFLOW_SWTRIG;


/* 更新主循环 ADC 更新分频计数。 */
adc_counter = 4;
/* 仅在定义 USE_RGB_LED 时编译以下代码。 */
#ifdef USE_RGB_LED
  /* 配置可选 RGB 状态灯引脚；仅在 USE_RGB_LED 编译分支中使用。 */
  LED_GPIO_init();
  /* 更新GPIOB 的兼容 GPIO 引脚清零寄存器。 */
  GPIOB->BRR = LL_GPIO_PIN_8; // turn on red
  /* 更新GPIOB 的兼容 GPIO 引脚置位/复位寄存器。 */
  GPIOB->BSRR = LL_GPIO_PIN_5;
  /* 更新GPIOB 的兼容 GPIO 引脚置位/复位寄存器。 */
  GPIOB->BSRR = LL_GPIO_PIN_3; //
/* 结束当前条件编译分支。 */
#endif

/* 仅在未定义 BRUSHED_MODE 时编译以下代码。 */
#ifndef BRUSHED_MODE             // commutation_timer priority 0
      /* 将COM_TIMER 的定时器计数使能设为 1。 */
	  COM_TIMER->ctrl1_bit.tmren = TRUE;
    /* 置位COM_TIMER 的软件事件寄存器。 */
    COM_TIMER->swevt |= TMR_OVERFLOW_SWTRIG;
    /* 按掩码更新COM_TIMER 的中断/DMA 请求使能寄存器。 */
    COM_TIMER->iden &= ~TMR_OVF_INT;
   /* 结束当前条件编译分支。 */
   #endif

 /* 将UTILITY_TIMER 的定时器计数使能设为 1。 */
 UTILITY_TIMER->ctrl1_bit.tmren = TRUE;
 //delayMillis(2000);

/* 将INTERVAL_TIMER 的定时器计数使能设为 1。 */
INTERVAL_TIMER->ctrl1_bit.tmren = TRUE;
/* 置位INTERVAL_TIMER 的软件事件寄存器。 */
INTERVAL_TIMER->swevt |= TMR_OVERFLOW_SWTRIG;
//TMR6->ists = (uint16_t)~TMR_OVF_FLAG;
/* 置位INTERVAL_TIMER 的中断/DMA 请求使能寄存器。 */
INTERVAL_TIMER->iden |= TMR_OVF_INT;


/* 更新主循环 ADC 更新分频计数。 */
adc_counter = 5;

 /* 将TEN_KHZ_TIMER 的定时器计数使能设为 1。 */
 TEN_KHZ_TIMER->ctrl1_bit.tmren = TRUE;
 /* 置位TEN_KHZ_TIMER 的软件事件寄存器。 */
 TEN_KHZ_TIMER->swevt |= TMR_OVERFLOW_SWTRIG;
 /* 置位TEN_KHZ_TIMER 的中断/DMA 请求使能寄存器。 */
 TEN_KHZ_TIMER->iden |= TMR_OVF_INT;


/* 更新主循环 ADC 更新分频计数。 */
adc_counter = 6;
/* 从 Flash 读取 176 字节配置，按字段解码模式和参数，并为部分越界值保留或选择默认值。 */
loadEEpromSettings();
  /* 检查固件主版本的配置值是否满足本分支要求。 */
  if(VERSION_MAJOR != eepromBuffer[3] || VERSION_MINOR != eepromBuffer[4]){
      /* 写入配置区中的固件主版本，稍后由 Flash 保存流程持久化。 */
	  eepromBuffer[3] = VERSION_MAJOR;
      /* 写入配置区中的固件次版本，稍后由 Flash 保存流程持久化。 */
	  eepromBuffer[4] = VERSION_MINOR;
      /* 按循环下标遍历本组数据或重复执行指定次数的处理。 */
	  for(int i = 0; i < 12 ; i ++){
          /* 写入176 字节 Flash 参数及旋律数据的 RAM 镜像中的当前元素，稍后由 Flash 保存流程持久化。 */
		  eepromBuffer[5+i] = (uint8_t)FIRMWARE_NAME[i];
	  }
      /* 把运行中的方向及模式等字段写回配置缓冲区，再保存整个 176 字节配置到 Flash。 */
	  saveEEpromSettings();
  }

  /* 当正弦启动使能非零时进入此分支。 */
  if(use_sin_start){
  /* 更新启动时最小 PWM 比较值。 */
  min_startup_duty = sin_mode_min_s_d;
  }
/* 仅在定义 POWER_ON_TEST_MODE 时编译以下代码。 */
#ifdef POWER_ON_TEST_MODE
    /* 按循环下标遍历本组数据或重复执行指定次数的处理。 */
	forward = POWER_ON_TEST_FORWARD;
/* 采用上一编译条件不成立时的备选实现。 */
#else
    /* 当保存的电机默认方向反转设置 等于 1 时进入此分支。 */
	if (dir_reversed == 1){
            /* 按循环下标遍历本组数据或重复执行指定次数的处理。 */
			forward = 0;
        /* 当前条件不满足时进入备选处理。 */
		}else{
            /* 按循环下标遍历本组数据或重复执行指定次数的处理。 */
			forward = 1;
		}
/* 结束当前条件编译分支。 */
#endif
    /* 更新当前实际使用的 PWM 周期值。 */
	tim1_arr = TIMER1_MAX_ARR;
    /* 更新启动阶段允许的最大 PWM 比较值。 */
	startup_max_duty_cycle = startup_max_duty_cycle * TIMER1_MAX_ARR / 2000  + dead_time_override;  // adjust for pwm frequency
    /* 更新低转速端允许的最大占空比。 */
	throttle_max_at_low_rpm  = throttle_max_at_low_rpm * TIMER1_MAX_ARR / 2000;  // adjust to new pwm frequency
    /* 更新高转速端允许的最大占空比。 */
    throttle_max_at_high_rpm = TIMER1_MAX_ARR;  // adjust to new pwm frequency
    /* 当互补 PWM 使能标志为零时进入此分支。 */
	if(!comp_pwm){
        /* 将正弦启动使能清零。 */
		use_sin_start = 0;  // sine start requires complementary pwm.
	}
/* 更新主循环 ADC 更新分频计数。 */
adc_counter = 7;
    /* 当车模先制动再反转模式标志非零时进入此分支。 */
	if (RC_CAR_REVERSE) {         // overrides a whole lot of things!
        /* 更新低转速端允许的最大占空比。 */
		throttle_max_at_low_rpm = 1000;
        /* 将双向油门模式标志设为 1。 */
		bi_direction = 1;
        /* 将正弦启动使能清零。 */
		use_sin_start = 0;
        /* 将低转速限制最大占空比的使能标志设为 1。 */
		low_rpm_throttle_limit = 1;
        /* 将随转速调整 PWM 周期的使能标志清零。 */
		VARIABLE_PWM = 0;
		//stall_protection = 1;
        /* 将互补 PWM 使能标志清零。 */
		comp_pwm = 0;
    /* 将堵转超时停机保护使能清零。 */
    stuck_rotor_protection = 0;
        /* 更新正常驱动最小 PWM 比较值。 */
		minimum_duty_cycle = minimum_duty_cycle + 50;
        /* 更新低速补偿模式的最小 PWM 比较值。 */
		stall_protect_minimum_duty = stall_protect_minimum_duty + 50;
        /* 更新启动时最小 PWM 比较值。 */
		min_startup_duty = min_startup_duty + 50;
	}
/* 当前实现将配置字节 39 同时用于启动 SystemDiagnose；该诊断会实际切换桥臂。 */
if(eepromBuffer[39]==1){//hull senser
    /* 逐相施加短时激励并采集四路反馈，通过数据一致性形成诊断提示音；此过程会驱动桥臂。 */
	SystemDiagnose();
}
/* 仅在定义 USE_ADC 时编译以下代码。 */
#ifdef USE_ADC
   /* 配置普通 ADC 扫描和 DMA 循环搬运；当前序列依次为 PA3、PA6、内部温度通道。 */
   ADC_Init();
/* 结束当前条件编译分支。 */
#endif

/* 仅在定义 USE_ADC_INPUT 时编译以下代码。 */
#ifdef USE_ADC_INPUT

/* 采用上一编译条件不成立时的备选实现。 */
#else
/* 配置指定主通道或互补通道的输出使能。 */
tmr_channel_enable(IC_TIMER_REGISTER, IC_TIMER_CHANNEL, TRUE);
/* 将IC_TIMER_REGISTER 的定时器计数使能设为 1。 */
IC_TIMER_REGISTER->ctrl1_bit.tmren = TRUE;

/* 结束当前条件编译分支。 */
#endif

/* 仅在定义 MCU_F031 时编译以下代码。 */
#ifdef MCU_F031
      /* 更新GPIOF 的兼容 GPIO 引脚置位/复位寄存器。 */
	  GPIOF->BSRR = LL_GPIO_PIN_6;            // uncomment to take bridge out of standby mode and set oc level
      /* 更新GPIOF 的兼容 GPIO 引脚清零寄存器。 */
	  GPIOF->BRR = LL_GPIO_PIN_7;				// out of standby mode
      /* 更新GPIOA 的兼容 GPIO 引脚清零寄存器。 */
	  GPIOA->BRR = LL_GPIO_PIN_11;
/* 结束当前条件编译分支。 */
#endif

/* 条件编译：defined(FIXED_DUTY_MODE) || defined(FIXED_SPEED_MODE)；条件满足时采用以下实现。 */
#if defined(FIXED_DUTY_MODE) || defined(FIXED_SPEED_MODE)
 /* 配置并启动独立看门狗，之后运行流程需定期写重载命令。 */
 MX_IWDG_Init();
 /* 重载看门狗，表明控制流程仍在正常执行。 */
 WDT->cmd = WDT_CMD_RELOAD;
 /* 将已识别输入信号类型的标志设为 1。 */
 inputSet = 1;
 /* 将电调已解锁并允许驱动的标志设为 1。 */
 armed = 1;
 /* 更新经过方向和死区处理的油门值。 */
 adjusted_input = 48;
 /* 更新输入协议刚解码得到的油门或命令数值。 */
 newinput = 48;
/* 仅在定义 POWER_ON_TEST_MODE 时编译以下代码。 */
#ifdef POWER_ON_TEST_MODE
 /* 更新电机 KV 参数。 */
 motor_kv = POWER_ON_TEST_MOTOR_KV;
 /* 更新电机磁极总数。 */
 motor_poles = POWER_ON_TEST_MOTOR_POLES;
 /* 更新换相提前角档位。 */
 advance_level = POWER_ON_TEST_ADVANCE_LEVEL;
 // Convert the 0..2000 test scale to the active 4000-count PWM period.
 // The old raw assignment produced only about 7.5% effective startup duty.
 /* 更新启动时最小 PWM 比较值。 */
 min_startup_duty = POWER_ON_TEST_MIN_STARTUP_DUTY * TIMER1_MAX_ARR / 2000 + dead_time_override;
 /* 更新正常驱动最小 PWM 比较值。 */
 minimum_duty_cycle = (POWER_ON_TEST_MIN_STARTUP_DUTY / 2) * TIMER1_MAX_ARR / 2000 + dead_time_override;
 /* 更新启动阶段允许的最大 PWM 比较值。 */
 startup_max_duty_cycle = POWER_ON_TEST_STARTUP_MAX_DUTY * TIMER1_MAX_ARR / 2000 + dead_time_override;
 /* 更新低速补偿模式的最小 PWM 比较值。 */
 stall_protect_minimum_duty = minimum_duty_cycle;
 /* 更新上升方向过零的有效读数门限。 */
 min_bemf_counts_up = POWER_ON_TEST_MIN_BEMF_COUNTS;
 /* 更新下降方向过零的有效读数门限。 */
 min_bemf_counts_down = POWER_ON_TEST_MIN_BEMF_COUNTS;
 /* 将堵转超时停机保护使能清零。 */
 stuck_rotor_protection = 0;
 /* 将正弦启动使能清零。 */
 use_sin_start = 0;
 /* 将低转速限制最大占空比的使能标志清零。 */
 low_rpm_throttle_limit = 0;
/* 结束当前条件编译分支。 */
#endif
/* 仅在定义 FIXED_SPEED_MODE 时编译以下代码。 */
#ifdef FIXED_SPEED_MODE
 /* 将转速闭环使能设为 1。 */
 use_speed_control_loop = 1;
 /* 将正弦启动使能清零。 */
 use_sin_start = 0;
 /* 更新转速环目标电周期。 */
 target_e_com_time = 60000000 / FIXED_SPEED_MODE_RPM / (motor_poles/2) ;
 /* 更新交给电机控制状态机的最终油门值。 */
 input = 48;
/* 结束当前条件编译分支。 */
#endif

/* 采用上一编译条件不成立时的备选实现。 */
#else
/* 仅在定义 BRUSHED_MODE 时编译以下代码。 */
#ifdef BRUSHED_MODE
        /* 屏蔽过零 EXINT 中断并清除挂起标志；比较器本身继续工作，仍可轮询输出。 */
		maskPhaseInterrupts();
        /* 更新单扇区的估计时间计数。 */
	 	commutation_interval = 5000;
        /* 将正弦启动使能清零。 */
	 	use_sin_start = 0;
        /* 播放有刷模式启动提示音，结束时关闭桥臂并恢复正常 PWM 周期。 */
		playBrushedStartupTune();
/* 采用上一编译条件不成立时的备选实现。 */
#else
	 //  playStartupTune();
/* 结束当前条件编译分支。 */
#endif
       /* 将连续检测到零油门的次数清零。 */
	   zero_input_count = 0;
       /* 配置并启动独立看门狗，之后运行流程需定期写重载命令。 */
	   MX_IWDG_Init();
     /* 重载看门狗，表明控制流程仍在正常执行。 */
     WDT->cmd = WDT_CMD_RELOAD;

/* 仅在定义 GIMBAL_MODE 时编译以下代码。 */
#ifdef GIMBAL_MODE
    /* 将双向油门模式标志设为 1。 */
	bi_direction = 1;
    /* 将正弦启动使能设为 1。 */
	use_sin_start = 1;
/* 结束当前条件编译分支。 */
#endif

/* 仅在定义 USE_ADC_INPUT 时编译以下代码。 */
#ifdef USE_ADC_INPUT
   /* 更新输入协议初始化时设置的解锁计数参数。 */
   armed_count_threshold = 5000;
   /* 将已识别输入信号类型的标志设为 1。 */
   inputSet = 1;

/* 采用上一编译条件不成立时的备选实现。 */
#else
/* 更新主循环 ADC 更新分频计数。 */
adc_counter = 8;

/* 配置输入信号定时器及其 DMA 请求，具体定时器和引脚由 targets.h 选择。 */
UN_TIM_Init();
/* 重建输入捕获 DMA 传输，将边沿时间戳写入 dma_buffer 并启动下一次接收。 */
receiveDshotDma();
 
/* 当按目标转速映射输入的使能标志非零时进入此分支。 */
if(drive_by_rpm){
     /* 将转速闭环使能设为 1。 */
	 use_speed_control_loop = 1;
 }
/* 结束当前条件编译分支。 */
#endif

/* 结束当前条件编译分支。 */
#endif      // end fixed duty mode ifdef

/* 更新主循环 ADC 更新分频计数。 */
adc_counter = 9;


 /* 持续运行主控制循环，周期性更新输入、采样、保护和换相状态。 */
 while (1)
  {
/* 重载看门狗，表明控制流程仍在正常执行。 */
WDT->cmd = WDT_CMD_RELOAD;

      /* 递增主循环 ADC 更新分频计数。 */
	  adc_counter++;
      /* 当主循环 ADC 更新分频计数 大于 200 时进入此分支。 */
	  if(adc_counter>200){   // for testing adc and telemetry

                    /* 更新温度通道 ADC 原始值。 */
					ADC_raw_temp = ADC_raw_temp - (temperature_offset);
                    /* 使用当前代码的内部温度传感器线性公式换算；这不是外接 NTC 的阻温换算。 */
					converted_degrees =(12600 - (int32_t)ADC_raw_temp * 33000 / 4096) / -42 + 25;
                    /* 用旧值 7/8、新值 1/8 做平滑更新，得到用于保护和遥测的滤波温度。 */
					degrees_celsius =(7 * degrees_celsius + converted_degrees) >> 3;
          /* 先以 3.3 V 参考和分压系数换算为 0.01 V，再做旧值 7/8、新值 1/8 的低通滤波。 */
          battery_voltage = ((7 * battery_voltage) + ((ADC_raw_volts * 3300 / 4095 * VOLTAGE_DIVIDER)/100)) >> 3;
          /* 用旧值 7/8、新值 1/8 做平滑更新，得到低通滤波后的电流 ADC 原始值。 */
          smoothed_raw_current = ((7*smoothed_raw_current + (ADC_raw_current) )>> 3);
          /* 按采样链 mV/A 系数换算电流；41 近似 4095/100，结果约以 0.01 A 表示。 */
          actual_current = (smoothed_raw_current * 3300/41) / (MILLIVOLT_PER_AMP)  + CURRENT_OFFSET;

            /* 通过软件启动一轮普通 ADC 转换。 */
			adc_ordinary_software_trigger_enable(ADC1, TRUE);
			

          /* 当低电压停机保护使能非零时进入此分支。 */
		  if(LOW_VOLTAGE_CUTOFF){
              /* 按条件 battery_voltage < (cell_count * low_cell_volt_cutoff) 选择当前处理路径。 */
			  if(battery_voltage < (cell_count * low_cell_volt_cutoff)){
                  /* 递增低电压连续检测计数。 */
				  low_voltage_count++;
                  /* 按条件 low_voltage_count > (20000 - (stepper_sine * 900)) 选择当前处理路径。 */
				  if(low_voltage_count > (20000 - (stepper_sine * 900))){
                  /* 将交给电机控制状态机的最终油门值清零。 */
				  input = 0;
                  /* 将 A、B、C 三相全部置为浮空，关闭各相高低侧驱动。 */
				  allOff();
                  /* 屏蔽过零 EXINT 中断并清除挂起标志；比较器本身继续工作，仍可轮询输出。 */
				  maskPhaseInterrupts();
                  /* 将电机控制流程正在运行的标志清零。 */
				  running = 0;
                  /* 将连续检测到零油门的次数清零。 */
				  zero_input_count = 0;
                  /* 将电调已解锁并允许驱动的标志清零。 */
				  armed = 0;

				  }
              /* 当前条件不满足时进入备选处理。 */
			  }else{
                  /* 将低电压连续检测计数清零。 */
				  low_voltage_count = 0;
			  }
		  }
          /* 将主循环 ADC 更新分频计数清零。 */
		  adc_counter = 0;
/* 仅在定义 USE_ADC_INPUT 时编译以下代码。 */
#ifdef USE_ADC_INPUT
          /* 当可选模拟油门 ADC 原始值 小于 10 时进入此分支。 */
		  if(ADC_raw_input < 10){
              /* 递增连续检测到零油门的次数。 */
			  zero_input_count++;
          /* 当前条件不满足时进入备选处理。 */
		  }else{
              /* 将连续检测到零油门的次数清零。 */
			  zero_input_count=0;
		  }
/* 结束当前条件编译分支。 */
#endif
	  }
/* 仅在定义 USE_ADC_INPUT 时编译以下代码。 */
#ifdef USE_ADC_INPUT

/* 将自最近有效输入以来的周期计数清零。 */
signaltimeout = 0;
/* 更新滤波后的模拟油门采样值。 */
ADC_smoothed_input = (((10*ADC_smoothed_input) + ADC_raw_input)/11);
/* 更新输入协议刚解码得到的油门或命令数值。 */
newinput = ADC_smoothed_input / 2;
/* 当输入协议刚解码得到的油门或命令数值 大于 2000 时进入此分支。 */
if(newinput > 2000){
    /* 更新输入协议刚解码得到的油门或命令数值。 */
	newinput = 2000;
}
/* 结束当前条件编译分支。 */
#endif
      /* 将主循环两次运行之间进入过零中断的次数清零。 */
	  stuckcounter = 0;

          /* 按条件 bi_direction == 1 && dshot == 0 选择当前处理路径。 */
  		  if (bi_direction == 1 && dshot == 0){
              /* 当车模先制动再反转模式标志非零时进入此分支。 */
  			  if(RC_CAR_REVERSE){
                  /* 按输入数值、方向和中位死区划分油门处理区间。 */
  				  if (newinput > (1000 + (servo_dead_band<<1))) {
                      /* 当当前六步旋转方向 等于 保存的电机默认方向反转设置 时进入此分支。 */
  					  if (forward == dir_reversed) {
                          /* 将经过方向和死区处理的油门值清零。 */
  						  adjusted_input = 0;
                          /* 当电机控制流程正在运行的标志非零时进入此分支。 */
  						  if(running){
                              /* 将比例制动当前生效标志设为 1。 */
  							  prop_brake_active = 1;
                          /* 当前条件不满足时进入备选处理。 */
  						  }else{
                              /* 按循环下标遍历本组数据或重复执行指定次数的处理。 */
  							  forward = 1 - dir_reversed;
  						  }
  					  }
                      /* 当比例制动当前生效标志 等于 0 时进入此分支。 */
  					  if (prop_brake_active == 0) {
                          /* 按当前输入区间限幅并线性映射，得到经过方向和死区处理的油门值。 */
  						  adjusted_input = map(newinput, 1000 + (servo_dead_band<<1), 2000, 47, 2047);
  					  }
  				  }
                  /* 按输入数值、方向和中位死区划分油门处理区间。 */
  				  if (newinput < (1000 -(servo_dead_band<<1))) {
                      /* 按条件 forward == (1 - dir_reversed) 选择当前处理路径。 */
  					  if (forward == (1 - dir_reversed)) {
                          /* 当电机控制流程正在运行的标志非零时进入此分支。 */
  						  if(running){
                              /* 将比例制动当前生效标志设为 1。 */
  							  prop_brake_active = 1;
                          /* 当前条件不满足时进入备选处理。 */
  						  }else{
                              /* 按循环下标遍历本组数据或重复执行指定次数的处理。 */
  							  forward = dir_reversed;
  						  }
                          /* 将经过方向和死区处理的油门值清零。 */
  						  adjusted_input = 0;

  					  }
                      /* 当比例制动当前生效标志 等于 0 时进入此分支。 */
  					  if (prop_brake_active == 0) {
                          /* 按当前输入区间限幅并线性映射，得到经过方向和死区处理的油门值。 */
  						  adjusted_input = map(newinput, 0, 1000-(servo_dead_band<<1), 2047, 47);
  					  }
  				  }


                  /* 按输入数值、方向和中位死区划分油门处理区间。 */
  				  if (newinput >= (1000 - (servo_dead_band << 1)) && newinput <= (1000 + (servo_dead_band <<1))) {
                      /* 将经过方向和死区处理的油门值清零。 */
  					  adjusted_input = 0;
                      /* 将比例制动当前生效标志清零。 */
  					  prop_brake_active = 0;
  				  }
              /* 当前条件不满足时进入备选处理。 */
  			  }else{
                  /* 按输入数值、方向和中位死区划分油门处理区间。 */
  				  if (newinput > (1000 + (servo_dead_band<<1))) {
                      /* 当当前六步旋转方向 等于 保存的电机默认方向反转设置 时进入此分支。 */
  					  if (forward == dir_reversed) {
                          /* 结合换相速度和过零累计情况选择启动、过滤或正常运行路径。 */
  						  if((commutation_interval > reverse_speed_threshold )|| stepper_sine){
                              /* 按循环下标遍历本组数据或重复执行指定次数的处理。 */
  							  forward = 1 - dir_reversed;
                              /* 将过零/换相累计计数清零。 */
  							  zero_crosses = 0;
                              /* 将过零检测路径选择设为 1。 */
  							  old_routine = 1;
                              /* 屏蔽过零 EXINT 中断并清除挂起标志；比较器本身继续工作，仍可轮询输出。 */
  							  maskPhaseInterrupts();
                            /* 将有刷模式桥臂方向已配置标志清零。 */
  							brushed_direction_set = 0;
                          /* 当前条件不满足时进入备选处理。 */
  						  }else{
                              /* 更新输入协议刚解码得到的油门或命令数值。 */
  							  newinput = 1000;
  						  }
  					  }
                      /* 按当前输入区间限幅并线性映射，得到经过方向和死区处理的油门值。 */
  					  adjusted_input = map(newinput, 1000 + (servo_dead_band<<1), 2000, 47, 2047);
  				  }
                  /* 按输入数值、方向和中位死区划分油门处理区间。 */
  				  if (newinput < (1000 -(servo_dead_band<<1))) {
                      /* 按条件 forward == (1 - dir_reversed) 选择当前处理路径。 */
  					  if (forward == (1 - dir_reversed)) {
                          /* 结合换相速度和过零累计情况选择启动、过滤或正常运行路径。 */
  						  if((commutation_interval > reverse_speed_threshold) || stepper_sine){
                              /* 将过零/换相累计计数清零。 */
  							  zero_crosses = 0;
                              /* 将过零检测路径选择设为 1。 */
  							  old_routine = 1;
                              /* 按循环下标遍历本组数据或重复执行指定次数的处理。 */
  							  forward = dir_reversed;
                              /* 屏蔽过零 EXINT 中断并清除挂起标志；比较器本身继续工作，仍可轮询输出。 */
  							  maskPhaseInterrupts();
                            /* 将有刷模式桥臂方向已配置标志清零。 */
  							brushed_direction_set = 0;
                          /* 当前条件不满足时进入备选处理。 */
  						  }else{
                              /* 更新输入协议刚解码得到的油门或命令数值。 */
  							  newinput = 1000;

  						  }
  					  }
                      /* 按当前输入区间限幅并线性映射，得到经过方向和死区处理的油门值。 */
  					  adjusted_input = map(newinput, 0, 1000-(servo_dead_band<<1), 2047, 47);
  				  }

                  /* 按输入数值、方向和中位死区划分油门处理区间。 */
 				  if (newinput >= (1000 - (servo_dead_band << 1)) && newinput <= (1000 + (servo_dead_band <<1))) {
                      /* 将经过方向和死区处理的油门值清零。 */
  					  adjusted_input = 0;
                    /* 将有刷模式桥臂方向已配置标志清零。 */
  					brushed_direction_set = 0;
  				  }
  			  }
          /* 按条件 dshot && bi_direction 选择当前处理路径。 */
 		  }else if (dshot && bi_direction) {
              /* 当输入协议刚解码得到的油门或命令数值 大于 1047 时进入此分支。 */
  			  if (newinput > 1047) {

                  /* 当当前六步旋转方向 等于 保存的电机默认方向反转设置 时进入此分支。 */
  				  if (forward == dir_reversed) {
                      /* 结合换相速度和过零累计情况选择启动、过滤或正常运行路径。 */
  					  if(commutation_interval > reverse_speed_threshold || stepper_sine){
                          /* 按循环下标遍历本组数据或重复执行指定次数的处理。 */
  						  forward = 1 - dir_reversed;
                          /* 将过零/换相累计计数清零。 */
  						  zero_crosses = 0;
                          /* 将过零检测路径选择设为 1。 */
  						  old_routine = 1;
                          /* 屏蔽过零 EXINT 中断并清除挂起标志；比较器本身继续工作，仍可轮询输出。 */
  						  maskPhaseInterrupts();
                        /* 将有刷模式桥臂方向已配置标志清零。 */
  						brushed_direction_set = 0;
                      /* 当前条件不满足时进入备选处理。 */
  					  }else{
                          /* 将输入协议刚解码得到的油门或命令数值清零。 */
  						  newinput = 0;

  					  }
  				  }
                  /* 更新经过方向和死区处理的油门值。 */
  				  adjusted_input = ((newinput - 1048) * 2 + 47) - reversing_dead_band;

  			  }
              /* 按输入数值、方向和中位死区划分油门处理区间。 */
  			  if (newinput <= 1047  && newinput > 47) {
  				  //	startcount++;

                  /* 按条件 forward == (1 - dir_reversed) 选择当前处理路径。 */
  				  if (forward == (1 - dir_reversed)) {
                      /* 结合换相速度和过零累计情况选择启动、过滤或正常运行路径。 */
  					  if(commutation_interval > reverse_speed_threshold || stepper_sine){
                          /* 将过零/换相累计计数清零。 */
  						  zero_crosses = 0;
                          /* 将过零检测路径选择设为 1。 */
  						  old_routine = 1;
                          /* 按循环下标遍历本组数据或重复执行指定次数的处理。 */
  						  forward = dir_reversed;
                          /* 屏蔽过零 EXINT 中断并清除挂起标志；比较器本身继续工作，仍可轮询输出。 */
  						  maskPhaseInterrupts();
                        /* 将有刷模式桥臂方向已配置标志清零。 */
  						brushed_direction_set = 0;
                      /* 当前条件不满足时进入备选处理。 */
  					  }else{
                          /* 将输入协议刚解码得到的油门或命令数值清零。 */
  						  newinput = 0;

  					  }
  				  }
                  /* 更新经过方向和死区处理的油门值。 */
  				  adjusted_input = ((newinput - 48) * 2 + 47) - reversing_dead_band;
  			  }
              /* 当输入协议刚解码得到的油门或命令数值 小于 48 时进入此分支。 */
  			  if ( newinput < 48) {
                  /* 将经过方向和死区处理的油门值清零。 */
  				  adjusted_input = 0;
                /* 将有刷模式桥臂方向已配置标志清零。 */
  				brushed_direction_set = 0;
  			  }


          /* 当前条件不满足时进入备选处理。 */
  		  }else{
              /* 更新经过方向和死区处理的油门值。 */
  			  adjusted_input = newinput;
  		  }
/* 仅在未定义 BRUSHED_MODE 时编译以下代码。 */
#ifndef BRUSHED_MODE

         /* 结合换相速度和过零累计情况选择启动、过滤或正常运行路径。 */
	 	 if ((zero_crosses > 1000) || (adjusted_input == 0)){
            /* 将累计过零超时次数清零。 */
 	 		bemf_timeout_happened = 0;
/* 仅在定义 USE_RGB_LED 时编译以下代码。 */
#ifdef USE_RGB_LED
            /* 结合解锁、运行和输入状态判断是否允许执行驱动处理。 */
 	 		if(adjusted_input == 0 && armed){
              /* 更新GPIOB 的兼容 GPIO 引脚置位/复位寄存器。 */
			  GPIOB->BSRR = LL_GPIO_PIN_8; // off red
              /* 更新GPIOB 的兼容 GPIO 引脚清零寄存器。 */
			  GPIOB->BRR = LL_GPIO_PIN_5;  // on green
              /* 更新GPIOB 的兼容 GPIO 引脚置位/复位寄存器。 */
			  GPIOB->BSRR = LL_GPIO_PIN_3;  //off blue
 	 		}
/* 结束当前条件编译分支。 */
#endif
 	 	 }
         /* 结合换相速度和过零累计情况选择启动、过滤或正常运行路径。 */
	 	 if(zero_crosses > 100 && adjusted_input < 200){
            /* 将累计过零超时次数清零。 */
	 		bemf_timeout_happened = 0;
	 	 }
         /* 按条件 use_sin_start && adjusted_input < 160 选择当前处理路径。 */
	 	 if(use_sin_start && adjusted_input < 160){
            /* 将累计过零超时次数清零。 */
	 		bemf_timeout_happened = 0;
	 	 }

         /* 当保留的爬车模式参数非零时进入此分支。 */
 	 	 if(crawler_mode){
            /* 当经过方向和死区处理的油门值 小于 400 时进入此分支。 */
 	 		if (adjusted_input < 400){
                /* 将累计过零超时次数清零。 */
 	 			bemf_timeout_happened = 0;
 	 		}
         /* 当前条件不满足时进入备选处理。 */
 	 	 }else{
            /* 当经过方向和死区处理的油门值 小于 150 时进入此分支。 */
 	 		if (adjusted_input < 150){              // startup duty cycle should be low enough to not burn motor
                /* 更新允许过零超时的次数门限。 */
 	 			bemf_timeout = 100;
             /* 当前条件不满足时进入备选处理。 */
 	 	 	 }else{
                /* 更新允许过零超时的次数门限。 */
 	 	 		bemf_timeout = 10;
 	 	 	 }
 	 	 }
      /* 按条件 bemf_timeout_happened > bemf_timeout * ( 1 + (crawler_mode*100))&& stuck_rotor_protection 选择当前处理路径。 */
	  if(bemf_timeout_happened > bemf_timeout * ( 1 + (crawler_mode*100))&& stuck_rotor_protection){
            /* 处理堵转停机：关闭桥臂、屏蔽过零并清零油门；试转模式同时取消待执行换相和 PWM。 */
	 		stopStuckMotor();
         /* 当前条件不满足时进入备选处理。 */
	 	 }else{
/* 仅在定义 FIXED_DUTY_MODE 时编译以下代码。 */
#ifdef FIXED_DUTY_MODE
/* 仅在定义 POWER_ON_TEST_MODE 时编译以下代码。 */
#ifdef POWER_ON_TEST_MODE
            /* 试转故障已经锁存，进入停机或禁止重启路径。 */
			if(power_on_test_fault){
                /* 将交给电机控制状态机的最终油门值清零。 */
				input = 0;
            /* 当前条件不满足时进入备选处理。 */
			}else{
                /* 更新交给电机控制状态机的最终油门值。 */
				input = FIXED_DUTY_MODE_POWER * 20;
			}
/* 采用上一编译条件不成立时的备选实现。 */
#else
            /* 更新交给电机控制状态机的最终油门值。 */
  			input = FIXED_DUTY_MODE_POWER * 20;
/* 结束当前条件编译分支。 */
#endif
/* 采用上一编译条件不成立时的备选实现。 */
#else
            /* 当正弦启动使能非零时进入此分支。 */
	  	  	if(use_sin_start){
                /* 当经过方向和死区处理的油门值 小于 30 时进入此分支。 */
  				if(adjusted_input < 30){           // dead band ?
                    /* 将交给电机控制状态机的最终油门值清零。 */
  					input= 0;
  					}

                    /* 按条件 adjusted_input > 30 && adjusted_input < (sine_mode_changeover_thottle_level * 20) 选择当前处理路径。 */
  					if(adjusted_input > 30 && adjusted_input < (sine_mode_changeover_thottle_level * 20)){
                    /* 按当前输入区间限幅并线性映射，得到交给电机控制状态机的最终油门值。 */
  					input= map(adjusted_input, 30 , (sine_mode_changeover_thottle_level * 20) , 47 ,160);
  					}
                    /* 按条件 adjusted_input >= (sine_mode_changeover_thottle_level * 20) 选择当前处理路径。 */
  					if(adjusted_input >= (sine_mode_changeover_thottle_level * 20)){
                    /* 按当前输入区间限幅并线性映射，得到交给电机控制状态机的最终油门值。 */
  					input = map(adjusted_input , (sine_mode_changeover_thottle_level * 20) ,2000 , 160, 2000);
  					}
                /* 当前条件不满足时进入备选处理。 */
  				}else{
                    /* 当转速闭环使能非零时进入此分支。 */
  					if(use_speed_control_loop){
                      /* 当按目标转速映射输入的使能标志非零时进入此分支。 */
  					  if (drive_by_rpm){
                        /* 按当前输入区间限幅并线性映射，得到转速环目标电周期。 */
 						target_e_com_time = 60000000 / map(adjusted_input , 47 ,2047 , MINIMUM_RPM_SPEED_CONTROL, MAXIMUM_RPM_SPEED_CONTROL) / (motor_poles/2);
                        /* 当经过方向和死区处理的油门值 小于 47 时进入此分支。 */
  		  				if(adjusted_input < 47){           // dead band ?
                            /* 将交给电机控制状态机的最终油门值清零。 */
  		  					input= 0;
                            /* 将PID 当前误差清零。 */
  		  					speedPid.error = 0;
                            /* 将转速 PID 生成的油门替代值清零。 */
  		  				    input_override = 0;
                        /* 当前条件不满足时进入备选处理。 */
  		  				}else{
                            /* 更新交给电机控制状态机的最终油门值。 */
  	  						input = (uint16_t)input_override;  // speed control pid override
                            /* 当转速 PID 生成的油门替代值 大于 2047 时进入此分支。 */
  	  						if(input_override > 2047){
                                /* 更新交给电机控制状态机的最终油门值。 */
  	  							input = 2047;
  	  						}
                            /* 当转速 PID 生成的油门替代值 小于 48 时进入此分支。 */
  	  						if(input_override < 48){
                                /* 更新交给电机控制状态机的最终油门值。 */
  	  							input = 48;
  	  						}
  		  				}
                        /* 当前条件不满足时进入备选处理。 */
					    }else{

                        /* 更新交给电机控制状态机的最终油门值。 */
  						input = (uint16_t)input_override;  // speed control pid override
                        /* 当转速 PID 生成的油门替代值 大于 1999 时进入此分支。 */
  						if(input_override > 1999){
                            /* 更新交给电机控制状态机的最终油门值。 */
  							input = 1999;
  						}
                        /* 当转速 PID 生成的油门替代值 小于 48 时进入此分支。 */
  						if(input_override < 48){
                            /* 更新交给电机控制状态机的最终油门值。 */
  							input = 48;
  						}
					    }
                    /* 当前条件不满足时进入备选处理。 */
  					}else{

                        /* 更新交给电机控制状态机的最终油门值。 */
  						input = adjusted_input;

  					}
  				}
/* 结束当前条件编译分支。 */
#endif
	 	  }
          /* 当当前处于正弦步进模式的标志 等于 0 时进入此分支。 */
		  if ( stepper_sine == 0){

  /* 将微秒电周期换算为电转速/100；先做整数除法会截断，结果用于遥测。 */
  e_rpm = running * (100000/ e_com_time) * 6;       // in tens of rpm
  /* 更新电转速的千转每分钟尺度值。 */
  k_erpm =  e_rpm / 10; // ecom time is time for one electrical revolution in microseconds

  /* 当低转速限制最大占空比的使能标志非零时进入此分支。 */
  if(low_rpm_throttle_limit){     // some hardware doesn't need this, its on by default to keep hardware / motors protected but can slow down the response in the very low end a little.

  /* 按当前输入区间限幅并线性映射，得到温度或低转速保护给出的占空比上限。 */
  duty_cycle_maximum = map(k_erpm, low_rpm_level, high_rpm_level, throttle_max_at_low_rpm, throttle_max_at_high_rpm);   // for more performance lower the high_rpm_level, set to a consvervative number in source.
   }

   /* 当用于保护和遥测的滤波温度 大于 温度降额门限 时进入此分支。 */
   if(degrees_celsius > TEMPERATURE_LIMIT){
       /* 按当前输入区间限幅并线性映射，得到温度或低转速保护给出的占空比上限。 */
	   duty_cycle_maximum = map(degrees_celsius, TEMPERATURE_LIMIT, TEMPERATURE_LIMIT+20, throttle_max_at_high_rpm/2, 1);
   }



/* 条件编译：defined(FAST_INTERRUPT)；条件满足时采用以下实现。 */
#if defined(FAST_INTERRUPT)
    /* 结合换相速度和过零累计情况选择启动、过滤或正常运行路径。 */
	if (zero_crosses < 100 || commutation_interval > 500) {
        /* 更新比较器中断软件过滤的读数次数参数。 */
		filter_level = 12;
    /* 当前条件不满足时进入备选处理。 */
	} else {
        /* 按当前输入区间限幅并线性映射，得到比较器中断软件过滤的读数次数参数。 */
		filter_level = map(average_interval, 100 , 500, 4 , 12);
	}
/* 采用上一编译条件不成立时的备选实现。 */
#else
    /* 结合换相速度和过零累计情况选择启动、过滤或正常运行路径。 */
	if (zero_crosses < 100 || commutation_interval > 500) {
        /* 更新比较器中断软件过滤的读数次数参数。 */
		filter_level = 8;
    /* 当前条件不满足时进入备选处理。 */
	} else {
        /* 按当前输入区间限幅并线性映射，得到比较器中断软件过滤的读数次数参数。 */
		filter_level = map(average_interval, 100 , 500, 2 , 8);
	}
/* 结束当前条件编译分支。 */
#endif
	
    /* 当单扇区的估计时间计数 小于 100 时进入此分支。 */
	if (commutation_interval < 100){
        /* 更新比较器中断软件过滤的读数次数参数。 */
		filter_level = 2;
	}

/* 当电机 KV 参数 小于 900 时进入此分支。 */
if(motor_kv < 900){

    /* 更新比较器中断软件过滤的读数次数参数。 */
	filter_level = filter_level * 2;
}

/**************** old routine*********************/
/* 结合解锁、运行和输入状态判断是否允许执行驱动处理。 */
if (old_routine && running){
    /* 屏蔽过零 EXINT 中断并清除挂起标志；比较器本身继续工作，仍可轮询输出。 */
	maskPhaseInterrupts();
             /* 轮询浮空相比较器电平，按预期方向累计有效读数；错误读数达到门限后清除有效计数。 */
	 		 getBemfState();
          /* 当轮询路径本次过零已处理的标志为零时进入此分支。 */
	 	  if (!zcfound){
              /* 当前扇区等待浮空相电压上升过零。 */
	 		  if (rising){
             /* 当轮询中满足预期电平的读数累计次数 大于 上升方向过零的有效读数门限 时进入此分支。 */
	 		 if (bemfcounter > min_bemf_counts_up){
                 /* 将轮询路径本次过零已处理的标志设为 1。 */
	 			 zcfound = 1;
             /* 轮询路径的过零处理；区分真实过零和强制换相，等待半扇区减提前量后执行下一步。 */
	 		 zcfoundroutine(0);
	 		}
              /* 当前条件不满足时进入备选处理。 */
	 		  }else{
                  /* 当轮询中满足预期电平的读数累计次数 大于 下降方向过零的有效读数门限 时进入此分支。 */
	 			  if (bemfcounter > min_bemf_counts_down){
                         /* 将轮询路径本次过零已处理的标志设为 1。 */
 			  			 zcfound = 1;
                 /* 轮询路径的过零处理；区分真实过零和强制换相，等待半扇区减提前量后执行下一步。 */
	 			 zcfoundroutine(0);
	 			  		}
	 		  }
	 	  }
}
/* 条件编译：defined(COMM_TIMEOUT_PROCESS)；条件满足时采用以下实现。 */
#if defined(COMM_TIMEOUT_PROCESS)
            /* 执行目标配置中定义的过零超时门限更新。 */
			COMM_TIMEOUT_PROCESS;			
/* 结束当前条件编译分支。 */
#endif

          /* 检查当前间隔计数是否落在本路径允许的过零或换相时间范围。 */
	 	  if (INTERVAL_TIMER->cval > comm_timeout  && running == 1){//zc timeout
                /* 处理过零等待超时；试转模式限制强制换相次数并锁存故障，正常回退路径转入轮询。 */
				IntervalTimerOverflowProcess(0);
	 	  }
          /* 当前条件不满足时进入备选处理。 */
	 	  }else{            // stepper sine

/* 仅在定义 GIMBAL_MODE 时编译以下代码。 */
#ifdef GIMBAL_MODE
                    /* 更新正弦模式每次角度推进的延时。 */
	 				step_delay = 300;
                    /* 屏蔽过零 EXINT 中断并清除挂起标志；比较器本身继续工作，仍可轮询输出。 */
	 				maskPhaseInterrupts();
                    /* 把三相切换到 PWM 模式，供正弦启动等三通道调制逻辑使用。 */
	 				allpwm();
                    /* 当输入协议刚解码得到的油门或命令数值 大于 1000 时进入此分支。 */
	 				if(newinput>1000){
                        /* 按当前输入区间限幅并线性映射，得到云台模式目标角度索引。 */
	 					desired_angle = map(newinput, 1000, 2000, 180, 360);
                    /* 当前条件不满足时进入备选处理。 */
	 				}else{
                        /* 按当前输入区间限幅并线性映射，得到云台模式目标角度索引。 */
	 					desired_angle = map(newinput, 0, 1000, 0, 180);
	 				}
                    /* 当云台模式当前角度索引 大于 云台模式目标角度索引 时进入此分支。 */
	 				if(current_angle > desired_angle){
                        /* 按循环下标遍历本组数据或重复执行指定次数的处理。 */
	 					forward = 1;
                        /* 按旋转方向推进三相正弦表索引，越界后回绕，并把查表结果缩放到 PWM 比较值。 */
	 					advanceincrement();
                        /* 使用 UTILITY_TIMER 忙等延时；会清零共享计数器，时间基准取决于当前定时器分频。 */
	 					delayMicros(step_delay);
                        /* 递减云台模式当前角度索引。 */
	 					current_angle--;
	 				}
                    /* 当云台模式当前角度索引 小于 云台模式目标角度索引 时进入此分支。 */
	 				if(current_angle < desired_angle){
                        /* 按循环下标遍历本组数据或重复执行指定次数的处理。 */
	 					forward = 0;
                        /* 按旋转方向推进三相正弦表索引，越界后回绕，并把查表结果缩放到 PWM 比较值。 */
	 					advanceincrement();
                        /* 使用 UTILITY_TIMER 忙等延时；会清零共享计数器，时间基准取决于当前定时器分频。 */
	 					delayMicros(step_delay);
                        /* 递增云台模式当前角度索引。 */
	 					current_angle++;
	 				}
/* 采用上一编译条件不成立时的备选实现。 */
#else



/* 结合解锁、运行和输入状态判断是否允许执行驱动处理。 */
if(input > 48 && armed){

              /* 按条件 input > 48 && input < 137 选择当前处理路径。 */
	 		  if (input > 48 && input < 137){// sine wave stepper
         /* 将电机控制流程正在运行的标志设为 1。 */
         running = 1;
                 /* 屏蔽过零 EXINT 中断并清除挂起标志；比较器本身继续工作，仍可轮询输出。 */
	 			 maskPhaseInterrupts();
                 /* 把三相切换到 PWM 模式，供正弦启动等三通道调制逻辑使用。 */
	 			 allpwm();
             /* 按旋转方向推进三相正弦表索引，越界后回绕，并把查表结果缩放到 PWM 比较值。 */
	 		 advanceincrement();
             /* 更新正弦模式每次角度推进的延时。 */
             step_delay = map (input, 48, 120, 7000/motor_poles, 1000/motor_poles);
             /* 使用 UTILITY_TIMER 忙等延时；会清零共享计数器，时间基准取决于当前定时器分频。 */
	 		 delayMicros(step_delay);
             /* 更新电转速除以 100 后的遥测数值。 */
			 e_rpm =   600/ step_delay ;         // in hundreds so 33 e_rpm is 3300 actual erpm
                /* 当正弦模式每次角度推进的延时 小于 181 时进入此分支。 */
				if(step_delay < 181){
                    /* 更新电气旋转一周的估计时间。 */
					e_com_time = step_delay * 360;
                /* 当前条件不满足时进入备选处理。 */
				}else{ 
                    /* 更新电气旋转一周的估计时间。 */
					e_com_time = 65535;
				}
              /* 当前条件不满足时进入备选处理。 */
	 		  }else{
                 /* 按旋转方向推进三相正弦表索引，越界后回绕，并把查表结果缩放到 PWM 比较值。 */
	 			 advanceincrement();
                  /* 当交给电机控制状态机的最终油门值 大于 200 时进入此分支。 */
	 			  if(input > 200){
                     /* 将A 相正弦查表角度索引清零。 */
	 				 phase_A_position = 0;
                     /* 更新正弦模式每次角度推进的延时。 */
	 				 step_delay = 80;
	 			  }

                 /* 使用 UTILITY_TIMER 忙等延时；会清零共享计数器，时间基准取决于当前定时器分频。 */
	 			 delayMicros(step_delay);
                  /* 当A 相正弦查表角度索引 等于 0 时进入此分支。 */
	 			  if (phase_A_position == 0){
                  /* 将当前处于正弦步进模式的标志清零。 */
	 			  stepper_sine = 0;
                  /* 将电机控制流程正在运行的标志设为 1。 */
	 			  running = 1;
                  /* 将过零检测路径选择设为 1。 */
				  old_routine = 1;
                  /* 更新单扇区的估计时间计数。 */
		 		  commutation_interval = 9000;
                  /* 更新由电周期折算的平均扇区计数。 */
		 		  average_interval = 9000;
                  /* 更新上一轮用于失步比较的平均扇区计数。 */
				  last_average_interval = average_interval;
                        /* 更新INTERVAL_TIMER 的当前计数值。 */
						INTERVAL_TIMER->cval = 9000;
                  /* 更新过零/换相累计计数。 */
				  zero_crosses = 10;
                  /* 将比例制动当前生效标志清零。 */
				  prop_brake_active = 0;
                  /* 更新当前六步换相步骤。 */
	 			  step = changeover_step;                    // rising bemf on a same as position 0.
		 		// comStep(step);// rising bemf on a same as position 0.
                /* 当低速防停转补偿使能非零时进入此分支。 */
				if(stall_protection){
                /* 更新正常驱动最小 PWM 比较值。 */
				minimum_duty_cycle = stall_protect_minimum_duty;
				}
                /* 推进或回退六步相序，更新电周期估计和过零方向，执行桥臂换相并切换比较器输入。 */
	 			commutate();
                /* 置位TMR1 的软件事件寄存器。 */
				TMR1->swevt |= TMR_OVERFLOW_SWTRIG;
	 			  }
	 		  }

/* 当前条件不满足时进入备选处理。 */
}else{
    /* 当零油门时制动的设置非零时进入此分支。 */
	if(brake_on_stop){
/* 仅在未定义 PWM_ENABLE_BRIDGE 时编译以下代码。 */
#ifndef PWM_ENABLE_BRIDGE
    /* 更新以基准 PWM 周期表示的目标比较值。 */
	duty_cycle = (TIMER1_MAX_ARR-19) + drag_brake_strength*2;
    /* 更新按当前 PWM 周期缩放后的实际比较值。 */
	adjusted_duty_cycle = TIMER1_MAX_ARR - ((duty_cycle * tim1_arr)/TIMER1_MAX_ARR)+1;

/* 更新TMR1 的通道 1 捕获/比较寄存器。 */
TMR1->c1dt = adjusted_duty_cycle;
/* 更新TMR1 的通道 2 捕获/比较寄存器。 */
TMR1->c2dt = adjusted_duty_cycle;
/* 更新TMR1 的通道 3 捕获/比较寄存器。 */
TMR1->c3dt = adjusted_duty_cycle;
    /* 关闭高侧驱动并把低侧切到 PWM 复用模式，用占空比控制制动力。 */
	proportionalBrake();
    /* 将比例制动当前生效标志设为 1。 */
	prop_brake_active = 1;
/* 采用上一编译条件不成立时的备选实现。 */
#else
		// todo add braking for PWM /enable style bridges.
/* 结束当前条件编译分支。 */
#endif
    /* 当前条件不满足时进入备选处理。 */
	}else{
        /* 将TMR1 的通道 1 捕获/比较寄存器清零。 */
		TMR1->c1dt = 0;
        /* 将TMR1 的通道 2 捕获/比较寄存器清零。 */
		TMR1->c2dt = 0;
        /* 将TMR1 的通道 3 捕获/比较寄存器清零。 */
		TMR1->c3dt = 0;
        /* 将 A、B、C 三相全部置为浮空，关闭各相高低侧驱动。 */
		allOff();
	}
    /* 将电机控制流程正在运行的标志清零。 */
	running = 0;
}

/* 结束当前条件编译分支。 */
#endif      // gimbal mode
	 }  // stepper/sine mode end
/* 结束当前条件编译分支。 */
#endif    // end of brushless mode

/* 仅在定义 BRUSHED_MODE 时编译以下代码。 */
#ifdef BRUSHED_MODE
          /* 更新TMR14 的通道 1 捕获/比较寄存器。 */
          TMR14->c1dt = adjusted_input;
                /* 按条件 brushed_direction_set == 0 && adjusted_input > 48 选择当前处理路径。 */
	  			if(brushed_direction_set == 0 && adjusted_input > 48){
                    /* 当当前六步旋转方向非零时进入此分支。 */
	  				if(forward){
                        /* 将 A、B、C 三相全部置为浮空，关闭各相高低侧驱动。 */
	  					allOff();
                        /* 使用 UTILITY_TIMER 忙等延时；会清零共享计数器，时间基准取决于当前定时器分频。 */
	  					delayMicros(10);
                        /* 设置有刷电机正向桥臂组合：A/C 相 PWM，B 相低侧导通。 */
	  					twoChannelForward();
                    /* 当前条件不满足时进入备选处理。 */
	  				}else{
                        /* 将 A、B、C 三相全部置为浮空，关闭各相高低侧驱动。 */
	  					allOff();
                        /* 使用 UTILITY_TIMER 忙等延时；会清零共享计数器，时间基准取决于当前定时器分频。 */
	  					delayMicros(10);
                        /* 设置有刷电机反向桥臂组合：A/C 相低侧导通，B 相 PWM。 */
	  					twoChannelReverse();
	  				}
                    /* 将有刷模式桥臂方向已配置标志设为 1。 */
	  				brushed_direction_set = 1;
	  			}
                /* 当经过方向和死区处理的油门值 大于 1900 时进入此分支。 */
	  			if(adjusted_input > 1900){
                    /* 更新经过方向和死区处理的油门值。 */
	  				adjusted_input = 1900;
	  			}
                /* 按当前输入区间限幅并线性映射，得到交给电机控制状态机的最终油门值。 */
	  			input = map(adjusted_input, 48, 2047, 0, TIMER1_MAX_ARR);

                /* 结合解锁、运行和输入状态判断是否允许执行驱动处理。 */
	  			if(input > 0 && armed){
                    /* 更新TMR1 的通道 1 捕获/比较寄存器。 */
	  				TMR1->c1dt = input;												// set duty cycle to 50 out of 768 to start.
                    /* 更新TMR1 的通道 2 捕获/比较寄存器。 */
	  				TMR1->c2dt = input;
                    /* 更新TMR1 的通道 3 捕获/比较寄存器。 */
	  				TMR1->c3dt = input;
                /* 当前条件不满足时进入备选处理。 */
	  			}else{
                    /* 将TMR1 的通道 1 捕获/比较寄存器清零。 */
	  				TMR1->c1dt = 0;												// set duty cycle to 50 out of 768 to start.
                    /* 将TMR1 的通道 2 捕获/比较寄存器清零。 */
	  				TMR1->c2dt = 0;
                    /* 将TMR1 的通道 3 捕获/比较寄存器清零。 */
	  				TMR1->c3dt = 0;
	  			}
/* 结束当前条件编译分支。 */
#endif
  		}
}
