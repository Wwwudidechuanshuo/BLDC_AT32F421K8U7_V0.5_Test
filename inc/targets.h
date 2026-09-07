/*
 * 硬件目标配置：选择 MCU、输入捕获资源、三相驱动引脚、比较器通道和默认参数。
 * 中文说明按当前实现编写；保留原代码、历史注释和编译条件。
 * 当前 AT32DEV_F421_540 的实际比较器相序是 A=PA0、B=PA4、C=PA5。
 * 目标名称不能替代寄存器值核对；诊断 ADC 通道与比较器通道分开配置。
 */


/* 仅在未定义 USE_MAKE 时编译以下代码。 */
#ifndef USE_MAKE

//#define AT32dev045_PA2
//#define AT32dev504_PA2
//#define AT32dev450_PA2
//#define AT32dev054_PA2
//#define AT32dev405_PA2
//#define AT32dev540_PA2

//#define AT32DEV_F421
//#define TEKKO32_F421
//#define FOXEER_F421
//#define AIKON_SINGLE_F421
//#define AIKON_55A_F421
//#define WRAITH32_F421

/* 结束当前条件编译分支。 */
#endif

//#define USE_DEBUG
//#define USE_INNER_STEP

/* 固件主版本号。 */
#define VERSION_MAJOR 3
/* 固件次版本号。 */
#define VERSION_MINOR 3

/* 仅在定义 AT32DEV_F421 时编译以下代码。 */
#ifdef AT32DEV_F421
/* 写入配置区供识别的固件名称。 */
#define FIRMWARE_NAME           "AT32PB4     "
/* 目标固件输出名称标识。 */
#define FILE_NAME                "AT32DEV_F421"
/* 硬件死区编码及相关补偿的默认值；不是直接以微秒表示。 */
#define DEAD_TIME               60
/* 选择 HARDWARE_GROUP_AT_A 硬件分组，启用下方对应的引脚和通道定义。 */
#define HARDWARE_GROUP_AT_A
/* 启用独立串口遥测输出。 */
#define USE_SERIAL_TELEMETRY
/* 按 PA6 为电压、PA3 为电流解释正常 ADC 扫描结果。 */
#define PA6_VOLTAGE
/* 目标专用的过零超时门限更新语句。 */
#define COMM_TIMEOUT_PROCESS comm_timeout=20000;//((zero_crosses<=3|| zero_crosses>=15 )? 30000 : comm_timeout*80/100 );
/* 结束当前条件编译分支。 */
#endif

/* 仅在定义 NEUTRONRC_F421 时编译以下代码。 */
#ifdef NEUTRONRC_F421
/* 写入配置区供识别的固件名称。 */
#define FIRMWARE_NAME           "NeutronRC Mi"
/* 目标固件输出名称标识。 */
#define FILE_NAME                "NeutronRC_F421"
/* 硬件死区编码及相关补偿的默认值；不是直接以微秒表示。 */
#define DEAD_TIME               60
/* 选择 HARDWARE_GROUP_AT_A 硬件分组，启用下方对应的引脚和通道定义。 */
#define HARDWARE_GROUP_AT_A
/* 启用独立串口遥测输出。 */
#define USE_SERIAL_TELEMETRY
/* 结束当前条件编译分支。 */
#endif

/* 仅在定义 NEUTRONRC_IH_F421 时编译以下代码。 */
#ifdef NEUTRONRC_IH_F421
/* 写入配置区供识别的固件名称。 */
#define FIRMWARE_NAME           "NeutronRC IH"
/* 目标固件输出名称标识。 */
#define FILE_NAME                "NTRC_IH_F421"
/* 硬件死区编码及相关补偿的默认值；不是直接以微秒表示。 */
#define DEAD_TIME               60
/* 选择 HARDWARE_GROUP_AT_A 硬件分组，启用下方对应的引脚和通道定义。 */
#define HARDWARE_GROUP_AT_A
/* 启用独立串口遥测输出。 */
#define USE_SERIAL_TELEMETRY
/* 该目标高侧驱动使用反向有效电平。 */
#define USE_INVERTED_HIGH
/* 结束当前条件编译分支。 */
#endif

/* 仅在定义 AT32DEV_F421_540 时编译以下代码。 */
#ifdef AT32DEV_F421_540
/* 写入配置区供识别的固件名称。 */
#define FIRMWARE_NAME           "AT32PB4 540 "
/* 目标固件输出名称标识。 */
#define FILE_NAME                "AT32DEV_F421_5"
/* 硬件死区编码及相关补偿的默认值；不是直接以微秒表示。 */
#define DEAD_TIME               60
/* 选择 HARDWARE_GROUP_AT_A 硬件分组，启用下方对应的引脚和通道定义。 */
#define HARDWARE_GROUP_AT_A
/* 启用独立串口遥测输出。 */
#define USE_SERIAL_TELEMETRY
/* 结束当前条件编译分支。 */
#endif

/* 仅在定义 TEKKO32_F421 时编译以下代码。 */
#ifdef TEKKO32_F421
/* 写入配置区供识别的固件名称。 */
#define FIRMWARE_NAME           "Tekko32 F4  "
/* 目标固件输出名称标识。 */
#define FILE_NAME                "TEKKO32_F421"
/* 硬件死区编码及相关补偿的默认值；不是直接以微秒表示。 */
#define DEAD_TIME               80
/* 选择 HARDWARE_GROUP_AT_A 硬件分组，启用下方对应的引脚和通道定义。 */
#define HARDWARE_GROUP_AT_A
/* 启用独立串口遥测输出。 */
#define USE_SERIAL_TELEMETRY
/* 结束当前条件编译分支。 */
#endif

/* 仅在定义 HAKRC_G_F421 时编译以下代码。 */
#ifdef HAKRC_G_F421                                     
/* 写入配置区供识别的固件名称。 */
#define FIRMWARE_NAME           "HAKRC F4 G  "
/* 目标固件输出名称标识。 */
#define FILE_NAME                "HAKRC_G_F421"
/* 硬件死区编码及相关补偿的默认值；不是直接以微秒表示。 */
#define DEAD_TIME               80
/* 选择 HARDWARE_GROUP_AT_B 硬件分组，启用下方对应的引脚和通道定义。 */
#define HARDWARE_GROUP_AT_B
/* 选择 HARDWARE_GROUP_AT_B450 硬件分组，启用下方对应的引脚和通道定义。 */
#define HARDWARE_GROUP_AT_B450
/* 启用独立串口遥测输出。 */
#define USE_SERIAL_TELEMETRY
/* 结束当前条件编译分支。 */
#endif

/* 仅在定义 HAKRC_K_F421 时编译以下代码。 */
#ifdef HAKRC_K_F421                                     
/* 写入配置区供识别的固件名称。 */
#define FIRMWARE_NAME           "HAKRC F4 K  "
/* 目标固件输出名称标识。 */
#define FILE_NAME               "HAKRC_K_F421"
/* 硬件死区编码及相关补偿的默认值；不是直接以微秒表示。 */
#define DEAD_TIME               80
/* 选择 HARDWARE_GROUP_AT_B 硬件分组，启用下方对应的引脚和通道定义。 */
#define HARDWARE_GROUP_AT_B
/* 选择 HARDWARE_GROUP_AT_B504 硬件分组，启用下方对应的引脚和通道定义。 */
#define HARDWARE_GROUP_AT_B504
/* 启用独立串口遥测输出。 */
#define USE_SERIAL_TELEMETRY
/* 结束当前条件编译分支。 */
#endif

/* 仅在定义 FOXEER_F421 时编译以下代码。 */
#ifdef  FOXEER_F421
/* 选择 HARDWARE_GROUP_AT_C 硬件分组，启用下方对应的引脚和通道定义。 */
#define HARDWARE_GROUP_AT_C
/* 写入配置区供识别的固件名称。 */
#define FIRMWARE_NAME           "FOXEER F4   "
/* 目标固件输出名称标识。 */
#define FILE_NAME                "FOXEER_F421"
/* 硬件死区编码及相关补偿的默认值；不是直接以微秒表示。 */
#define DEAD_TIME               60
/* 选择 HARDWARE_GROUP_AT_C540 硬件分组，启用下方对应的引脚和通道定义。 */
#define HARDWARE_GROUP_AT_C540
/* 启用独立串口遥测输出。 */
#define USE_SERIAL_TELEMETRY
/* 结束当前条件编译分支。 */
#endif


/* 仅在定义 WRAITH32_F421 时编译以下代码。 */
#ifdef  WRAITH32_F421
/* 写入配置区供识别的固件名称。 */
#define FIRMWARE_NAME           "WRAITH32_F4  "
/* 目标固件输出名称标识。 */
#define FILE_NAME                "WRAITH32_F421"
/* 硬件死区编码及相关补偿的默认值；不是直接以微秒表示。 */
#define DEAD_TIME               60
/* 选择 HARDWARE_GROUP_AT_C045 硬件分组，启用下方对应的引脚和通道定义。 */
#define HARDWARE_GROUP_AT_C045
/* 选择 HARDWARE_GROUP_AT_C 硬件分组，启用下方对应的引脚和通道定义。 */
#define HARDWARE_GROUP_AT_C
/* 启用独立串口遥测输出。 */
#define USE_SERIAL_TELEMETRY
/* 结束当前条件编译分支。 */
#endif

/* 仅在定义 AIKON_55A_F421 时编译以下代码。 */
#ifdef  AIKON_55A_F421
/* 写入配置区供识别的固件名称。 */
#define FIRMWARE_NAME           "AIKON F4 55A"
/* 目标固件输出名称标识。 */
#define FILE_NAME                "AIKON_55A_F421"
/* 硬件死区编码及相关补偿的默认值；不是直接以微秒表示。 */
#define DEAD_TIME               60
/* 选择 HARDWARE_GROUP_AT_C 硬件分组，启用下方对应的引脚和通道定义。 */
#define HARDWARE_GROUP_AT_C
/* 选择 HARDWARE_GROUP_AT_C045 硬件分组，启用下方对应的引脚和通道定义。 */
#define HARDWARE_GROUP_AT_C045
/* 启用独立串口遥测输出。 */
#define USE_SERIAL_TELEMETRY
/* 结束当前条件编译分支。 */
#endif

/* 仅在定义 AIKON_S_F421 时编译以下代码。 */
#ifdef  AIKON_S_F421
/* 写入配置区供识别的固件名称。 */
#define FIRMWARE_NAME           "AIKON S F4  "
/* 目标固件输出名称标识。 */
#define FILE_NAME                "AIKON_S_F421"
/* 硬件死区编码及相关补偿的默认值；不是直接以微秒表示。 */
#define DEAD_TIME               60
/* 选择 HARDWARE_GROUP_AT_A 硬件分组，启用下方对应的引脚和通道定义。 */
#define HARDWARE_GROUP_AT_A
/* 启用独立串口遥测输出。 */
#define USE_SERIAL_TELEMETRY
/* 结束当前条件编译分支。 */
#endif

/* 仅在定义 AT32DEV_F415 时编译以下代码。 */
#ifdef AT32DEV_F415
/* 写入配置区供识别的固件名称。 */
#define FIRMWARE_NAME           "AT32F415    "
/* 目标固件输出名称标识。 */
#define FILE_NAME               "AT32DEV_F415"
/* 硬件死区编码及相关补偿的默认值；不是直接以微秒表示。 */
#define DEAD_TIME               100
/* 选择 HARDWARE_GROUP_AT_D 硬件分组，启用下方对应的引脚和通道定义。 */
#define HARDWARE_GROUP_AT_D
/* 启用独立串口遥测输出。 */
#define USE_SERIAL_TELEMETRY
/* 结束当前条件编译分支。 */
#endif

/* 仅在定义 TEKKO32_F415 时编译以下代码。 */
#ifdef TEKKO32_F415
/* 写入配置区供识别的固件名称。 */
#define FIRMWARE_NAME           "Tekko32 F4  "
/* 目标固件输出名称标识。 */
#define FILE_NAME               "TEKKO32_F415"
/* 硬件死区编码及相关补偿的默认值；不是直接以微秒表示。 */
#define DEAD_TIME               100
/* 选择 HARDWARE_GROUP_AT_D 硬件分组，启用下方对应的引脚和通道定义。 */
#define HARDWARE_GROUP_AT_D
/* 启用独立串口遥测输出。 */
#define USE_SERIAL_TELEMETRY
/* 结束当前条件编译分支。 */
#endif
/********************************** defaults if not set ***************************/

/* 仅在未定义 TARGET_VOLTAGE_DIVIDER 时编译以下代码。 */
#ifndef 	TARGET_VOLTAGE_DIVIDER
/* 电压分压倍数乘 10 的默认整数系数。 */
#define 	TARGET_VOLTAGE_DIVIDER  	110
/* 结束当前条件编译分支。 */
#endif

/* 仅在未定义 SINE_DIVIDER 时编译以下代码。 */
#ifndef 	SINE_DIVIDER
/* 正弦表输出幅度的除数。 */
#define 	SINE_DIVIDER 				2
/* 结束当前条件编译分支。 */
#endif

/* 仅在未定义 MILLIVOLT_PER_AMP 时编译以下代码。 */
#ifndef  	MILLIVOLT_PER_AMP
/* 电流检测链每安培对应的输出毫伏数。 */
#define   MILLIVOLT_PER_AMP           20
/* 结束当前条件编译分支。 */
#endif 

/* 仅在未定义 CURRENT_OFFSET 时编译以下代码。 */
#ifndef 	  CURRENT_OFFSET
/* 电流换算后附加的偏移修正。 */
#define     CURRENT_OFFSET              0
/* 结束当前条件编译分支。 */
#endif

/* 仅在未定义 TARGET_STALL_PROTECTION_INTERVAL 时编译以下代码。 */
#ifndef TARGET_STALL_PROTECTION_INTERVAL
/* 低速补偿的目标换相间隔默认值。 */
#define TARGET_STALL_PROTECTION_INTERVAL 6500
/* 结束当前条件编译分支。 */
#endif


/* 仅在定义 HARDWARE_GROUP_AT_A 时编译以下代码。 */
#ifdef HARDWARE_GROUP_AT_A

/* 选择 MCU_AT421 的 MCU 寄存器和外设适配分支。 */
#define MCU_AT421
/* 选择信号输入使用的定时器及通道：USE_TIMER_3_CHANNEL_1。 */
#define USE_TIMER_3_CHANNEL_1
/* 外部油门信号 GPIO 引脚位掩码。 */
#define INPUT_PIN               GPIO_PINS_4
/* 外部油门信号 GPIO 复用源编号。 */
#define INPUT_PIN_SOURCE        GPIO_PINS_SOURCE4
/* 外部油门信号 GPIO 端口。 */
#define INPUT_PIN_PORT          GPIOB
/* 信号输入捕获通道。 */
#define IC_TIMER_CHANNEL        TMR_SELECT_CHANNEL_1
/* 信号输入捕获及双向输出使用的定时器。 */
#define IC_TIMER_REGISTER       TMR3
/* 输入捕获及双向 DShot 输出共用的 DMA 通道。 */
#define INPUT_DMA_CHANNEL       DMA1_CHANNEL4
/* 信号 DMA 对应的中断号。 */
#define IC_DMA_IRQ_NAME         DMA1_Channel5_4_IRQn

/* A 相低侧引脚位掩码。 */
#define PHASE_A_GPIO_LOW        GPIO_PINS_1
/* A 相低侧引脚复用源编号。 */
#define PHASE_A_PIN_SOURCE_LOW  GPIO_PINS_SOURCE1
/* A 相低侧 GPIO 端口。 */
#define PHASE_A_GPIO_PORT_LOW   GPIOB
/* A 相高侧引脚位掩码。 */
#define PHASE_A_GPIO_HIGH       GPIO_PINS_10
/* A 相高侧引脚复用源编号。 */
#define PHASE_A_PIN_SOURCE_HIGH  GPIO_PINS_SOURCE10
/* A 相高侧 GPIO 端口。 */
#define PHASE_A_GPIO_PORT_HIGH  GPIOA

/* B 相低侧引脚位掩码。 */
#define PHASE_B_GPIO_LOW        GPIO_PINS_0
/* B 相低侧引脚复用源编号。 */
#define PHASE_B_PIN_SOURCE_LOW  GPIO_PINS_SOURCE0
/* B 相低侧 GPIO 端口。 */
#define PHASE_B_GPIO_PORT_LOW   GPIOB
/* B 相高侧引脚位掩码。 */
#define PHASE_B_GPIO_HIGH       GPIO_PINS_9
/* B 相高侧引脚复用源编号。 */
#define PHASE_B_PIN_SOURCE_HIGH  GPIO_PINS_SOURCE9
/* B 相高侧 GPIO 端口。 */
#define PHASE_B_GPIO_PORT_HIGH  GPIOA

/* C 相低侧引脚位掩码。 */
#define PHASE_C_GPIO_LOW        GPIO_PINS_7
/* C 相低侧引脚复用源编号。 */
#define PHASE_C_PIN_SOURCE_LOW  GPIO_PINS_SOURCE7
/* C 相低侧 GPIO 端口。 */
#define PHASE_C_GPIO_PORT_LOW   GPIOA
/* C 相高侧引脚位掩码。 */
#define PHASE_C_GPIO_HIGH       GPIO_PINS_8
/* C 相高侧引脚复用源编号。 */
#define PHASE_C_PIN_SOURCE_HIGH  GPIO_PINS_SOURCE8
/* C 相高侧 GPIO 端口。 */
#define PHASE_C_GPIO_PORT_HIGH  GPIOA


//#define PHASE_A_COMP COMP_INMInput_IN3  // pa0
//#define PHASE_B_COMP COMP_INMInput_IN1  // pa4
//#define PHASE_C_COMP COMP_INMInput_IN2  // pa5



/* 条件编译：defined(AT32DEV_F421_540)；条件满足时采用以下实现。 */
#if defined(AT32DEV_F421_540)
/* 诊断 ADC 的 A 相通道；这里仍按原实现选择，与比较器路径应分别核对。 */
#define PHASE_A_COMP_CHANNEL ADC_CHANNEL_5
/* 诊断 ADC 的 B 相通道。 */
#define PHASE_B_COMP_CHANNEL ADC_CHANNEL_4
/* 诊断 ADC 的 C 相通道；这里仍按原实现选择，与比较器路径应分别核对。 */
#define PHASE_C_COMP_CHANNEL ADC_CHANNEL_0
/* 诊断 ADC 的虚拟中性点通道。 */
#define PHASE_CC_COMP_CHANNEL ADC_CHANNEL_1
/* A 相比较器配置：负端选 PA0，正端选 PA1；整字写入也会覆盖迟滞和消隐位。 */
#define PHASE_A_COMP  0x400000E1
/* B 相比较器配置：负端选 PA4，正端选 PA1；整字写入也会覆盖迟滞和消隐位。 */
#define PHASE_B_COMP  0x400000C1
/* C 相比较器配置：负端选 PA5，正端选 PA1；整字写入也会覆盖迟滞和消隐位。 */
#define PHASE_C_COMP  0x400000D1
/* 采用上一编译条件不成立时的备选实现。 */
#else
/* 诊断 ADC 的 A 相通道；这里仍按原实现选择，与比较器路径应分别核对。 */
#define PHASE_A_COMP_CHANNEL ADC_CHANNEL_5
/* 诊断 ADC 的 B 相通道。 */
#define PHASE_B_COMP_CHANNEL ADC_CHANNEL_4
/* 诊断 ADC 的 C 相通道；这里仍按原实现选择，与比较器路径应分别核对。 */
#define PHASE_C_COMP_CHANNEL ADC_CHANNEL_0
/* 诊断 ADC 的虚拟中性点通道。 */
#define PHASE_CC_COMP_CHANNEL ADC_CHANNEL_1
/* A 相比较器配置：负端选 PA0，正端选 PA1；整字写入也会覆盖迟滞和消隐位。 */
#define PHASE_A_COMP  0x400000E1
/* B 相比较器配置：负端选 PA4，正端选 PA1；整字写入也会覆盖迟滞和消隐位。 */
#define PHASE_B_COMP  0x400000C1
/* C 相比较器配置：负端选 PA5，正端选 PA1；整字写入也会覆盖迟滞和消隐位。 */
#define PHASE_C_COMP  0x400000D1
/* 结束当前条件编译分支。 */
#endif

/* 当前实现按通道号加 1 生成位掩码；与 PA0～PA6 同号 ADC 引脚的对应存在偏移。 */
#define CHANNEL_TO_PIN(CANNNEL) (1<<(CANNNEL+1))

/* 结束当前条件编译分支。 */
#endif



/* 仅在定义 HARDWARE_GROUP_AT_B 时编译以下代码。 */
#ifdef HARDWARE_GROUP_AT_B

/* 选择 MCU_AT421 的 MCU 寄存器和外设适配分支。 */
#define MCU_AT421
/* 选择信号输入使用的定时器及通道：USE_TIMER_3_CHANNEL_1。 */
#define USE_TIMER_3_CHANNEL_1
/* 外部油门信号 GPIO 引脚位掩码。 */
#define INPUT_PIN               GPIO_PINS_4
/* 外部油门信号 GPIO 复用源编号。 */
#define INPUT_PIN_SOURCE        GPIO_PINS_SOURCE4
/* 外部油门信号 GPIO 端口。 */
#define INPUT_PIN_PORT          GPIOB
/* 信号输入捕获通道。 */
#define IC_TIMER_CHANNEL        TMR_SELECT_CHANNEL_1
/* 信号输入捕获及双向输出使用的定时器。 */
#define IC_TIMER_REGISTER       TMR3
/* 输入捕获及双向 DShot 输出共用的 DMA 通道。 */
#define INPUT_DMA_CHANNEL       DMA1_CHANNEL4
/* 信号 DMA 对应的中断号。 */
#define IC_DMA_IRQ_NAME         DMA1_Channel5_4_IRQn

/* A 相低侧引脚位掩码。 */
#define PHASE_A_GPIO_LOW        GPIO_PINS_1
/* A 相低侧引脚复用源编号。 */
#define PHASE_A_PIN_SOURCE_LOW  GPIO_PINS_SOURCE1
/* A 相低侧 GPIO 端口。 */
#define PHASE_A_GPIO_PORT_LOW   GPIOB
/* A 相高侧引脚位掩码。 */
#define PHASE_A_GPIO_HIGH       GPIO_PINS_10
/* A 相高侧引脚复用源编号。 */
#define PHASE_A_PIN_SOURCE_HIGH  GPIO_PINS_SOURCE10
/* A 相高侧 GPIO 端口。 */
#define PHASE_A_GPIO_PORT_HIGH  GPIOA

/* B 相低侧引脚位掩码。 */
#define PHASE_B_GPIO_LOW        GPIO_PINS_0
/* B 相低侧引脚复用源编号。 */
#define PHASE_B_PIN_SOURCE_LOW  GPIO_PINS_SOURCE0
/* B 相低侧 GPIO 端口。 */
#define PHASE_B_GPIO_PORT_LOW   GPIOB
/* B 相高侧引脚位掩码。 */
#define PHASE_B_GPIO_HIGH       GPIO_PINS_9
/* B 相高侧引脚复用源编号。 */
#define PHASE_B_PIN_SOURCE_HIGH  GPIO_PINS_SOURCE9
/* B 相高侧 GPIO 端口。 */
#define PHASE_B_GPIO_PORT_HIGH  GPIOA

/* C 相低侧引脚位掩码。 */
#define PHASE_C_GPIO_LOW        GPIO_PINS_7
/* C 相低侧引脚复用源编号。 */
#define PHASE_C_PIN_SOURCE_LOW  GPIO_PINS_SOURCE7
/* C 相低侧 GPIO 端口。 */
#define PHASE_C_GPIO_PORT_LOW   GPIOA
/* C 相高侧引脚位掩码。 */
#define PHASE_C_GPIO_HIGH       GPIO_PINS_8
/* C 相高侧引脚复用源编号。 */
#define PHASE_C_PIN_SOURCE_HIGH  GPIO_PINS_SOURCE8
/* C 相高侧 GPIO 端口。 */
#define PHASE_C_GPIO_PORT_HIGH  GPIOA

/* 仅在定义 HARDWARE_GROUP_AT_B450 时编译以下代码。 */
#ifdef HARDWARE_GROUP_AT_B450
/* A 相比较器配置：负端选 PA4，正端选 PA1；整字写入也会覆盖迟滞和消隐位。 */
#define PHASE_A_COMP  0x400000C1       //pa4            // works for polling mode
/* B 相比较器配置：负端选 PA5，正端选 PA1；整字写入也会覆盖迟滞和消隐位。 */
#define PHASE_B_COMP  0x400000D1       //pa5      
/* C 相比较器配置：负端选 PA0，正端选 PA1；整字写入也会覆盖迟滞和消隐位。 */
#define PHASE_C_COMP  0x400000E1       //pa0       
/* 结束当前条件编译分支。 */
#endif

/* 仅在定义 HARDWARE_GROUP_AT_B504 时编译以下代码。 */
#ifdef HARDWARE_GROUP_AT_B504
/* A 相比较器配置：负端选 PA5，正端选 PA1；整字写入也会覆盖迟滞和消隐位。 */
#define PHASE_A_COMP  0x400000D1       //pa5            // works for polling mode
/* B 相比较器配置：负端选 PA0，正端选 PA1；整字写入也会覆盖迟滞和消隐位。 */
#define PHASE_B_COMP  0x400000E1       //pa0       
/* C 相比较器配置：负端选 PA4，正端选 PA1；整字写入也会覆盖迟滞和消隐位。 */
#define PHASE_C_COMP  0x400000C1       //pa4       
/* 结束当前条件编译分支。 */
#endif

//#define PHASE_A_COMP  0x400000D1            // works for polling mode
//#define PHASE_B_COMP  0x400000C1
//#define PHASE_C_COMP  0x400000E1

/* 结束当前条件编译分支。 */
#endif

/* 仅在定义 HARDWARE_GROUP_AT_C 时编译以下代码。 */
#ifdef HARDWARE_GROUP_AT_C

/* 选择 MCU_AT421 的 MCU 寄存器和外设适配分支。 */
#define MCU_AT421
/* 选择信号输入使用的定时器及通道：USE_TIMER_15_CHANNEL_1。 */
#define USE_TIMER_15_CHANNEL_1
/* 外部油门信号 GPIO 引脚位掩码。 */
#define INPUT_PIN               GPIO_PINS_2
/* 外部油门信号 GPIO 复用源编号。 */
#define INPUT_PIN_SOURCE        GPIO_PINS_SOURCE2
/* 外部油门信号 GPIO 端口。 */
#define INPUT_PIN_PORT          GPIOA
/* 信号输入捕获通道。 */
#define IC_TIMER_CHANNEL        TMR_SELECT_CHANNEL_1
/* 信号输入捕获及双向输出使用的定时器。 */
#define IC_TIMER_REGISTER       TMR15
/* 输入捕获及双向 DShot 输出共用的 DMA 通道。 */
#define INPUT_DMA_CHANNEL       DMA1_CHANNEL5
/* 信号 DMA 对应的中断号。 */
#define IC_DMA_IRQ_NAME         DMA1_Channel5_4_IRQn

/* A 相低侧引脚位掩码。 */
#define PHASE_A_GPIO_LOW        GPIO_PINS_1
/* A 相低侧引脚复用源编号。 */
#define PHASE_A_PIN_SOURCE_LOW  GPIO_PINS_SOURCE1
/* A 相低侧 GPIO 端口。 */
#define PHASE_A_GPIO_PORT_LOW   GPIOB
/* A 相高侧引脚位掩码。 */
#define PHASE_A_GPIO_HIGH       GPIO_PINS_10
/* A 相高侧引脚复用源编号。 */
#define PHASE_A_PIN_SOURCE_HIGH  GPIO_PINS_SOURCE10
/* A 相高侧 GPIO 端口。 */
#define PHASE_A_GPIO_PORT_HIGH  GPIOA

/* B 相低侧引脚位掩码。 */
#define PHASE_B_GPIO_LOW        GPIO_PINS_0
/* B 相低侧引脚复用源编号。 */
#define PHASE_B_PIN_SOURCE_LOW  GPIO_PINS_SOURCE0
/* B 相低侧 GPIO 端口。 */
#define PHASE_B_GPIO_PORT_LOW   GPIOB
/* B 相高侧引脚位掩码。 */
#define PHASE_B_GPIO_HIGH       GPIO_PINS_9
/* B 相高侧引脚复用源编号。 */
#define PHASE_B_PIN_SOURCE_HIGH  GPIO_PINS_SOURCE9
/* B 相高侧 GPIO 端口。 */
#define PHASE_B_GPIO_PORT_HIGH  GPIOA

/* C 相低侧引脚位掩码。 */
#define PHASE_C_GPIO_LOW        GPIO_PINS_7
/* C 相低侧引脚复用源编号。 */
#define PHASE_C_PIN_SOURCE_LOW  GPIO_PINS_SOURCE7
/* C 相低侧 GPIO 端口。 */
#define PHASE_C_GPIO_PORT_LOW   GPIOA
/* C 相高侧引脚位掩码。 */
#define PHASE_C_GPIO_HIGH       GPIO_PINS_8
/* C 相高侧引脚复用源编号。 */
#define PHASE_C_PIN_SOURCE_HIGH  GPIO_PINS_SOURCE8
/* C 相高侧 GPIO 端口。 */
#define PHASE_C_GPIO_PORT_HIGH  GPIOA



/* 仅在定义 HARDWARE_GROUP_AT_C045 时编译以下代码。 */
#ifdef HARDWARE_GROUP_AT_C045
/* A 相比较器配置：负端选 PA0，正端选 PA1；整字写入也会覆盖迟滞和消隐位。 */
#define PHASE_A_COMP  0x400000E1       //pa0     // works for polling mode
/* B 相比较器配置：负端选 PA4，正端选 PA1；整字写入也会覆盖迟滞和消隐位。 */
#define PHASE_B_COMP  0x400000C1       //pa4
/* C 相比较器配置：负端选 PA5，正端选 PA1；整字写入也会覆盖迟滞和消隐位。 */
#define PHASE_C_COMP  0x400000D1       //pa5
/* 结束当前条件编译分支。 */
#endif
/* 仅在定义 HARDWARE_GROUP_AT_C504 时编译以下代码。 */
#ifdef HARDWARE_GROUP_AT_C504
/* A 相比较器配置：负端选 PA5，正端选 PA1；整字写入也会覆盖迟滞和消隐位。 */
#define PHASE_A_COMP  0x400000D1       //pa5            // works for polling mode
/* B 相比较器配置：负端选 PA0，正端选 PA1；整字写入也会覆盖迟滞和消隐位。 */
#define PHASE_B_COMP  0x400000E1       //pa0       
/* C 相比较器配置：负端选 PA4，正端选 PA1；整字写入也会覆盖迟滞和消隐位。 */
#define PHASE_C_COMP  0x400000C1       //pa4       
/* 结束当前条件编译分支。 */
#endif
/* 仅在定义 HARDWARE_GROUP_AT_C450 时编译以下代码。 */
#ifdef HARDWARE_GROUP_AT_C450
/* A 相比较器配置：负端选 PA4，正端选 PA1；整字写入也会覆盖迟滞和消隐位。 */
#define PHASE_A_COMP  0x400000C1       //pa4            // works for polling mode
/* B 相比较器配置：负端选 PA5，正端选 PA1；整字写入也会覆盖迟滞和消隐位。 */
#define PHASE_B_COMP  0x400000D1       //pa5      
/* C 相比较器配置：负端选 PA0，正端选 PA1；整字写入也会覆盖迟滞和消隐位。 */
#define PHASE_C_COMP  0x400000E1       //pa0       
/* 结束当前条件编译分支。 */
#endif
/* 仅在定义 HARDWARE_GROUP_AT_C054 时编译以下代码。 */
#ifdef HARDWARE_GROUP_AT_C054
/* A 相比较器配置：负端选 PA0，正端选 PA1；整字写入也会覆盖迟滞和消隐位。 */
#define PHASE_A_COMP  0x400000E1       //pa0            // works for polling mode
/* B 相比较器配置：负端选 PA5，正端选 PA1；整字写入也会覆盖迟滞和消隐位。 */
#define PHASE_B_COMP  0x400000D1       //pa5       
/* C 相比较器配置：负端选 PA4，正端选 PA1；整字写入也会覆盖迟滞和消隐位。 */
#define PHASE_C_COMP  0x400000C1       //pa4      
/* 结束当前条件编译分支。 */
#endif
/* 仅在定义 HARDWARE_GROUP_AT_C405 时编译以下代码。 */
#ifdef HARDWARE_GROUP_AT_C405
/* A 相比较器配置：负端选 PA4，正端选 PA1；整字写入也会覆盖迟滞和消隐位。 */
#define PHASE_A_COMP  0x400000C1       //pa4            // works for polling mode
/* B 相比较器配置：负端选 PA0，正端选 PA1；整字写入也会覆盖迟滞和消隐位。 */
#define PHASE_B_COMP  0x400000E1       //pa0       
/* C 相比较器配置：负端选 PA5，正端选 PA1；整字写入也会覆盖迟滞和消隐位。 */
#define PHASE_C_COMP  0x400000D1       //pa5       
/* 结束当前条件编译分支。 */
#endif
/* 仅在定义 HARDWARE_GROUP_AT_C540 时编译以下代码。 */
#ifdef HARDWARE_GROUP_AT_C540
/* A 相比较器配置：负端选 PA5，正端选 PA1；整字写入也会覆盖迟滞和消隐位。 */
#define PHASE_A_COMP  0x400000D1       //pa5           // works for polling mode
/* B 相比较器配置：负端选 PA4，正端选 PA1；整字写入也会覆盖迟滞和消隐位。 */
#define PHASE_B_COMP  0x400000C1       //pa4       
/* C 相比较器配置：负端选 PA0，正端选 PA1；整字写入也会覆盖迟滞和消隐位。 */
#define PHASE_C_COMP  0x400000E1       //pa0       
/* 结束当前条件编译分支。 */
#endif

/* 结束当前条件编译分支。 */
#endif

/* 仅在定义 HARDWARE_GROUP_AT_D 时编译以下代码。 */
#ifdef HARDWARE_GROUP_AT_D

/* 选择 MCU_AT415 的 MCU 寄存器和外设适配分支。 */
#define MCU_AT415
/* 选择信号输入使用的定时器及通道：USE_TIMER_3_CHANNEL_1。 */
#define USE_TIMER_3_CHANNEL_1
/* 外部油门信号 GPIO 引脚位掩码。 */
#define INPUT_PIN               GPIO_PINS_4
/* 外部油门信号 GPIO 端口。 */
#define INPUT_PIN_PORT          GPIOB
/* 信号输入捕获通道。 */
#define IC_TIMER_CHANNEL        TMR_SELECT_CHANNEL_1
/* 信号输入捕获及双向输出使用的定时器。 */
#define IC_TIMER_REGISTER       TMR3
/* 输入捕获及双向 DShot 输出共用的 DMA 通道。 */
#define INPUT_DMA_CHANNEL       DMA1_CHANNEL6
/* 信号 DMA 对应的中断号。 */
#define IC_DMA_IRQ_NAME         DMA1_Channel6_IRQn

/* A 相低侧引脚位掩码。 */
#define PHASE_A_GPIO_LOW        GPIO_PINS_1
/* A 相低侧引脚复用源编号。 */
#define PHASE_A_PIN_SOURCE_LOW  GPIO_PIN_SOURCE1
/* A 相低侧 GPIO 端口。 */
#define PHASE_A_GPIO_PORT_LOW   GPIOB
/* A 相高侧引脚位掩码。 */
#define PHASE_A_GPIO_HIGH       GPIO_PINS_10
/* A 相高侧引脚复用源编号。 */
#define PHASE_A_PIN_SOURCE_HIGH GPIO_PIN_SOURCE10
/* A 相高侧 GPIO 端口。 */
#define PHASE_A_GPIO_PORT_HIGH  GPIOA

/* B 相低侧引脚位掩码。 */
#define PHASE_B_GPIO_LOW        GPIO_PINS_0
/* B 相低侧引脚复用源编号。 */
#define PHASE_B_PIN_SOURCE_LOW  GPIO_PIN_SOURCE0
/* B 相低侧 GPIO 端口。 */
#define PHASE_B_GPIO_PORT_LOW   GPIOB
/* B 相高侧引脚位掩码。 */
#define PHASE_B_GPIO_HIGH       GPIO_PINS_9
/* B 相高侧引脚复用源编号。 */
#define PHASE_B_PIN_SOURCE_HIGH  GPIO_PIN_SOURCE9
/* B 相高侧 GPIO 端口。 */
#define PHASE_B_GPIO_PORT_HIGH  GPIOA

/* C 相低侧引脚位掩码。 */
#define PHASE_C_GPIO_LOW        GPIO_PINS_7
/* C 相低侧引脚复用源编号。 */
#define PHASE_C_PIN_SOURCE_LOW  GPIO_PIN_SOURCE7
/* C 相低侧 GPIO 端口。 */
#define PHASE_C_GPIO_PORT_LOW   GPIOA
/* C 相高侧引脚位掩码。 */
#define PHASE_C_GPIO_HIGH       GPIO_PINS_8
/* C 相高侧引脚复用源编号。 */
#define PHASE_C_PIN_SOURCE_HIGH  GPIO_PIN_SOURCE8
/* C 相高侧 GPIO 端口。 */
#define PHASE_C_GPIO_PORT_HIGH  GPIOA


//#define PHASE_A_COMP COMP_INMInput_IN3  // pa0
//#define PHASE_B_COMP COMP_INMInput_IN1  // pa4
//#define PHASE_C_COMP COMP_INMInput_IN2  // pa5

/* A 相比较器配置：负端选 PA0，正端选 PA1；整字写入也会覆盖迟滞和消隐位。 */
#define PHASE_A_COMP  0x400000E1            // works for polling mode
/* B 相比较器配置：负端选 PA4，正端选 PA1；整字写入也会覆盖迟滞和消隐位。 */
#define PHASE_B_COMP  0x400000C1
/* C 相比较器配置：负端选 PA5，正端选 PA1；整字写入也会覆盖迟滞和消隐位。 */
#define PHASE_C_COMP  0x400000D1

/* 结束当前条件编译分支。 */
#endif
/************************************ MCU COMMON PERIPHERALS **********************************************/


/* 仅在定义 MCU_AT421 时编译以下代码。 */
#ifdef MCU_AT421
/* 代码用于定时换算的 CPU 频率参数，单位 MHz。 */
#define CPU_FREQUENCY_MHZ   120

/* 模拟 EEPROM 的 Flash 起始地址。 */
#define EEPROM_START_ADD  (uint32_t)0x08007C00

/* 测量过零/换相间隔的定时器。 */
#define INTERVAL_TIMER     TMR6
/* 周期控制任务使用的定时器。 */
#define TEN_KHZ_TIMER      TMR14
/* 阻塞延时和耗时测量共用的辅助定时器。 */
#define UTILITY_TIMER      TMR17
/* 过零后延迟换相使用的定时器。 */
#define COM_TIMER          TMR16
/* PWM 默认周期基准。 */
#define TIM1_AUTORELOAD    4000
/* 应用程序起始地址。 */
#define APPLICATION_ADDRESS 0x08000000
/* 比较器过零事件对应的 EXINT 线路。 */
#define EXTI_LINE   EXINT_LINE_21
/* 轮询过零需要的有效读数默认门限。 */
#define TARGET_MIN_BEMF_COUNTS 8
/* 启用 ADC 采样模块。 */
#define USE_ADC
/* 双向 DShot 输出波形的定时器周期参数。 */
#define DSHOT_PRE            76
/* 结束当前条件编译分支。 */
#endif

/* 仅在定义 MCU_AT415 时编译以下代码。 */
#ifdef MCU_AT415
/* 代码用于定时换算的 CPU 频率参数，单位 MHz。 */
#define CPU_FREQUENCY_MHZ   150

/* 模拟 EEPROM 的 Flash 起始地址。 */
#define EEPROM_START_ADD  (uint32_t)0x08007C00

/* 测量过零/换相间隔的定时器。 */
#define INTERVAL_TIMER     TMR4
/* 周期控制任务使用的定时器。 */
#define TEN_KHZ_TIMER      TMR9
/* 阻塞延时和耗时测量共用的辅助定时器。 */
#define UTILITY_TIMER      TMR10
/* 过零后延迟换相使用的定时器。 */
#define COM_TIMER          TMR11
/* PWM 默认周期基准。 */
#define TIM1_AUTORELOAD    4000
/* 应用程序起始地址。 */
#define APPLICATION_ADDRESS 0x08000000
/* 比较器过零事件对应的 EXINT 线路。 */
#define EXTI_LINE   EXINT_LINE_19
/* 轮询过零需要的有效读数默认门限。 */
#define TARGET_MIN_BEMF_COUNTS 6
/* 启用 ADC 采样模块。 */
#define USE_ADC
/* 双向 DShot 输出波形的定时器周期参数。 */
#define DSHOT_PRE            95
/* 结束当前条件编译分支。 */
#endif



/* 仅在定义 MCU_AT415 时编译以下代码。 */
#ifdef MCU_AT415
    /* 当前 MCU 比较器输出状态位。 */
	#define CMP_VALUE (CMP->ctrlsts1_bit.cmp1value)
/* 采用上一编译条件不成立时的备选实现。 */
#else
    /* 当前 MCU 比较器输出状态位。 */
	#define CMP_VALUE (CMP->ctrlsts_bit.cmpvalue)
/* 结束当前条件编译分支。 */
#endif
