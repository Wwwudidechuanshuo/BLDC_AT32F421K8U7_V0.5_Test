/*
 * 电调主控制：配置加载、启动、六步换相、过零判断、保护、油门和遥测调度。
 * 中文说明按当前实现编写；保留原代码、历史注释和编译条件。
 */
/*!
    \file    main.h
    \brief   the header file of main
    
    \version 2019-02-19, V1.0.0, firmware for GD32E23x
    \version 2020-12-12, V1.1.0, firmware for GD32E23x
*/

/*
    Copyright (c) 2020, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

/* 仅在未定义 MAIN_H 时编译以下代码。 */
#ifndef MAIN_H
/* 头文件重复包含保护标记。 */
#define MAIN_H

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
//#include "at32421_conf.h"
//#include "at32f421_dma.h"
//#include "at32f421_tmr.h"
//#include "at32f421_crm.h"
//#include "at32f421_gpio.h"
//#include "at32f421_adc.h"
//#include "at32f421_exint.h"
//#include "at32f421_usart.h"
//#include "at32f421_wdt.h"
//#include "at32f421_cmp.h"
//#include "at32f421_misc.h"


/* 结束当前条件编译分支。 */
#endif


