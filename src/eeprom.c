/*
 * 使用片内 Flash 保存和读取配置；此处是软件模拟 EEPROM，并非外接 EEPROM。
 * 中文说明按当前实现编写；保留原代码、历史注释和编译条件。
 */
/* 引入 eeprom.h：使用片内 Flash 保存和读取配置；此处是软件模拟 EEPROM，并非外接 EEPROM。 */
#include "eeprom.h"
/* 引入 string.h：芯片或运行库接口。 */
#include <string.h>
/* 引入 targets.h：硬件目标配置：选择 MCU、输入捕获资源、三相驱动引脚、比较器通道和默认参数。 */
#include "targets.h"

/* 仅在定义 MCU_ATF421 时编译以下代码。 */
#ifdef MCU_ATF421
/* 引入 at32f421_flash.h：芯片或运行库接口。 */
#include "at32f421_flash.h"
/* 结束当前条件编译分支。 */
#endif
/* 仅在定义 MCU_AT415 时编译以下代码。 */
#ifdef MCU_AT415
/* 引入 at32f415_flash.h：芯片或运行库接口。 */
#include "at32f415_flash.h"
/* 结束当前条件编译分支。 */
#endif

/* Flash 写入缓冲页大小，单位字节。 */
#define page_size 0x400                   // 1 kb for f051


/**
 * @brief 把字节缓冲区按小端顺序组装为 32 位字后写入 Flash；最多写一页，尾部不足 4 字节不写入。
 *
 * 仅在起始地址按 1024 字节对齐时擦除；Flash 不支持通过普通编程把 0 位直接恢复为 1。
 * 本函数没有返回擦写状态，现有实现会清除相关错误标志而不会向调用方报告失败。
 * @param data Flash 读出目标缓冲区或写入源缓冲区。
 * @param length 请求写入字节数，最多一页且按完整 32 位字写入。
 * @param add Flash 映射地址；写入时仅在地址按 1024 字节对齐时擦页。
 */
void save_flash_nolib(uint8_t *data, int length, uint32_t add){    ///todo
	
 //fmc_wscnt_set(2);

//fmc_prefetch_enable();
	
	
    /* 按页分配 32 位写入缓冲区；字节数据会按小端顺序装入。 */
	uint32_t data_to_FLASH[page_size / 4];
    /* 当Flash 请求写入字节数 大于 Flash 写入缓冲页大小 时进入此分支。 */
	if (length > page_size) {
        /* 更新Flash 请求写入字节数。 */
		length = page_size;
	}
    /* 按字节初始化缓冲区内容。 */
	memset(data_to_FLASH, 0, sizeof(data_to_FLASH));
        /* 按完整的四字节组遍历源缓冲区并组织待写入数据。 */
		for(int i = 0; i < length / 4 ; i ++ ){
        /* 把连续四个字节组合为一个 32 位小端字；原英文注释中的 16 位表述不适用。 */
		data_to_FLASH[i] =  data[i*4+3] << 24 |data[i*4+2] << 16|data[i*4+1] << 8| data[i*4];   // make 16 bit
	}
    /* 保存本次需要写入的完整 32 位字数量。 */
	volatile uint32_t data_length = length / 4;

	// unlock flash

        /* 解除 Flash 写保护锁，允许擦除和编程。 */
		flash_unlock();

	// erase page if address even divisable by 1024
     /* 写入起始地址按 1024 字节对齐时，先执行扇区擦除。 */
	 if((add % 1024) == 0){

   /* 擦除配置地址所在扇区，原有数据将被清除。 */
   flash_sector_erase(add);

	 }

     /* 保存保留的 Flash 写入计数变量。 */
	 volatile uint32_t write_cnt=0, index=0;
     /* 重复检查等待条件：当Flash 逐字写入位置 小于 本次需要写入的完整 32 位字数量 时继续循环。 */
	 while(index < data_length)
			  {

            /* 向指定 Flash 地址写入一个 32 位字。 */
	        flash_word_program(add + (index*4),data_to_FLASH[index]);
            /* 清除 Flash 操作完成和错误状态标志。 */
			flash_flag_clear(FLASH_PROGRAM_ERROR | FLASH_EPP_ERROR | FLASH_OPERATE_DONE);
                  /* 递增Flash 逐字写入位置。 */
				  index++;
		  }
     /* 重新锁定 Flash，防止后续意外写入。 */
	 flash_lock();
}




/**
 * @brief 从 Flash 映射地址逐字节读取到调用方缓冲区；调用方保证地址和容量有效。
 * @param data Flash 读出目标缓冲区或写入源缓冲区。
 * @param add Flash 映射地址；写入时仅在地址按 1024 字节对齐时擦页。
 * @param out_buff_len 需要读出的字节数，调用方保证目标缓冲区足够大。
 */
void read_flash_bin(uint8_t*  data , uint32_t add , int out_buff_len){
	//volatile uint32_t read_data;
    /* 逐字节读取指定 Flash 范围并复制到目标缓冲区。 */
	for (int i = 0; i < out_buff_len ; i ++){
        /* 更新data中的当前元素。 */
		data[i] = *(uint8_t*)(add + i);
	}
}
