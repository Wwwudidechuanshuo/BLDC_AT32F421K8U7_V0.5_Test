/*
 * 使用片内 Flash 保存和读取配置；此处是软件模拟 EEPROM，并非外接 EEPROM。
 * 中文说明按当前实现编写；保留原代码、历史注释和编译条件。
 */
/* 引入 main.h：电调主控制：配置加载、启动、六步换相、过零判断、保护、油门和遥测调度。 */
#include "main.h"


//void save_to_flash(uint8_t *data);
//void read_flash(uint8_t* data, uint32_t address);
//void save_to_flash_bin(uint8_t *data, int length, uint32_t add);
/* 从 Flash 映射地址逐字节读取到调用方缓冲区；调用方保证地址和容量有效。 */
void read_flash_bin(uint8_t*  data , uint32_t add ,int  out_buff_len);
/* 把字节缓冲区按小端顺序组装为 32 位字后写入 Flash；最多写一页，尾部不足 4 字节不写入。 */
void save_flash_nolib(uint8_t *data, int length, uint32_t add);
