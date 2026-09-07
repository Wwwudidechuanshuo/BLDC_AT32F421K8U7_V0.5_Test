/*
 * 固件识别字符串的导出接口；当前 test 数组保存 FILE_NAME，不是数值版本数组。
 * 中文说明按当前实现编写；保留原代码、历史注释和编译条件。
 */
/* 引入 firmwareversion.h：固件识别字符串的导出接口；当前 test 数组保存 FILE_NAME，不是数值版本数组。 */
#include "firmwareversion.h"
/* 引入 targets.h：硬件目标配置：选择 MCU、输入捕获资源、三相驱动引脚、比较器通道和默认参数。 */
#include "targets.h"

/* 导出固定长度的固件目标名称；used 属性要求编译器保留该对象。 */
const char test[14] __attribute__((used)) = FILE_NAME;
