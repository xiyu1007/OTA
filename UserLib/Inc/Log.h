#ifndef __LOG_H__
#define __LOG_H__

#include <stdio.h>

#define LOG(fmt, ...) printf("%s : %d : %s : " fmt "\n", __FILE__, __LINE__, __func__, ##__VA_ARGS__)

/*
##__VA_ARGS__
可变参数宏的特性
当没有额外参数时，会把多余的逗号去掉，防止语法错误
当有参数时，就替换为这些参数
*/

#endif /* __LOG_H__ */
