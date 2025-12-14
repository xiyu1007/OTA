#include <stdio.h>
#include "Log.h"
#include "UserUSART.h"


// int fputc(int ch, FILE *f)
// {
//     // 1.发送数据寄存器为空
//     while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);

//     // 2.写入数据到数据寄存器, int 类型转换为字节类型
//     USART_SendData(USARTx, (uint8_t)ch);

//     while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);

//     return ch;
// }

