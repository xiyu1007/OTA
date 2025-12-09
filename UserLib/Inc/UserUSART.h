#ifndef __USER_USART_H__
#define __USER_USART_H__

#include "stm32f10x.h"
#include <stdio.h>

#ifndef USARTx
#define USARTx USART1
#define USARTTx GPIO_Pin_9
#define USARTRx GPIO_Pin_10
#define GPIOx_USART GPIOA
#define DMAx DMA1
#define USART_DMA_TX_CH DMA1_Channel4 // USART1 TX
#define USART_DMA_RX_CH DMA1_Channel5 // USART1 RX
// #define USART_DMA_TX_CH   DMA1_Channel6 // USART2 TX
// #define USART_DMA_RX_CH   DMA1_Channel7 // USART2 RX
// #define USART_DMA_TX_CH   DMA1_Channel2 // USART3 TX
// #define USART_DMA_RX_CH   DMA1_Channel3 // USART3 RX
#endif

#define U_TX_SIZE   2048    // 环形缓冲区总大小（字节数）
#define U_RX_SIZE   2048    // 环形缓冲区总大小（字节数）
#define U_RX_MAX    256     // 每次单独接收最大字节数
#define NUM         10      // 分段缓冲区数量

/**
 * @brief  表示环形缓冲区中的一段连续内存
 * @note   start: 段起始地址
 *         end:   段结束地址
 *         一个环形缓冲区由 NUM 个段组成
 */
typedef struct {
    uint8_t *start;   // 段起始地址
    uint8_t *end;     // 段结束地址
} UCB_URxBuffPtr;

/**
 * @brief  环形缓冲区控制结构
 * @note
 *  - URxDataPtr[NUM] : 保存每个段的起始和结束地址
 *  - URxDataIN       : DMA 或串口写入指针，写入数据时向前移动
 *                      到达缓冲区末尾时回绕到起始地址，实现循环
 *  - URxDataOUT      : 应用程序读取指针，读取数据后向前移动
 *  - URxDataEND      : 缓冲区末尾地址，用于判断回绕
 *  - URxCount        : 当前缓冲区有效数据量，可判断空或满
 */
typedef struct {
    UCB_URxBuffPtr URxDataPtr[NUM];  // 分段缓冲区数组
    UCB_URxBuffPtr *URxDataIN;               // 写入指针（DMA/串口接收）
    UCB_URxBuffPtr *URxDataOUT;              // 读取指针（应用程序）
    UCB_URxBuffPtr *URxDataEND;              // 缓冲区末尾指针
    uint32_t URxCount;                // 当前缓冲区有效数据量(字节数)
} UCB_CB;


void User_USART_Init(void);
void USARTx_IRQHandler(void);
extern UCB_CB UxCB;

#endif /* __UASRT_H__ */
