#include "stm32f10x.h"
#include "UserUSART.h"
// #include <stdio.h>

uint8_t URxBuffer[U_RX_SIZE]; // 接收缓冲区
UCB_CB UxCB;

void UxCB_Init(void);
void User_DMA_Init(void);

void User_USART_Init(void)
{

    // 配置 NVIC 中断
    NVIC_InitTypeDef NVIC_InitStruct;
    if (USARTx == USART1)
        NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
    else if (USARTx == USART2)
        NVIC_InitStruct.NVIC_IRQChannel = USART2_IRQn;
    else if (USARTx == USART3)
        NVIC_InitStruct.NVIC_IRQChannel = USART3_IRQn;

    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
    

    if (USARTx == USART1)
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    else if (USARTx == USART2)
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    else if (USARTx == USART3)
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    USART_InitTypeDef USART_InitStruct = {0};
    USART_InitStruct.USART_BaudRate = 115200;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_Init(USARTx, &USART_InitStruct);
    
    
    if (GPIOx_USART == GPIOA)
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    else if (GPIOx_USART == GPIOB)
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    else if (GPIOx_USART == GPIOC)
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Pin = USARTTx;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOx_USART, &GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStruct.GPIO_Pin = USARTRx;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOx_USART, &GPIO_InitStruct);
    

    // 使能串口 IDLE 中断
    USART_ITConfig(USARTx, USART_IT_IDLE, ENABLE);
    // USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);

    UxCB_Init();
    User_DMA_Init();
    USART_Cmd(USARTx, ENABLE);  
}

void USARTx_IRQHandler(void)
{
    if (USART_GetITStatus(USARTx, USART_IT_IDLE) == SET)
    {
        // 清空中断需要读取状态寄存器和数据寄存器
        (void)USARTx->SR; // 读取状态寄存器
        (void)USARTx->DR; // 读取数据寄存器

        // DMA 初始化时设置的总数据量为 `DMA_BufferSize`， 传输过程中，这个值会递减
        UxCB.URxCount += U_RX_MAX + 1 - DMA_GetCurrDataCounter(USART_DMA_RX_CH);
        // URxDataIN 指向当前段的起始地址
        UxCB.URxDataIN->end = &URxBuffer[UxCB.URxCount - 1]; // 设置当前段的结束地址
        UxCB.URxDataIN++;                                    // 指向下一个段的地址
        if (UxCB.URxDataIN == UxCB.URxDataEND)
            UxCB.URxDataIN = &UxCB.URxDataPtr[0]; // 移动到第一个段的地址

        if (U_RX_SIZE - UxCB.URxCount >= U_RX_MAX)             // 确保下一个段有足够空间
            UxCB.URxDataIN->start = &URxBuffer[UxCB.URxCount]; // 0 ~ UxCB.URxCount-1是已接收数据
        else
        {
            UxCB.URxDataIN->start = URxBuffer;
            UxCB.URxCount = 0;
        }

        DMA_Cmd(USART_DMA_RX_CH, DISABLE);
        DMA_SetCurrDataCounter(USART_DMA_RX_CH, U_RX_MAX + 1);   // 设置本次搬运数据量
        USART_DMA_RX_CH->CMAR = (uint32_t)UxCB.URxDataIN->start; // 重设 DMA 内存起始地址
        DMA_Cmd(USART_DMA_RX_CH, ENABLE);
    }
}

void UxCB_Init(void)
{
    UxCB.URxDataIN = &UxCB.URxDataPtr[0];          // 指向第一个段的地址
    UxCB.URxDataOUT = &UxCB.URxDataPtr[0];         // 指向第一个段的地址
    UxCB.URxDataEND = &(UxCB.URxDataPtr[NUM - 1]); // 指向最后一个段的地址
    UxCB.URxDataIN->start = URxBuffer;
    UxCB.URxCount = 0;
}

int fputc(int ch, FILE *f)
{
    // 1.发送数据寄存器为空
    while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET)
        ;
    // 2.写入数据到数据寄存器, int 类型转换为字节类型
    USART_SendData(USARTx, (uint8_t)ch);
    return ch;
}

void User_DMA_Init(void)
{
    if (DMAx == DMA1)
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); // 开启DMA时钟
    else if (DMAx == DMA2)
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE); // 开启DMA时钟

    // 指定 DMA 通道恢复成上电复位状态——寄存器全部清零，所有中断、使能、优先级、数据宽度、地址等配置擦除。
    DMA_DeInit(USART_DMA_TX_CH); // 串口发送DMA通道
    DMA_DeInit(USART_DMA_RX_CH); // 串口接收DMA通道

    DMA_InitTypeDef DMA_InitStructure;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USARTx->DR);
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)URxBuffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;               // 串口数据(外设)到内存
    DMA_InitStructure.DMA_BufferSize = U_RX_MAX + 1;                 // 无须DMA完成中断，因此设置为单次接收数据个数（字节）最大值加1
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; // 串口地址不会变，无须自增
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;          // 内存地址会变，需要自增
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;         // 无须回到内存起始地址，因为是连续接收数据
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; // 中等优先级
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;          //
    DMA_Init(USART_DMA_RX_CH, &DMA_InitStructure);

    DMA_Cmd(USART_DMA_RX_CH, ENABLE); // 使能 DMA 接收通道

    USART_DMACmd(USARTx, USART_DMAReq_Rx, ENABLE); // 当 USART 收到数据时，会发出 DMA 请求，让 DMA 自动从 DR 寄存器读取数据。
}
