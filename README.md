# 环形指针
![[Fig/OTA/Pasted image 20251201034134.png|644]]
# USART 配置
## 引脚
1. 引脚配置
	![[Fig/OTA/Pasted image 20251130231934.png]]
2. 引脚模式配置
	![[Fig/OTA/Pasted image 20251130232030.png]]
3. 地址
	- USART1 范围：0x4001 3800 - 0x4001 3BFF （参考手册 表 1）
	- USART1_BASE ：0x4001 3800
	- DR 偏移地址：0x04  （参考手册 p 541）
	- `&(USART1->DR) = USART1_BASE + 0x40 = 0x40013800 + 0x04 = 0x40013804`
## 中断
###  UART 空闲中断（IDLE Line Interrupt）是什么？
> **UART 空闲中断用于检测一帧数据是否结束。**  
> 当 UART 在 RX 线上 **一个字节时间内没有新数据到来** → 触发 IDLE 中断。

场景：
- DMA 接收不定长数据（常见：Modbus、ESP8266、AT 指令等）
- 不需要每个字节都进中断
- 自动判断“本帧结束”
### 空闲中断的触发条件（非常重要）

IDLE 触发条件：

> **UART 在收到至少 1 个字节后，在一个帧的时长内没有再收到新字节**

比如：
115200 波特率 → 1 字节时间 = 1 / 115200 * 10bit ≈ **87µs**
若 87µs 内没有新数据 → 空闲中断触发。
### 空闲中断的寄存器位

|功能|寄存器|位|
|---|---|---|
|空闲中断标志|USART_SR|**IDLE (bit 4)**|
|空闲中断使能|USART_CR1|**IDLEIE (bit 4)**|

### 开启空闲中断（标准库）

1. 开启 UART 空闲中断（CR1）
```cpp
USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
```
2. 开启 NVIC
```cpp
NVIC_InitTypeDef NVIC_InitStructure;
NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

NVIC_Init(&NVIC_InitStructure);
```
3.  UART 空闲中断处理函数（最关键）
	**⚠ 必须读 SR → DR 来清除 IDLE 标志，否则中断会一直进入。**
```cpp
void USART1_IRQHandler(void)
{
    /* 空闲中断 */
    if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
    {
        volatile uint32_t temp;

        /* 必须先读 SR 再读 DR 清除 IDLE 标志 */
        temp = USART1->SR;
        temp = USART1->DR;
        (void)temp;

        /* ---- 这里处理接收完成 ---- */
        UART_IDLE_Callback();
    }
}
```


## OTA 代码
```cpp
#ifndef __USER_USART_H__
#define __USER_USART_H__

#include "stm32f10x.h"
#include <stdio.h>

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
void USART1_IRQHandler(void);
extern UCB_CB UxCB;

#endif /* __UASRT_H__ */

```

```cpp
#include "stm32f10x.h"
#include "UserUSART.h"
#include <stdio.h>

#ifndef USARTx
#define USARTx USART1
#define USARTTx GPIO_Pin_9
#define USARTRx GPIO_Pin_10
#define GPIOx GPIOA
#define DMAx DMA1
#define USART_DMA_TX_CH DMA1_Channel4 // USART1 TX
#define USART_DMA_RX_CH DMA1_Channel5 // USART1 RX
// #define USART_DMA_TX_CH   DMA1_Channel6 // USART2 TX
// #define USART_DMA_RX_CH   DMA1_Channel7 // USART2 RX
// #define USART_DMA_TX_CH   DMA1_Channel2 // USART3 TX
// #define USART_DMA_RX_CH   DMA1_Channel3 // USART3 RX
#endif

uint8_t URxBuffer[U_RX_SIZE]; // 接收缓冲区
UCB_CB UxCB;

void UxCB_Init(void);
void USARTx_IRQHandler(void);
void User_DMA_Init(void);

void USART1_IRQHandler(void)
{
    USARTx_IRQHandler();
}

void User_USART_Init(void)
{
    // 配置 NVIC 中断
    NVIC_InitTypeDef NVIC_InitStruct;
    if (USARTx == USART1) NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
    else if (USARTx == USART2) NVIC_InitStruct.NVIC_IRQChannel = USART2_IRQn;
    else if (USARTx == USART3) NVIC_InitStruct.NVIC_IRQChannel = USART3_IRQn;

    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    if (GPIOx == GPIOA) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    else if (GPIOx == GPIOB) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    else if (GPIOx == GPIOC) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Pin = USARTTx;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOx, &GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStruct.GPIO_Pin = USARTRx;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOx, &GPIO_InitStruct);

    if (USARTx == USART1) RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    else if (USARTx == USART2) RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    else if (USARTx == USART3) RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    USART_InitTypeDef USART_InitStruct = {0};
    USART_InitStruct.USART_BaudRate = 115200;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_Init(USARTx, &USART_InitStruct);

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

```
# DMA 配置
## 通道
1. DMA1
		![[Fig/OTA/Pasted image 20251130232824.png]]
	2. DMA2
		![[Fig/OTA/Pasted image 20251130233139.png]]


## DMA 传输种类

| 类型      | 描述                         |
| ------- | -------------------------- |
| 外设 → 内存 | ADC、串口接收                   |
| 内存 → 外设 | 串口发送、DAC                   |
| 内存 → 内存 | 快速内存复制，必须设置 `M2M = ENABLE` |

`DMA_DIR_PeripheralSRC`：外设 → 内存（如 ADC）  
`DMA_DIR_PeripheralDST`：内存 → 外设（如 USART 发送）

| 传输模型                         | DMA_DIR               | DMA_M2M   | 说明                 |
| ---------------------------- | --------------------- | --------- | ------------------ |
| **外设 → 内存**（ADC、USART RX 等）  | DMA_DIR_PeripheralSRC | ❌ Disable | 数据源是外设寄存器，必须依外设触发  |
| **内存 → 外设**（USART TX、SPI TX） | DMA_DIR_PeripheralDST | ❌ Disable | 数据目标是外设寄存器，必须依外设触发 |
| **内存 → 内存**（M2M copy）        | 任意                    | ✔ Enable  | 两地址都是内存，与外设无关      |

## DMA_InitTypeDef初始化结构体
		
```cpp
void DMA_MTM_Config(void)
{
	/*定义DMA初始化结构体*/
	DMA_InitTypeDef DMA_InitStruct;
	/*打开DMA外设时钟，DMA挂载在AHB总线时钟上*/
	RCC_AHBPeriphClockCmd(DMA_MTM_CLK, ENABLE);
	/*设置外设基地址，即源地址*/
	DMA_InitStruct.DMA_PeripheralBaseAddr =	(uint32_t) aSRC_Const_Buffer;
	/*设置目标存储器地址，即目标地址*/
	DMA_InitStruct.DMA_MemoryBaseAddr	  =	(uint32_t) aDST_Buffer;
	/*配置传输方向，这里为止解决的是从哪里来，到哪里去*/
	DMA_InitStruct.DMA_DIR 				  =	DMA_DIR_PeripheralSRC;
	/*设置要传输的数量*/
	// DMA 要搬运的 **数据个数**- 单位取决于 DataSize（Byte/HalfWord/Word）
	// DMA_BufferSize 减到 0 → 硬件置位内部 **TC（Transfer Complete）** 标志；
	DMA_InitStruct.DMA_BufferSize		  =	BUFFER_SIZE;
	/*设置外设的字宽*/
	DMA_InitStruct.DMA_PeripheralDataSize =	DMA_PeripheralDataSize_Word;
	/*外设地址是否自增*/
	DMA_InitStruct.DMA_PeripheralInc	  =	DMA_PeripheralInc_Enable;
	/*存储器地址是否自增*/
	DMA_InitStruct.DMA_MemoryInc		  =	DMA_MemoryInc_Enable;
	/*存储器数据宽度 - 目标端的数据宽度 - 设置为 32bit，与源端一致 → 正确。*/
	DMA_InitStruct.DMA_MemoryDataSize	  =	DMA_MemoryDataSize_Word;
	/*设置传输模式是一次性传输，还是循环传输*/
	// Circular：传输完自动返回起点（常用于 ADC 连续传输）
	// 重新回到起始地址
	DMA_InitStruct.DMA_Mode				  =	DMA_Mode_Normal;
	/*设置优先级*/
	DMA_InitStruct.DMA_Priority			  =	DMA_Priority_High;
	/*开启存储器到存储器传输*/
	// 存储器到存储器必须开启，否则 M2M 复制无法进行 - 开启后 DMA 不依赖外设触发，可自行运行。
	DMA_InitStruct.DMA_M2M				  =	DMA_M2M_Enable;
	/*DMA初始化*/
	DMA_Init(DMA_MTM_CHANNEL, &DMA_InitStruct);
	/*打开DMA通道*/
	DMA_Cmd(DMA_MTM_CHANNEL,  ENABLE);
 }
```


## DMA 中断

> **DMA 中断需要：DMA_ITConfig 开启控制位 + NVIC_Init 开启 NVIC 通道**  
> **TC/HT/TE 三种中断分别对应 TCIFx、HTIFx、TEIFx 标志位。**
### 标志位说明

| 中断事件       | 标志位（状态寄存器）                         | 控制位（使能寄存器）                             | 说明                     |
| ---------- | ---------------------------------- | -------------------------------------- | ---------------------- |
| **传输完成中断** | `TCIFx`，**不会自动触发中断**，除非 `TCIE` 被使能 | `TCIE`，控制 DMA 是否在 `TCIFx` 置位时 **触发中断** | 完成全部缓冲区传输时触发           |
| **半传输中断**  | `HTIFx`                            | `HTIE`                                 | 完成一半传输时触发（常用于双缓冲/流式处理） |
| **传输错误中断** | `TEIFx`                            | `TEIE`                                 | DMA 发现错误时触发（优先级最高）     |
|            |                                    |                                        |                        |

1. 什么时候用半传输中断（HTIE）？
	- 双缓冲数据处理
	- 串口长帧接收（DMA + HT 中断）
	- 音频数据流连续采样（ADC + DMA + HT）
	- 处理“前半段时处理前半缓冲，后半段处理后半缓冲”
	如果只需要一次性传输或环形传输→通常只开 **TCIE**。

2. 什么时候用传输错误中断（TEIE）？
	几乎所有项目都建议开启：`DMA_IT_TE`，可以发现：
	- 地址越界
	- 配置错误
	- DMA 未正确使能外设触发

### 启用 DMA 中断的流程

DMA 中断必须 **同时开启两个地方**：
1. DMA 本身的中断控制位
	```cpp
	DMA_ITConfig(DMA_ChannelX, DMA_IT_TC, ENABLE);  // 传输完成中断
	DMA_ITConfig(DMA_ChannelX, DMA_IT_HT, ENABLE);  // 半传输中断 
	DMA_ITConfig(DMA_ChannelX, DMA_IT_TE, ENABLE);  // 传输错误中断
	```
	示例：
	`DMA_ITConfig(DMA1_Channel1, DMA_IT_TC | DMA_IT_TE, ENABLE);`

2. NVIC 中断使能
	DMA 中断属于 Cortex-M NVIC，需要配置：
	```cpp
	NVIC_InitTypeDef NVIC_InitStructure; // 定义 NVIC 初始化结构体
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn; // 选择中断源：DMA1 通道1
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // 抢占优先级 1（0~15，数值越小越优先）
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; // 子优先级 1（0~15，同级抢占下再细分）
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; // 使能该中断通道
	NVIC_Init(&NVIC_InitStructure); // 写入 NVIC 寄存器，配置生效
	```
---

### DMA 中断服务函数（ISR）

根据通道不同，ISR 名字如下：
```cpp
void DMA1_Channel1_IRQHandler(void);   // 仅 Channel1
void DMA1_Channel2_IRQHandler(void);   // Channel2 + Channel3 共用
void DMA1_Channel3_IRQHandler(void);   // 同上，实际进同一函数
void DMA1_Channel4_IRQHandler(void);   // Channel4 + Channel5 共用
void DMA1_Channel5_IRQHandler(void);   // 同上
void DMA1_Channel6_IRQHandler(void);   // Channel6 + Channel7 共用
void DMA1_Channel7_IRQHandler(void);   // 同上
```

模板：

```cpp
void DMA1_Channel1_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_IT_TC1)) { // 确认是 Channel1 传输完成
        DMA_ClearITPendingBit(DMA1_IT_TC1);
        // 传输完成处理
    }

    if (DMA_GetITStatus(DMA1_IT_HT1)) {
        DMA_ClearITPendingBit(DMA1_IT_HT1);
        // 半传输处理
    }

    if (DMA_GetITStatus(DMA1_IT_TE1)) {
        DMA_ClearITPendingBit(DMA1_IT_TE1);
        // 错误处理
    }
}
```

### 接口函数
```cpp
// 使能DMA时钟  
RCC_AHBPeriphClockCmd(); 
// 初始化DMA通道参数  
DMA_Init();
// 使能串口DMA发送,串口DMA使能函数：  
USART_DMACmd();
// 使能DMA1通道，启动传输
DMA_Cmd();
// 查询DMA传输状态  
DMA_GetFlagStatus();
// 获取/设置通道当前剩余数据量：  
// DMA 初始化时设置的总数据量为 `DMA_BufferSize`， 传输过程中，这个值会递减
// 如果 DMA 已搬运 100 字节，remaining = 156
DMA_GetCurrDataCounter(); 
DMA_SetCurrDataCounter(); // /设置通道当前剩余数据量
```
## OTA 代码
```cpp
void User_DMA_Init(void)
{
    if (DMAx == DMA1) RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); // 开启DMA时钟
    else if (DMAx == DMA2) RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE); // 开启DMA时钟

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
```

# I2C

## I2C 时序要求
1. I²C 空闲状态必须是 SCL=1、SDA=1。
2. I²C 协议规定，一个 bit 的结束必须将 SCL 拉回低电平。
3. ACK 位也是一个 bit，因此也必须按同样规律：SCL_L → SCL_H → SCL_L
4. 只能在 SCL 为低时改变 SDA
5. 在 SCL 高电平稳定时读取
	```cpp
	SCL:  _|‾|_|‾|_|‾|_|‾|_ ...
	SDA:    ↑   ↑   ↑   ↑
	       slave 输出数据位
	```

## OTA 代码

```cpp
#ifndef __USERIIC_H__
#define __USERIIC_H__

#define GPIOx GPIOB
#define IIC_SCL_Pin GPIO_Pin_6
#define IIC_SDA_Pin GPIO_Pin_7

#define I2C_DELAY    2
static int16_t TIMEOUT = 1000;

#define IIC_SCL_H GPIO_SetBits(GPIOx, IIC_SCL_Pin)
#define IIC_SCL_L GPIO_ResetBits(GPIOx, IIC_SCL_Pin)
#define IIC_SDA_H GPIO_SetBits(GPIOx, IIC_SDA_Pin)
#define IIC_SDA_L GPIO_ResetBits(GPIOx, IIC_SDA_Pin)

#define IIC_SDA_READ GPIO_ReadInputDataBit(GPIOx, IIC_SDA_Pin)

void UserIICInit(void);
void IICStart(void);
void IICStop(void);
void IICAck(void);
uint8_t IICWaitACk(void);
void IICSendByte(uint8_t Byte);
uint8_t IICReceiveByte(uint8_t Ack);

#endif /* __USERI2C_H__ */

```

```cpp
#include "stm32f10x.h"
#include "UserIIC.h"
#include "Delay.h"

/*
SCL:  _|‾|_|‾|_|‾|_|‾|_ ...
SDA:    ↑   ↑   ↑   ↑
       slave 输出数据位

// I²C 空闲状态必须是 SCL=1、SDA=1。
// I²C 协议规定，一个 bit 的结束必须将 SCL 拉回低电平。
// ACK 位也是一个 bit，因此也必须按同样规律：SCL_L → SCL_H → SCL_L。
// 高位先发送
*/

void IICStart(void)
{
    IIC_SDA_H;
    IIC_SCL_H;
    Delay_us(I2C_DELAY);
    IIC_SDA_L;
    Delay_us(I2C_DELAY);
    IIC_SCL_L;
}

void IICStop(void)
{
    IIC_SCL_L;
    IIC_SDA_L;
    Delay_us(I2C_DELAY);
    IIC_SCL_H;
    Delay_us(I2C_DELAY);
    IIC_SDA_H;
    Delay_us(I2C_DELAY);
}

// 高位先发送
void IICSendByte(uint8_t Byte)
{
    for (int8_t i = 7; i >= 0; i--)
    {
        IIC_SCL_L;
        Delay_us(I2C_DELAY);
        if (Byte & (1 << i)){ // 1左移7位：0x 1000 0000
            IIC_SDA_H; // 注意分号
        } 
        else{
            IIC_SDA_L; 
        }

        Delay_us(I2C_DELAY);
        IIC_SCL_H;
        Delay_us(I2C_DELAY);
    }
    IIC_SCL_L; // 结束后拉低 SCL
    IIC_SDA_H; // 释放 SDA 恢复空闲状态
}

uint8_t IICReceiveByte(uint8_t Ack)
{
    uint8_t Byte = 0;
    for (int8_t i = 7; i >= 0; i--)
    {
        IIC_SCL_L;
        Delay_us(I2C_DELAY);
        IIC_SCL_H;
        Delay_us(I2C_DELAY);
        if (IIC_SDA_READ) Byte |= (1 << i);
    }
    IIC_SCL_L; 
    Delay_us(I2C_DELAY);

    if (Ack){
        IIC_SDA_L;  // 发送应答
        IIC_SCL_H; // 拉高 SCL 以通知从机
        Delay_us(I2C_DELAY); //确保稳定，下面需要释放 SDA
        IIC_SCL_L; // 先拉低避免发送停止位
        IIC_SDA_H; // 释放 SDA 恢复空闲状态
    }else{
        IIC_SDA_H;
        IIC_SCL_H;
        Delay_us(I2C_DELAY);
        IIC_SCL_L;
        Delay_us(I2C_DELAY);
    }
    return Byte;
}

uint8_t IICWaitACk(void)
{
    // IIC_SDA_H; // 释放 SDA，准备接收从机应答
    // Delay_us(I2C_DELAY);
    // IIC_SCL_H; // 拉高 SCL，通知从机发送应答
    // Delay_us(I2C_DELAY);
    int16_t TIMEOUT_FAIL = TIMEOUT;
    do{
        TIMEOUT_FAIL--;
    } while (IIC_SDA_READ && (TIMEOUT_FAIL > 0)); // 等待从机拉低 SDA，表示应答

    if (TIMEOUT_FAIL < 0)  return 1; // 超时，认为从机无应答

    IIC_SCL_H; // 拉高确保是稳定的SDA低电平
    Delay_us(I2C_DELAY);
    if(IIC_SDA_READ) return 2; // 再次检查SDA，确保是稳定的低电平
    IIC_SCL_L; // 拉低SCL，完成时序
    Delay_us(I2C_DELAY);
    return 0; // 成功接收从机应答
}

void UserIICInit(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (GPIOx == GPIOA)
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    else if (GPIOx == GPIOB)
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    else if (GPIOx == GPIOC)
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStruct.GPIO_Pin = IIC_SCL_Pin | IIC_SDA_Pin;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOx, &GPIO_InitStruct);

    IIC_SCL_H;
    IIC_SDA_H;
}

```


# AT24C256

## 地址
- 前 4 位固定为 1010（十六进制 0xA，这是 _所有 I²C EEPROM 的厂商规定（行业标准）_，表示这是一个 EEPROM 器件。
- 接下来三位 A2 A1 A0 是“硬件器件地址”，**如果 A2/A1/A0 不接，芯片会自动认为它是 0（内部下拉）**。
- 第 8 位是 Read / Write 选择位：R/W = 0   → 写操作， R/W = 1   → 读操作

如果 A2=A1=A0 都接 GND：
1. 写地址：1010 0000 = 0xA0
2. 读地址：1010 0001 = 0xA1

WP 引脚是写保护（Write Protect）
- WP = 1 → 整个 EEPROM 写保护，不能写
- WP = 0 → 可以正常读写

![[Fig/OTA/Pasted image 20251202113757.png|628]]

## 写

###### 字节 
AT24C256 是 32KB（地址 0x0000 ~ 0x7FFF），所以地址是 **16 位**，需要发送两个字节

流程示意
```cpp
START
↓
发送 8-bit 设备地址（写，例如 0xA0）
↓
EEPROM 回 ACK（0）
↓
发送地址高字节（AH）
↓
EEPROM ACK
↓
发送地址低字节（AL）
↓
EEPROM ACK
↓
发送一个数据字节（D0）
↓
EEPROM ACK
↓
STOP（结束写序列）
↓
EEPROM 内部进行写周期 tWR (5ms 左右)

```

- 芯片内部擦写 Flash 单元
- 整个芯片“关闭输入”，不会响应新的指令

![[Fig/OTA/Pasted image 20251202120119.png]]
###### 页
EEPROM 的存储空间按“页”划分，**每页 64 字节**（因为低 6 位地址正好 0-63）。
Page Write 与 Byte Write 区别只有一点：
**写完第一个数据后不发送 STOP，而是继续发送更多数据（最多 64 个）。**

示意
```cpp
START
设备地址(A0) → ACK
地址高字节 → ACK
地址低字节 → ACK

数据1 → ACK
数据2 → ACK
…
数据64 → ACK

STOP 结束写序列
EEPROM 内部写64字节
```

###### 地址自动递增（Auto-increment）+ 页回卷（roll-over）
因为低 6 位可以表示 0~63 → 对应 64 字节，高地址字节保持不变，所以不会跨页。
```cpp
Address = HHHHHHHH LLLLLLxx
                       ↑↑↑↑↑↑
                     自动递增的部分（0~63）
```
如果从一个页的末尾地址开始写，例如：
地址 0x003F  (页的最后一个字节)
所有连续超过 64 字节的写入，会**覆盖本页的前面的数据**。

![[Fig/OTA/Pasted image 20251202120148.png]]
###### ACK 轮询
写入期间（5ms 内）：
- EEPROM 不接受任何命令
- 不会 ACK 器件地址
循环发送器件地址（写模式）
- 如果 EEPROM 还在写，它不会 ACK（即 SDA=1）。  
- 当写完后，它会 ACK（拉低 SDA）。

###### 注意
- **不要在 page write 中发送超过 64 字节**，因为会覆盖本页。
- **写操作后执行 ACK-polling**：写操作触发设备内部写周期（tWR），在此期间设备不会 ACK。通过轮询设备地址直到 ACK 即可检测写完成，避免盲等固定延时。
- **Read 操作的 auto-increment**：读（sequential read）也会在地址自动递增，且读取到**地址末（不是页末）** 会继续回卷到**芯片首地址（不是页）**（same page wrap）——注意读也遵循相同的低 6 位回卷规则（不过常见用法是跨页读时先设置地址然后连续读多个字节，芯片会按同样的低位回卷行为返回数据）。

## 读

I²C EEPROM（ AT24C256 系列）共有三种读操作：

1. **Current Address Read（当前地址读）**
    
2. **Random Read（随机地址读）** —— 实际上由 Dummy Write + Current Read 构成
    
3. **Sequential Read（顺序读）**
    

核心机制是：
- 读出数据后，只要 MCU 继续发 ACK，EEPROM 就 **自动递增地址**（Auto-Increment）继续输出下一字节。
- 若地址达到最大（如 0x7FFF），则 **回卷到 0x0000**（Roll-Over）。
- **当且仅当达到址达最大值才会回卷** ，页末是不会回卷的。
- **所有读操作（Current / Random / Sequential Read）在地址溢出时，回卷（roll-over）的目标都是整个 EEPROM 的最起始地址（0x0000）**。


###### Current Address Read（当前地址读）
EEPROM 内部维护一个 **地址计数器（address pointer）**。
- 是上一次操作所使用的地址 **+ 1**
- 如果电源保持不断电，此地址一直有效。

时序过程
1. MCU 发 START
2. MCU 发 **设备地址 + R（读）**
3. EEPROM ACK
4. EEPROM 输出计数器当前指向的地址内的数据
5. MCU 发 NACK + STOP（表示不再读更多）

自动递增 + 回卷
- 读完 1 字节后，EEPROM 自动执行：`address++`
- 若到**最后地址**（如 0x7FFF）：
    - 再++ → 回卷为 **0x0000**

###### Random Read（随机地址读）
“先写地址，但不写数据，再读”

时序步骤
1. MCU 发 START
2. MCU 发设备地址 + **W（写）**
3. EEPROM ACK
4. MCU 发送 **地址高字节**
5. EEPROM ACK
6. MCU 发送 **地址低字节**
7. EEPROM ACK  
    （这里就是所谓的 “dummy write”，只是写地址，不写数据）
8. MCU 发 **重复起始（Repeated START）**
9. MCU 发设备地址 + **R（读）**
10. EEPROM ACK
11. EEPROM 输出 “刚才指定的地址” 的数据
12. MCU NACK + STOP 结束

自动递增
- EEPROM 输出 1 字节后会将内部地址执行：`address++`
- 如果继续发 ACK，而不是 NACK，那么就进入顺序读（Sequential Read）。

###### Sequential Read（顺序读）
顺序读等于： Current Address Read / Random Read 的拓展版

只要 MCU 对每个字节发送 **ACK（0）**，EEPROM 就继续：
- 输出下一字节 → 地址自动递增

时序
- MCU 收到 EEPROM 发的 1 字节数据
- MCU 发 ACK
- EEPROM 自动递增地址计数器 → 输出下一字节
- MCU 再 ACK
- 无限循环直到 MCU 发 NACK + STOP

地址回卷（roll-over）
1. 当 EEPROM **地址达到最大值**，例如 24LC256（32KB）→ 最后地址 0x7FFF
2. 当继续 auto-increment：`0x7FFF + 1 → 0x0000`
3. 顺序读会从 EEPROM 开头继续读（循环读整个芯片）。
> 注意：  
> 回卷不是进入下一页，而是直接回到 **整个芯片的开始**，回卷只会发生在读到**地址达到最大值**。

###### 总结

| 类型                   | 是否需要 dummy write? | 地址自动递增  | 回卷方式  | 使用场景       |
| -------------------- | ----------------- | ------- | ----- | ---------- |
| Current Address Read | ❌                 | ✔       | 芯片级回卷 | 连续读取，不指定地址 |
| Random Read          | ✔（只写地址）           | ✔       | 芯片级回卷 | 从任意地址读取    |
| Sequential Read      | ✔（隐式/显式）          | ✔✔✔持续递增 | 芯片级回卷 | 连续大量读取     |

![[Fig/OTA/Pasted image 20251202131959.png]]
![[Fig/OTA/Pasted image 20251202132009.png|766]]
![[Fig/OTA/Pasted image 20251202132022.png]]


## OTA 代码
```cpp
#ifndef __AT24C256_H__
#define __AT24C256_H__

#include "UserIIC.h"
#include "stm32f10x.h"

// 器件地址为 1010  A2 A1 A0 X
/// A2 A1 A0 接GND 时，器件地址为 1010 000X
#define AT24C256_ADDR_WRITE   0xA0
#define AT24C256_ADDR_READ    0xA1
#define AT24C256_PAGE_SIZE 64  // 每页64字节
#define ERR_OK 0
#define ERR_NAK -1
#define ERR_TIMEOUT -2
#define ERR_BUSY -3
#define ERR_ADD_NAK -4
#define ERR_MEM_NAK -5

int8_t AT24C256_WriteByte(uint16_t memAddr, uint8_t byte);
int8_t AT24C256_WritePage(uint16_t memAddr, uint8_t *bytes, uint8_t writeLen);
int8_t AT24C256_ReadBytes(uint16_t memAddr, uint8_t *bytes, uint8_t readLen);

#endif /* __AT24C256_H__ */

```

```cpp
#include "stm32f10x.h"
#include "AT24C256.h"
#include "UserIIC.h"


int8_t AT24C256_WriteByte(uint16_t memAddr, uint8_t byte)
{
    IICStart();
    IICSendByte(AT24C256_ADDR_WRITE);
    if (!IICWaitACk())
    {
        IICStop();
        return ERR_ADD_NAK;
    }
    IICSendByte(memAddr >> 8);
    if (!IICWaitACk())
    {
        IICStop();
        return ERR_MEM_NAK;
    }
    IICSendByte(memAddr & 0xFF);
    if (!IICWaitACk())
    {
        IICStop();
        return ERR_MEM_NAK;
    }
    IICSendByte(byte);
    if (!IICWaitACk())
    {
        IICStop();
        return ERR_NAK;
    }
    IICStop();
    return 0;
}

int8_t AT24C256_WritePage(uint16_t memAddr, uint8_t *bytes, uint8_t writeLen)
{
    IICStart();
    IICSendByte(AT24C256_ADDR_WRITE);
    if (!IICWaitACk())
    {
        IICStop();
        return ERR_ADD_NAK;
    }
    IICSendByte(memAddr >> 8);
    if (!IICWaitACk())
    {
        IICStop();
        return ERR_MEM_NAK;
    }
    IICSendByte(memAddr & 0xFF);
    if (!IICWaitACk())
    {
        IICStop();
        return ERR_MEM_NAK;
    }

    for (uint8_t i = 0; i < writeLen; i++)
    {
        IICSendByte(bytes[i]);
        if (!IICWaitACk())
        {
            IICStop();
            return ERR_NAK; 
        }
    }
    IICStop();
    return 0;
}

int8_t AT24C256_ReadBytes(uint16_t memAddr, uint8_t *bytes, uint8_t readLen)
{
    IICStart();
    IICSendByte(AT24C256_ADDR_WRITE);
    if (!IICWaitACk())
    {
        IICStop();
        return ERR_ADD_NAK;
    }
    IICSendByte(memAddr >> 8);
    if (!IICWaitACk())
    {
        IICStop();
        return ERR_MEM_NAK;
    }
    IICSendByte(memAddr & 0xFF);
    if (!IICWaitACk())
    {
        IICStop();
        return ERR_MEM_NAK;
    }

    IICStart(); // 完成dummy write, 然后再次发送起始信号
    IICSendByte(AT24C256_ADDR_READ);
    if (!IICWaitACk())
    {
        IICStop();
        return ERR_ADD_NAK;
    }
    for (uint8_t i = 0; i < readLen; i++)
    {
        bytes[i] = IICReceiveByte(i == readLen - 1 ? 1 : 0); // NAK on last byte
    }
    IICStop();
    return 0;
}



```
