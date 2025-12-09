#include "stm32f10x.h"
#include "UserSPI.h"

void User_SPI_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (GPIOx_SPI == GPIOA)
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    else if (GPIOx_SPI == GPIOB)
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    else if (GPIOx_SPI == GPIOC)
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    else if (GPIOx_SPI == GPIOD)
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);


    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Pin = SPI_SCK_Pin | SPI_MOSI_Pin;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOx_SPI, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStruct.GPIO_Pin = SPI_MISO_Pin;
    GPIO_Init(GPIOx_SPI, &GPIO_InitStruct);

    if (SPIx == SPI1)
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
    else if (SPIx == SPI2)
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

    SPI_InitTypeDef SPI_InitStruct = {0};
    // SPI_Direction参数：
    // SPI_Direction_2Lines_FullDuplex：全双工模式，同时发送和接收数据
    // SPI_Direction_2Lines_RxOnly：全双工模式，只接收数据
    // SPI_Direction_1Line_RxOnly：单工模式，只接收数据
    // SPI_Direction_1Line_TxOnly：单工模式，只发送数据
    SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    // SPI_Mode参数：
    // SPI_Mode_Master：主模式
    // SPI_Mode_Slave：从模式
    SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
    // SPI_DataSize参数：
    // SPI_DataSize_8b：8位数据大小
    // SPI_DataSize_16b：16位数据大小
    SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
    // SPI_CPOL参数：
    // SPI_CPOL_Low：时钟极性为低电平
    // SPI_CPOL_High：时钟极性为高电平
    // SPI_CPHA参数：
    // SPI_CPHA_1Edge：时钟相位为第1个时钟沿
    // SPI_CPHA_2Edge：时钟相位为第2个时钟沿
    SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
    // // OR
    // SPI_InitStruct.SPI_CPOL = SPI_CPOL_High;
    // SPI_InitStruct.SPI_CPHA = SPI_CPHA_2Edge;
    // SPI_NSS参数：
    // SPI_NSS_Soft：软件NSS管理
    // SPI_NSS_Hard：硬件NSS管理
    SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
    // SPI_BaudRatePrescaler参数：
    // SPI_BaudRatePrescaler_2：波特率预分频器为2
    // SPI_BaudRatePrescaler_4：波特率预分频器为4
    // SPI_BaudRatePrescaler_8：波特率预分频器为8
    // SPI_BaudRatePrescaler_16：波特率预分频器为16
    // SPI_BaudRatePrescaler_32：波特率预分频器为32
    // SPI_BaudRatePrescaler_64：波特率预分频器为64
    // SPI_BaudRatePrescaler_128：波特率预分频器为128
    // SPI_BaudRatePrescaler_256：波特率预分频器为256
    SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
    // SPI_FirstBit参数：
    // SPI_FirstBit_MSB：先发送或接收MSB位
    // SPI_FirstBit_LSB：先发送或接收LSB位
    SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
    // SPI_CRCPolynomial参数：
    // 7：CRC多项式为7
    // 10：CRC多项式为10
    SPI_InitStruct.SPI_CRCPolynomial = 7;
    SPI_Init(SPIx, &SPI_InitStruct);
    SPI_Cmd(SPIx, ENABLE);
}

uint8_t SPIRWByte(uint8_t byte)
{
    // TXE: 发送缓冲区：RESET(0), DR未空，不可写。SET(1), 可以写入数据
    while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET);

    SPI_I2S_SendData(SPIx, byte);

    // RXNE: 接收缓冲区: RESET(0), 接收缓冲区为空，不可读。SET(1), 可以读取数据
    while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET);

    return SPI_I2S_ReceiveData(SPIx);
}

void SPIWBytes(uint8_t *wBytes, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)
        SPIRWByte(wBytes[i]);
}

void SPIRBytes(uint8_t *rBytes, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)
        rBytes[i] = SPIRWByte(SPI_WRITE_NULL);
}


