#include "stm32f10x.h"
#include "W25Q64.h"
#include "UserSPI.h"


void W25Q64_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (W25Q64_SPI_CS_PORT == GPIOA)
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    else if (W25Q64_SPI_CS_PORT == GPIOB)
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    else if (W25Q64_SPI_CS_PORT == GPIOC)
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Pin = W25Q64_SPI_CS_PIN;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(W25Q64_SPI_CS_PORT, &GPIO_InitStruct);

    CS_DISABLE;
    User_SPI_Init();
}

void W25Q64_WriteBusy(void)
{
    uint8_t status;
    do
    {
        CS_ENABLE;
        SPIRWByte(W25Q64_CMD_RDSR1);
        status = SPIRWByte(W25Q64_WRITE_NULL);
        CS_DISABLE;
    } while (status & 0x01);
}

void W25Q64_WriteEnable(void)
{   W25Q64_WriteBusy();

    CS_ENABLE;
    SPIRWByte(W25Q64_CMD_WREN);
    CS_DISABLE;
}

void W25Q64_EraseBlock64K(uint8_t blockNum)
{
    uint8_t wCMD[4];
    wCMD[0] = W25Q64_CMD_ERASE_64KB;
    wCMD[1] = ((blockNum*W25Q64_BLOCK_SIZE) >> 16) & 0xFF;
    wCMD[2] = ((blockNum*W25Q64_BLOCK_SIZE) >> 8) & 0xFF;
    wCMD[3] = (blockNum*W25Q64_BLOCK_SIZE) & 0xFF;

    W25Q64_WriteBusy();
    W25Q64_WriteEnable();
    CS_ENABLE;
    SPIWBytes(wCMD, 4);
    CS_DISABLE;
    W25Q64_WriteBusy(); // query erase status
}

void W25Q64_PageWrite(uint8_t *wBuff, uint16_t pageNum)
{
    uint8_t wCMD[4];
    wCMD[0] = W25Q64_CMD_PAGE_PROG;
    wCMD[1] = ((pageNum*W25Q64_PAGE_SIZE) >> 16) & 0xFF;
    wCMD[2] = ((pageNum*W25Q64_PAGE_SIZE) >> 8) & 0xFF;
    wCMD[3] = (pageNum*W25Q64_PAGE_SIZE) & 0xFF;

    W25Q64_WriteBusy();
    W25Q64_WriteEnable();
    CS_ENABLE;
    SPIWBytes(wCMD, 4);
    SPIWBytes(wBuff, W25Q64_PAGE_SIZE);
    CS_DISABLE;
    
    W25Q64_WriteBusy(); // query erase status
}

void W25Q64_PageRead(uint8_t *rBuff, uint16_t pageNum)
{
    uint32_t addr = pageNum * W25Q64_PAGE_SIZE;
    uint8_t wCMD[4];
    wCMD[0] = W25Q64_CMD_READ;
    wCMD[1] = (addr >> 16) & 0xFF;
    wCMD[2] = (addr >> 8) & 0xFF;
    wCMD[3] = addr & 0xFF;
    
    W25Q64_WriteBusy();
    CS_ENABLE;
    SPIWBytes(wCMD, 4);
    SPIRBytes(rBuff, W25Q64_PAGE_SIZE);
    CS_DISABLE;
}


void W25Q64_Read(uint8_t *rBuff, uint32_t addr, uint32_t dataLen)
{
    uint8_t wCMD[4];
    wCMD[0] = W25Q64_CMD_READ;
    wCMD[1] = (addr >> 16) & 0xFF;
    wCMD[2] = (addr >> 8) & 0xFF;
    wCMD[3] = addr & 0xFF;
    
    W25Q64_WriteBusy();
    CS_ENABLE;
    SPIWBytes(wCMD, 4);
    SPIRBytes(rBuff, dataLen);
    CS_DISABLE;
}



