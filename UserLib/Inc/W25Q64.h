#ifndef __W25Q64_H__
#define __W25Q64_H__

#define W25Q64_SPI_CS_PORT GPIOA
#define W25Q64_SPI_CS_PIN GPIO_Pin_4

#define W25Q64_PAGE_SIZE 256
#define W25Q64_BLOCK_SIZE (64*1024)
#define W25Q64_BLOCK_PAGE (W25Q64_BLOCK_SIZE / W25Q64_PAGE_SIZE)

#define W25Q64_WRITE_NULL 0xFF // 不写字节指令

#define W25Q64_CMD_RDSR1 0x05 // 读取状态寄存器1指令
#define W25Q64_CMD_WREN 0x06 // 写使能指令
#define W25Q64_CMD_ERASE_64KB 0xD8 // 64KB 块擦除指令
#define W25Q64_CMD_PAGE_PROG 0x02 // 写页编程指令
#define W25Q64_CMD_READ 0x03 // 读数据指令

#define CS_ENABLE GPIO_ResetBits(W25Q64_SPI_CS_PORT, W25Q64_SPI_CS_PIN)
#define CS_DISABLE GPIO_SetBits(W25Q64_SPI_CS_PORT, W25Q64_SPI_CS_PIN)


void W25Q64_Init(void);
void W25Q64_EraseBlock64K(uint8_t blockNum);
void W25Q64_PageWrite(uint8_t *wBuff, uint16_t pageNum);
void W25Q64_PageRead(uint8_t *rBuff, uint16_t pageNum);
void W25Q64_Read(uint8_t *rBuff, uint32_t addr, uint32_t dataLen);

#endif /* __W25Q64_H__ */

