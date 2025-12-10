#ifndef __FLASH_H__
#define __FLASH_H__

#include "stm32f10x.h"

#define FLASH_BASE_ADDR (uint32_t)(0x08000000)
#define FLASH_BASE_PTR ((__IO uint32_t*) (FLASH_BASE_ADDR))
#define FLASH_PAGE_BYTE_NUM (0x00000400) // 1 KB，1024 字节 = 256 字
#define FLASH_PAGE_WORD_NUM FLASH_PAGE_BYTE_NUM / sizeof(uint32_t) // 对于字读，每页的偏移(字)数 = 256
#define FLASH_PAGE_NUM 64 // 64 页，STM31F103CT6的页数（64k）

// FLASH_BASE_PTR[0]  -> 0x08000000
// FLASH_BASE_PTR[1]  -> 0x08000004
// FLASH_BASE_PTR[256]-> 0x08000400  (Page 1 起始)

// #define     __IO    volatile   

// 读取字
uint32_t FLASH_ReadWord(uint32_t addr);
// 读取半字
uint16_t FLASH_ReadHalfWord(uint32_t addr);
// 读取字节
uint8_t FLASH_ReadByte(uint32_t addr);

void UserFLASH_EraseAllPages(void);
void UserFLASH_ErasePage(uint8_t pageBegin, uint8_t pageNum);
// 从指定页开始写数据
void UserFLASH_WriteFromPage(uint32_t pageBegin, uint32_t *wdata, uint32_t wlen);
// 从指定地址开始写数据, 字节对齐
void UserFLASH_Write(uint32_t addr, uint32_t *wdata, uint32_t wlen);

#endif /* __FLASH_H__ */


