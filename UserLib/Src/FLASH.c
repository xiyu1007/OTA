#include "stm32f10x.h"
#include "FLASH.h"
#include "Log.h"

// 字读
uint32_t FLASH_ReadWord(uint32_t addr)
{
    return FLASH_BASE_PTR[addr];
}

uint16_t FLASH_ReadHalfWord(uint32_t addr)
{
    return ((uint16_t*)FLASH_BASE_PTR)[addr] & 0xFFFF;
}

// 字节读
uint8_t FLASH_ReadByte(uint32_t addr)
{
    return ((uint8_t*)FLASH_BASE_PTR)[addr] & 0xFF;
}

// typedef enum
// {
//   FLASH_BUSY = 1,
//   FLASH_ERROR_PG,
//   FLASH_ERROR_WRP,
//   FLASH_COMPLETE,
//   FLASH_TIMEOUT
// }FLASH_Status;
void User_FLASH_EraseAllPages(void)
{
    FLASH_Unlock();
    // 擦除页
    if (FLASH_EraseAllPages() != FLASH_COMPLETE)
        // 关闭FMC写操作
        FLASH_Lock();
}

void User_FLASH_ErasePage(uint8_t pageBegin, uint8_t pageNum)
{
    // FLASH_Status FLASH_ErasePage(uint32_t Page_Address);
    FLASH_Unlock();
    // ！！！是字节数而不是“字数偏移” -> 每页偏移 FLASH_PAGE_BYTE_NUM 1024 字节
    for(uint8_t i = 0; i < pageNum; i++){
        FLASH_ErasePage(FLASH_BASE_ADDR + (pageBegin + i) * FLASH_PAGE_BYTE_NUM);
    }
    // 关闭FMC写操作
    FLASH_Lock();
}

void User_FLASH_WriteFromPage(uint32_t pageBegin, uint32_t *wdata, uint32_t wlen)
{
    // 一次写4字节，地址每次增加4字节
    if (wlen % 4 != 0)
    {
        LOG("wlen must be multiple of 4");
        return;
    }
    FLASH_Unlock();
    for (uint32_t i = 0; i < wlen / 4; i++)
    {
        FLASH_ProgramWord(FLASH_BASE_ADDR + pageBegin * FLASH_PAGE_BYTE_NUM + (i * 4), wdata[i]);
    }
    // 关闭FMC写操作
    FLASH_Lock();
}

void User_FLASH_Write(uint32_t addr, uint32_t *wdata, uint32_t wlen)
{
    // 一次写4字节，地址每次增加4字节
    if (wlen % 4 != 0)
    {
        LOG("wlen must be multiple of 4");
        return;
    }
    FLASH_Unlock();
    for (uint32_t i = 0; i < wlen / 4; i++)
    {
        FLASH_ProgramWord(FLASH_BASE_ADDR + addr + (i * 4), wdata[i]);
    }
    // 关闭FMC写操作
    FLASH_Lock();
}
