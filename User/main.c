#include "stm32f10x.h" // 设备头文件
// #include <stdio.h>
// #include "Delay.h"
// #include "OLED.h"
// #include "UserIIC.h"
#include "UserUSART.h"
// #include "AT24C256.h"
// #include "W25Q64.h"
// #include "UserSPI.h"
#include "FLASH.h"
#include "Log.h"

// void printError(int err);
// void USART1_IRQHandler(void);

uint32_t i, j;

int main(void)
{
    User_USART_Init();
    // User_IIC_Init();
    // OLED_Init();
    // W25Q64_Init();

    printf("HelloWorld!\n");

    UserFLASH_ErasePage(60, 1);
    uint32_t wdata[3] = {0x9ABCDEF0, 0x01234567, 0x01234567}; // 共 3 字，12 字节
    UserFLASH_WriteFromPage(60, wdata, 12);
    // 下面读一个字
    printf("%08x\r\n", FLASH_ReadWord(60 * FLASH_PAGE_WORD_NUM)); // 字读则字偏移
    printf("%08x\r\n", *(uint32_t *)(FLASH_BASE_ADDR + 60 * FLASH_PAGE_BYTE_NUM)); // 字节读则字节偏移
    printf("%08x\r\n", FLASH_ReadWord(60 * FLASH_PAGE_WORD_NUM + 1)); // 字读则字偏移
    printf("%08x\r\n", *(uint32_t *)(FLASH_BASE_ADDR + 60 * FLASH_PAGE_BYTE_NUM + 4)); // 字节读则字节偏移



    while (1)
    {
        // if (UxCB.URxDataOUT != UxCB.URxDataIN) // 有数据可读
        // {
        //     printf("Received data %d bytes: ", UxCB.URxDataOUT->end - UxCB.URxDataOUT->start + 1);
        //     for (uint8_t *ptr = UxCB.URxDataOUT->start; ptr <= UxCB.URxDataOUT->end; ptr++)
        //     {
        //         printf("%c", *ptr);
        //     }
        //     UxCB.URxDataOUT++; // 指向下一个段的地址
        //     if (UxCB.URxDataOUT == UxCB.URxDataEND)
        //         UxCB.URxDataOUT = &UxCB.URxDataPtr[0]; // 移动到第一个段的地址
        // }

    };
}

// TODO 修改为对应的USARTx中断处理函数
// void USART1_IRQHandler(void)
// {
//     USARTx_IRQHandler();
// }

// void printError(int err)
// {
//     switch (err)
//     {
//     case ERR_OK:
//         break;
//     case ERR_ADD_NAK:
//         printf("ERR_ADD_NAK\r\n");
//         break;
//     case ERR_MEM_NAK:
//         printf("ERR_MEM_NAK\r\n");
//         break;
//     case ERR_DATA_NAK:
//         printf("ERR_DATA_NAK\r\n");
//         break;
//     default:
//         printf("Unknown error code: %d\r\n", err);
//         break;
//     }
// }
