#include "stm32f10x.h" // 设备头文件
#include <stdio.h>
#include "Delay.h"
#include "OLED.h"
#include "UserIIC.h"
#include "UserUSART.h"
#include "AT24C256.h"
#include "W25Q64.h"
#include "UserSPI.h"

void printError(int err);
void USART1_IRQHandler(void);

uint32_t i, j;

int main(void)
{
    User_USART_Init();
    User_IIC_Init();
    OLED_Init();
    W25Q64_Init();

    // OLED_ShowString(1, 3, "HelloWorld!");
    OLED_Clear();

    printf("HelloWorld!\n");

    uint8_t wBuff[W25Q64_PAGE_SIZE];
    uint8_t rBuff[W25Q64_PAGE_SIZE];

    W25Q64_EraseBlock64K(0);

    for (i = 0; i < W25Q64_BLOCK_PAGE; i++)
    {
        for (j = 0; j < W25Q64_PAGE_SIZE; j++)
            wBuff[j] = i;
        W25Q64_PageWrite(wBuff, i);
    }

    for (i = 0; i < W25Q64_BLOCK_PAGE; i++)
    {
        W25Q64_PageRead(rBuff, i);
        for (j = 0; j < W25Q64_PAGE_SIZE; j++)
        {
            printf("addr %lu = %d\r\n",
                   (uint32_t)i * W25Q64_PAGE_SIZE + j,
                   rBuff[j]);
        }
    }

    while (1)
    {
        if (UxCB.URxDataOUT != UxCB.URxDataIN) // 有数据可读
        {
            printf("Received data %d bytes: ", UxCB.URxDataOUT->end - UxCB.URxDataOUT->start + 1);
            for (uint8_t *ptr = UxCB.URxDataOUT->start; ptr <= UxCB.URxDataOUT->end; ptr++)
            {
                printf("%c", *ptr);
            }
            UxCB.URxDataOUT++; // 指向下一个段的地址
            if (UxCB.URxDataOUT == UxCB.URxDataEND)
                UxCB.URxDataOUT = &UxCB.URxDataPtr[0]; // 移动到第一个段的地址
        }
        // printf("Hello world. \r\n");
        Delay_ms(1000);
    };
}

// TODO 修改为对应的USARTx中断处理函数
void USART1_IRQHandler(void)
{
    USARTx_IRQHandler();
}

void printError(int err)
{
    switch (err)
    {
    case ERR_OK:
        break;
    case ERR_ADD_NAK:
        printf("ERR_ADD_NAK\r\n");
        break;
    case ERR_MEM_NAK:
        printf("ERR_MEM_NAK\r\n");
        break;
    case ERR_DATA_NAK:
        printf("ERR_DATA_NAK\r\n");
        break;
    default:
        printf("Unknown error code: %d\r\n", err);
        break;
    }
}
