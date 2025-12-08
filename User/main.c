#include "stm32f10x.h" // 设备头文件
#include <stdio.h>
#include "Delay.h"
#include "OLED.h"
#include "I2C.h"
#include "UserUSART.h"
#include "AT24C256.h"

extern UCB_CB UxCB;
volatile uint16_t  i;
uint16_t buffLen =64;

void printError(int err)
{
    switch (err)
    {
    case ERR_OK:
        printf("ERR_OK\r\n");
        break;
    case ERR_NAK:
        printf("ERR_NAK\r\n");
        break;
    case ERR_TIMEOUT:
        printf("ERR_TIMEOUT\r\n");
        break;
    case ERR_BUSY:
        printf("ERR_BUSY\r\n");
        break;
    case ERR_ADD_NAK:
        printf("ERR_ADD_NAK\r\n");
        break;
    case ERR_MEM_NAK:
        printf("ERR_MEM_NAK\r\n");
        break;
    default:
        printf("Unknown error code: %d\r\n", err);
        break;
    }
}

int main(void)
{
    User_USART_Init();
    UserIICInit();
    OLED_Init();



    // OLED_ShowString(1, 3, "HelloWorld!");
    OLED_Clear();

    printf("HelloWorld!\n");

    for (i = 0; i < buffLen; i++)
    {
        printError(AT24C256_WriteByte(i, (uint8_t)i));
        Delay_ms(6); // 等待写入完成
        OLED_ShowNum(1,1,(uint8_t)i,3);
    }
    uint8_t buf[buffLen];
    printError(AT24C256_ReadBytes(0, buf, buffLen));
    for (i = 0; i < buffLen; i++)
    {
        printf("地址%d= %d\r\n", i, buf[i]);
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
