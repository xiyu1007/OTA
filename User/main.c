#include "stm32f10x.h" // 设备头文件
#include <stdio.h>
#include "Delay.h"
#include "OLED.h"
#include "UserIIC.h"
#include "UserUSART.h"
#include "AT24C256.h"
#include "W25Q64.h"
#include "UserSPI.h"
#include "FLASH.h"
#include "Log.h"
#include "main.h"

void USART1_IRQHandler(void);

ota_info_t g_ota_info = {OTA_FLAG_STATE_INVALID};
uint32_t i, j;

int main(void)
{
    User_USART_Init();
    User_IIC_Init();
    OLED_Init();
    W25Q64_Init();

    OLED_ShowString(1, 1, "HelloWorld!");

    ota_flag_state_t ota_flag = OTA_FLAG_STATE_VALID;
    printf("ota_flag: 0x%08X\n", ota_flag);
    AT24C256_WriteBytes(G_OTA_INFO_ADDR, (uint8_t *)&ota_flag, sizeof(ota_flag));
    Delay_ms(6);
    // AT24C256_ReadBytes(OTA_FLAG_STATE_VALID, (uint8_t *)&g_ota_info, sizeof(g_ota_info));
    AT24C256_ReadOtaInfo();
    printf("OTA_FLAG: 0x%08X\n", g_ota_info.ota_flag);

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

    };
}

// TODO 修改为对应的USARTx中断处理函数
void USART1_IRQHandler(void)
{
    USARTx_IRQHandler();
}
