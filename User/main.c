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
#include "BOOT.h"
#include "main.h"

// W25Q64   存放服务器发送的 OTA 更新文件
// AT24C256 存放 OTA 标志位和文件长度
// FLASH    运行代码的起始地址为 STM_APP_BASE_ADDR

extern uint32_t i;
void USART1_IRQHandler(void);

ota_info_t g_ota_info = {OTA_FLAG_INVALID};

boot_update_t g_boot_update = {0};
boot_flag_t g_boot_flag = BOOT_FLAG_INVALID;

int main(void)
{
    User_USART_Init();
    User_IIC_Init();
    User_SPI_Init();
    W25Q64_Init();
    OLED_Init();
    OLED_ShowString(1, 1, "BootLoader!");

    printf("\r\n\r\nBootLoader Start!\r\n");
    BootLoader();

    while (1)
    {
        if (UxCB.URxDataOUT != UxCB.URxDataIN) // 有数据可读
        {
            BootLoader_Event(UxCB.URxDataOUT->start, UxCB.URxDataOUT->end - UxCB.URxDataOUT->start + 1);
            UxCB.URxDataOUT++; // 指向下一个段的地址
            if (UxCB.URxDataOUT == UxCB.URxDataEND)
                UxCB.URxDataOUT = &UxCB.URxDataPtr[0]; // 移动到第一个段的地址
        }

        if ((g_boot_flag & BOOT_FLAG_XMODEM_IAP) || (g_boot_flag & BOOT_FLAG_XMODEM_2_EXT_FLASH))
        {
            // 通过Xmodem协议，串口IAP下载A区程序，请使用bin文件

            if (g_boot_update.Xmodem_timer > 100)
            {
                printf("C");
                g_boot_update.Xmodem_timer = 0; // 1s的间隔发送 C
                // g_boot_flag &= ~BOOT_FLAG_XMODEM_IAP;
            }
            g_boot_update.Xmodem_timer++;
        }
        Delay_ms(10);

        if (g_boot_flag & BOOT_FLAG_UPDATE_A)
        {
            // 更新应用程序
            // BootUpdateApp();
            printf("BootUpdateApp()\r\n");
            uint32_t block_num = g_boot_update.block_num;
            uint32_t file_len = g_ota_info.ota_file_len[block_num];
            if (file_len % 4 == 0)
            {
                printf("OTA update file length: %d bytes\r\n", file_len);
                // FLASH 每页 1024 字节 FLASH_PAGE_BYTE_NUM
                // W25Q64 每页 256 字节 W25Q64_PAGE_SIZE
                // W25Q64 每块(64*1024) 字节（64KB） W25Q64_BLOCK_SIZE,

                // 擦除 FLASH 所有页
                User_FLASH_ErasePage(STM_APP_BASE_PAGE_NUM, STM_APP_PAGE_NUM);
                for (i = 0; i < file_len / FLASH_PAGE_BYTE_NUM; i++)
                {
                    // 从 W25Q64 读取 FLASH_PAGE_BYTE_NUM 个字节的数据
                    W25Q64_Read(g_boot_update.update_buff, i * FLASH_PAGE_BYTE_NUM + block_num * W25Q64_BLOCK_SIZE, FLASH_PAGE_BYTE_NUM);
                    // 每次写入 一个页 = FLASH_PAGE_BYTE_NUM 个字节
                    User_FLASH_WriteFromPage(STM_APP_BASE_PAGE_NUM + i, (uint32_t *)g_boot_update.update_buff, FLASH_PAGE_BYTE_NUM);
                }
                if (file_len % FLASH_PAGE_BYTE_NUM != 0)
                {
                    // 不满足页写的字节数，需要单独处理
                    // 从 W25Q64 读取 file_len % FLASH_PAGE_BYTE_NUM 个字节的数据
                    W25Q64_Read(g_boot_update.update_buff, i * FLASH_PAGE_BYTE_NUM + block_num * W25Q64_BLOCK_SIZE, file_len % FLASH_PAGE_BYTE_NUM);
                    // 每次写入 file_len % FLASH_PAGE_BYTE_NUM 个字节
                    User_FLASH_WriteFromPage(STM_APP_BASE_PAGE_NUM + i, (uint32_t *)g_boot_update.update_buff, file_len % FLASH_PAGE_BYTE_NUM);
                }
                if (block_num == 0)
                {
                    g_ota_info.ota_flag = OTA_FLAG_INVALID; // 升级完成后，将标志位设为无效
                    AT24C256_WriteOtaInfo();                // 写入 OTA 标志位和文件长度
                }
                NVIC_SystemReset(); // 复位系统，按了一次复位键
            }
            else
            {
                printf("OTA update file bytes length is not multiple of 4.\r\n");
                g_boot_flag &= ~BOOT_FLAG_UPDATE_A;
            }
        }
    };
}

// TODO 修改为对应的USARTx中断处理函数
void USART1_IRQHandler(void)
{
    USARTx_IRQHandler();
}
