#include "stm32f10x.h" // 设备头文件
#include <stdio.h>
#include "Delay.h"
#include "OLED.h"
#include "UserIIC.h"
#include "UserUSART.h"
#include "AT24C256.h"
#include "W25Q64.h"
#include "FLASH.h"
#include "Log.h"
#include "main.h"
#include "BOOT.h"
#include "CRC.h"


AppEntryFunc g_appEntry = NULL;
uint32_t timeout = 0, i;

void BootLoader(void)
{
    if (BootLoader_Enter(3) == 0){
        if (g_ota_info.ota_flag == OTA_FLAG_VALID){
            g_boot_flag |= BOOT_FLAG_UPDATE_A;
            g_boot_update.block_num = 0;
        }else{
            BootJumpToApp(STM_APP_BASE_ADDR);
            BootLoader_CommandLine();
        }
    }else{
        // 进入bootloader命令行
        // printf("Enter BootLoader Command Line\r\n");
        UxCB.URxDataOUT++; // 指向下一个段的地址
        if (UxCB.URxDataOUT == UxCB.URxDataEND)
            UxCB.URxDataOUT = &UxCB.URxDataPtr[0]; // 移动到第一个段的地址

        BootLoader_CommandLine();
    }


}

void BootLoader_Event(uint8_t *data, uint32_t len)
{
    if (len == 1 && data[0] == (char)0x03){
        g_boot_flag &= BOOT_FLAG_INVALID;
        printf("\r\n=>");   
        return;
    }
    if(g_boot_flag == BOOT_FLAG_INVALID && len == 1){
        BootLoader_EchoRxBuffer(1);
        switch(data[0]){
            case '0':
                // printf("\r\n");
                BootLoader_CommandLine();
                goto ECHO_OFF;
            case '1':
                BootJumpToApp(STM_APP_BASE_ADDR);
                break;
            case '2':
                User_FLASH_ErasePage(STM_APP_BASE_PAGE_NUM, STM_APP_PAGE_NUM);
                printf("App Erase Success!\r\n");
                break;
            case '3':
                // 通过Xmodem协议，串口IAP下载A区程序，请使用bin文件
                printf("Please send OTA update file (bin) via Xmodem protocol!\r\n");
                g_boot_flag |= (BOOT_FLAG_XMODEM_IAP | BOOT_FLAG_XMODEM_2_FLASH);
                g_boot_update.Xmodem_timer = 0;
                break;
            case '4':
                g_boot_flag |= BOOT_FLAG_SET_VERSION;
                // V1.0.0 2015/12/13 20:22
                printf("Please input OTA Version (V1.0.0 2015/12/13 20:22): \r\n");
                break;
            case '5':
                AT24C256_ReadOtaInfo();
                printf("Current OTA Version: %s\r\n", g_ota_info.ota_version);
                break;
            case '6':
                g_boot_flag |= BOOT_FLAG_2_FLASH;
                // 输入使用的块编号：1~5
                printf("Please input Block Number (1~5): \r\n");
                break;
            case '7':
                g_boot_flag |= BOOT_FLAG_EXT_FLASH_2_APP;
                printf("Please input Block Number need to Download (1~5): \r\n");
                break;
            case '8':
                printf("Begin to Restart!\r\n");
                Delay_ms(100);
                NVIC_SystemReset();
                
            default:
                break;  
                // printf("\r\n");
                // BootLoader_CommandLine();
                // break;
        }
        printf("=>");   
        ECHO_OFF:
        ;
    }else if(g_boot_flag & BOOT_FLAG_XMODEM_2_FLASH | (g_boot_flag & BOOT_FLAG_XMODEM_2_EXT_FLASH) ){
        // 校验Xmodem包, 每次介绍的实际数据大小是 (XMODEM_PACKET_SIZE)128字节, 即data的实际数据大小是128+3=131字节
        // g_boot_update.update_buff，用于接收满 FLASH_PAGE_BYTE_NUM （1024字节）后再写入FLASH
        // PER_FLASH_XMODEM_PACKET_SIZE 每个FLASH页可以接收的Xmodem包数量
        
        if( (len == 133) && (data[0] == 0x01)){
            if ( g_boot_flag & BOOT_FLAG_XMODEM_2_FLASH ){
                User_FLASH_ErasePage(STM_APP_BASE_PAGE_NUM, STM_APP_PAGE_NUM);
            }
            g_boot_flag &= ~ BOOT_FLAG_XMODEM_IAP;
            g_boot_update.packet_crc = CRC16_XMODEM(&data[3], XMODEM_PACKET_SIZE);
            if (g_boot_update.packet_crc == data[131]*256 + data[132]){
                g_boot_update.packet_num++; 
                memcpy(&g_boot_update.update_buff[((g_boot_update.packet_num-1) % PER_FLASH_XMODEM_PACKET_SIZE)*XMODEM_PACKET_SIZE]
                        , &data[3], XMODEM_PACKET_SIZE);
                
                // 校验完后，判断是否是填充完缓冲区update_buff的最后一包
                if (g_boot_update.packet_num % PER_FLASH_XMODEM_PACKET_SIZE == 0){
                    // 最后一包，校验完成，写入FLASH
                    if (g_boot_flag & BOOT_FLAG_XMODEM_2_EXT_FLASH){
                        for (i = 0; i < PER_W25Q64; i++){
                            // W25Q64_BLOCK_PAGE 每个块包含的页数
                           W25Q64_PageWrite(&g_boot_update.update_buff[i*W25Q64_PAGE_SIZE], 
                            g_boot_update.block_num * W25Q64_BLOCK_PAGE 
                            + ( (g_boot_update.packet_num / PER_FLASH_XMODEM_PACKET_SIZE -1 ) * PER_W25Q64 ) + i);
                        }
                    }else{
                        User_FLASH_WriteFromPage(STM_APP_BASE_PAGE_NUM + ( (g_boot_update.packet_num-1) / PER_FLASH_XMODEM_PACKET_SIZE), 
                        (uint32_t *)g_boot_update.update_buff, FLASH_PAGE_BYTE_NUM);
                    }
                    
                }
                printf("\x06");
            }else{
                printf("\x15");
            }
        }
        // 0x04：XMODEM发送完成
        if (len == 1 && data[0] == 0x04){
            printf("\x06");
            // 对于不满 FLASH_PAGE_BYTE_NUM的最后一包，需要写入FLASH
            if(g_boot_update.packet_num % PER_FLASH_XMODEM_PACKET_SIZE != 0){
                if (g_boot_flag & BOOT_FLAG_XMODEM_2_EXT_FLASH){
                    for (i = 0; i < PER_W25Q64; i++){
                        // W25Q64_BLOCK_PAGE 每个块包含的页数
                        W25Q64_PageWrite(&g_boot_update.update_buff[i*W25Q64_PAGE_SIZE], 
                        g_boot_update.block_num * W25Q64_BLOCK_PAGE 
                        + ( (g_boot_update.packet_num / PER_FLASH_XMODEM_PACKET_SIZE ) * PER_W25Q64 ) + i);
                    }
                    g_boot_flag &= ~BOOT_FLAG_XMODEM_2_EXT_FLASH;
                    g_ota_info.ota_file_len[g_boot_update.block_num] = g_boot_update.packet_num * XMODEM_PACKET_SIZE;
                    AT24C256_WriteOtaInfo();
                    Delay_ms(100);
                    printf("Success Download File to Block %d!\r\n", g_boot_update.block_num);
                    printf("=>");
                }else{
                     User_FLASH_WriteFromPage(STM_APP_BASE_PAGE_NUM + (g_boot_update.packet_num / PER_FLASH_XMODEM_PACKET_SIZE), 
                        (uint32_t *)g_boot_update.update_buff, g_boot_update.packet_num % PER_FLASH_XMODEM_PACKET_SIZE * XMODEM_PACKET_SIZE);
                    g_boot_flag &= ~BOOT_FLAG_XMODEM_2_FLASH;
                    printf("IAP update success, begin to restart!\r\n");
                    Delay_ms(100);
                    NVIC_SystemReset();
                }
            }

        }
    }else if(g_boot_flag & BOOT_FLAG_SET_VERSION){
       int temp;
       // V1.0.0 2015/12/13 20:22
       if (len == 23){
            if(sscanf((const char *)data, "V%1d.%1d.%1d %4d/%2d/%2d %2d:%2d",
                &temp, &temp, &temp,&temp, &temp, &temp,&temp, &temp) == 8)
            {
                memset(&g_ota_info.ota_version, 0, sizeof(g_ota_info.ota_version));
                memcpy(&g_ota_info.ota_version, data, 23);
                AT24C256_WriteOtaInfo();
                printf("Success Set OTA Version: %s\r\n", g_ota_info.ota_version);
                g_boot_flag &= ~BOOT_FLAG_SET_VERSION;
            }else
                printf("Invalid OTA Version Format!\r\n");
        }
        else{                
            printf("Invalid OTA Version Length!\r\n");
        }
    }else if(g_boot_flag & BOOT_FLAG_2_FLASH){
        BootLoader_EchoRxBuffer(1);
        if (len == 1){
            if(data[0] >= 0x31 && data[0] <= 0x35){ // 1对应的16进制数为0x31，5对应的16进制数为0x35
                g_boot_update.block_num = data[0] - 0x30; // 1~5
                g_boot_flag |= BOOT_FLAG_XMODEM_2_EXT_FLASH;
                g_boot_flag |= BOOT_FLAG_XMODEM_IAP;
                g_boot_update.Xmodem_timer = 0;
                g_boot_update.packet_num = 0;
                g_ota_info.ota_file_len[g_boot_update.block_num] = 0;
                W25Q64_EraseBlock64K(g_boot_update.block_num);
                // 通过Xmodem协议，串口下载A区程序，请使用bin文件, 每个块最大64KB。
                printf("Please send OTA File (Max %d Bytes, type bin) Length for Block %d: \r\n", W25Q64_BLOCK_SIZE, g_boot_update.block_num);
                g_boot_flag &= ~ BOOT_FLAG_2_FLASH;
            }
        }else{
            // 块编号错误
            printf("Invalid Block Number!\r\n");
        }
    }else if(g_boot_flag & BOOT_FLAG_EXT_FLASH_2_APP){
        BootLoader_EchoRxBuffer(1);
        if (len == 1){
            if(data[0] >= 0x31 && data[0] <= 0x35){ // 1对应的16进制数为0x31，5对应的16进制数为0x35
                g_boot_update.block_num = data[0] - 0x30; // 1~5
                g_boot_flag |= BOOT_FLAG_UPDATE_A;
                g_boot_flag &= ~ BOOT_FLAG_EXT_FLASH_2_APP;

            }
        }else{
            // 块编号错误
            printf("Invalid Block Number!\r\n");
        }
    }
}

void BootLoader_EchoRxBuffer(uint8_t echo)
{
    if(echo){
        for (uint8_t *ptr = UxCB.URxDataOUT->start; ptr <= UxCB.URxDataOUT->end; ptr++)
        {
            printf("%c", *ptr);
        }
    }
    printf("\r\n");
    // UxCB.URxDataOUT++; // 指向下一个段的地址
    // if (UxCB.URxDataOUT == UxCB.URxDataEND)
    //     UxCB.URxDataOUT = &UxCB.URxDataPtr[0]; // 移动到第一个段的地址
}

void BootLoader_CommandLine(void)
{
    printf("Enter BootLoader Command Line\r\n");
    printf(
        "==========================================\r\n"
        "[1] Enter App\r\n"
        "[2] Erase App\r\n"
        "[3] Update App (IAP)\r\n"
        "[4] Set OTA Version\r\n"
        "[5] Get OTA Version\r\n"
        "[6] Download App to Ext Flash\r\n"
        "[7] Download App from Ext Flash\r\n"
        "[8] Restart\r\n"
        "[0] Show Menu\r\n"
        "==========================================\r\n"
    );
    printf("=>");
}


uint8_t BootLoader_Enter(uint8_t timeout)
{
    // 在3秒内按下任意键进入 BootLoader
    printf("Press any key to enter BootLoader in %d seconds ...\r\n", timeout); 
    while(timeout--){
        if (URxBuffer[0] != 0 || URxBuffer[0] == '\r'){
            return 1;
        }
        Delay_ms(1000);
    }
    return 0;
}



/* 设置 MSP 的汇编函数 */
__asm void SetMainStackPointer(uint32_t stackAddr)
{
    MSR MSP, r0        ; MSP = r0
    BX  lr             ; return
}

/* 跳转到应用程序入口 */
void BootJumpToApp(uint32_t appBaseAddr)
{
    /* 1. 判断栈顶地址有效性（SRAM 范围） */
    uint32_t appStack = *(uint32_t *)appBaseAddr; // 从地址 appBaseAddr 取出应用程序栈顶地址
    uint32_t appResetHandler = *(uint32_t *)(appBaseAddr + 4);

    if (appStack < 0x20000000 || appStack > 0x20004FFF)
    {
        LOG("ERROR: Invalid application stack pointer: 0x%08X\r\n", appStack);
        return;
    }


    /* 2. 关闭所有中断 */
    __disable_irq();

    /* 3. 设置 MSP */
    SetMainStackPointer(appStack);

    /* 4. 设置向量表偏移 */
    // SCB->VTOR = appBaseAddr; 无效， APP加载后会被覆盖

    /* 5. 跳转到应用程序的 Reset_Handler */
    BootClearAppEntry();
    __enable_irq();  // 开启全局中断
    g_appEntry = (AppEntryFunc)appResetHandler;
    g_appEntry();
}

/* ===============================
   GPIOx -> RCC_APB2Periph_GPIOx 映射函数
   =============================== */
static uint32_t GPIOxToRCC(GPIO_TypeDef *GPIOx)
{
    if (GPIOx == GPIOA) return RCC_APB2Periph_GPIOA;
    if (GPIOx == GPIOB) return RCC_APB2Periph_GPIOB;
    if (GPIOx == GPIOC) return RCC_APB2Periph_GPIOC;
    if (GPIOx == GPIOD) return RCC_APB2Periph_GPIOD;
    if (GPIOx == GPIOE) return RCC_APB2Periph_GPIOE;
    return 0; // 无效
}

/**
 * @brief  清理外设状态，避免 Bootloader 跳转到应用程序后外设冲突
 * @note   清理完成后关闭对应 GPIO 时钟
 */
void BootClearAppEntry(void)
{
    uint32_t rcc_gpio;

    /* ===========================
       USART 清理
       =========================== */
    #ifdef GPIOx_USART
        rcc_gpio = GPIOxToRCC(GPIOx_USART);
        if (rcc_gpio)
        {
            RCC_APB2PeriphClockCmd(rcc_gpio, ENABLE);

            #ifdef USARTx
            // TC：Transmit Complete-发送完成,当TDR空且移位寄存器空时
            while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET);
            
            NVIC_DisableIRQ((IRQn_Type)(
            (USARTx == USART1) ? USART1_IRQn :
            (USARTx == USART2) ? USART2_IRQn :
            (USARTx == USART3) ? USART3_IRQn : 0));
            
            USART_ITConfig(USART1, USART_IT_TXE | USART_IT_TC | USART_IT_RXNE | USART_IT_IDLE, DISABLE);
            USART_Cmd(USARTx, DISABLE);
            USART_DeInit(USARTx);
            #endif
            
            #ifdef USART_DMA_TX_CH
            DMA_DeInit(USART_DMA_TX_CH);
            DMA_Cmd(USART_DMA_TX_CH, DISABLE);
            #endif
            
            #ifdef USART_DMA_RX_CH
            DMA_Cmd(USART_DMA_RX_CH, DISABLE);  
            DMA_DeInit(USART_DMA_RX_CH);
            #endif
            
            #ifdef USARTTx
            GPIO_ResetBits(GPIOx_USART, USARTTx);
            #endif
            
            #ifdef USARTRx
            GPIO_ResetBits(GPIOx_USART, USARTRx);
            #endif
            
            RCC_APB2PeriphClockCmd(rcc_gpio, DISABLE);
        }
    #endif

}


