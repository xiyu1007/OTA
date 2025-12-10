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
#include "BOOT.h"

void BootLoader(void)
{
    if (g_ota_info.ota_flag == OTA_FLAG_STATE_VALID)
        printf("Begin to update App!\n");
        // BootJumpToApp();  
    else
        printf("Jump to run App!\n");
        // BootJumpToBoot();
}









