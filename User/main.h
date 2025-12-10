#ifndef __MAIN_H__
#define __MAIN_H__
#include "stm32f10x.h"
#include "FLASH.h"
#include <string.h>

// 0x08000000
#define STM_FLASH_BASE_ADDR FLASH_BASE_ADDR // STM32F103CT6 的 FLASH 基地址
#define STM_FLASH_PAGE_NUM FLASH_PAGE_NUM // STM32F103CT6 的 FLASH 页数
#define STM_BL_PAGE_NUM 20 // 前 20 页为 BootLoader 页
#define STM_APP_PAGE_NUM (STM_FLASH_PAGE_NUM - STM_BL_PAGE_NUM) // 应用程序占用的页数
#define STM_APP_BASE_PAGE_NUM STM_BL_PAGE_NUM // 应用程序起始页
#define STM_PAGE_BYTE_NUM FLASH_PAGE_BYTE_NUM // 每页的字节数 = 1 KB，1024 字节
// 0x08000000 + 20 * 1024 = 0x0800 0500
#define STM_APP_BASE_ADDR (STM_FLASH_BASE_ADDR + STM_APP_BASE_PAGE_NUM * FLASH_PAGE_BYTE_NUM) // 应用程序起始地址


#define OTA_INFO_T_SIZE sizeof(ota_info_t) // OTA 信息结构体大小，单位：字节
#define G_OTA_INFO_ADDR 0

typedef enum
{
    OTA_FLAG_STATE_INVALID = 0x00000000U,   /* OTA 标志无效 */
    OTA_FLAG_STATE_VALID   = 0x55AA55AAU,   /* OTA 标志有效，需要升级 */
} ota_flag_state_t;

typedef struct
{
    ota_flag_state_t ota_flag;   /* OTA 标志位，0x55AA55AA 表示需要 OTA 升级 */
} ota_info_t;

extern ota_info_t g_ota_info;





#endif /* __MAIN_H__ */


