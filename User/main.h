#ifndef __MAIN_H__
#define __MAIN_H__
#include "stm32f10x.h"
#include "FLASH.h"
#include <string.h>

// 0x08000000
#define STM_FLASH_BASE_ADDR FLASH_BASE_ADDR                     // STM32F103CT6 的 FLASH 基地址
#define STM_FLASH_PAGE_NUM FLASH_PAGE_NUM                       // STM32F103CT6 的 FLASH 页数
#define STM_BL_PAGE_NUM 20                                      // 前 20 页为 BootLoader 页
#define STM_APP_PAGE_NUM (STM_FLASH_PAGE_NUM - STM_BL_PAGE_NUM) // 应用程序占用的页数
#define STM_APP_BASE_PAGE_NUM STM_BL_PAGE_NUM                   // 应用程序起始页
#define STM_PAGE_BYTE_NUM FLASH_PAGE_BYTE_NUM                   // 每页的字节数 = 1 KB，1024 字节
// 0x08000000 + 20 * 1024 = 0x0800 5000
#define STM_APP_BASE_ADDR (STM_FLASH_BASE_ADDR + STM_APP_BASE_PAGE_NUM * FLASH_PAGE_BYTE_NUM) // 应用程序起始地址

#define OTA_INFO_T_SIZE sizeof(ota_info_t) // OTA 信息结构体大小，单位：字节
#define G_OTA_INFO_ADDR 0

#define XMODEM_PACKET_SIZE 128 // Xmodem 包大小，单位：字节
#define PER_FLASH_XMODEM_PACKET_SIZE (STM_PAGE_BYTE_NUM / XMODEM_PACKET_SIZE) // 每个FLASH页可以接收的Xmodem包数量
#define PER_W25Q64 (FLASH_PAGE_BYTE_NUM / W25Q64_PAGE_SIZE) // 每个1024的缓冲区可以写入的W25Q64页数量

typedef enum
{
    OTA_FLAG_INVALID = 0x00000000U, /* 没有OTA时，跳转到应用程序 */
    OTA_FLAG_VALID =   0x55AA55AAU,   /* OTA 标志有效，需要升级 */
} ota_flag_t; // 由服务器发送的 OTA 标志位

typedef struct
{
    ota_flag_t ota_flag; /* OTA 标志位，0x55AA55AA 表示需要 OTA 升级， 4 字节 */
    uint8_t ota_version[32]; /* OTA 版本号 */
    uint32_t ota_file_len[ (AT24C256_PAGE_SIZE - 32 - 1) / 4]; // 凑齐 64 字节, 0~6, 仅1-5可使用

} ota_info_t; // 包含服务器发送的 OTA 标志位和文件长度，存放在 AT24C256 的第 0 页

// ==================================================================================
typedef enum
{
    BOOT_FLAG_INVALID =                (uint32_t)0x00000000, /* 默认值，不更新应用程序 */
    BOOT_FLAG_UPDATE_A =               (uint32_t)0x00000001, 
    BOOT_FLAG_XMODEM_IAP =             (uint32_t)0x00000002, 
    BOOT_FLAG_XMODEM_2_FLASH =         (uint32_t)0x00000004, 
    BOOT_FLAG_SET_VERSION =            (uint32_t)0x00000008, 
    BOOT_FLAG_2_FLASH =                (uint32_t)0x00000010,   
    BOOT_FLAG_XMODEM_2_EXT_FLASH =     (uint32_t)0x00000020, 
    BOOT_FLAG_EXT_FLASH_2_APP =     (uint32_t)0x00000040, 
} boot_flag_t; 


typedef struct
{
    uint8_t update_buff[FLASH_PAGE_BYTE_NUM]; // 每次写入 FLASH 的页缓冲区，每页 1024 字节
    uint32_t block_num; // 当前写入 W25Q64 的块数
    uint32_t Xmodem_timer; // Xmodem 超时时间，单位：毫秒
    uint32_t packet_num; // Xmodem 包数
    uint32_t packet_crc; // Xmodem 包的 CRC 校验值

} boot_update_t;


extern ota_info_t g_ota_info;
extern boot_update_t g_boot_update;
extern boot_flag_t g_boot_flag;

#endif /* __MAIN_H__ */
