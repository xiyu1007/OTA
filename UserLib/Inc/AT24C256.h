#ifndef __AT24C256_H__
#define __AT24C256_H__

#include "UserIIC.h"
#include "stm32f10x.h"

// 器件地址为 1010  A2 A1 A0 X
/// A2 A1 A0 接GND 时，器件地址为 1010 000X
#define AT24C256_ADDR_WRITE   0xA0 // 写地址
#define AT24C256_ADDR_READ    0xA1 // 读地址
#define AT24C256_PAGE_SIZE 64  // 每页64字节
#define AT24C_ADDR_LEN 2 // 地址字节数

#define ERR_OK 0
#define ERR_ADD_NAK 1
#define ERR_MEM_NAK 2
#define ERR_DATA_NAK 3

int8_t AT24C256_WriteByte(uint16_t memAddr, uint8_t byte);
int8_t AT24C256_WritePage(uint16_t memAddr, uint8_t *bytes, uint16_t writeLen);
int8_t AT24C256_ReadBytes(uint16_t memAddr, uint8_t *bytes, uint16_t readLen);

#endif /* __AT24C256_H__ */

