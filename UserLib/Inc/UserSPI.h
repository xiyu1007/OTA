#ifndef __USERSPI_H__
#define __USERSPI_H__

#include "stm32f10x.h" 
// #include "stm32f10x_spi.h" // Library\stm32f10x_spi.h
#include "W25Q64.h"

#define SPIx SPI1
#define GPIOx_SPI GPIOA
#define SPI_SCK_Pin GPIO_Pin_5
#define SPI_MISO_Pin GPIO_Pin_6
#define SPI_MOSI_Pin GPIO_Pin_7

// #define SPIx SPI2
// #define GPIOx_SPI GPIOB
// #define SPI_SCK_Pin GPIO_Pin_13
// #define SPI_MISO_Pin GPIO_Pin_14
// #define SPI_MOSI_Pin GPIO_Pin_15

#define SPI_WRITE_NULL W25Q64_WRITE_NULL

void User_SPI_Init(void);
uint8_t SPIRWByte(uint8_t byte);
void SPIWBytes(uint8_t *wBytes, uint16_t len);
void SPIRBytes(uint8_t *rBytes, uint16_t len);



#endif /* __USERSPI_H__ */

