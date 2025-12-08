#ifndef __USERIIC_H__
#define __USERIIC_H__

#define GPIOx GPIOA
#define IIC_SCL_Pin GPIO_Pin_6
#define IIC_SDA_Pin GPIO_Pin_7

#define IIC_DELAY  2
#define IIC_ACK_OK    0
#define IIC_ACK_TIMEOUT    -1
#define IIC_ACK_ERR    -2
#define IIC_ACK_MEM_H  -3
#define IIC_ACK_MEM_L  -4

static int16_t TIMEOUT = 1000;

#define IIC_SCL_H GPIO_SetBits(GPIOx, IIC_SCL_Pin)
#define IIC_SCL_L GPIO_ResetBits(GPIOx, IIC_SCL_Pin)
#define IIC_SDA_H GPIO_SetBits(GPIOx, IIC_SDA_Pin)
#define IIC_SDA_L GPIO_ResetBits(GPIOx, IIC_SDA_Pin)

#define IIC_SDA_READ GPIO_ReadInputDataBit(GPIOx, IIC_SDA_Pin)
#define IIC_SCL_READ GPIO_ReadInputDataBit(GPIOx, IIC_SCL_Pin)

void UserIICInit(void);
void IICStart(void);
void IICStop(void);
int8_t IICWaitACk(void);
void IICSendByte(uint8_t Byte);
uint8_t IICReceiveByte(uint8_t Ack);

#endif /* __USERI2C_H__ */
