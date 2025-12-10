#include "stm32f10x.h"
#include "UserIIC.h"
#include "Delay.h"

/*
SCL:  _|‾|_|‾|_|‾|_|‾|_ ...
SDA:    ↑   ↑   ↑   ↑
       slave 输出数据位

// I²C 空闲状态必须是 SCL=1、SDA=1。
// I²C 协议规定，一个 bit 的结束必须将 SCL 拉回低电平。
// ACK 位也是一个 bit，因此也必须按同样规律：SCL_L → SCL_H → SCL_L。
// 高位先发送
*/

void IICStart(void)
{
    IIC_SDA_H;
    IIC_SCL_H;
    Delay_us(IIC_DELAY);
    IIC_SDA_L;
    Delay_us(IIC_DELAY);
    IIC_SCL_L;
}

void IICStop(void)
{
    IIC_SCL_L;
    IIC_SDA_L;
    Delay_us(IIC_DELAY);
    IIC_SCL_H;
    Delay_us(IIC_DELAY);
    IIC_SDA_H;
    Delay_us(IIC_DELAY);
}

// 高位先发送
void IICSendByte(uint8_t Byte)
{
    for (int8_t i = 7; i >= 0; i--)
    {
        IIC_SCL_L;
        Delay_us(IIC_DELAY);
        if (Byte & (1 << i)){              
            // 1左移7位：0x 1000 0000
            IIC_SDA_H; // 注意分号
        }
        else{
            IIC_SDA_L;
        }

        Delay_us(IIC_DELAY);
        IIC_SCL_H;
        Delay_us(IIC_DELAY);
    }
    IIC_SCL_L; // 结束后拉低 SCL
    IIC_SDA_H; // 释放 SDA 恢复空闲状态
}

uint8_t IICReceiveByte(uint8_t Ack)
{
    uint8_t Byte = 0;
    for (int8_t i = 7; i >= 0; i--)
    {
        IIC_SCL_L;
        Delay_us(IIC_DELAY);
        IIC_SCL_H;
        Delay_us(IIC_DELAY);
        if (IIC_SDA_READ)
            Byte |= (1 << i);
    }
    IIC_SCL_L;
    Delay_us(IIC_DELAY);

    if (Ack)
    {
        IIC_SDA_L;           // 发送应答
        IIC_SCL_H;           // 拉高 SCL 以通知从机
        Delay_us(IIC_DELAY); // 确保稳定，下面需要释放 SDA
        IIC_SCL_L;           // 先拉低避免发送停止位
        IIC_SDA_H;           // 释放 SDA 恢复空闲状态
    } else
    {
        IIC_SDA_H;
        IIC_SCL_H;
        Delay_us(IIC_DELAY);
        IIC_SCL_L;
        Delay_us(IIC_DELAY);
    }
    return Byte;
}

int8_t IICWaitACk(void)
{
    // IIC_SDA_H; // 释放 SDA，准备接收从机应答
    Delay_us(IIC_DELAY);
    // IIC_SCL_H; // 拉高 SCL，通知从机发送应答
    // Delay_us(IIC_DELAY);
    int16_t TIMEOUT_FAIL = TIMEOUT;
    do
    {
        TIMEOUT_FAIL--;
    } while (IIC_SDA_READ && (TIMEOUT_FAIL > 0)); // 等待从机拉低 SDA，表示应答
    if (TIMEOUT_FAIL < 0){
        return IIC_ACK_TIMEOUT; // 超时，认为从机无应答
    }

    IIC_SCL_H; // 拉高确保是稳定的SDA低电平
    Delay_us(IIC_DELAY);
    if (IIC_SDA_READ){
        return IIC_ACK_ERR; // 再次检查SDA，确保是稳定的低电平
    }

    IIC_SCL_L;    // 拉低SCL，完成时序
    Delay_us(IIC_DELAY);
    return IIC_ACK_OK; // 成功接收从机应答
}

void User_IIC_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (GPIOx_IIC == GPIOA)
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    else if (GPIOx_IIC == GPIOB)
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    else if (GPIOx_IIC == GPIOC)
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStruct.GPIO_Pin = IIC_SCL_Pin | IIC_SDA_Pin;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOx_IIC, &GPIO_InitStruct);

    IIC_SCL_H;
    IIC_SDA_H;
}
