#include "stm32f10x.h"
#include "AT24C256.h"
#include "UserIIC.h"

int AT24C_SendAddr(uint16_t memAddr)
{
    if (AT24C_ADDR_LEN == 2)
    {
        IICSendByte(memAddr >> 8);
        if (IICWaitACk())
            goto G_ERR_MEM_NAK;
    }

    IICSendByte(memAddr & 0xFF);
    if (IICWaitACk())
        goto G_ERR_MEM_NAK;
    
    return ERR_OK;

    G_ERR_MEM_NAK:
        return ERR_MEM_NAK;
}

int8_t AT24C256_WriteByte(uint16_t memAddr, uint8_t byte)
{
    IICStart();
    IICSendByte(AT24C256_ADDR_WRITE);
    if (IICWaitACk())
        goto G_ERR_ADD_NAK;
    if (AT24C_SendAddr(memAddr))
        goto G_ERR_MEM_NAK;

    IICSendByte(byte);
    if (IICWaitACk())
        goto G_ERR_DATA_NAK;
    IICStop();
    return ERR_OK;

    G_ERR_ADD_NAK:
        IICStop();
        return ERR_ADD_NAK;
    G_ERR_MEM_NAK:
        IICStop();
        return ERR_MEM_NAK;
    G_ERR_DATA_NAK:
        IICStop();
        return ERR_DATA_NAK;
}

int8_t AT24C256_WritePage(uint16_t memAddr, uint8_t *bytes, uint16_t writeLen)
{
    IICStart();
    IICSendByte(AT24C256_ADDR_WRITE);
    if (IICWaitACk())
        goto G_ERR_ADD_NAK;

    if (AT24C_SendAddr(memAddr))
        goto G_ERR_MEM_NAK;

    for (uint16_t i = 0; i < writeLen; i++)
    {
        IICSendByte(bytes[i]);
        if (IICWaitACk())
            goto G_ERR_DATA_NAK;
    }
    IICStop();
    return ERR_OK;

    G_ERR_ADD_NAK:
        IICStop();
        return ERR_ADD_NAK;
    G_ERR_MEM_NAK:
        IICStop();
        return ERR_MEM_NAK;
    G_ERR_DATA_NAK:
        IICStop();
        return ERR_DATA_NAK;
}

int8_t AT24C256_ReadBytes(uint16_t memAddr, uint8_t *bytes, uint16_t readLen)
{
    IICStart();
    IICSendByte(AT24C256_ADDR_WRITE);
    if (IICWaitACk())
        goto G_ERR_ADD_NAK;
    if (AT24C_SendAddr(memAddr))
        goto G_ERR_MEM_NAK;

    IICStart(); // 完成dummy write, 然后再次发送起始信号
    IICSendByte(AT24C256_ADDR_READ);
    if (IICWaitACk())
        goto G_ERR_ADD_NAK;
    for (uint16_t i = 0; i < readLen; i++)
    {
        bytes[i] = IICReceiveByte(i != (readLen - 1) ? 1 : 0); // NAK on last byte
    }

    IICStop();
    return ERR_OK;

    G_ERR_ADD_NAK:
        IICStop();
        return ERR_ADD_NAK;
    G_ERR_MEM_NAK:
        IICStop();
        return ERR_MEM_NAK;
}
