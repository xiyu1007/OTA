#include "CRC.h"
#include "stm32f10x.h" 

// CRC-16/CCITT（XMODEM 变体）
uint16_t CRC16_XMODEM(uint8_t *data, uint16_t len)
{
    uint16_t crc = 0x0000;
    uint16_t poly = 0x1021;

    while(len--)
    {
        crc = (*data << 8) ^ crc;
        for(uint8_t i = 0; i < 8; i++)
        {
            if(crc & 0x8000)
                crc = (crc << 1) ^ poly;
            else
                crc <<= 1;
        }
        data++;
    }
    return crc;
}


// uint16_t CRC16_XMODEM(uint8_t *data, uint16_t len)
// {
//     uint16_t crc = 0x0000;

//     for (uint16_t i = 0; i < len; i++)
//     {
//         crc = (crc >> 8) | (crc << 8);
//         crc ^= data[i];
//         crc ^= (uint8_t)(crc & 0xFF) >> 4;
//         crc ^= (crc << 12);
//         crc ^= ((crc & 0xFF) << 5);
//     }
//     return crc;
// }



