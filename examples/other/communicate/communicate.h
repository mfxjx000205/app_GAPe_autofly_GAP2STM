//
// Created by 紫意菌 on 4/2/23.
//

#ifndef __COMMUNICATE_H__
#define __COMMUNICATE_H__
void CPXListeningInit(void);
void itoa(uint8_t number,char*numberArray);
typedef struct
{
    uint16_t x;
    uint16_t y;
    uint16_t z;
} coordinate_t;
#endif //__COMMUNICATE_H__
