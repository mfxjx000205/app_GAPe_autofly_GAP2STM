// Created by Ziyi on 4/2/23.
#ifndef __COMMUNICATE_H__
#define __COMMUNICATE_H__

#define MAPPING_REQ 1
#define EXPLORE_REQ 2
#define PATH_REQ 3
#define EXPLORE_RESP 4
#define PATH_RESP 5

typedef struct
{
    uint16_t x;
    uint16_t y;
    uint16_t z;
} coordinate_t;

typedef struct
{
    coordinate_t startPoint;
    coordinate_t endPoint;
} coordinate_pair_t;

void CPXListeningInit(void);
void itoa(uint8_t number,char*numberArray);
#endif //__COMMUNICATE_H__
