#include "pmsis.h"
#include "bsp/bsp.h"
#include "cpx.h"
#include <string.h>
#define COORDS_LENGTH 5

typedef struct
{
    uint16_t x;
    uint16_t y;
    uint16_t z;
} coordinate_t;

static CPXPacket_t packet;

void CPXAdListeningInit(void)
{
    cpxInit();
    cpxEnableFunction(CPX_F_APP);
    while (1)
    {
        cpxPrintToConsole(LOG_TO_CRTP, "[AD] Listening...\n");
        cpxReceivePacketBlocking(CPX_F_APP,&packet);
        coordinate_t coords[5];
        memcpy(coords, packet.data, packet.dataLength);
        char msg[30]="[AD] Get Msg, coord1: ";
        char coord1[10];
        coord1[0]='(';
        coord1[1]=coords[0].x+'0';
        coord1[2]=',';
        coord1[3]=coords[0].y+'0';
        coord1[4]=',';
        coord1[5]=coords[0].z+'0';
        coord1[6]=')';
        coord1[7]='\n';
        strcat(msg,coord1);
        cpxPrintToConsole(LOG_TO_CRTP, msg);
    }
}

int main(void)
{
    pi_bsp_init();
  return pmsis_kickoff((void *)CPXAdListeningInit);
}
