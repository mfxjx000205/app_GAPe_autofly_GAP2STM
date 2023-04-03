#include "pmsis.h"
#include "bsp/bsp.h"
#include "cpx.h"
#include "communicate.h"
#include <string.h>

static CPXPacket_t packet;

void CPXListeningInit(void)
{
    cpxInit();
    cpxEnableFunction(CPX_F_APP);
    while (1)
    {
        cpxPrintToConsole(LOG_TO_CRTP, "[AD] Listening...\n");
        cpxReceivePacketBlocking(CPX_F_APP,&packet);
        uint8_t other_id=packet.data[0];
        uint8_t reqType=packet.data[1];
        coordinate_t coords[5];
        memcpy(coords, &packet.data[2], packet.dataLength-2*sizeof(uint8_t));
        char msg[50]="[AD] Get Msg from: ";
        itoa(other_id,msg+ strlen(msg)-1);
        strcat(msg," ,reqType: ");
        msg[strlen(msg)]=reqType+'0';
        strcat(msg," ,coord1: ");
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
void itoa(uint8_t number,char*numberArray)
{
    uint8_t n=0,temp=number;
    while(temp>0){
        temp/=10;
        n++;
    }
    for (int8_t i = n-1; i >= 0; --i, number /= 10)
    {
        numberArray[i] = (number % 10) + '0';
    }
    return;
}
//bool SendCoords(coordinate_t* coords){
//
//    // Initialize the p2p packet
//    static P2PPacket packet;
//    packet.port=0x00;
//
//    // Get the current address of the crazyflie and obtain
//    //   the last two digits and send it as the first byte
//    //   of the payload
//    uint64_t address = configblockGetRadioAddress();
//    uint8_t my_id =(uint8_t)((address) & 0x00000000ff);
//    packet.data[0]=my_id;
//
//    memcpy(&packet.data[1], coords, sizeof(coordinate_t)*COORDS_LENGTH);
//
//    // Set the size, which is the amount of bytes the payload with ID and the string
//    packet.size=sizeof(coordinate_t)*COORDS_LENGTH+1;
//    // Send the P2P packet
//    DEBUG_PRINT("P2P Msg Sent by:%d, First Coord is: (%d,%d,%d)\n",my_id,coords[0].x,coords[0].y,coords[0].z);
//    return radiolinkSendP2PPacketBroadcast(&packet);
//}
//void CPXSendCoords(void){
//    coordinate_t coords[5];
//    for(int i=0;i<5;i++){
//        coords[i].x=i;
//        coords[i].y=i+1;
//        coords[i].z=i+2;
//    }
//    CPXPacket_t cpxPacket;
//    cpxInitRoute(CPX_T_STM32,CPX_T_GAP8,CPX_F_APP,&cpxPacket.route);
//    cpxPacket.dataLength=sizeof(coordinate_t)*COORDS_LENGTH;
//    memcpy(cpxPacket.data, msg, cpxPacket.dataLength);
//    while(1)
//    {
//        bool flag= cpxSendPacket(&cpxPacket,1000);
//        DEBUG_PRINT("Send %s\n",flag==false?"timeout":"success");
//    }
//}

int main(void)
{
    pi_bsp_init();
  return pmsis_kickoff((void *)CPXListeningInit);
}
