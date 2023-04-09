#include <string.h>
#include "pmsis.h"
#include "bsp/bsp.h"
#include "cpx.h"
#include "octoMap.h"
#include "octoTree.h"
#include "communicate.h"

static CPXPacket_t packet;
octoMap_t octoMapData;

void mapInit()
{
    octoMap_t* octoMap = &octoMapData;
    octoMapInit(octoMap);

    // print octoMap
    cpxPrintToConsole(LOG_TO_CRTP, "[GAP8-Edge]sizeof(octoNode) = %lu\n", sizeof(octoNode_t));
    cpxPrintToConsole(LOG_TO_CRTP, "[GAP8-Edge]octoTree->center = (%d, %d, %d), origin = (%d, %d, %d), resolution = %d, maxDepth = %d, width = %d\n", 
        octoMap->octoTree->center.x, octoMap->octoTree->center.y, octoMap->octoTree->center.z, 
        octoMap->octoTree->origin.x, octoMap->octoTree->origin.y, octoMap->octoTree->origin.z,
        octoMap->octoTree->resolution, octoMap->octoTree->maxDepth, octoMap->octoTree->width);
    cpxPrintToConsole(LOG_TO_CRTP, "[GAP8-Edge]root->children = %d, logOdds = %d, isLeaf = %d\n", 
        octoMap->octoTree->root->children, octoMap->octoTree->root->logOdds, octoMap->octoTree->root->isLeaf);
    cpxPrintToConsole(LOG_TO_CRTP, "[GAP8-Edge]octoNodeSet->freeQE = %d, fullQE = %d, length = %d, numFree = %d, numOccupied = %d\n", 
        octoMap->octoNodeSet->freeQueueEntry, octoMap->octoNodeSet->fullQueueEntry, 
        octoMap->octoNodeSet->length, octoMap->octoNodeSet->numFree, octoMap->octoNodeSet->numOccupied);
}

void CPXListeningTask(void)
{
    cpxInit();
    cpxEnableFunction(CPX_F_APP);
    mapInit();
    octoMap_t* octoMap = &octoMapData;
    char msg[MSG_LENGTH];

    while (1)
    {
        cpxPrintToConsole(LOG_TO_CRTP, "[GAP8-Edge]Listening...\n");
        cpxReceivePacketBlocking(CPX_F_APP, &packet);
        uint8_t sourceId = packet.data[0];
        uint8_t reqType = packet.data[1];
        uint16_t seq = packet.data[2];
        if (reqType == MAPPING_REQ) {
            uint8_t mappingRequestPayloadLength = packet.data[3];
            coordinate_pair_t mappingRequestPayload[mappingRequestPayloadLength];
            memcpy(mappingRequestPayload, &packet.data[4], sizeof(coordinate_pair_t)*mappingRequestPayloadLength);
            sprintf(msg, "[GAP8-Edge]Receive CPX mapping request from: %d, seq: %d, payloadLength: %d\n", sourceId, seq, mappingRequestPayloadLength);
            for (int i = 0; i < mappingRequestPayloadLength; i++) {
                sprintf(msg, "[GAP8-Edge]Coordinate pair raycasted: (%d, %d, %d), (%d, %d, %d)\n",
                    mappingRequestPayload[i].startPoint.x, mappingRequestPayload[i].startPoint.y, mappingRequestPayload[i].startPoint.z,
                    mappingRequestPayload[i].endPoint.x, mappingRequestPayload[i].endPoint.y, mappingRequestPayload[i].endPoint.z);
                cpxPrintToConsole(LOG_TO_CRTP, msg);
                octoTreeRayCasting(octoMap->octoTree, octoMap, &mappingRequestPayload[i].startPoint, &mappingRequestPayload[i].endPoint);
            }
        } else {
            sprintf(msg, "[GAP8-Edge]Receive CPX other request from: %d, seq: %d, reqType: %d\n", sourceId, seq, reqType);
            cpxPrintToConsole(LOG_TO_CRTP, msg);
        }
    }
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
    return pmsis_kickoff((void *)CPXListeningTask);
}
