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
    static uint16_t TotalPacketCount = 0;
    static uint16_t Loss_UAV_1=0
    static uint16_t Loss_UAV_2=0
    static uint16_t Loss_UAV_3=0
    while (1)
    {
        cpxPrintToConsole(LOG_TO_CRTP, "[GAP8-Edge]Listening...\n");
        cpxReceivePacketBlocking(CPX_F_APP, &packet);
        // Packet Loss Rate Calculate Module
        TotalPacketCount++;
        if(TotalPacketCount % 10==0){
            cpxPrintToConsole(LOG_TO_CRTP, "\n[GAP8-Edge]Packet Loss Total: The GAP8 has processed %d Packet!\n\n", TotalPacketCount);
        }
        uint8_t sourceId = packet.data[0];
        switch (sourceId)
        {
        case 0x01:
            Loss_UAV_1++;
            if(TotalPacketCount % 50==0){
                cpxPrintToConsole(LOG_TO_CRTP, "\n[GAP8-Edge]Packet Loss UAV1: The GAP8 has processed %d Packet!\n\n", Loss_UAV_1);
                Loss_UAV_1 = 0;
            }
            break;
        case 0x02:
            Loss_UAV_2++;
            if(TotalPacketCount % 50==0){
                cpxPrintToConsole(LOG_TO_CRTP, "\n[GAP8-Edge]Packet Loss UAV2: The GAP8 has processed %d Packet!\n\n", Loss_UAV_2);
                Loss_UAV_2 = 0;
            }
            break;
        
        case 0x03:
            Loss_UAV_3++;
            if(TotalPacketCount % 50==0){
                cpxPrintToConsole(LOG_TO_CRTP, "\n[GAP8-Edge]Packet Loss UAV3: The GAP8 has processed %d Packet!\n\n", Loss_UAV_3);
                Loss_UAV_3 = 0;
            }
            break;
        }
        uint8_t reqType = packet.data[1];
        // Calculate the sequence number
        uint8_t a = packet.data[2];
        uint8_t b = packet.data[3];
        uint16_t c = a << 8;
        uint16_t seq = c | b;

        if (reqType == MAPPING_REQ) {
            uint8_t mappingRequestPayloadLength = packet.data[4];
            coordinate_pair_t mappingRequestPayload[mappingRequestPayloadLength];
            memcpy(mappingRequestPayload, &packet.data[5], sizeof(coordinate_pair_t)*mappingRequestPayloadLength);
            cpxPrintToConsole(LOG_TO_CRTP, "[GAP8-Edge]First pair: (%d, %d, %d) - (%d, %d, %d)\n", 
                mappingRequestPayload[0].startPoint.x, mappingRequestPayload[0].startPoint.y, mappingRequestPayload[0].startPoint.z,
                mappingRequestPayload[0].endPoint.x, mappingRequestPayload[0].endPoint.y, mappingRequestPayload[0].endPoint.z);

            cpxPrintToConsole(LOG_TO_CRTP, "[GAP8-Edge]Receive CPX mapping request from: %d, seq: %d, payloadLength: %d\n", sourceId, seq, mappingRequestPayloadLength);

            // update octotree
            for (int i = 0; i < mappingRequestPayloadLength; i++) {
                octoTreeRayCasting(octoMap->octoTree, octoMap, &mappingRequestPayload[i].startPoint, &mappingRequestPayload[i].endPoint);
            }
        } else {
            cpxPrintToConsole(LOG_TO_CRTP, "[GAP8-Edge]Receive CPX other request from: %d, seq: %d, reqType: %d\n", sourceId, seq, reqType);
        }
    }
}



bool ProcessAndSend(){
    bool flag = false;
    static RespInfo_t RespInfo = {0};
    RespInfo.reqType = EXPLORE_RESP;
    //Initiate fake, need to be replaced by real data
    RespInfo.seq = 0;
    RespInfo.PayloadDataLength = 1;
    RespInfo.data[0].x = 1;
    RespInfo.data[0].y = 2;
    RespInfo.data[0].z = 3;
    // static RespInfo_t RespInfo = GetRespInfo();
    uint8_t sourceId = 0X3F;
    uint8_t reqType = RespInfo.reqType;
    uint8_t seq = RespInfo.seq;
    uint8_t RespDataLength = RespInfo.PayloadDataLength;
    static CPXPacket_t GAPTxSTM;
    cpxInitRoute(CPX_T_GAP8, CPX_T_STM32, CPX_F_APP, &GAPTxSTM.route);
    GAPTxSTM.data[0] = sourceId;
    GAPTxSTM.data[1] = reqType;
    GAPTxSTM.data[2] = seq;
    GAPTxSTM.data[3] = RespDataLength;
    memcpy(&GAPTxSTM.data[4], RespInfo.data, RespDataLength * sizeof(coordinate_t));
    GAPTxSTM.dataLength = 4 + RespDataLength * sizeof(coordinate_t);
    cpxSendPacketBlocking(&GAPTxSTM);
    cpxPrintToConsole(LOG_TO_CRTP, "[GAP8-Edge]: Send to STM32, ReqType = %d, Seq = %d, RespDataLength = %d\n\n", reqType, seq, RespDataLength);
    flag=true;
    return flag;
}

void GAP8SendTask(void)
{
    pi_bsp_init();
    cpxInit();
    cpxEnableFunction(CPX_F_APP);
    while(1){
        
        bool flag = ProcessAndSend();
        cpxPrintToConsole(LOG_TO_CRTP, "flag = %d\n", flag);
        pi_time_wait_us(1000*10000);
    }
}



int main(void)
{
    pi_bsp_init();
    return pmsis_kickoff((void *)GAP8SendTask);
}
