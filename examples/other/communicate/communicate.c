#include <string.h>
#include "pmsis.h"
#include "bsp/bsp.h"
#include "cpx.h"
#include "octoMap.h"
#include "octoTree.h"
#include "communicate.h"
#define GAP8Edge 0x3F

static CPXPacket_t packet;
octoMap_t octoMapData;
static mapping_req_packet_t mapping_req_packet;
static explore_req_packet_t explore_req_packet;
static RespInfo_t RespInfo;

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

void SplitAndAssembleMapping(){
    memcpy(&mapping_req_packet, packet.data, sizeof(mapping_req_packet_t));
    cpxPrintToConsole(LOG_TO_CRTP, "[GAP8-Edge]Receive mapping request! OctoMap processing...");
}

void SplitAndAssembleExplore(){
    memcpy(&explore_req_packet, packet.data, sizeof(explore_req_packet_t));
    cpxPrintToConsole(LOG_TO_CRTP, "[GAP8-Edge]Receive explore request! OctoMap processing...");
}

void ReceiveAndGive(void)
{
    mapInit();
    octoMap_t* octoMap = &octoMapData;
    static uint16_t TotalPacketCount = 0;
    static uint16_t Loss_UAV_1 = 0;
    static uint16_t Loss_UAV_2 = 0;
    static uint16_t Loss_UAV_3 = 0;
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

        //Split and Assemble the packet to OctoMAP process
        uint8_t ReqType = packet.data[1];
        if(ReqType==MAPPING_REQ){
            SplitAndAssembleMapping();
            //process
            
        }else{
            SplitAndAssembleExplore();
            //process
            //generate RespInfo_t
        }
    }
}



bool GetAndSend(){
    bool Sendflag = false;
    RespInfo = GetRespInfo();
    static CPXPacket_t GAP2STMTx;
    cpxInitRoute(CPX_T_GAP8, CPX_T_STM32, CPX_F_APP, &GAPTxSTM.route);
    memcpy(&GAPTxSTM.data, &RespInfo, sizeof(RespInfo_t));
    GAPTxSTM.dataLength = sizeof(RespInfo_t);
    cpxSendPacketBlocking(&GAPTxSTM);
    cpxPrintToConsole(LOG_TO_CRTP, "[GAP8-Edge]: Send to STM32, Destination = %d, Seq = %d \n\n", RespInfo.destinationId, RespInfo.seq);
    flag=true;
    return flag;
}

void GAP8SendTask(void)
{
    while(1){     
        bool flag = GetAndSend();
        cpxPrintToConsole(LOG_TO_CRTP, "flag = %d\n", flag);
        pi_time_wait_us(1000*100);
    }
}

void InitTask(void){
    while(1){
    ReceiveAndGive();
    GetAndSend();
    pi_time_wait_us(1000 * 1000);
    }
}


int main(void)
{
    pi_bsp_init();
    cpxInit();
    cpxEnableFunction(CPX_F_APP);
    return pmsis_kickoff((void *)InitTask);
}
