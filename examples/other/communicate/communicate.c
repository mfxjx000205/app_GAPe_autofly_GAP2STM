#include <string.h>
#include "pmsis.h"
#include "bsp/bsp.h"
#include "cpx.h"
#include "octoMap.h"
#include "octoTree.h"
#include "communicate.h"
#define GAP8Edge 0x3F
#define AIDECK_ID 0x00
#define UAVS_LIDAR_NUM 3

static CPXPacket_t packet;
octoMap_t octoMapData;
static mapping_req_packet_t mapping_req_packet;
static explore_req_packet_t explore_req_packet;
static RespInfo_t RespInfo;
uav_t uavs[UAVS_LIDAR_NUM];

// void sendSumUpInfo(){
//     octoNodeSetItem_t* base= (&octoMapData)->octoNodeSet->setData;
//     octoNodeSetItem_t* cur=base+(&octoMapData)->octoNodeSet->fullQueueEntry;
//     u_int8_t nodesCount=0;
//     while(cur->next!=-1){
//         nodesCount++;
//         cpxPrintToConsole(LOG_TO_CRTP, "[SumUp-Info]: Seq = %d, \t(%d,%d,%d)@%d (%d,%d,%d)@%d (%d,%d,%d)@%d (%d,%d,%d)@%d\n", nodesCount,cur->data[0].origin.x,cur->data[0].origin.y,cur->data[0].origin.z,cur->data[0].width
//                                                         ,cur->data[1].origin.x,cur->data[1].origin.y,cur->data[1].origin.z,cur->data[1].width
//                                                         ,cur->data[2].origin.x,cur->data[2].origin.y,cur->data[2].origin.z,cur->data[2].width
//                                                         ,cur->data[3].origin.x,cur->data[3].origin.y,cur->data[3].origin.z,cur->data[3].width);

//         pi_time_wait_us(1000 * 1000);
//         cpxPrintToConsole(LOG_TO_CRTP, "[SumUp-Info]: Seq = %d.5, \t(%d,%d,%d)@%d (%d,%d,%d)@%d (%d,%d,%d)@%d (%d,%d,%d)@%d\n\n", nodesCount,cur->data[4].origin.x,cur->data[4].origin.y,cur->data[4].origin.z,cur->data[4].width
//                 ,cur->data[5].origin.x,cur->data[5].origin.y,cur->data[5].origin.z,cur->data[5].width
//                 ,cur->data[6].origin.x,cur->data[6].origin.y,cur->data[6].origin.z,cur->data[6].width
//                 ,cur->data[7].origin.x,cur->data[7].origin.y,cur->data[7].origin.z,cur->data[7].width);
//         cur=base+cur->next;
//         pi_time_wait_us(1000 * 1000);
//     }
//     cpxPrintToConsole(LOG_TO_CRTP, "[SumUp-Info]: Finished!, totalPacketCount = %d\n\n", nodesCount);
// }

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

void processMappingPacket(){
    short uav_id = mapping_req_packet.sourceId;
    short len = mapping_req_packet.mappingRequestPayloadLength;
    for (short i = 0; i < len; i++)
    {
        UpdateMap(&octoMapData,&mapping_req_packet.mappingRequestPayload[i].startPoint, &mapping_req_packet.mappingRequestPayload[i].endPoint);
    }
}

void processExplorePacket(){
    short uav_id = explore_req_packet.sourceId;
    if(uav_id >= UAVS_LIDAR_NUM){
        cpxPrintToConsole(LOG_TO_CRTP, "UavID error!\n");
        return;
    }
    //计算新的路径点
    coordinate_t currentI = explore_req_packet.exploreRequestPayload.startPoint;
    coordinateF_t currentF = {currentI.x, currentI.y, currentI.z};
    example_measure_t measurement = explore_req_packet.exploreRequestPayload.measurement;
    short index_loop = ((currentI.x + currentI.y + currentI.z) / TREE_RESOLUTION) % WINDOW_SIZE;
    ++uavs[uav_id].loops[index_loop];
    if (uavs[uav_id].loops[index_loop] < MAX_LOOP)
    {
        push(&uavs[uav_id].queue, index_loop);
        if (uavs[uav_id].queue.len >= WINDOW_SIZE)
        {
            index_loop = pop(&uavs[uav_id].queue);
            --uavs[uav_id].loops[index_loop];
        }
        if(isCoordinateQueueEmpty(&uavs[uav_id].paths) && !CalBestCandinates(&octoMapData, &measurement, &currentF, &uavs[uav_id])){
            initCoordinateQueue(&uavs[uav_id].paths);
            JumpLocalOp(&currentF, &measurement, &uavs[uav_id].paths);
        }
    }
    else
    {
        initCoordinateQueue(&uavs[uav_id].paths);
        JumpLocalOp(&currentF, &measurement, &uavs[uav_id].paths);
    }
    coordinate_t nextpoint;
    //存在未行走的路径点
    if(GetNextPoint(&uavs[uav_id].paths, &nextpoint)){
        RespInfo.sourceId = AIDECK_ID;
        RespInfo.destinationId = uav_id;
        RespInfo.seq = explore_req_packet.seq;
        RespInfo.reqType = EXPLORE_RESP;
        RespInfo.exploreResponsePayload.endPoint = nextpoint;
        return ;
    }
    else{
        cpxPrintToConsole(LOG_TO_CRTP, "Uav%d doesn't have next point\n",uav_id);
        return;
    }
}


void SplitAndAssembleMapping(){
    memcpy(&mapping_req_packet, packet.data, sizeof(mapping_req_packet_t));
    cpxPrintToConsole(LOG_TO_CRTP, "[GAP8-Edge]Receive mapping request! OctoMap processing...\n");
}

void SplitAndAssembleExplore(){
    memcpy(&explore_req_packet, packet.data, sizeof(explore_req_packet_t));
    cpxPrintToConsole(LOG_TO_CRTP, "[GAP8-Edge]Receive explore request! OctoMap processing...\n");
}

void ReceiveAndGive(void)
{
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
        uint8_t ReqType = packet.data[2];
        if(ReqType==MAPPING_REQ){
            SplitAndAssembleMapping();
            processMappingPacket();
        }else{
            SplitAndAssembleExplore();
            processExplorePacket();
        }
    }
}

bool ProcessAndSend(){
    bool flag = false;
    // static RespInfo_t RespInfo = GetRespInfo();
    static CPXPacket_t GAPTxSTM;
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
        bool flag = ProcessAndSend();
        cpxPrintToConsole(LOG_TO_CRTP, "flag = %d\n", flag);
        pi_time_wait_us(1000*100);
    }
}

void InitTask(void){
    for(int i=0;i<UAVS_LIDAR_NUM;++i){
        UAVInit(&uavs[i]);
    }
    mapInit();
    while(1){
        ReceiveAndGive();
        ProcessAndSend();
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
