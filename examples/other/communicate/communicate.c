#include <string.h>
#include "pmsis.h"
#include "bsp/bsp.h"
#include "cpx.h"
#include "octoMap.h"
#include "octoTree.h"
#include "communicate.h"
#define GAP8Edge 0x3F
#define AIDECK_ID 0x7E
#define FINISH_NUM 0xFFFF
#define UAVS_LIDAR_NUM 3

static CPXPacket_t packet;
octoMap_t octoMapData;
static mapping_req_packet_t mapping_req_packet;
static explore_req_packet_t explore_req_packet;
static metrics_req_payload_t metrics_req_payload;
static RespInfo_t RespInfo;
static uint16_t TotalPacketCount = 0;
static uint16_t UAV1count=0;
static uint16_t UAV2count=0;
static uint16_t UAV3count=0;
static bool Sendflag=false;
static bool PacketLoss=false;

uav_t uavs[UAVS_LIDAR_NUM];

void sendSumUpInfo(){
    octoNodeSetItem_t* base = (&octoMapData)->octoNodeSet->setData;
    octoNodeSetItem_t* cur = base+(&octoMapData)->octoNodeSet->fullQueueEntry;
    short length=(&octoMapData)->octoNodeSet->length;
    u_int8_t nodesCount=0;
    if(PacketLoss==false){
        cpxPrintToConsole(LOG_TO_CRTP, "[SumUpInfo]Finished! TotalPacketCount = %d,\n", nodesCount);
        cpxPrintToConsole(LOG_TO_CRTP, "[SumUpInfo]UAV1:%d, UAV2:%d, UAV3:%d, total:%d\n\n", UAV1count, UAV2count, UAV3count, TotalPacketCount);
        PacketLoss=true;
    }
    while(nodesCount < length){
        nodesCount++;
        cpxPrintToConsole(LOG_TO_CRTP, "[SumUpInfo]Seq = %d, \t",nodesCount);
        for(uint8_t i=0;i<8;i++){
            if(cur->data[i].logOdds==LOG_ODDS_FREE||cur->data[i].logOdds==LOG_ODDS_OCCUPIED)
            {
                cpxPrintToConsole(LOG_TO_CRTP, "(%d,%d,%d)#%d@%d ",cur->data[i].origin.x,cur->data[i].origin.y,cur->data[i].origin.z,cur->data[i].logOdds,cur->data[i].width);
                pi_time_wait_us(1000);
            }
        }
        cpxPrintToConsole(LOG_TO_CRTP, "\n");
        pi_time_wait_us(10 * 1000);
        cur = base+cur->next;
    }
    Sendflag=true;
}

void mapInit()
{
    octoMap_t* octoMap = &octoMapData;
    octoMapInit(octoMap);
    // print octoMap
    cpxPrintToConsole(LOG_TO_CRTP, "[MapInit]sizeof(octoNode) = %lu\n", sizeof(octoNode_t));
    cpxPrintToConsole(LOG_TO_CRTP, "[MapInit]octoTree->center = (%d, %d, %d), origin = (%d, %d, %d), resolution = %d, maxDepth = %d, width = %d\n", 
        octoMap->octoTree->center.x, octoMap->octoTree->center.y, octoMap->octoTree->center.z, 
        octoMap->octoTree->origin.x, octoMap->octoTree->origin.y, octoMap->octoTree->origin.z,
        octoMap->octoTree->resolution, octoMap->octoTree->maxDepth, octoMap->octoTree->width);
    cpxPrintToConsole(LOG_TO_CRTP, "[MapInit]root->children = %d, logOdds = %d, isLeaf = %d\n", 
        octoMap->octoTree->root->children, octoMap->octoTree->root->logOdds, octoMap->octoTree->root->isLeaf);
    cpxPrintToConsole(LOG_TO_CRTP, "[MapInit]octoNodeSet->freeQE = %d, fullQE = %d, length = %d, numFree = %d, numOccupied = %d\n", 
        octoMap->octoNodeSet->freeQueueEntry, octoMap->octoNodeSet->fullQueueEntry, 
        octoMap->octoNodeSet->length, octoMap->octoNodeSet->numFree, octoMap->octoNodeSet->numOccupied);
}

void processMappingPacket(){
    uint8_t uav_id = mapping_req_packet.sourceId;
    short len = mapping_req_packet.mappingRequestPayloadLength;
    for (short i = 0; i < len; i++)
    {
        UpdateMap(&octoMapData,&mapping_req_packet.mappingRequestPayload[i].startPoint
                , &mapping_req_packet.mappingRequestPayload[i].endPoint
                , mapping_req_packet.mappingRequestPayload[i].mergedNums
                , uav_id);
    }
    // cpxPrintToConsole(LOG_TO_CRTP, "[Edge-GAP8]Process mapping packet successfully\n");
}

void processExplorePacket(){
    short uav_id = explore_req_packet.sourceId;
    if(uav_id >= UAVS_LIDAR_NUM){
        // cpxPrintToConsole(LOG_TO_CRTP, "UavID error!\n");
        return;
    }
    // Calculate new waypoint
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
        initQueue(&uavs[uav_id].queue);
        for(int i = 0; i < WINDOW_SIZE; ++i)
            uavs[uav_id].loops[i] = 0;
        JumpLocalOp(&currentF, &measurement, &uavs[uav_id].paths);
    }
    coordinate_t nextpoint;
    // Have next waypoint
    if(GetNextPoint(&uavs[uav_id].paths, &nextpoint)){
        RespInfo.sourceId = AIDECK_ID;
        RespInfo.destinationId = uav_id;
        RespInfo.seq = explore_req_packet.seq;
        RespInfo.reqType = EXPLORE_RESP;
        RespInfo.exploreResponsePayload.endPoint = nextpoint;

        // Send response packet
        static CPXPacket_t GAPTxSTM;
        cpxInitRoute(CPX_T_GAP8, CPX_T_STM32, CPX_F_APP, &GAPTxSTM.route);
        memcpy(&GAPTxSTM.data, &RespInfo, sizeof(RespInfo_t));
        GAPTxSTM.dataLength = sizeof(RespInfo_t);
        cpxSendPacketBlocking(&GAPTxSTM);
        pi_time_wait_us(10 * 1000);
        // cpxPrintToConsole(LOG_TO_CRTP, "[Edge-GAP8]Send explore response packet, destinationId = %d, seq = %d\n\n", RespInfo.destinationId, RespInfo.seq);
    }
    else{
        // cpxPrintToConsole(LOG_TO_CRTP, "[Edge-GAP8]Uav%d doesn't have next point\n\n",uav_id);
        return;
    }
}


void SplitAndAssembleMapping(){
    memcpy(&mapping_req_packet, packet.data, sizeof(mapping_req_packet_t));
    // cpxPrintToConsole(LOG_TO_CRTP, "[Edge-GAP8]CPX: Receive mapping request from: %d, seq: %d, payloadLength: %d\n", 
    //     mapping_req_packet.sourceId, mapping_req_packet.seq, mapping_req_packet.mappingRequestPayloadLength);
}

void SplitAndAssembleExplore(){
    memcpy(&explore_req_packet, packet.data, sizeof(explore_req_packet_t));
    // cpxPrintToConsole(LOG_TO_CRTP, "[Edge-GAP8]CPX: Receive explore request from: %d, seq: %d\n", 
    //     explore_req_packet.sourceId, explore_req_packet.seq);
}

void processMetrics(){
    memcpy(&metrics_req_payload, packet.data, sizeof(metrics_req_payload_t));
    if(flag==false){
        sendSumUpInfo();
    }
}

void ReceiveAndGive(void)
{
    octoMap_t* octoMap = &octoMapData;
    cpxReceivePacketBlocking(CPX_F_APP, &packet);
    
    // Packet Loss Rate Calculate Module
    // count and split packet from other UAV
    TotalPacketCount++;
    uint8_t sourceId = packet.data[0];
    switch (sourceId)
    {
    case 0x00:
        {
            UAV1count++;
            break;
        }
    case 0x01:
        {
            UAV2count++;
            break;
        }
    case 0x02:
        {
            UAV3count++;
            break;
        }
    default:
            break;
    }

    // Split and Assemble the packet to OctoMAP process
    uint8_t ReqType = packet.data[2];
    switch(ReqType){
        case MAPPING_REQ:
        {
            SplitAndAssembleMapping();
            processMappingPacket();
            break;
        }
        case EXPLORE_REQ:
        {
            SplitAndAssembleExplore();
            processExplorePacket();
            break;
        }
        case METRICS:
        {
            processMetrics();
            break;   
        }
    }
}

void InitTask(void){
    for (int i = 0; i < UAVS_LIDAR_NUM; ++i) {
        UAVInit(&uavs[i]);
    }
    mapInit();
    while(1) {
        ReceiveAndGive();
        pi_time_wait_us(10 * 1000);
    }
}


int main(void)
{
    pi_bsp_init();
    cpxInit();
    cpxEnableFunction(CPX_F_APP);
    return pmsis_kickoff((void *)InitTask);
}
