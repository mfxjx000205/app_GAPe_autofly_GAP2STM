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

    while (1)
    {
        cpxPrintToConsole(LOG_TO_CRTP, "[GAP8-Edge]Listening...\n");
        cpxReceivePacketBlocking(CPX_F_APP, &packet);
        uint8_t sourceId = packet.data[0];
        uint8_t reqType = packet.data[1];
        // Calculate the sequence number
        uint8_t a = packet.data[2];
        uint8_t b = packet.data[3];
        uint8_t c = a << 8;
        uint16_t seq = c | b;

        if (reqType == MAPPING_REQ) {
            uint8_t mappingRequestPayloadLength = packet.data[3];
            coordinate_pair_t mappingRequestPayload[mappingRequestPayloadLength];
            memcpy(mappingRequestPayload, &packet.data[4], sizeof(coordinate_pair_t)*mappingRequestPayloadLength);
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

int main(void)
{
    pi_bsp_init();
    return pmsis_kickoff((void *)CPXListeningTask);
}
