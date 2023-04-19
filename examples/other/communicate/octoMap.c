/* octoMap.c: Do the mapping task */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
#include "octoMap.h"
#include "octoTree.h"
#include "octoNodeSet.h"
#include "rrtConnect.h"

#include "debug.h"

#define FILE_LENGTH 1000

int count = 0;

void octoMapInit(octoMap_t *octoMap)
{
    // init node set
    DEBUG_PRINT("octoMapInit\n");
    octoNodeSet_t* nodeSet;
    nodeSet = malloc(sizeof(octoNodeSet_t));
    // print nodeSet size
    DEBUG_PRINT("sizeof(octoNodeSet_t) = %d\n", sizeof(octoNodeSet_t));
    octoNodeSetInit(nodeSet);

    // init octoMap
    octoMap->octoTree = octoTreeInit(nodeSet);
    octoMap->octoNodeSet = nodeSet;
    // avoid index 0 is used (octoNodeHasChildren will fail)
    octoMap->octoTree->root->children = octoNodeSetMalloc(octoMap->octoNodeSet);

    // print octoMap
    DEBUG_PRINT("octoMap.octoTree->center = (%d, %d, %d)\n", octoMap->octoTree->center.x, octoMap->octoTree->center.y, octoMap->octoTree->center.z);
    DEBUG_PRINT("octoMap.octoTree->origin = (%d, %d, %d)\n", octoMap->octoTree->origin.x, octoMap->octoTree->origin.y, octoMap->octoTree->origin.z);
    DEBUG_PRINT("octoMap.octoTree->resolution = %d\n", octoMap->octoTree->resolution);
    DEBUG_PRINT("octoMap.octoTree->maxDepth = %d\n", octoMap->octoTree->maxDepth);
    DEBUG_PRINT("octoMap.octoTree->width = %d\n", octoMap->octoTree->width);
    // print octoMap.octoTree->root
    DEBUG_PRINT("octoMap.octoTree->root->children = %d\n", octoMap->octoTree->root->children);
    DEBUG_PRINT("octoMap.octoTree->root->logOdds = %d\n", octoMap->octoTree->root->logOdds);
    DEBUG_PRINT("octoMap.octoTree->root->isLeaf = %d\n", octoMap->octoTree->root->isLeaf);
    // print octoMap.octoNodeSet
    DEBUG_PRINT("octoMap.octoNodeSet->freeQueueEntry = %d, octoMap.octoNodeSet->fullQueueEntry = %d\n\n", octoMap->octoNodeSet->freeQueueEntry, octoMap->octoNodeSet->fullQueueEntry);
    //print the length and numFree and numOccupied
    DEBUG_PRINT("octoMap.octoNodeSet->length = %d, octoMap.octoNodeSet->numFree = %d, octoMap.octoNodeSet->numOccupied = %d\n\n", octoMap->octoNodeSet->length, octoMap->octoNodeSet->numFree, octoMap->octoNodeSet->numOccupied);
    count = 0;
    DEBUG_PRINT("octoMap.octoNodeSet->volumeFree = %d, octoMap.octoNodeSet->volumeOccupied = %d\n\n", octoMap->octoNodeSet->volumeFree, octoMap->octoNodeSet->volumeOccupied);
}

void recursiveExportOctoMap(octoMap_t* octoMap, octoNode_t* node, coordinate_t origin, uint16_t width) {
    if (node->isLeaf) {
        if(LOG_ODDS_FREE == node->logOdds ){
            ++count;
            DEBUG_PRINT("[app]FN:(%.2f,%.2f,%.2f),seq:%d,width:%d\n", (double)origin.x, (double)origin.y, (double)origin.z, count, width);
            vTaskDelay(100);
        }
        else if(LOG_ODDS_OCCUPIED == node->logOdds){
            ++count;
            DEBUG_PRINT("[app]ON:(%.2f,%.2f,%.2f),seq:%d,width:%d\n", (double)origin.x, (double)origin.y, (double)origin.z, count, width);
            vTaskDelay(100);
        }
        // DEBUG_PRINT("node->x = %d, node->y = %d, node->z = %d, node->width = %d, node->logOdds = %d\n", node->origin.x, node->origin.y, node->origin.z, width, node->logOdds);
        // fprintf(fp, "%d, %d, %d, %d, %d\n", node->origin.x, node->origin.y, node->origin.z, width, node->logOdds);
    } else {
        for (int i = 0; i < 8; i++) {
            if (octoNodeHasChildren(node) && width > octoMap->octoTree->resolution) {
                coordinate_t newOrigin = calOrigin(i,origin,width);
                recursiveExportOctoMap(octoMap, &octoMap->octoNodeSet->setData[node->children].data[i], newOrigin, width / 2);
            }
        }
    }
}
