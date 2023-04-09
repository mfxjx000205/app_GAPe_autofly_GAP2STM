/* octoMap.c: Do the mapping task */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "octoMap.h"
#include "octoTree.h"
#include "octoNodeSet.h"

// init node set
octoNodeSet_t nodeSet;
void octoMapInit(octoMap_t *octoMap)
{
    octoNodeSetInit(&nodeSet);
    // init octoMap
    octoMap->octoTree = octoTreeInit(&nodeSet);
    octoMap->octoNodeSet = &nodeSet;
    // avoid index 0 is used (octoNodeHasChildren will fail)
    octoMap->octoTree->root->children = octoNodeSetMalloc(octoMap->octoNodeSet);
}

int leafCountRecursive = 0;
void recursiveExportOctoMap(octoMap_t* octoMap, octoNode_t* node, uint16_t width) {
    if (node->isLeaf) {
        if (octoNodeLogOddsIsOccupiedOrFree(node)) {
            // printf("%d, %d, %d, %d, %d\n", node->origin.x, node->origin.y, node->origin.z, width, node->logOdds);
            leafCountRecursive++;
        }
    } else {
        for (int i = 0; i < 8; i++) {
            if (octoNodeHasChildren(node) && width > octoMap->octoTree->resolution) {
                recursiveExportOctoMap(octoMap, &octoMap->octoNodeSet->setData[node->children].data[i], width / 2);
            }
        }
    }
}

void iterativeExportOctoMap(octoMap_t* octoMap) {
    octoNode_t* cur;
    int leafCount = 0;
    for (int i = 0; i < NODE_SET_SIZE; i++) {
        for (int j = 0; j < 8; j++) {
            cur = &octoMap->octoNodeSet->setData[i].data[j];
            if (cur->isLeaf) {
                if (octoNodeLogOddsIsOccupiedOrFree(cur)) {
                    leafCount++;
                }
            }
        }
    }
    // printf("[iterativeExportOctoMap]leafCount = %d\n", leafCount);
}

void exportOctoMap(octoMap_t* octoMap) {
    octoNode_t* node = octoMap->octoTree->root;
    recursiveExportOctoMap(octoMap, node, octoMap->octoTree->width);
    iterativeExportOctoMap(octoMap);
    // printf("[recursiveExportOctoMap]leafCount = %d\n", leafCountRecursive);
}

void printOctoMapNodeDistribution(octoMap_t* octoMap, int times) {
    int nodeCount = 0;
    int occupiedCount = 0;
    int freeCount = 0;
    int unknownCount = 0;
    for (int i = 0; i < NODE_SET_SIZE; i++) {
        for (int j = 0; j < 8; j++) {
            nodeCount++;
            octoNode_t* node = &octoMap->octoNodeSet->setData[i].data[j];
            if (node->logOdds == LOG_ODDS_OCCUPIED) {
                occupiedCount++;
            } else if (node->logOdds == LOG_ODDS_FREE) {
                freeCount++;
            } else {
                unknownCount++;
            }
        }
    }
}
