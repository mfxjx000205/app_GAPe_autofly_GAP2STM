#ifndef __OCTOTREE_H__
#define __OCTOTREE_H__
#include <stdint.h>
#include "octoMap.h"
#include "octoNode.h"

//#define BOOL int
#define TRUE 1
#define FALSE 0

#define P_GLOBAL 0.5
#define MIN_OCCUPIED 5
#define MAX_NOT_OCCUPIED 2

typedef enum direction_t{
    UP = 0,
    DOWN,
    LEFT,
    RIGHT,
    FRONT,
    BACK
}direction_t;

octoTree_t* octoTreeInit(octoNodeSet_t* nodeSet);
void octoTreeInsertPoint(octoTree_t *octoTree, octoMap_t *octoMap, coordinate_t *point, uint8_t diffLogOdds);
void octoTreeRayCasting(octoTree_t *octoTree, octoMap_t *octoMap, coordinate_t *startPoint, coordinate_t *endPoint);
uint8_t octoTreeGetLogProbability(octoTree_t *octoTree, octoMap_t *octoMap, coordinate_t *point);
void bresenham3D(octoTree_t *octoTree, octoMap_t *octoMap, coordinate_t *startPoint, coordinate_t *endPoint);
#endif
