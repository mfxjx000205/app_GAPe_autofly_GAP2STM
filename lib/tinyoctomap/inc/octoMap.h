#ifndef __OCTOMAP_H__
#define __OCTOMAP_H__
#include <stdint.h>
#include <stdbool.h>

// for strong platform
// remember to change the bit length of the children member in octoNode_t
// #define uint16_t uint32_t
// #define int16_t int32_t
// typedef int32_t setIndex_t;
// #define BIT_SCALE 2

// for weak platform
typedef short setIndex_t;

// for strong or weak(<=4096) platform
// determined by the bit width of the children member in octoNode_t
#define NODE_SET_SIZE 4096

#define BOOL uint16_t
#define TRUE 1
#define FALSE 0

#define TREE_CENTER_X 128
#define TREE_CENTER_Y 128
#define TREE_CENTER_Z 128
#define TREE_RESOLUTION 4
#define TREE_MAX_DEPTH 6

#define LOG_ODDS_OCCUPIED 6
#define LOG_ODDS_FREE 0
#define LOG_ODDS_UNKNOWN 3
#define LOG_ODDS_OCCUPIED_FLAG 1
#define LOG_ODDS_FREE_FLAG 0
#define LOG_ODDS_DIFF_STEP 3


// coordinate
typedef struct
{
    uint16_t x;
    uint16_t y;
    uint16_t z;
} coordinate_t;

// OctoNode

typedef struct
{
    uint16_t children : 12 ; // first child node index (the following 7 children are in order, rft, rbt, lbt, lft, rfn, rbn, lbn, lfn)
    uint16_t logOdds : 3 ;   // occupation probability level
    uint16_t isLeaf : 1 ;    // whether is leaf node
    // just for readable logging, you can remove this member onboard
    coordinate_t origin;    // origin coordinate of the voxel node
    uint16_t width;
} octoNode_t;
// strong platform (test fail)
// typedef struct
// {
//     uint16_t children; // first child node index (the following 7 children are in order, rft, rbt, lbt, lft, rfn, rbn, lbn, lfn)
//     uint16_t logOdds;   // occupation probability level
//     uint16_t isLeaf;    // whether is leaf node
//     // just for readable logging, you can remove this member onboard
//     coordinate_t origin;    // origin coordinate of the voxel node
//     uint16_t width;
// } octoNode_t;

// OctoNodeSet
typedef struct
{
    octoNode_t data[8]; // data of the item
    setIndex_t next; // next item index
} octoNodeSetItem_t;

typedef struct
{
    octoNodeSetItem_t setData[NODE_SET_SIZE];     // data set
    setIndex_t freeQueueEntry;                    // first free item index
    setIndex_t fullQueueEntry;                    // first full item index
    short numOccupied;
    short numFree;
    short length;
} octoNodeSet_t;

// OctoTree
typedef struct
{
    coordinate_t center;     // the coordinate of the center --- (x,y,z): tuple
    coordinate_t origin;     // the origin coordinate of this tree --- (x,y,z): tuple
    uint8_t resolution;      // resolution of the tree
    uint8_t maxDepth;        // max depth of the tree
    uint16_t width;          // width of the tree
    octoNode_t *root;        // root node of the tree
} octoTree_t;

// OctoMap
typedef struct
{
    octoTree_t *octoTree;
    octoNodeSet_t *octoNodeSet;
} octoMap_t;

void octoMapInit(octoMap_t *octoMap);
#endif
