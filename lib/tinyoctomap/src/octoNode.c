#include <stdio.h>
#include "octoNode.h"
#include "octoNodeSet.h"

/**
 * @brief Initialize the node
 *
 * @param octoNode self
 */
void octoNodeInit(octoNode_t *octoNode)
{
    octoNode->children = 0;
    octoNode->logOdds = LOG_ODDS_UNKNOWN;
    octoNode->isLeaf = TRUE;
    octoNode->width = 0;
}

/**
 * @brief whether this is a leaf node
 *
 * @param octoNode self
 */
BOOL octoNodeHasChildren(octoNode_t *octoNode)
{
    return octoNode->children != 0;
}

/**
 * @brief Splits the node into 8 child nodes.
 * Child nodes are given the occupancy probability of this parent node as the initial probability
 *
 * @param octoNode self
 * @param octoMap the octoMap to get the nodeset
 */
void octoNodeSplit(octoNode_t *octoNode, octoMap_t *octoMap)
{
    octoNode->children = octoNodeSetMalloc(octoMap->octoNodeSet);
    if(octoNode->logOdds == LOG_ODDS_FREE)
        octoMap->octoNodeSet->numFree += 7;
    else if(octoNode->logOdds == LOG_ODDS_OCCUPIED)
        octoMap->octoNodeSet->numOccupied += 7;
    for (uint8_t i = 0; i < 8; i++)
    {
        octoMap->octoNodeSet->setData[octoNode->children].data[i].logOdds = octoNode->logOdds;
        octoMap->octoNodeSet->setData[octoNode->children].data[i].children = 0;
        octoMap->octoNodeSet->setData[octoNode->children].data[i].isLeaf = TRUE;
    }
    octoNode->isLeaf = FALSE;
    octoNode->logOdds = LOG_ODDS_UNKNOWN;
}

/**
 * @brief Merges the node's 8 child nodes into a single node.
 * The occupancy probability of the parent node is set to the occupancy probability of the first child node.
 *
 * @param octoNode self
 * @param octoMap the octoMap to get the nodeset
 */
void octoNodePrune(octoNode_t *octoNode, octoMap_t *octoMap)
{
    octoNode->logOdds = octoMap->octoNodeSet->setData[octoNode->children].data[0].logOdds;
    octoNode->isLeaf = TRUE;
    if(octoNode->logOdds == LOG_ODDS_FREE)
    {
        octoMap->octoNodeSet->numFree -= 7;
    }
    else if(octoNode->logOdds == LOG_ODDS_OCCUPIED)
    {
        octoMap->octoNodeSet->numOccupied -= 7;
    }
    octoNodeSetFree(octoMap->octoNodeSet, (setIndex_t)octoNode->children);
    octoNode->children = 0;
    octoNode->isLeaf = TRUE;
}

/**
 * @brief Calculates the index of the child containing point.
 *
 * @param point the coordinate of the child node --- (x,y,z): tuple
 * @param origin the origin coordinate of the parent node -- (x,y,z): tuple
 * @param width the width of the parent node --- int
 * @return uint8_t the index of the child
 */
uint8_t octoNodeIndex(coordinate_t *point, coordinate_t origin, uint16_t width)
{
    uint8_t index = 0;
    uint8_t halfWidth = width / 2;
    if (point->x >= origin.x + halfWidth)
    {
        index += 1;
        origin.x += halfWidth;
    }
    if (point->y >= origin.y + halfWidth)
    {
        index += 2;
        origin.y += halfWidth;
    }
    if (point->z >= origin.z + halfWidth)
    {
        index += 4;
        origin.z += halfWidth;
    }
    return index;
}

/**
 * @brief Calculates the origin of the node with given index.
 *
 * @param index
 * @param origin
 * @param width
 * @return coordinate_t
 */
coordinate_t calOrigin(uint8_t index, coordinate_t origin, uint16_t width)
{
    coordinate_t newOrigin = origin;
    uint8_t halfWidth = width / 2;
    if (index & 1)
    {
        newOrigin.x += halfWidth;
    }
    if (index & 2)
    {
        newOrigin.y += halfWidth;
    }
    if (index & 4)
    {
        newOrigin.z += halfWidth;
    }
    return newOrigin;
}

/**
 * @brief Updates the node with a new observation.
 * @param octoNode self
 * @param point the point coordinate of the observation --- (x,y,z): tuple
 * @param diffLogOdds the difference value of logodds --- int
 * @param origin origin of this node --- (x,y,z): tuple
 * @param width width of this node --- int
 * @param maxDepth maximum depth this node can be branched --- int
 */
void octoNodeUpdate(octoNode_t *octoNode, octoMap_t *octoMap, coordinate_t *point, uint8_t diffLogOdds, coordinate_t origin, uint16_t width, uint8_t maxDepth)
{
    octoNode->origin = origin;
    octoNode->width = width;
    if (maxDepth == 0)
    {
        // if(octoNodeLogOddsIsOccupiedOrFree(octoNode))
        //         return;
        octoNodeUpdateLogOdds(octoMap, octoNode, diffLogOdds);
        return;
    }
    else
    {
        // check whether to split
        if (!octoNodeHasChildren(octoNode))
        {
            // if(octoNodeLogOddsIsOccupiedOrFree(octoNode))
            //     return;
            if (octoNode->logOdds == LOG_ODDS_OCCUPIED && diffLogOdds == LOG_ODDS_OCCUPIED_FLAG) {
                return;
            } else if (octoNode->logOdds == LOG_ODDS_FREE && diffLogOdds == LOG_ODDS_FREE_FLAG) {
                return;
            }
            octoNodeSplit(octoNode, octoMap);
        }
        // update child
        uint8_t index = octoNodeIndex(point, origin, width);
        coordinate_t newOrigin = calOrigin(index, origin, width);
        octoNodeUpdate(&octoMap->octoNodeSet->setData[octoNode->children].data[index], octoMap, point, diffLogOdds, newOrigin, width / 2, maxDepth - 1);

        // check whether to prune
        if (octoNode->isLeaf == FALSE && octoNodeCheckChildrenLogOdds(octoNode, octoMap))
        {
            octoNodePrune(octoNode, octoMap);
        }
    }
}

/**
 * @brief Return whether the logodds of all children are the same and arrive thresholds
 *
 * @param octoNode self
 * @return true
 * @return false
 */
BOOL octoNodeCheckChildrenLogOdds(octoNode_t *octoNode, octoMap_t *octoMap)
{
    uint8_t logOdds = octoMap->octoNodeSet->setData[octoNode->children].data[0].logOdds;
    // check whether the first child is occupied or free
    if (!octoNodeLogOddsIsOccupiedOrFree(&octoMap->octoNodeSet->setData[octoNode->children].data[0]))
    {
        return FALSE;
    }
    // check whether the logodds of all children are the same
    for (int i = 1; i < 8; i++)
    {
        if (octoMap->octoNodeSet->setData[octoNode->children].data[i].logOdds != logOdds)
        {
            return FALSE;
        }
    }
    return TRUE;
}

/**
 * @brief Return the occupancy probability of the voxel at a given point coordinate.
 *
 * @param octoNode self
 * @param diffLogOdds the difference flag of logodds --- int
 * @return uint8_t occupancy probability of the corresponding voxel
 */
void octoNodeUpdateLogOdds(octoMap_t* octoMap,octoNode_t *octoNode, uint8_t diffLogOdds)
{
    if (octoNode->logOdds > LOG_ODDS_FREE && diffLogOdds == LOG_ODDS_FREE_FLAG) {
        if(octoNode->logOdds == LOG_ODDS_OCCUPIED)
            --octoMap->octoNodeSet->numOccupied;
        octoNode->logOdds -= LOG_ODDS_DIFF_STEP;
        if(octoNode->logOdds == LOG_ODDS_FREE)
            ++octoMap->octoNodeSet->numFree;
    } else if (octoNode->logOdds < LOG_ODDS_OCCUPIED && diffLogOdds == LOG_ODDS_OCCUPIED_FLAG) {
        if(octoNode->logOdds == LOG_ODDS_FREE)
            --octoMap->octoNodeSet->numFree;
        octoNode->logOdds += LOG_ODDS_DIFF_STEP;
        if(octoNode->logOdds == LOG_ODDS_OCCUPIED)
            ++octoMap->octoNodeSet->numOccupied;
    }
}

BOOL octoNodeLogOddsIsOccupiedOrFree(octoNode_t *octoNode)
{
    return octoNode->logOdds == LOG_ODDS_OCCUPIED || octoNode->logOdds == LOG_ODDS_FREE;
}

/**
 * @brief Return the occupancy probability of the voxel at a given point coordinate.
 *
 * @param octoNode self
 * @param point coordinate of some voxel to get probability --- (x,y,z): tuple
 * @param origin the origin coordinate of the parent node -- (x,y,z): tuple
 * @param width the width of the parent node --- int
 * @return uint8_t occupancy probability of the corresponding voxel
 */
uint8_t octoNodeLogOddsAt(octoNode_t *octoNode, octoMap_t *octoMap, coordinate_t *point, coordinate_t origin, uint16_t width)
{
    if (octoNode->isLeaf)
    {
        return octoNode->logOdds;
    }
    else
    {
        uint8_t index = octoNodeIndex(point, origin, width);
        coordinate_t newOrigin = calOrigin(index, origin, width);
        return octoNodeLogOddsAt(&octoMap->octoNodeSet->setData[octoNode->children].data[index], octoMap, point, newOrigin, width / 2);
    }
}
