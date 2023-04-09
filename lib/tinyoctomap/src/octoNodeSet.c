#include <stdio.h>
#include <stdlib.h>
#include "octoNodeSet.h"
#include "octoNode.h"

void octoNodeSetInit(octoNodeSet_t *nodeSet)
{
    setIndex_t i;
    nodeSet->freeQueueEntry = 0;
    nodeSet->fullQueueEntry = -1;
    nodeSet->length = 0;
    nodeSet->numFree = 0;
    nodeSet->numOccupied = 0;
    for (i = 0; i < NODE_SET_SIZE; i++)
    {
        nodeSet->setData[i].next = i;
        nodeSet->setData[i].next += 1;
        for (int j = 0; j < 8; j++)
        {
            octoNodeInit(&nodeSet->setData[i].data[j]);
        }
        if (i == NODE_SET_SIZE - 1)
        {
            nodeSet->setData[i].next = -1;
        }
    }
}

setIndex_t octoNodeSetMalloc(octoNodeSet_t *nodeSet)
{
    if (nodeSet->freeQueueEntry == -1)
    {
        // printf("Full of sets!!! can not malloc!!!\n");
        exit(0);
    }
    else
    {
        setIndex_t candidate = nodeSet->freeQueueEntry;
        nodeSet->freeQueueEntry = nodeSet->setData[candidate].next;
        // insert to full queue
        setIndex_t tmp = nodeSet->fullQueueEntry;
        nodeSet->fullQueueEntry = candidate;
        nodeSet->setData[candidate].next = tmp;
        ++nodeSet->length;
        return candidate;
    }
}

BOOL octoNodeSetFree(octoNodeSet_t *nodeSet,
                     setIndex_t delItem)
{
    if (-1 == delItem)
        return TRUE;
    // init each node
    for (int i = 0; i < 8; i++)
    {
        octoNodeInit(&nodeSet->setData[delItem].data[i]);
    }
    // del from full queue
    setIndex_t pre = nodeSet->fullQueueEntry;
    if (delItem == pre)
    {
        nodeSet->fullQueueEntry = nodeSet->setData[pre].next;
        // insert to empty queue
        nodeSet->setData[delItem].next = nodeSet->freeQueueEntry;
        nodeSet->freeQueueEntry = delItem;
        --nodeSet->length;
        return TRUE;
    }
    else
    {
        while (pre != -1)
        {
            if (nodeSet->setData[pre].next == delItem)
            {
                nodeSet->setData[pre].next = nodeSet->setData[delItem].next;
                // insert to empty queue
                nodeSet->setData[delItem].next = nodeSet->freeQueueEntry;
                nodeSet->freeQueueEntry = delItem;
                --nodeSet->length;
                return TRUE;
            }
            pre = nodeSet->setData[pre].next;
        }
    }
    return FALSE;
}
