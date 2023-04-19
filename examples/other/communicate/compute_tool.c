
#include <math.h>
#include <stdlib.h>
#include "pmsis.h"
#include "bsp/bsp.h"
#include "compute_tool.h"
#include "config_autofly.h"
#include "octoMap.h"
#include "octoTree.h"
#include "octoNodeSet.h"
#include "octoNode.h"
#include "circularQueue.h"
#include "coordinateQueue.h"
#include "auxiliary_tool.h"
#include "coordinateQueue.h"
#include "cpx.h"

//#include "../../../../aideck-latest/gap_riscv_toolchain_ubuntu/riscv32-unknown-elf/sys-include/math.h"
#define PROBABILITY_MEM(octomap) (double)octomap->octoNodeSet->length / NODE_SET_SIZE

void UAVInit(uav_t* uav){
    for(int i = 0;i<6;++i){
        uav->direction_weight[i] = 1;
    }
    uav->lastdir = 0;
    initQueue(&uav->queue);
    initCoordinateQueue(&uav->paths);
}

void CalCandidates(coordinateF_t *candidates, example_measure_t *measurement, coordinateF_t *current_F)
{
    float pitch = -1 * measurement->pitch;
    float roll = measurement->roll;
    float yaw = measurement->yaw;
    for (rangeDirection_t dir = rangeFront; dir <= rangeDown; ++dir)
    {
        if (measurement->data[dir] > AVOID_DISTANCE + STRIDE)
        {
            cal_PointByLength(STRIDE, pitch, roll, yaw, current_F, dir, &candidates[dir]);
        }
        else
        {
            candidates[dir].x = 30000;
            candidates[dir].y = 30000;
            candidates[dir].z = 30000;
        }
    }
}

bool CalBestCandinates(octoMap_t *octoMap,example_measure_t *measurement, coordinateF_t *current_point, uav_t* uav){
    coordinateF_t candinates[6];
    coordinate_t item_point;
    double item_candinateCost = 0, max_candinateCost = 0;
    Cost_C_t item_sum,item_cost;
    CalCandidates(candinates, measurement, current_point);
    max_candinateCost = 0;
    short dir_next = -1;
    for(int i = 0;i<6;++i){
        item_candinateCost = 0;
        item_sum.cost_prune = 0;
        item_sum.income_info = 0;
        if (candinates[i].x == 30000 && candinates[i].y == 30000 && candinates[i].z == 30000)
        {
            continue;
        }
        for (rangeDirection_t dir = rangeFront; dir <= rangeDown; ++dir)
        {
            item_point.x = candinates[i].x;
            item_point.y = candinates[i].y;
            item_point.z = candinates[i].z;
            item_cost = Cost_Sum(octoMap->octoTree, octoMap, &item_point, dir);
            item_sum.cost_prune += item_cost.cost_prune;
            item_sum.income_info += item_cost.income_info;
        }
        if (item_sum.income_info == 0)
        {
            item_sum.income_info = DISCIPLINE;
        }
        item_candinateCost = (double)uav->direction_weight[i] * (PROBABILITY_MEM(octoMap) * item_sum.cost_prune * COST_PRUNE_TIMES +
                                                    (1.0 - PROBABILITY_MEM(octoMap)) * item_sum.income_info * INCOME_INFO_TIMES);
        if (item_candinateCost > max_candinateCost){
            dir_next = i;
            max_candinateCost = item_candinateCost;
        }
    }
    if(dir_next != -1){
        uav->direction_weight[dir_next] = DIRECTION_AWARD;
        uav->direction_weight[(uav->lastdir)] = 1;
        (uav->lastdir) = dir_next;
        push_CoordinateQueue(&uav->paths, candinates[dir_next]);
        return true;
    }
    else{
        cpxPrintToConsole(LOG_TO_CRTP,"no next point\n");
        return false;
    }
}

rangeDirection_t GetRandomDir(example_measure_t *measurement)
{
    // Randomly sample twice to choose the larger
    rangeDirection_t dir = (rangeDirection_t)rand() % 6;
    rangeDirection_t maxdir = (rangeDirection_t)rand() % 6;
    int i = 0;
    // Guaranteed to get a feasible direction
    while (measurement->data[maxdir] < STRIDE + AVOID_DISTANCE && i < 20)
    {
        maxdir = (rangeDirection_t)rand() % 6;
        ++i;
    }
    // Try to get a better and feasible direction
    dir = (rangeDirection_t)rand() % 6;
    ++i;
    if (i == 20)
        return 10;
    if (measurement->data[dir] > measurement->data[maxdir])
        maxdir = dir;
    return maxdir;
}

void JumpLocalOp(coordinateF_t *current_point, example_measure_t* measurement,CoordinateQueue_t* paths){
    rangeDirection_t dir = GetRandomDir(measurement);
    if(dir == 10){
        cpxPrintToConsole(LOG_TO_CRTP,"no next dir\n");
        return;
    }
    // rangeDirection_t dir = rand()%6;
    float length = measurement->data[dir];
    coordinateF_t item_start_point = {current_point->x,current_point->y,current_point->z};
    coordinateF_t item_end_point;
    while(length > STRIDE + AVOID_DISTANCE){
        cal_PointByLength(STRIDE, -1 * measurement->pitch, measurement->roll, measurement->yaw, &item_start_point, dir, &item_end_point);
        push_CoordinateQueue(paths, item_end_point);
        item_start_point = item_end_point;
        length -= STRIDE;
    }
}

bool GetNextPoint(CoordinateQueue_t* paths, coordinate_t* next_point){
    if(isCoordinateQueueEmpty(paths)){
        return false;
    }
    else{
        coordinateF_t item_point = pop_CoordinateQueue(paths);
        next_point->x = item_point.x;
        next_point->y = item_point.y;
        next_point->z = item_point.z;
        return true;
    }
}

void UpdateMap(octoMap_t* octoMap, coordinate_t* current_I, coordinate_t* end_point){
    octoTreeRayCasting(octoMap->octoTree, octoMap, current_I, end_point); 
}
