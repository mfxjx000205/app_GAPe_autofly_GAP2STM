#include "octoMap.h"
#include "circularQueue.h"
#include "coordinateQueue.h"
#include "config_autofly.h"

typedef struct {
    float direction_weight[6];
    short loops[WINDOW_SIZE];
    Queue_t queue;
    CoordinateQueue_t paths;
    short lastdir;
}uav_t;

void UAVInit(uav_t* uav)
void CalCandidates(coordinateF_t *candidates, example_measure_t *measurement, coordinateF_t *current_F);
bool CalBestCandinates(octoMap_t *octoMap,example_measure_t *measurement, coordinateF_t *current_point, uav_t* uav);
rangeDirection_t GetRandomDir(example_measure_t *measurement);
void JumpLocalOp(coordinateF_t *current_point, example_measure_t* measurement,CoordinateQueue_t* paths);
bool GetNextPoint(CoordinateQueue_t* paths, coordinateF_t* next_point);
void UpdateMap(octoMap_t* octoMap, coordinate_t* current_I, coordinateF_t* end_point);