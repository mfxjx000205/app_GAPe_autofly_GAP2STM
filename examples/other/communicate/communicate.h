// Created by Ziyi on 4/2/23.
#ifndef __COMMUNICATE_H__
#define __COMMUNICATE_H__
#include "circularQueue.h"
#include "compute_tool.h"
#include "config_autofly.h"
#include "coordinateQueue.h"
#include "pmsis.h"
#include "bsp/bsp.h"

#define MAPPING_REQ 1
#define EXPLORE_REQ 2
#define PATH_REQ 3
#define EXPLORE_RESP 4
#define PATH_RESP 5
#define METRICS 9

#define MSG_LENGTH 10
#define METRICS_MSG_LENGTH 10
#define MAPPING_REQUEST_PAYLOAD_LENGTH_LIMIT 4
typedef struct
{
    coordinate_t endPoint;
} explore_resp_payload_t;
typedef struct{
    uint8_t sourceId;
    uint8_t destinationId;
    uint8_t reqType;
    uint16_t seq;
    explore_resp_payload_t exploreResponsePayload;
} RespInfo_t;

typedef struct
{
    coordinate_t startPoint;
    coordinate_t endPoint;
    uint8_t mergedNums;
} mapping_req_payload_t;

typedef struct
{
    uint8_t sourceId;
    uint8_t destinationId;
    uint8_t packetType;
    uint16_t seq;
    uint8_t mappingRequestPayloadLength;
    mapping_req_payload_t mappingRequestPayload[MAPPING_REQUEST_PAYLOAD_LENGTH_LIMIT];
} mapping_req_packet_t;

typedef struct
{
    coordinate_t startPoint;
    example_measure_t measurement;
} explore_req_payload_t;

typedef struct{
    uint8_t sourceId;
    uint8_t destinationId;
    uint8_t packetType;
    uint16_t seq;
    uint8_t payload[METRICS_MSG_LENGTH];
} metrics_req_payload_t;

typedef struct
{
    uint8_t sourceId;
    uint8_t destinationId;
    uint8_t packetType;
    uint16_t seq;
    explore_req_payload_t exploreRequestPayload;
} explore_req_packet_t;


void CPXListeningInit(void);
void processMetrics();
void itoa(uint8_t number,char*numberArray);
#endif //__COMMUNICATE_H__
