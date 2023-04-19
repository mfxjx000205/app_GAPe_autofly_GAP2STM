// Created by Ziyi on 4/2/23.
#ifndef __COMMUNICATE_H__
#define __COMMUNICATE_H__

#define MAPPING_REQ 1
#define EXPLORE_REQ 2
#define PATH_REQ 3
#define EXPLORE_RESP 4
#define PATH_RESP 5

#define MSG_LENGTH 100

typedef struct
{
    coordinate_t startPoint;
    coordinate_t endPoint;
} coordinate_pair_t;

typedef struct{
    uint8_t sourceId;
    uint8_t reqType;
    uint8_t destinationId;
    uint16_t seq;
    uint8_t msgLength;
    coordinate_t data[MSG_LENGTH];
} RespInfo_t;

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
    uint8_t sourceId;
    uint8_t destinationId;
    uint8_t packetType;
    uint16_t seq;
    explore_req_payload_t exploreRequestPayload;
} explore_req_packet_t;

void CPXListeningInit(void);
void itoa(uint8_t number,char*numberArray);
#endif //__COMMUNICATE_H__
