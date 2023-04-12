#include "pmsis.h"
#include "bsp/bsp.h"
#include "cpx.h"
#define MAPPING_REQ 1
#define EXPLORE_REQ 2
#define PATH_REQ 3
#define EXPLORE_RESP 4
#define PATH_RESP 5

typedef struct
{
    uint16_t x;
    uint16_t y;
    uint16_t z;
} coordinate_t;

typedef struct{
  uint8_t reqType;
  uint8_t seq;
  uint8_t PayloadDataLength;
  coordinate_t data[2];
} RespInfo_t;
bool ProcessAndSend(){
  bool flag = false;
  static RespInfo_t RespInfo = {0};
  //static RespInfo_t RespInfo = GetRespInfo();
  RespInfo.reqType = EXPLORE_RESP;
  RespInfo.seq = 0;
  RespInfo.PayloadDataLength = 1;
  RespInfo.data[0].x = 1;
  RespInfo.data[0].y = 2;
  RespInfo.data[0].z = 3;
  // static RespInfo_t RespInfo = GetRespInfo();
  uint8_t sourceId = 0X3F;
  uint8_t reqType = RespInfo.reqType;
  uint8_t seq = RespInfo.seq;
  uint8_t RespDataLength = RespInfo.PayloadDataLength;
  static CPXPacket_t GAPTxSTM;
  cpxInitRoute(CPX_T_GAP8, CPX_T_STM32, CPX_F_APP, &GAPTxSTM.route);
  GAPTxSTM.data[0] = sourceId;
  GAPTxSTM.data[1] = reqType;
  GAPTxSTM.data[2] = seq;
  GAPTxSTM.data[3] = RespDataLength;
  memcpy(&GAPTxSTM.data[4], RespInfo.data, RespDataLength * sizeof(coordinate_t));
  GAPTxSTM.dataLength = 4 + RespDataLength * sizeof(coordinate_t);
  cpxSendPacketBlocking(&GAPTxSTM);
  cpxPrintToConsole(LOG_TO_CRTP, "[GAP8-Edge]: Send to STM32, ReqType = %d, Seq = %d, RespDataLength = %d\n\n", reqType, seq, RespDataLength);
  flag=true;
  
  return flag;
}
void GAPY2STM(void)
{
    pi_bsp_init();
    cpxInit();
    cpxEnableFunction(CPX_F_APP);
    // CPXPacket_t StmTx;
    // cpxInitRoute(CPX_T_GAP8, CPX_T_STM32, CPX_F_APP, &StmTx.route);
    while(1){
        
        bool flag = ProcessAndSend();
        cpxPrintToConsole(LOG_TO_CRTP, "flag = %d\n", flag);
        pi_time_wait_us(1000*10000);
    }
}

int main(void)
{
  return pmsis_kickoff((void *)GAPY2STM);
}