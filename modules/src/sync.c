#include <string.h>
#include "stm32f10x_conf.h"
#include "FreeRTOS.h"
#include "task.h"
#include "crtp.h"
#include "usec_time.h"


static bool isInit;
static void syncCrtpCB(CRTPPacket* pk);

void syncInit(void) {
  if(isInit)
    return;

  crtpInit();
  crtpRegisterPortCB(CRTP_PORT_SYNC, syncCrtpCB);

  isInit = TRUE;
}

bool syncTest(void) {
  crtpTest();
  return isInit;
}

static void syncCrtpCB(CRTPPacket* pk) {
  uint64_t time = usecTimestamp();
  memcpy(pk->data+8, &time, sizeof(uint64_t));
  pk->size=16;
  crtpSendPacket(pk);
}

