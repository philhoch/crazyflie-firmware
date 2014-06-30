#include "stm32f10x_conf.h"
#include "FreeRTOS.h"
#include "utilization.h"
#include "task.h"
#include "log.h"
#include "param.h"
#include "usec_time.h"
#include "system.h"

static bool isInit;

static uint16_t freeMemory = 0;

// CPU utilization meassurement

static float idleTime = 0.;
static uint32_t duration = 0;
static uint64_t switchedIn = 0;


static void utilizationTask(void* param);

void taskSwitchedIn(signed char *pcTaskName) {
    int count = 0;
    while(pcTaskName[count]) count++;
    if(count == 4 && pcTaskName[0] == 'I' && pcTaskName[1] == 'D' && pcTaskName[2] == 'L' && pcTaskName[3] == 'E') {
        switchedIn = usecTimestamp();
    }
}

void taskSwitchedOut(signed char *pcTaskName) {
    int count = 0;
    while(pcTaskName[count]) count++;
    if(count == 4 && pcTaskName[0] == 'I' && pcTaskName[1] == 'D' && pcTaskName[2] == 'L' && pcTaskName[3] == 'E') {
        duration += usecTimestamp() - switchedIn;
    }
}


void utilizationInit(void) {
  if(isInit)
    return;

  xTaskCreate(utilizationTask, (const signed char * const)"utility",
              2*configMINIMAL_STACK_SIZE, NULL, /*Piority*/3, NULL);

  isInit = TRUE;
}

bool utilizationTest(void) {
    return isInit;
}

static void utilizationTask(void* param) {
  uint32_t lastWakeTime;
  //Wait for the system to be fully started to start utilization loop
  systemWaitStart();
  lastWakeTime = xTaskGetTickCount ();

  while(1) {
    vTaskDelayUntil(&lastWakeTime, F2T(UTILIZATION_UPDATE_RATE)); // 100Hz

    idleTime = duration / 10000.;
    duration = 0;

    freeMemory = xPortGetFreeHeapSize();
   }
}

LOG_GROUP_START(utilization)
LOG_ADD(LOG_FLOAT, idle, &idleTime)
LOG_ADD(LOG_INT16, freeMemory, &freeMemory)
LOG_GROUP_STOP(utilization)



