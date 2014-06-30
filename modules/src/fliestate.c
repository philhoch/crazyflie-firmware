#include "fliestate.h"
#include "FreeRTOS.h"
#include "task.h"
#include "crtp.h"
#include "param.h"
#include "system.h"
#include "log.h"
#include "commander.h"

/* TODO
 * enable/disable thrust for security purposes
 * more sophisticated joy control only enable/disable single joy components
*/

struct ControlStateCrtpValues {
      uint8_t controlState;
} __attribute__((packed));

static bool isInit;
static ControlState controlState = MANUAL;
static FlieState flieState = DEAD;

static void deadmanCrtpCB(CRTPPacket* pk);
static void controlStateCrtpCB(CRTPPacket* pk);
static void deadmanWatchdogReset(void);

static uint32_t lastUpdate;
static uint16_t stabilizerTimeout = 100;
static uint16_t shutdownTimeout = 250;

void flieStateInit(void) {
    if(isInit)
        return;

    crtpInit();
    crtpRegisterPortCB(CRTP_PORT_FLIESTATE, controlStateCrtpCB);
    crtpRegisterPortCB(CRTP_PORT_DEADMAN, deadmanCrtpCB);

    lastUpdate = xTaskGetTickCount();

    isInit = true;
}

static void controlStateCrtpCB(CRTPPacket* pk) {
    struct ControlStateCrtpValues packet = *((struct ControlStateCrtpValues*) pk->data);
    controlState = packet.controlState;
}

static void deadmanCrtpCB(CRTPPacket* pk) {
    deadmanWatchdogReset();
}

bool flieStateTest(void) {
    bool pass = true;
    pass &= crtpTest();
    return pass && isInit;
}

void deadmanWatchdog(void) {
    uint32_t ticktimeSinceUpdate;

    ticktimeSinceUpdate = xTaskGetTickCount() - lastUpdate;

    if (ticktimeSinceUpdate < stabilizerTimeout) {
        flieState = ALIVE;
    } else if((ticktimeSinceUpdate  >= stabilizerTimeout) && (ticktimeSinceUpdate < shutdownTimeout)) {
        flieState = ZOMBIE;
    } else {
        flieState = DEAD;
    }
}

static void deadmanWatchdogReset(void) {
    lastUpdate = xTaskGetTickCount();
}

FlieState getFlieState(void) {
    deadmanWatchdog();
    return flieState;
}

ControlState getControlState(void) {
    if(commanderIsJoy() == 0) {
        return MANUAL;
    }
    return controlState;
}


PARAM_GROUP_START(deadman)
PARAM_ADD(PARAM_UINT16, stabilizeTimeout, &stabilizerTimeout)
PARAM_ADD(PARAM_UINT16, shutdownTimeout, &shutdownTimeout)
PARAM_GROUP_STOP(deadman)


LOG_GROUP_START(cfstate)
LOG_ADD(LOG_UINT8, flieState, &flieState)
LOG_ADD(LOG_UINT8, controlState, &controlState)
LOG_GROUP_STOP(cfstate)







