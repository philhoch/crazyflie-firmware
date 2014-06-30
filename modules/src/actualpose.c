#include "stm32f10x_conf.h"
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "actualpose.h"
#include "crtp.h"
#include "configblock.h"



struct PoseCrtpValues {
  float x;
  float y;
  float z;
  float roll;
  float pitch;
  float yaw;
} __attribute__((packed));


static struct PoseCrtpValues actualPose[2];
static int side;
static bool isInit;
static uint32_t lastUpdate;
static bool isInactive;

static void actualPoseCrtpCB(CRTPPacket* pk);
static void actualPoseWatchdogReset(void);

void actualPoseInit(void) {
  if(isInit)
    return;


  crtpInit();
  crtpRegisterPortCB(CRTP_PORT_ACTUALPOSE, actualPoseCrtpCB); // x, y, z, roll, pitch, yaw

  lastUpdate = xTaskGetTickCount();
  isInactive = TRUE;
  isInit = TRUE;
}

bool actualPoseTest(void) {
  crtpTest();
  return isInit;
}

static void actualPoseCrtpCB(CRTPPacket* pk) {
    actualPose[!side] = *((struct PoseCrtpValues*)pk->data);
    side = !side;
    actualPoseWatchdogReset();
}


static void actualPoseWatchdogReset(void) {
  lastUpdate = xTaskGetTickCount();
}

uint32_t actualPoseGetInactivityTime(void) {
  return xTaskGetTickCount() - lastUpdate;
}

void actualPoseGetRPY(float* roll, float* pitch, float* yaw) {
    int usedSide = side;
    *roll = actualPose[usedSide].roll;
    *pitch = actualPose[usedSide].pitch;
    *yaw = actualPose[usedSide].yaw;
}

void actualPoseGetXYZ(float* x, float* y, float* z) {
    int usedSide = side;
    *x = actualPose[usedSide].x;
    *y = actualPose[usedSide].y;
    *z = actualPose[usedSide].z;
}

void actualPoseGetXYZRPY(float* x, float* y, float* z, float* roll, float* pitch, float* yaw) {
    actualPoseGetXYZ(x, y, z);
    actualPoseGetRPY(roll, pitch, yaw);
}

void actualPoseGetXYZYAW(float* x, float* y, float* z, float* yaw) {
    int usedSide = side;
    actualPoseGetXYZ(x, y, z);
    *yaw = actualPose[usedSide].yaw;
}

void actualPoseGetPose(Pose *pose) {
    int usedSide = side;
    pose->x = actualPose[usedSide].x;
    pose->y = actualPose[usedSide].y;
    pose->z = actualPose[usedSide].z;
    pose->yaw = actualPose[usedSide].yaw;
}

