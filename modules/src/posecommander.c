#include "stm32f10x_conf.h"
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "system.h"
#include "math.h"
#include "posecommander.h"
#include "crtp.h"
#include "configblock.h"
#include "log.h"
#include "param.h"
#include "linkedlist.h"
#include "datastructure.h"
#include "actualpose.h"

struct DesiredPoseCrtpValues {
    float x;
    float y;
    float z;
    float yaw;
    uint8_t index;
} __attribute__((packed));

typedef struct DesiredPoseCrtpValues DesiredPoseCrtpValues;


// goal reached parameter
static float radius = 0.05; // in m
//static float maxX = 0.05; // in m
//static float maxY = 0.05; // in m
//static float maxZ = 0.05; // in m
static float maxYaw = 5; // in degree
static uint8_t yawEnabled = 0;

// log variables
static float error = 0.;
static float xError = 0.;
static float yError = 0.;
static float zError = 0.;
static float yawError = 0.;

static uint32_t id;

static linkedlist desiredPoses;
static Pose desiredPose;
static Pose actualPose;

static bool isInit = FALSE;
static uint32_t lastUpdate;
static bool isInactive;

static void poseCommanderCrtpCB(CRTPPacket* pk);
static void poseCommanderTask(void* param);


void poseCommanderInit(void) {
  if(isInit)
    return;

  crtpInit();
  crtpRegisterPortCB(CRTP_PORT_POSE, poseCommanderCrtpCB); // x, y, z, yaw

  desiredPoses.front = 0;
  desiredPoses.rear = 0;

  xTaskCreate(poseCommanderTask, (const signed char * const)"POSECMD",
              2*configMINIMAL_STACK_SIZE, NULL, /*Piority*/2, NULL);

  lastUpdate = xTaskGetTickCount();
  isInactive = TRUE;
  isInit = TRUE;
}

bool poseCommanderTest(void) {
  crtpTest();
  return isInit;
}

static void addDesiredPose(Pose *desiredPose) {
    linkedlistItem *item = (linkedlistItem *) pvPortMalloc(sizeof(linkedlistItem));
    Pose* value = (Pose*)  pvPortMalloc(sizeof(Pose));
    item->item = value;
    *value = *desiredPose;
    item->next = 0;
    linkedlistAddItem(&desiredPoses, item);
}

static void poseCommanderCrtpCB(CRTPPacket* pk) {
    DesiredPoseCrtpValues desiredCrtpPose = *((DesiredPoseCrtpValues*) pk->data);
    Pose desiredPose;
    desiredPose.x = desiredCrtpPose.x;
    desiredPose.y = desiredCrtpPose.y;
    desiredPose.z = desiredCrtpPose.z;
    desiredPose.yaw = desiredCrtpPose.yaw;
    id = desiredCrtpPose.index;
    if(desiredCrtpPose.index == 0) {
        clearLinkedList(&desiredPoses);
    } else {
        addDesiredPose(&desiredPose);
    }
}

static float euclideanDistance(Pose *referencePose, Pose *actualPose) {
   return sqrt(pow(referencePose->x - actualPose->x, 2) + pow(referencePose->y - actualPose->y, 2) + pow(referencePose->z - actualPose->z, 2));
}

static void updateErrors(Pose *referencePose, Pose *actualPose) {
    error = euclideanDistance(referencePose, actualPose);
    xError = fabs(referencePose->x - actualPose->x);
    yError = fabs(referencePose->y - actualPose->y);
    zError = fabs(referencePose->z - actualPose->z);
    yawError = fabs(referencePose->yaw - actualPose->yaw);
}

static float yawDistance(float referenceYaw, float actualYaw) {
    if(fabs(referenceYaw - actualYaw) > 180) {
        return 360. - fabs(referenceYaw - actualYaw);
    }
    return fabs(referenceYaw - actualYaw);
}

static bool reached(Pose *referencePose, Pose *actualPose) {
    if(euclideanDistance(referencePose, actualPose) < radius) {
        if(yawEnabled) {
            if(yawDistance(referencePose->yaw, actualPose->yaw) < maxYaw) {
                return true;
            }
            return false;
        }
        return true;
    }
    return false;
}



static void poseCommanderTask(void* param) {
    uint32_t lastWakeTime;

    //Wait for the system to be fully started to start pose controller loop
    systemWaitStart();

    lastWakeTime = xTaskGetTickCount ();

    while(1) {
        vTaskDelayUntil(&lastWakeTime, F2T(POSECOMMANDERFREQUENCY)); // 100Hz

        // read actual pose value
        actualPoseGetPose(&actualPose);
        // get desired pose value
        if(desiredPoses.front != 0) {
            desiredPose = *(Pose*)(desiredPoses.front->item);
        }

        // update data for logging
        updateErrors(&desiredPose, &actualPose);

        // test for goal reached and shift
        if(reached(&desiredPose, &actualPose)) {
            if(desiredPoses.front != 0) {
                if(desiredPoses.front->next != 0) {
                    linkedlistItem front = *linkedListPopFront(&desiredPoses);
                    deleteLinkedlistItem(&front);
                }
            }
        }
    }
}

void poseCommanderGetXYZYAW(float* x, float* y, float* z, float* yaw) {
    *x = desiredPose.x;
    *y = desiredPose.y;
    *z = desiredPose.z;
    *yaw = desiredPose.yaw;
}


PARAM_GROUP_START(posereached)
PARAM_ADD(PARAM_FLOAT, radius, &radius)
//PARAM_ADD(PARAM_FLOAT, maxX, &maxX)
//PARAM_ADD(PARAM_FLOAT, maxY, &maxY)
//PARAM_ADD(PARAM_FLOAT, maxZ, &maxZ)
PARAM_ADD(PARAM_FLOAT, maxYaw, &maxYaw)
PARAM_ADD(PARAM_UINT8, yawEnabled, &yawEnabled)
PARAM_GROUP_STOP(posereached)

LOG_GROUP_START(posecommander)
LOG_ADD(LOG_UINT32, index, &id)
LOG_GROUP_STOP(posecommander)

LOG_GROUP_START(goals)
LOG_ADD(LOG_UINT32, numberOfGoals, &desiredPoses.elements)
//LOG_ADD(LOG_UINT8, endReached, &endReached)
LOG_GROUP_STOP(goals)

LOG_GROUP_START(distance)
LOG_ADD(LOG_FLOAT, euclideanDistance, &error)
LOG_ADD(LOG_FLOAT, x, &xError)
LOG_ADD(LOG_FLOAT, y, &yError)
LOG_ADD(LOG_FLOAT, z, &zError)
LOG_ADD(LOG_FLOAT, yaw, &yawError)
LOG_GROUP_STOP(distance)

