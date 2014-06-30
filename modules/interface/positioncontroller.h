

#ifndef POSITIONCONTROLLER_H_
#define POSITIONCONTROLLER_H_

#include <stdbool.h>
#include "commander.h"

#define POSITION_CONTROLLER_FREQ 10

struct FLIEPARAMETER {
    float mass; // mass (kg)
    int32_t minFlieThrust;
    int32_t maxCmdThrust; // as command to the flie
//    int32_t maxFlieThrust; //as value from the flie
    int32_t minThrust; // percent of maximum cmd thrust
    int32_t maxThrust; // percent of maximum cmd thrust
//    int32_t baseThrust; // percent of maximum cmd thrust
    int32_t maxRollAngle; // (deg)
    int32_t minRollAngle; // (deg)
    int32_t maxPitchAngle; // (deg)
    int32_t minPitchAngle; // (deg)
//    int32_t maxYawAngle;
//    int32_t minYawAngle;
};

struct POSITIONCONTROLLERVALUES {
  float roll;
  float pitch;
  float yaw;
  uint16_t thrust;
};

typedef struct FLIEPARAMETER FlieParameter;
typedef struct POSITIONCONTROLLERVALUES PositionControllerValues;


void positionControllerInit(void);
bool positionControllerTest(void);
void positionControllerGetRPY(float* eulerRollDesired, float* eulerPitchDesired, float* eulerYawDesired);
void positionControllerGetThrust(uint16_t* thrust);
void positionControllerGetRPYType(RPYType* rollType, RPYType* pitchType, RPYType* yawType);


#endif /* POSITIONCONTROLLER_H_ */
