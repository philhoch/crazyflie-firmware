#define _USE_MATH_DEFINES

#include "positioncontroller.h"
#include "FreeRTOS.h"
#include "task.h"
#include "pid2dofcontroller.h"
#include "system.h"
#include "controller2dof.h"
#include "transferfunction.h"
#include "param.h"
#include "log.h"
#include "math.h"
#include "sensfusion6.h"
#include "fliestate.h"
#include "actualpose.h"
#include "posecommander.h"


static const float G = 9.81; // gravity constant (m/s^2)

static bool isInit;
static void positionControllerTask(void* param);

static PID2DOFController yawPID; // yaw controller
static controller2dof xPositionController; // x controller
static controller2dof yPositionController; // y controller
static controller2dof zPositionController; // z controller

static float xruNumerator[6] = {0.000000, 0.027239, -0.068226, 0.060743, -0.019348, 0.000066};//{0.000000, 0.010486, -0.031005, 0.030624, -0.010139, 0.000035}; //{0.000000, 0.010184, -0.030112, 0.029741, -0.009847, 0.000034};
static float xruDenumerator[5] = {-1.828498, 1.255403, -0.428595, 0.006040, -0.004351};//{-3.625145, 4.932403, -2.988555, 0.683626, -0.002330}; //{-3.566864, 4.780506, -2.856305, 0.644865, -0.002202};

static float xyuNumerator[6] = {0.000000, -1.228250, 4.323599, -5.863830, 3.650998, -0.882991};//{0.000000, -3.361245, 13.294201, -19.727163, 13.016579, -3.222372}; //{0.000000, -7.901474, 31.261792, -46.404245, 30.628762, -7.584836};
static float xyuDenumerator[5] = {-1.828498, 1.255403, -0.428595, 0.006040, -0.004351};//{-3.625145, 4.932403, -2.988555, 0.683626, -0.002330}; //{-3.566864, 4.780506, -2.856305, 0.644865, -0.002202};

static float zruNumerator[4] = {0.000000, 0.006293, -0.005834, 0.000126};//{0.000000, 0.001178, -0.001190, 0.000026}; //{0.000000, 0.001178, -0.001190, 0.000026};
static float zruDenumerator[3] = {-1.240543, 0.319705, -0.079163};//{-1.896241, 0.917237, -0.020996}; //{-1.896241, 0.917237, -0.020996};

static float zyuNumerator[4] = {0.000000, -0.291764, 0.561899, -0.270721};//{0.000000, -0.395391, 0.786817, -0.391441}; //{0.000000, -0.395391, 0.786817, -0.391441};
static float zyuDenumerator[3] = {-1.240543, 0.319705, -0.079163};//{-1.896241, 0.917237, -0.020996}; //{-1.896241, 0.917237, -0.020996};

//static float xruNumerator[4] = {0.000000, 0.001178, -0.001190, 0.000026}; //{0.000000, 0.001178, -0.001190, 0.000026};
//static float xruDenumerator[3] = {-1.896241, 0.917237, -0.020996}; //{-1.896241, 0.917237, -0.020996};

//static float xyuNumerator[4] = {0.000000, -0.395391, 0.786817, -0.391441}; //{0.000000, -0.395391, 0.786817, -0.391441};
//static float xyuDenumerator[3] = {-1.896241, 0.917237, -0.020996}; //{-1.896241, 0.917237, -0.020996};


static float xruInput[6] = {0.};
static float xruOutput[5] = {0.};

static float xyuInput[6] = {0.};
static float xyuOutput[5] = {0.};

static float yruInput[6] = {0.};
static float yruOutput[5] = {0.};

static float yyuInput[6] = {0.};
static float yyuOutput[5] = {0.};

static float zruInput[4] = {0.};
static float zruOutput[3] = {0.};

static float zyuInput[4] = {0.};
static float zyuOutput[3] = {0.};

static TransferFunction xru;
static TransferFunction xyu;
static TransferFunction yru;
static TransferFunction yyu;
static TransferFunction zru;
static TransferFunction zyu;

static FlieParameter flieParameter;
static PositionControllerValues positionControllerValues[2];
static int side = 0;

static float Fx = 0.;
static float Fy = 0.;
static float Fz = 0.;

static float xActual = 0.;
static float yActual = 0.;
static float zActual = 0.;
static float eulerYawActual = 0.;
static float xDesired = 0.;
static float yDesired = 0.;
static float zDesired = 0.;
static float eulerYawDesired = 0.;

static float saturate(float min, float max, float signal) {
    if(signal < min) {
        return min;
    } else if (signal > max) {
        return max;
    }
    return signal;
}

static float rad2deg(float rad) {
    return rad * 180 / M_PI;
}

static float deg2rad(float deg) {
    return deg * M_PI / 180;
}

static int rawThrustToFlieThrust(float thrust) {
    int flieThrust = (int)  210695.042862605 * thrust + 8432.23798311926;
    if(flieThrust > flieParameter.maxCmdThrust*flieParameter.maxThrust/100.) {
        return flieParameter.maxCmdThrust*flieParameter.maxThrust/100.;
    } else if(flieThrust < flieParameter.maxCmdThrust*flieParameter.minThrust/100.) {
        return (int) flieParameter.maxCmdThrust*flieParameter.minThrust/100.;
    }
    return flieThrust;
}

static float pitch(float Fx, float Fy, float Fz, float yaw) {
    return atan((cos(yaw) * Fx + sin(yaw) * Fy)/ (Fz - flieParameter.mass*G)); // atan2
}

static float roll(float Fx, float Fy, float Fz, float yaw, float pitch) {
    return atan(((sin(yaw) * Fx - cos(yaw) * Fy) * cos(pitch))/ (Fz - flieParameter.mass*G)); // atan2
}

static float thrust(float Fz, float pitch, float roll) {
    return (flieParameter.mass*G - Fz) / (cos(pitch) * cos(roll));
}

void positionControllerInit(void) {
  if(isInit)
    return;

    PID2DOFinitialize(&yawPID, 0.244980418445, 0.000479067924985, -0.518497051262, 0.461577449714, 1.0, 1.0, -1000, 1000); // yaw controller


    setNominatorCoefficients(&xru, xruNumerator, xruInput, 6);
    setDenominatorCoefficients(&xru, xruDenumerator, xruOutput, 5);
    setRUTransferFunction(&xPositionController, &xru);

    setNominatorCoefficients(&xyu, xyuNumerator, xyuInput, 6);
    setDenominatorCoefficients(&xyu, xyuDenumerator, xyuOutput, 5);
    setYUTransferFunction(&xPositionController, &xyu);

    setNominatorCoefficients(&yru, xruNumerator, yruInput, 6);
    setDenominatorCoefficients(&yru, xruDenumerator, yruOutput, 5);
    setRUTransferFunction(&yPositionController, &yru);

    setNominatorCoefficients(&yyu, xyuNumerator, yyuInput, 6);
    setDenominatorCoefficients(&yyu, xyuDenumerator, yyuOutput, 5);
    setYUTransferFunction(&yPositionController, &yyu);



    setNominatorCoefficients(&zru, zruNumerator,zruInput, 4);
    setDenominatorCoefficients(&zru, zruDenumerator, zruOutput, 3);
    setRUTransferFunction(&zPositionController, &zru);

    setNominatorCoefficients(&zyu, zyuNumerator, zyuInput, 4);
    setDenominatorCoefficients(&zyu, zyuDenumerator, zyuOutput, 3);
    setYUTransferFunction(&zPositionController, &zyu);


    flieParameter.mass = 18.61;
    flieParameter.minFlieThrust = 10000;
    flieParameter.maxCmdThrust = 60000;
//    flieParameter.maxFlieThrust = 65535;
    flieParameter.minThrust = 10;
    flieParameter.maxThrust = 90;
//    flieParameter.baseThrust = 39;
    flieParameter.maxRollAngle = 30;
    flieParameter.minRollAngle = -30;
    flieParameter.maxPitchAngle = 30;
    flieParameter.minPitchAngle = -30;
//    flieParameter.maxYawAngle = 10;
//    flieParameter.minYawAngle = -10;


    xTaskCreate(positionControllerTask, (const signed char * const)"POSCTRL",
              2*configMINIMAL_STACK_SIZE, NULL, /*Piority*/2, NULL);

    isInit = true;
}




bool positionControllerTest(void) {
    return isInit;
}

static void positionControllerTask(void* param) {
    uint32_t lastWakeTime;

    float yaw = 0.;

    //Wait for the system to be fully started to start positioncontroller loop
    systemWaitStart();

    lastWakeTime = xTaskGetTickCount();

    while(1) {
        vTaskDelayUntil(&lastWakeTime, F2T(POSITION_CONTROLLER_FREQ));

        poseCommanderGetXYZYAW(&xDesired, &yDesired, &zDesired, &eulerYawDesired);

//#ifdef MOTION_CAPTURE
        actualPoseGetXYZYAW(&xActual, &yActual, &zActual, &eulerYawActual);
        sensfusion6GetEulerY(&eulerYawActual);
//#else
        // TODO fusioned position and heading
//#endif

        // only run controller if system is not dead to prevent from "windup"
        if((getFlieState() != DEAD) && getControlState() == AUTONOMOUS) {
            yaw = PID2DOFupdate(&yawPID, eulerYawDesired, eulerYawActual); // read onboard yaw data
            Fx = processControllerInput(&xPositionController, &xDesired, &xActual);
            Fy = processControllerInput(&yPositionController, &yDesired, &yActual);
            Fz = processControllerInput(&zPositionController, &zDesired, &zActual);

//            if(Fx != Fx) {
//                Fx = 0.; // initialize register
//            }

//            if(Fy != Fy) {
//                Fy = 0.; // initialize register
//            }

//            if(Fz != Fz) {
//                Fz = 0.; // initialize register
//            }

            positionControllerValues[!side].yaw = yaw;
            positionControllerValues[!side].pitch = saturate(flieParameter.minPitchAngle, flieParameter.maxPitchAngle, rad2deg(pitch(Fx, Fy, Fz, deg2rad(positionControllerValues[!side].yaw))));
            positionControllerValues[!side].roll = saturate(flieParameter.minRollAngle, flieParameter.maxRollAngle, rad2deg(roll(Fx, Fy, Fz, deg2rad(positionControllerValues[!side].yaw), deg2rad(positionControllerValues[!side].pitch))));
            positionControllerValues[!side].thrust = rawThrustToFlieThrust(thrust(Fz, deg2rad(positionControllerValues[!side].pitch), deg2rad(positionControllerValues[!side].roll)));
          } else {
            positionControllerValues[!side].yaw = 0.;
            positionControllerValues[!side].pitch = 0.;
            positionControllerValues[!side].roll = 0.;
            positionControllerValues[!side].thrust = 0;
        }
        side = !side;
    }
}

void positionControllerGetRPY(float* eulerRollDesired, float* eulerPitchDesired, float* eulerYawDesired) {
    if((getFlieState() != DEAD) && getControlState() == AUTONOMOUS) {
        int usedSide = side;
        *eulerRollDesired  = positionControllerValues[usedSide].roll;
        *eulerPitchDesired = positionControllerValues[usedSide].pitch;
        *eulerYawDesired   = positionControllerValues[usedSide].yaw;
    } else {
        *eulerRollDesired = 0.;
        *eulerPitchDesired = 0.;
        *eulerYawDesired = 0.;
    }
}

void positionControllerGetThrust(uint16_t* thrust) {
    uint16_t rawThrust = positionControllerValues[side].thrust;

    if(getFlieState() == ZOMBIE) {
        *thrust = rawThrustToFlieThrust(flieParameter.mass * G);
    } else if(getFlieState() == DEAD) {
        *thrust = 0;
        return;
    }

    if (rawThrust > flieParameter.minFlieThrust) {
        *thrust = rawThrust;
    } else {
        *thrust = 0;
    }

    if (rawThrust > flieParameter.maxCmdThrust) {
        *thrust = flieParameter.maxCmdThrust;
    }

}

void positionControllerGetRPYType(RPYType* rollType, RPYType* pitchType, RPYType* yawType) {
    *rollType  = ANGLE;
    *pitchType = ANGLE;
    *yawType   = RATE;
}

LOG_GROUP_START(positionController)
LOG_ADD(LOG_FLOAT, roll, &positionControllerValues[0].roll)
LOG_ADD(LOG_FLOAT, pitch, &positionControllerValues[0].pitch)
LOG_ADD(LOG_FLOAT, yaw, &positionControllerValues[0].yaw)
LOG_ADD(LOG_UINT16, thrust, &positionControllerValues[0].thrust)
LOG_GROUP_STOP(positionController)

LOG_GROUP_START(desiredPose)
LOG_ADD(LOG_FLOAT, x, &xDesired)
LOG_ADD(LOG_FLOAT, y, &yDesired)
LOG_ADD(LOG_FLOAT, z, &zDesired)
LOG_ADD(LOG_FLOAT, yaw, &eulerYawDesired)
LOG_GROUP_STOP(desiredPose)

LOG_GROUP_START(actualPose)
LOG_ADD(LOG_FLOAT, x, &xActual)
LOG_ADD(LOG_FLOAT, y, &yActual)
LOG_ADD(LOG_FLOAT, z, &zActual)
LOG_ADD(LOG_FLOAT, yaw, &eulerYawActual)
LOG_GROUP_STOP(actualPose)

LOG_GROUP_START(forces)
LOG_ADD(LOG_FLOAT, Fx, (float*) &Fx)
LOG_ADD(LOG_FLOAT, Fy, (float*) &Fy)
LOG_ADD(LOG_FLOAT, Fz, (float*) &Fz)
LOG_GROUP_STOP(forces)

LOG_GROUP_START(debug0)
LOG_ADD(LOG_FLOAT, yruInput0, &yruInput[0])
LOG_ADD(LOG_FLOAT, yruInput1, &yruInput[1])
LOG_ADD(LOG_FLOAT, yruInput2, &yruInput[2])
LOG_ADD(LOG_FLOAT, yruInput3, &yruInput[3])
LOG_ADD(LOG_FLOAT, yruInput4, &yruInput[4])
LOG_ADD(LOG_FLOAT, yruInput5, &yruInput[5])
LOG_GROUP_STOP(debug0)
LOG_GROUP_START(debug1)
LOG_ADD(LOG_FLOAT, yruOutput0, &yruOutput[0])
LOG_ADD(LOG_FLOAT, yruOutput1, &yruOutput[1])
LOG_ADD(LOG_FLOAT, yruOutput2, &yruOutput[2])
LOG_ADD(LOG_FLOAT, yruOutput3, &yruOutput[3])
LOG_ADD(LOG_FLOAT, yruOutput4, &yruOutput[4])
LOG_GROUP_STOP(debug1)
LOG_GROUP_START(debug2)
LOG_ADD(LOG_FLOAT, yyuInput0, &yyuInput[0])
LOG_ADD(LOG_FLOAT, yyuInput1, &yyuInput[1])
LOG_ADD(LOG_FLOAT, yyuInput2, &yyuInput[2])
LOG_ADD(LOG_FLOAT, yyuInput3, &yyuInput[3])
LOG_ADD(LOG_FLOAT, yyuInput4, &yyuInput[4])
LOG_ADD(LOG_FLOAT, yyuInput5, &yyuInput[5])
LOG_GROUP_STOP(debug2)
LOG_GROUP_START(debug3)
LOG_ADD(LOG_FLOAT, yyuOutput0, &yyuOutput[0])
LOG_ADD(LOG_FLOAT, yyuOutput1, &yyuOutput[1])
LOG_ADD(LOG_FLOAT, yyuOutput2, &yyuOutput[2])
LOG_ADD(LOG_FLOAT, yyuOutput3, &yyuOutput[3])
LOG_ADD(LOG_FLOAT, yyuOutput4, &yyuOutput[4])
LOG_GROUP_STOP(debug3)

//static float yruInput[6] = {0.};
//static float yruOutput[5] = {0.};

//static float yyuInput[6] = {0.};
//static float yyuOutput[5] = {0.};


// Params for altitude hold
PARAM_GROUP_START(cfParameter)
PARAM_ADD(PARAM_FLOAT, mass, &flieParameter.mass)
PARAM_ADD(PARAM_INT32, minFlieThrust, &flieParameter.minFlieThrust)
PARAM_ADD(PARAM_INT32, maxCmdThrust, &flieParameter.maxCmdThrust)
//PARAM_ADD(PARAM_INT32, maxFlieThrust, &flieParameter.maxFlieThrust)
PARAM_ADD(PARAM_INT32, minThrust, &flieParameter.minThrust)
PARAM_ADD(PARAM_INT32, maxThrust, &flieParameter.maxThrust)
//PARAM_ADD(PARAM_INT32, baseThrust, &flieParameter.baseThrust)
PARAM_ADD(PARAM_INT32, maxRollAngle, &flieParameter.maxRollAngle)
PARAM_ADD(PARAM_INT32, minRollAngle, &flieParameter.minRollAngle)
PARAM_ADD(PARAM_INT32, maxPitchAngle, &flieParameter.maxPitchAngle)
PARAM_ADD(PARAM_INT32, minPitchAngle, &flieParameter.minPitchAngle)
//PARAM_ADD(PARAM_INT32, maxYawAngle, &flieParameter.maxYawAngle)
//PARAM_ADD(PARAM_INT32, minYawAngle, &flieParameter.minYawAngle)
PARAM_GROUP_STOP(cfParameter)
