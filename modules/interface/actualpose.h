#ifndef ACTUALPOSE_H_
#define ACTUALPOSE_H_

#include <stdint.h>
#include <stdbool.h>
#include "datastructure.h"

void actualPoseInit(void);
bool actualPoseTest(void);
uint32_t actualPoseGetInactivityTime(void);
void actualPoseGetRPY(float* roll, float* pitch, float* yaw);
void actualPoseGetXYZ(float* x, float* y, float* z);
void actualPoseGetXYZRPY(float* x, float* y, float* z, float* roll, float* pitch, float* yaw);
void actualPoseGetXYZYAW(float* x, float* y, float* z, float* yaw);
void actualPoseGetPose(Pose *pose);


#endif /* ACTUALPOSE_H_ */
