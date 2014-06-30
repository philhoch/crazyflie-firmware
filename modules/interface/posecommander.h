#ifndef POSECOMMANDER_H_
#define POSECOMMANDER_H_

#include <stdint.h>
#include <stdbool.h>
#include "linkedlist.h"

#define POSECOMMANDERFREQUENCY 100

void poseCommanderInit(void);
bool poseCommanderTest(void);
void poseCommanderGetXYZYAW(float* x, float* y, float* z, float* yaw);

#endif /* POSECOMMANDER_H_ */
