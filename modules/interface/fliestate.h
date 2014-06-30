#ifndef FLIESTATE_H
#define FLIESTATE_H

#include <stdbool.h>

typedef enum{
  MANUAL,
  AUTONOMOUS
} ControlState;

typedef enum{
  ALIVE,
  ZOMBIE,
  DEAD
} FlieState;

void flieStateInit(void);
bool flieStateTest(void);
FlieState getFlieState(void);
ControlState getControlState(void);

#endif // FLIESTATE_H
