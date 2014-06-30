#include "pid2dofcontroller.h"
#include <stdint.h>
#include "usec_time.h"

int clamping(PID2DOFController* pid, float r, float y) {
    if(((r-y) < pid->saturationMinimum) || ((r-y) > pid->saturationMaximum))
        return 0;
    return 1;
}

void PID2DOFinitialize(PID2DOFController* pid, float kp, float ki, float kd, float n, float b, float c, float saturationMinimum, float saturationMaximum) {
    pid->integralError = 0;
    pid->integralResult = 0;
    pid->derivativeError = 0;
    pid->derivativeResult = 0;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->n = n;
    pid->b = b;
    pid->c = c;
    pid->saturationMinimum = -1000000;
    pid->saturationMaximum = 1000000;
    pid->previousTime = usecTimestamp();
    pid->saturationMinimum = saturationMinimum;
    pid->saturationMaximum = saturationMaximum;
}

float PID2DOFupdate(PID2DOFController* pid, float r, float y) {
    float T = (usecTimestamp() - pid->previousTime) / 1000000.;
    pid->previousTime = usecTimestamp();
    // proportional control
    float p = pid->kp * (pid->b * r - y);
    // integral control
    pid->integralResult = clamping(pid, r, y) * pid->ki * T/2. * ((r-y) + pid->integralError) + pid->integralResult;
    pid->integralError = r-y;
    // derivative control
    pid->derivativeResult = pid->kd * pid->n * (pid->c * r - y - pid->derivativeError) + pid->derivativeResult * (1 - pid->n * T);
    pid->derivativeError = pid->c * r - y;

    return p + pid->integralResult + pid->derivativeResult;
}


