#ifndef PID2DOFCONTROLLER_H
#define PID2DOFCONTROLLER_H


typedef struct {
    float kp;
    float ki;
    float kd;
    float n; // filter coefficient
    float b; // setpoint weight
    float c; // setpoint weight
    float saturationMinimum;
    float saturationMaximum;
    uint64_t previousTime;
    float integralError;
    float integralResult;
    float derivativeError;
    float derivativeResult;
} PID2DOFController;

void PID2DOFinitialize(PID2DOFController* pid, float kp, float ki, float kd, float n, float b, float c, float saturationMinimum, float saturationMaximum);

float PID2DOFupdate(PID2DOFController* pid, float r, float y);

#endif // PID2DOFCONTROLLER_H
