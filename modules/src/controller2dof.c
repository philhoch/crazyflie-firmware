#include "../interface/controller2dof.h"
#include "transferfunction.h"



void setRUTransferFunction(controller2dof *controller, TransferFunction *transferFunction) {
    controller->ru = *transferFunction;
}

void setYUTransferFunction(controller2dof *controller, TransferFunction *transferFunction) {
    controller->yu = *transferFunction;
}

float processControllerInput(controller2dof *controller, float *r, float *y) {
    float result = 0.;
    result += evaluate(&controller->ru, r);
    result += evaluate(&controller->yu, y);
    return result;
}


