#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "transferfunction.h"

struct CONTROLLER_2DOF {
    TransferFunction ru;
    TransferFunction yu;
};

typedef struct CONTROLLER_2DOF controller2dof;

void setRUTransferFunction(controller2dof *controller, TransferFunction* transferFunction);
void setYUTransferFunction(controller2dof *controller, TransferFunction* transferFunction);
float processControllerInput(controller2dof *controller, float *r, float *y);


#endif // CONTROLLER_H
