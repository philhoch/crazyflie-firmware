#ifndef TRANSFERFUNCTION_H
#define TRANSFERFUNCTION_H

//#include "linkedlist.h"


typedef struct {
    float* numeratorCoefficients;
    float* denumeratorCoefficients;
    float* inputRegister;
    float* outputRegister;
    //linkedlist inputRegister;
    //linkedlist outputRegister;
    uint16_t numeratorSize;
    uint16_t denumeratorSize;
} TransferFunction;

//void initializeInputRegister(TransferFunction *transferFunction);
//void updateInputRegister(TransferFunction* transferFunction, float input);
//void updateOutputRegister(TransferFunction* transferFunction, float output);
//void updateRegister(TransferFunction* transferFunction, float input, float output);
void setNominatorCoefficients(TransferFunction* transferFunction, float* numeratorCoefficients, float* inputRegister, uint16_t numeratorSize);
void setDenominatorCoefficients(TransferFunction* transferFunction, float* denumeratorCoefficients, float* outputRegister, uint16_t denumeratorSize);
float evaluate(TransferFunction* transferFunction, float *input);

#endif // TRANSFERFUNCTION_H
