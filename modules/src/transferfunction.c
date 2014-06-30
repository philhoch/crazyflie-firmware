#include "../interface/transferfunction.h"
//#include "../../utils/interface/linkedlist.h"
#include "FreeRTOS.h"
#include "log.h"


//static void shiftLinkedList(linkedlist *list, float value) {
//    linkedlistItem* item = list->front;
//    while(item != list->rear) {
//        if(item->next == list->rear) {
//            list->rear = item; // new rear element
//            item->next->next = list->front;
//            list->front = item->next;
//            *(float*)(list->front->item) = value; // set new value
//            return;
//        }
//        item = item->next;
//    }
//}

//void initializeInputRegister(TransferFunction *transferFunction) {
//    transferFunction->inputRegister.front = 0;
//    transferFunction->inputRegister.rear = 0;
//    int i;
////    for(i = 0; i < transferFunction->numeratorSize - 1; i++) {
////        linkedlistItem *item = (linkedlistItem *) pvPortMalloc(sizeof(linkedlistItem)); //= malloc(sizeof(linkedlistItem));
////        float* value = (float*)  pvPortMalloc(sizeof(float));
////        *value = 0.;
////        item->item = value;
////        item->next = 0;
////        linkedlistAddItem(&(transferFunction->inputRegister), item);
////    }
//}

//void initializeOutputRegister(TransferFunction *transferFunction) {
//    transferFunction->outputRegister.front = 0;
//    transferFunction->outputRegister.rear = 0;
//    int i;
////    for(i = 0; i < transferFunction->denumeratorSize; i++) {
////        linkedlistItem *item = (linkedlistItem *) pvPortMalloc(sizeof(linkedlistItem));
////        float* value = (float*)  pvPortMalloc(sizeof(float));
////        *value = 0.;
////        item->item = value;
////        item->next = 0;
////        linkedlistAddItem(&(transferFunction->outputRegister), item);
////    }
//}

//static float output;
//static float numerator;
//static float denumerator;
//static int counter = 0;

static void shift(float *reg, int length, float value) {
    int i = 0;
    if(length > 0)
        reg[0] = value;
    for(i = length-1; i > 0; --i) {
        reg[i] = reg[i-1];
    }
}

static void updateRegister(TransferFunction *transferFunction, float input, float output) {
    shift(transferFunction->inputRegister, transferFunction->numeratorSize, input);
    shift(transferFunction->outputRegister, transferFunction->denumeratorSize ,output);
}


void setNominatorCoefficients(TransferFunction* transferFunction, float* numeratorCoefficients, float *inputRegister, uint16_t numeratorSize) {
    transferFunction->numeratorCoefficients = numeratorCoefficients;
    transferFunction->numeratorSize = numeratorSize;
    transferFunction->inputRegister = inputRegister;
}

void setDenominatorCoefficients(TransferFunction* transferFunction, float* denumeratorCoefficients, float *outputRegister, uint16_t denumeratorSize) {
    transferFunction->denumeratorCoefficients = denumeratorCoefficients;
    transferFunction->denumeratorSize = denumeratorSize;
    transferFunction->outputRegister = outputRegister;
}


float evaluate(TransferFunction* transferFunction, float *input) {
    float numerator = 0.;
    int i;
    if(transferFunction->numeratorSize > 0) {
        numerator = transferFunction->numeratorCoefficients[0] * *input;
        for(i = 1; i < transferFunction->numeratorSize; i++) { //
            numerator += transferFunction->numeratorCoefficients[i] * transferFunction->inputRegister[i];
        }
    }
    float denumerator = 0.;
    for(i = 0; i < transferFunction->denumeratorSize; i++) { //
        denumerator += transferFunction->denumeratorCoefficients[i] * transferFunction->outputRegister[i];
    }
    float output = numerator - denumerator;
    updateRegister(transferFunction, *input, output);
    return output;
}





//LOG_GROUP_START(tf)
//LOG_ADD(LOG_FLOAT, numerator, &numerator)
//LOG_ADD(LOG_FLOAT, denumerator, &denumerator)
//LOG_ADD(LOG_FLOAT, output, &output)
//LOG_GROUP_STOP(forces)





