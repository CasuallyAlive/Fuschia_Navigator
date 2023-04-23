#include "stm32f0xx.h"
#define ARM_MATH_CM4


void initOutputs();


void initLEDS();

void initIRSensors();



//currently uses one channel (10) for single conversion
void ADCInitSingleConversion();

void ADCStartSingleConversion();

void ADCSingleConversion();
void ADCChangeChannel(int channel);


void ADCDisable();
