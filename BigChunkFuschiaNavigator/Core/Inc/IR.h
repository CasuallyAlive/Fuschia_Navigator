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
float getSensorData(int sensor);
float getSensor1cm(int data);
float getSensor2cm(int data);
float getSensor3cm(int data);

void ADCDisable();
