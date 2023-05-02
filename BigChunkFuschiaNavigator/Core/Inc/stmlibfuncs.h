#ifndef STMLIBFUNCS_H_
#define STMLIBFUNCS_H_

#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include "stm32f0xx.h"

extern const uint8_t MODER_RESET_STATE;
extern const uint8_t MODER_GEN_OUT;
extern const uint8_t MODER_ALT_FUNC;
extern const uint8_t MODER_ANALOG_MODE;

// Set moder bits at port
void setModerBits(char port, uint8_t pin, uint8_t mode);

// Get pointer to GPIO struct pointer at port
void* getGPIOStruct(char port);

// Get adjust speed between [0, MOTOR_MAX_RPM]
uint8_t getAdjustedSpeed(uint8_t speed);

// Sets alt function bits to mode for pin at port
void setAltFuncBits(char port, uint8_t pin, uint8_t alt_func);

// Set pin to high via ODR register
void setPinState(char port, uint8_t pin, uint8_t out);

#endif /* STMLIBFUNCS_H_ */