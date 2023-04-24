
#ifndef MOTOR_H_
#define MOTOR_H_

#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include "stm32f0xx.h"

/* -------------------------------------------------------------------------------------------------------------
 *  Global Variable and Type Declarations
 *  -------------------------------------------------------------------------------------------------------------
 */
extern volatile int16_t error_integral;    // Integrated error signal
extern volatile int8_t adc_value;      // ADC measured motor current
extern volatile int16_t error;         // Speed error signal
extern volatile uint8_t Kp;            // Proportional gain
extern volatile uint8_t Ki;            // Integral gain
extern volatile uint8_t Kd;						 // Derivative gain

extern uint8_t enc_interr_ratio;

const uint8_t MOTOR_REDUCTION_RATIO = 45; // reduction ratio of 45, so 45 full turns for one revolution
const uint8_t MOTOR_MAX_RPM = 100;
const uint8_t MOTOR_COUNTS_PER_REV_SHAFT = 48; // counts for one turn of the shaft
const uint16_t MOTOR_COUNTS_PER_REV = MOTOR_COUNTS_PER_REV_SHAFT*MOTOR_REDUCTION_RATIO; // total counts for one full revolution
const uint8_t MOTOR_ENC_INTERR_RATIO = 2; // encoder counts per revolution

const uint16_t PID_INTEGRAL_CLAMP = 3200; // integral clamp val

const uint8_t MODER_RESET_STATE = 0x0;
const uint8_t MODER_GEN_OUT = 0x1;
const uint8_t MODER_ALT_FUNC = 0x2;
const uint8_t MODER_ANALOG_MODE = 0x3;

struct PID_Params
{
	int16_t prev_error;
	int16_t error_integral;
};

struct MotorStruct
{
	uint8_t dir;
	char pwm_port, in1_port, in2_port, encA_port, encB_port;
	uint8_t pwm_pin, in1_pin, in2_pin, encA_pin, encB_pin;
	uint8_t in1, in2;
	uint8_t alt_func;
	void *pwm_tim, *enc_tim; // Assume {}_tim vars are TIM_Typedef pointers
	uint8_t currentSpeed;
	uint8_t target;
	struct PID_Params pid_params;
};



extern int (*PID_Func)(struct MotorStruct* motorL);


/* -------------------------------------------------------------------------------------------------------------
 *  Motor Control and Initialization Functions
 * -------------------------------------------------------------------------------------------------------------
 */

// Sets up the entire motor drive system
void motor_init(struct MotorStruct *n_motorL, struct MotorStruct *n_motorR, uint32_t psc, uint32_t arr,
					uint32_t RCC_TIMxEN_PWM1, uint32_t RCC_TIMxEN_PWM2, uint32_t RCC_TIMxEN_ENC1, uint32_t RCC_TIMxEN_ENC2)
// Set the duty cycle of the PWM, accepts (0-100)
void pwm_setDutyCycle(uint8_t duty, TIM_TypeDef *motorTimer);

// Set PID function. 0 -> PID_Standard, 1 -> PID_Rotate, 2
void setPIDFunc(uint8_t plantMode);

// PID control for motors to reach a target speed.
int PID_Standard(struct MotorStruct *motor);

// PID control for motors to go forward or backward with feedback from imu (Z rotation). 
//void PID_Forward(void);

// PID control for motors to remain a certain distance from a wall to the right of the navigator.
int PID_WallFollow(struct MotorStruct *motor);

// PID control for motors to reach a certain 
int PID_Rotate(struct MotorStruct *motor);

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

/* -------------------------------------------------------------------------------------------------------------
 *  Internal-Use Initialization Functions
 * -------------------------------------------------------------------------------------------------------------
 */

// Sets up the PWM and direction signals to drive the H-Bridge
void pwm_init(struct MotorStruct *motorL, struct MotorStruct *motorR, uint32_t RCC_TIMxEN1, uint32_t RCC_TIMxEN2);
void pwm_timer_init(TIM_TypeDef *TIMx, uint32_t RCC_TIMxEN);

// Sets up encoder interface to read motor speed
void encoder_init(struct MotorStruct *motorL, struct MotorStruct *motorR, uint32_t psc, uint32_t arr, uint32_t RCC_TIMxEN1, uint32_t RCC_TIMxEN2);

// Sets up ADC to measure motor current
void ADC_init(void);

#endif /* MOTOR_H_ */