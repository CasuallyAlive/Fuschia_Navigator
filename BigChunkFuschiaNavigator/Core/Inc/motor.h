
#ifndef MOTOR_H_
#define MOTOR_H_

#include <stdio.h>
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include "stm32f0xx.h"
#include "stmlibfuncs.h"

/* -------------------------------------------------------------------------------------------------------------
 *  Global Variable and Type Declarations
 *  -------------------------------------------------------------------------------------------------------------
 */
 
extern volatile float error_integral_wall_follow;    // Integrated error signal
extern volatile float prev_error_wall_follow;         // Speed error signal
extern volatile int target_wall_follow; // target for wall following

extern volatile uint8_t Kp;            // Proportional gain
extern volatile uint8_t Ki;            // Integral gain
extern volatile uint8_t Kd;						 // Derivative gain

extern uint8_t enc_interr_ratio;

extern const uint8_t MOTOR_REDUCTION_RATIO; // reduction ratio of 45, so 45 full turns for one revolution
extern const uint8_t MOTOR_MAX_RPM;
extern const uint8_t MOTOR_COUNTS_PER_REV_SHAFT; // counts for one turn of the shaft
extern const uint16_t MOTOR_COUNTS_PER_REV; // total counts for one full revolution
extern const uint8_t MOTOR_ENC_INTERR_RATIO; // encoder counts per revolution

extern const uint16_t PID_INTEGRAL_CLAMP; // integral clamp val

extern const float AVG_TIME_ELAPSED;

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
	void *pwm_tim, *enc_tim, *pwm_gpio, *enc_gpio; // Assume {}_tim vars are TIM_Typedef pointers
	uint8_t currentSpeed;
	uint8_t target;
	struct PID_Params pid_params;
};



extern int (*PID_Func)(struct MotorStruct* motorL, int);

/* -------------------------------------------------------------------------------------------------------------
 *  Motor Control and Initialization Functions
 * -------------------------------------------------------------------------------------------------------------
 */
// Get adjust speed between [0, MOTOR_MAX_RPM]
uint8_t getAdjustedSpeed(uint8_t speed);
void resetPID(struct MotorStruct *motorL, struct MotorStruct *motorR);
// Sets up the entire motor drive system
//void motor_init(struct MotorStruct *n_motorL, struct MotorStruct *n_motorR, uint32_t psc, uint32_t arr,
//					uint32_t RCC_TIMxEN_PWM1, uint32_t RCC_TIMxEN_PWM2, uint32_t RCC_TIMxEN_ENC1, uint32_t RCC_TIMxEN_ENC2);
// Set the duty cycle of the PWM, accepts (0-100)
void pwm_setDutyCycle(uint8_t duty, TIM_TypeDef *motorTimer);

// Set PID function. 0 -> PID_Standard, 1 -> PID_Rotate, 2
void setPIDFunc(uint8_t plantMode);

// PID control for motors to reach a target speed.
int PID_Standard(struct MotorStruct *motor, int);

// PID control for motors to go forward or backward with feedback from imu (Z rotation). 
//void PID_Forward(void);

// PID control for motors to remain a certain distance from a wall to the right of the navigator.
int PID_WallFollow(struct MotorStruct *motor, int);
void setWallTargetDist(int new_target);

// PID control for motors to reach a certain 
int PID_Rotate(struct MotorStruct *motor, int);

/* -------------------------------------------------------------------------------------------------------------
 *  Internal-Use Initialization Functions
 * -------------------------------------------------------------------------------------------------------------
 */

// Sets up the PWM and direction signals to drive the H-Bridge
void pwm_init(struct MotorStruct *motorL, struct MotorStruct *motorR, uint32_t RCC_TIMxEN1, uint32_t RCC_TIMxEN2, uint8_t optReg1, uint8_t optReg2);
void pwm_init_motor(void *gpio_ptr, uint8_t pin, uint8_t alt_func);
void pwm_timer_init(TIM_TypeDef *TIMx, uint32_t RCC_TIMxEN, uint8_t optReg); //optReg: 1 enable on APB1, 0 enable on APB2

// Sets up encoder interface to read motor speed
void encoder_timer_init(TIM_TypeDef *TIMx, uint32_t RCC_TIMxEN, uint8_t opt);
void encoder_init(struct MotorStruct *motorL, struct MotorStruct *motorR, uint32_t psc, uint32_t arr, uint32_t RCC_TIMxEN1, uint32_t RCC_TIMxEN2, uint8_t optReg1, uint8_t optReg2);

// Sets up ADC to measure motor current
void ADC_init(void);

#endif /* MOTOR_H_ */