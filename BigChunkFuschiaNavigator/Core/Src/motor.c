/* -------------------------------------------------------------------------------------------------------------
 *  Motor Control and Initialization Functions
 * -------------------------------------------------------------------------------------------------------------
 */
#include "motor.h"

volatile float error_integral_wall_follow = 0;    // Integrated error signal
volatile float prev_error_wall_follow = 0;         	// Speed error signal
volatile int target_wall_follow = 16;						// in cm

volatile uint8_t Kp = 0;            	// Proportional gain
volatile uint8_t Ki = 0;            	// Integral gain
volatile uint8_t Kd = 0;						 // Derivative gain

const float AVG_TIME_ELAPSED = 0.056; // in seconds
const uint8_t MOTOR_REDUCTION_RATIO = 45; // reduction ratio of 45, so 45 full turns for one revolution
const uint8_t MOTOR_MAX_RPM = 130;
const uint8_t MOTOR_COUNTS_PER_REV_SHAFT = 48; // counts for one turn of the shaft
const uint16_t MOTOR_COUNTS_PER_REV = MOTOR_COUNTS_PER_REV_SHAFT*MOTOR_REDUCTION_RATIO; // total counts for one full revolution
const uint8_t MOTOR_ENC_INTERR_RATIO = 2; // encoder counts per revolution

const uint16_t PID_INTEGRAL_CLAMP = 3200; // integral clamp val


int (*PID_Func)(struct MotorStruct* motorL, int) = &PID_Standard;
uint8_t enc_interr_ratio = MOTOR_ENC_INTERR_RATIO;

void setPIDWeights(uint8_t n_Kp, uint8_t n_Ki, uint8_t n_Kd){
	Kp = n_Kp;
	Ki = n_Ki;
	Kd = n_Kd;
}
void setWallTargetDist(int new_target){
	if(new_target > 80 || new_target < 0)
		return;
	target_wall_follow = new_target;
}
//void setIRDistance

// SetsPIDFunc for control.
void setPIDFunc(uint8_t plantMode){
	if(plantMode > 2){
		printf("PlantMode selected is not supported!");
		return;
	}
	
	if(plantMode == 0)
		PID_Func = &PID_Standard;
	else if(plantMode == 1)
		PID_Func = &PID_Rotate;
	else
		PID_Func = &PID_WallFollow;
	
	return;
}

// returns speed that is within limits of motors.
uint8_t getAdjustedSpeed(uint8_t speed){
	if(speed < 0)
		return 0;
	if(speed > MOTOR_MAX_RPM)
		return MOTOR_MAX_RPM;
	
	return speed;
}

// Sets up the entire motor drive system
//void motor_init(struct MotorStruct *motorL, struct MotorStruct *motorR, uint32_t psc, uint32_t arr,
//									uint32_t RCC_TIMxEN_PWM1, uint32_t RCC_TIMxEN_PWM2, uint32_t RCC_TIMxEN_ENC1, uint32_t RCC_TIMxEN_ENC2){
//	
//	pwm_init(motorL, motorR, RCC_TIMxEN_PWM1, RCC_TIMxEN_PWM2);
//	encoder_init(motorL, motorR, psc, arr, RCC_TIMxEN_ENC1, RCC_TIMxEN_ENC2);
//	//ADC_init();
//}

// Sets target speed for param motor to the newTarget speed if is within the capabilities of the motor.
void setTargetSpeed(struct MotorStruct *motor, uint8_t newTarget){
	motor->target = getAdjustedSpeed(newTarget);
	
	return;
}

// Sets the motor direction.
void setMotorDirection(struct MotorStruct *motor, uint8_t newDir){
	if(newDir < 0 || newDir > 1)
		printf("Couldn't set the new direction! Need 0 <= NewDir <= 1!");
		return;
	
	// CW -> dir == 1 -> {in1,in2} = {1,0}; CCW -> dir == 0 -> {in1,in2} = {0,1}
	motor->dir = newDir;
	
	motor->in1 = newDir;
	motor->in2 = ~newDir;

	setPinState(motor->pwm_gpio, motor->in1_pin, motor->in1);
	setPinState(motor->pwm_gpio, motor->in2_pin, motor->in2);
}

// Setup PWM for motor
void pwm_init_motor(void *gpio_ptr, uint8_t pin, uint8_t alt_func){

	// Set alt func mode at pin for H-bridge PWM output
	setModerBits(gpio_ptr, pin, MODER_ALT_FUNC);
	
	// Set AFR at pin to alt_func
	setAltFuncBits(gpio_ptr, pin, alt_func);
	return;
}

// Setup for motor direction pins
void dir_init_motor(void *gpio_ptr, char port2, uint8_t in1, uint8_t in2){
	
	// clear bits for pins at ports and set to general purpose output mode
	setModerBits(gpio_ptr, in1, MODER_GEN_OUT);
	setModerBits(gpio_ptr, in2, MODER_GEN_OUT);
	
	// Initialize in1 to high
	setPinState(gpio_ptr, in1, 0x1);
	
	// Initialize in2 to high
	setPinState(gpio_ptr, in2, 0x0);
	
	return;
}
// setup pwm timer
void pwm_timer_init(TIM_TypeDef *TIMx, uint32_t RCC_TIMxEN, uint8_t optReg){
	/// Set up PWM timer
	if(optReg == 1){
		RCC->APB1ENR |= RCC_TIMxEN;
	}
	else{
		RCC->APB2ENR |= RCC_TIMxEN;
	}
	TIMx->CR1 = 0;                         // Clear control registers
	TIMx->CCMR1 = 0;                       // (prevents having to manually clear bits)
	TIMx->CCER = 0;

	/// Set output-compare CH1 to PWM1 mode and enable CCR1 preload buffer
	TIMx->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE);
	TIMx->CCER |= TIM_CCER_CC1E;           // Enable capture-compare channel 1
	TIMx->PSC = 1;                         // Run timer on 24Mhz
	TIMx->ARR = 1200;                      // PWM at 20kHz
	TIMx->CCR1 = 0;                        // Start PWM at 0% duty cycle
	
	TIMx->CR1 |= TIM_CR1_CEN;              // Enable timer
}

// Sets up the PWM and direction signals to drive the H-Bridge
void pwm_init(struct MotorStruct *motorL, struct MotorStruct *motorR, uint32_t RCC_TIMxEN1, uint32_t RCC_TIMxEN2, uint8_t optReg1, uint8_t optReg2) {		
			
	// Pin setup for left motor //
	pwm_init_motor(motorL->pwm_gpio, motorL->pwm_pin, motorL->alt_func); // initialize pin PA4 with alt function TIM14_CH1 (AF4).

	dir_init_motor(motorL->pwm_gpio,motorL->in2_port, motorL->in1_pin,motorL->in2_pin); // initialize pins PA5, PA6 with PA5->High, PA6->Low for CW motor direction.
	setMotorDirection(motorL, 1);
	
	pwm_timer_init(motorL->pwm_tim, RCC_TIMxEN1, optReg1);
	// Pin setup for left motor Done //
	
	// Pin setup for right motor //
	pwm_init_motor(motorR->pwm_gpio, motorR->pwm_pin, motorR->alt_func);

	dir_init_motor(motorR->pwm_gpio,motorR->in2_port, motorR->in1_pin,motorR->in2_pin);
	setMotorDirection(motorR, 1);
	
	pwm_timer_init(motorR->pwm_tim, RCC_TIMxEN2, optReg2);
	// Pin setup for right motor Done //
}

// Set the duty cycle of the PWM, accepts (0-100)
void pwm_setDutyCycle(uint8_t duty, TIM_TypeDef *motorTimer) {
    if(duty <= 100) {
        motorTimer->CCR1 = ((uint32_t)duty*motorTimer->ARR)/100;  // Use linear transform to produce CCR1 value
        // (CCR1 == "pulse" parameter in PWM struct used by peripheral library)
    }
}

//void PID_FORWARD(

void encoder_timer_init(TIM_TypeDef *TIMx, uint32_t RCC_TIMxEN, uint8_t ccmrRegOpt){
	// Set up encoder interface (TIMx encoder input mode)
	RCC->APB1ENR |= RCC_TIMxEN;
	if(ccmrRegOpt == 1){
		TIMx->CCMR1 = 0;
	}
	else{
		TIMx->CCMR2 = 0;
	}
	TIMx->CCER = 0;
	TIMx->SMCR = 0;
	TIMx->CR1 = 0;
	if(ccmrRegOpt == 1){
		TIMx->CCMR1 |= (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0);   // TI1FP1 and TI2FP2 signals connected to CH1 and CH2
	}
	else{
		TIMx->CCMR2 |= (TIM_CCMR2_CC3S_0 | TIM_CCMR2_CC4S_0);
	}
	TIMx->SMCR |= (TIM_SMCR_SMS_1 | TIM_SMCR_SMS_0);        // Capture encoder on both rising and falling edges
	TIMx->ARR = 0xFFFF;                                     // Set ARR to top of timer (longest possible period)
	TIMx->CNT = 0x7FFF;                                     // Bias at midpoint to allow for negative rotation
	// (Could also cast unsigned register to signed number to get negative numbers if it rotates backwards past zero
	//  just another option, the mid-bias is a bit simpler to understand though.)
	TIMx->CR1 |= TIM_CR1_CEN;                               // Enable timer
}

// Sets up encoder interface to read motor speed, uses TIM6 for ISR update event.
void encoder_init(struct MotorStruct *motorL, struct MotorStruct *motorR, uint32_t psc, uint32_t arr, 
										uint32_t RCC_TIMxEN1, uint32_t RCC_TIMxEN2, uint8_t optReg1, uint8_t optReg2) {
    
	// Set up encoder for motorL
	setModerBits(motorL->enc_gpio, motorL->encA_pin, MODER_ALT_FUNC);
	setModerBits(motorL->enc_gpio, motorL->encB_pin, MODER_ALT_FUNC);

  /// Set up timer for motorL encoder
	encoder_timer_init(motorL->enc_tim, RCC_TIMxEN1, optReg1);
	// Done with Setup for motorL encoder
	
	// Set up encoder for motorR
	setModerBits(motorR->enc_gpio, motorR->encA_pin, MODER_ALT_FUNC);
	setModerBits(motorR->enc_gpio, motorR->encB_pin, MODER_ALT_FUNC);

  /// Set up timer for motorR encoder
	encoder_timer_init(motorR->enc_tim, RCC_TIMxEN2, optReg2);
	// Done witth setup for motorR encoder
	
	// Configure a second timer (TIM6) to fire an ISR on update event
	// Used to periodically check and update speed variable
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
	
	// Select PSC and ARR values that give an appropriate interrupt rate

	TIM6->PSC = psc;
	TIM6->ARR = arr;
	
	TIM6->DIER |= TIM_DIER_UIE;             // Enable update event interrupt
	TIM6->CR1 |= TIM_CR1_CEN;               // Enable Timer

	NVIC_EnableIRQ(TIM6_DAC_IRQn);          // Enable interrupt in NVIC
	NVIC_SetPriority(TIM6_DAC_IRQn,2);
}


//void ADC_init(void) {

    // Configure PA1 for ADC input (used for current monitoring)
    //GPIOA->MODER |= (GPIO_MODER_MODER1_0 | GPIO_MODER_MODER1_1);

    // Configure ADC to 8-bit continuous-run mode, (asynchronous clock mode)
  //  RCC->APB2ENR |= RCC_APB2ENR_ADCEN;

    //ADC1->CFGR1 = 0;                        // Default resolution is 12-bit (RES[1:0] = 00 --> 12-bit)
    //ADC1->CFGR1 |= ADC_CFGR1_CONT;          // Set to continuous mode
    //ADC1->CHSELR |= ADC_CHSELR_CHSEL1;      // Enable channel 1

    //ADC1->CR = 0;
    //ADC1->CR |= ADC_CR_ADCAL;               // Perform self calibration
    //while(ADC1->CR & ADC_CR_ADCAL);         // Delay until calibration is complete

    //ADC1->CR |= ADC_CR_ADEN;                // Enable ADC
    //while(!(ADC1->ISR & ADC_ISR_ADRDY));    // Wait until ADC ready
    //ADC1->CR |= ADC_CR_ADSTART;             // Signal conversion start
//}

void resetPID(struct MotorStruct *motorL, struct MotorStruct *motorR){
	Kp = Ki = Kd = 0;
	error_integral_wall_follow = prev_error_wall_follow = 0;
	motorL->pid_params = (struct PID_Params){.prev_error = 0, .error_integral = 0};
	motorR->pid_params = (struct PID_Params){.prev_error = 0, .error_integral = 0};
}

int PID_WallFollow(struct MotorStruct *motor, int refVal){
	
	int curr_error = target_wall_follow - refVal;
	
	int prop = 	Kp*curr_error;
	int integ = Ki*curr_error + error_integral_wall_follow;
	// Derivative Gain = kd * {delta}Error/{delta}Time
	float der = (float) Kd*((float)(curr_error - prev_error_wall_follow)/(float)(AVG_TIME_ELAPSED)); 
	
	// clamp integral
	if(integ < 0) 
		integ =0;
	else if(integ > PID_INTEGRAL_CLAMP)
		integ = PID_INTEGRAL_CLAMP;
	// Update error values for next PID call
	error_integral_wall_follow = integ;
	prev_error_wall_follow = curr_error;
	
	// get output
	int output = prop + integ + (int)floor(der);

	/// TODO: Divide the output into the proper range for output adjustment (Divide by five which is equal to maximum integral clamp)
	output = output >> 5;
	/// TODO: Clamp the output value between 0 and MOTOR_MAX_RPM 
	return output;
}

int PID_Rotate(struct MotorStruct *motor, int target){
	
	return 0;
}

// Uses encoder values for plant (reaches a target speed)
int PID_Standard(struct MotorStruct *motor, int target) {
    
	/* Run PI control loop
	 *
	 * Make sure to use the indicated variable names. This allows STMStudio to monitor
	 * the condition of the system!
	 *
	 * target_rpm -> target motor speed in RPM
	 * motor_speed -> raw motor speed in encoder counts
	 * error -> error signal (difference between measured speed and target)
	 * error_integral -> integrated error signal
	 * Kp -> Proportional Gain
	 * Ki -> Integral Gain
	 * output -> raw output signal from PI controller
	 * duty_cycle -> used to report the duty cycle of the system 
	 * adc_value -> raw ADC counts to report current
	 *
	 */
	// 0.0375 s or 37.5 ms *Change this*
	int16_t current_rpm = (motor->currentSpeed / enc_interr_ratio);
	/// calculate error signal and write to "error" variable
	int error = target - current_rpm;
	
	motor->pid_params.prev_error = error;

	int16_t error_proportional = error*Kp;
	/// Calculate integral portion of PI controller, write to "error_integral" variable
	int error_integral = motor->pid_params.error_integral + error*Ki;
	
	/// Clamp the value of the integral to a limited positive range
	if(error_integral < 0) 
		error_integral =0;

	else if(error_integral > PID_INTEGRAL_CLAMP)
		error_integral = PID_INTEGRAL_CLAMP;

	motor->pid_params.error_integral = error_integral;
	
	/// TODO: Calculate proportional portion, add integral and write to "output" variable

	int16_t output = error_proportional + error_integral;

	/// TODO: Divide the output into the proper range for output adjustment (Divide by five which is equal to maximum integral clamp)
	output = output >> 5;
	/// TODO: Clamp the output value between 0 and MOTOR_MAX_RPM 
	output = getAdjustedSpeed(output);
		
	// Read the ADC value for current monitoring, actual conversion into meaningful units 
	// will be performed by STMStudio
	//if(ADC1->ISR & ADC_ISR_EOC) {   // If the ADC has new data for us
			//adc_value = ADC1->DR;       // Read the motor current for debug viewing
	//}
	return output;
}