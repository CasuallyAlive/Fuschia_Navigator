/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

#define PI 4.0 * atan(1.0)
#define deg2rad(val) (val/180.0)*PI

#define DEG1 deg2rad(3*45.0)
#define DEG2 deg2rad(2*45.0)
#define DEG3 deg2rad(1*45.0)
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile struct MotorStruct *motorL_e;
volatile struct MotorStruct *motorR_e;

const int IDLE_STATE = 0;
const int FOLLOW_WALL = 1;
const int GO_FORWARD = 2;
const int MANUAL = 3;
const int STOP = 4;

volatile int state = IDLE_STATE;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Encoder interrupt to calculate motor speed, also manages PI controller
void TIM6_DAC_IRQHandler(void) {
	/* Calculate the motor speed in raw encoder counts
	 * Note the motor speed is signed! Motor can be run in reverse.
	 * Speed is measured by how far the counter moved from center point
	 */

	motorL_e->currentSpeed = ((TIM_TypeDef*) motorL_e->enc_tim)->CNT - 0x7FFF;
	((TIM_TypeDef*) motorL_e->enc_tim)->CNT = 0x7FFF;
	
	motorR_e->currentSpeed = ((TIM_TypeDef*) motorR_e->enc_tim)->CNT - 0x7FFF;
	((TIM_TypeDef*) motorR_e->enc_tim)->CNT = 0x7FFF;

	//motor_speed = (TIM3->CNT - 0x7FFF);
	//TIM3->CNT = 0x7FFF; // Reset back to center point

	// Call the PID update function
	if(state == GO_FORWARD){
		// output for MotorL
		struct MotorStruct motor1 = *motorL_e;
		int output1 = PID_Func(&motor1, motor1.target); // warning here because using motorL as val (which is fine)
		motorL_e->pid_params = motor1.pid_params;

		pwm_setDutyCycle(output1, motorL_e->pwm_tim);

		// output for MotorR
		struct MotorStruct motor2 = *motorR_e;
		int output2 = PID_Func(&motor2, motor2.target);
		motorR_e->pid_params = motor2.pid_params;
		
		pwm_setDutyCycle(output2, motorR_e->pwm_tim);
	}
	// Follow wall state PID update.
	else if(state == FOLLOW_WALL){
		
		float sensID, magn, deg; // magnitude of point
		retrieveRefParams(&sensID, &magn, &deg);
		
		int ref = ceil(magn*cos(deg));
		int output = PID_Func(NULL, ref); // careful Kd
		
		uint8_t ms1 = motorL_e->target, ms2 = motorR_e->target;
		// Point between h1 and h2 or at h1
		if(sensID >= 1.0 || sensID <= 1.5){
			// set speed for motorL_e
			ms1 -= output;
			// set speed for motorR_e
			ms2 += output;
		}
		// Point between h2 and h3 or at h3
		else if(sensID >= 2.0 || sensID <= 2.5){
			// set speed for motorL_e
			ms1 += output;
			// set speed for motorR_e
			ms2 -= output;
		}
		// Send pwm pulse to motorL
		pwm_setDutyCycle(ms1, motorL_e->pwm_tim);
		motorL_e->currentSpeed = ms1;
		
		// Send pwm pulse to motorR
		pwm_setDutyCycle(ms2, motorR_e->pwm_tim);
		motorL_e->currentSpeed = ms2;
	}
	TIM6->SR &= ~TIM_SR_UIF;        // Acknowledge the interrupt
}

void retrieveRefParams(float *sensID, float *magn, float *deg){
	// if wall detected on left and wall on left is the closest wall
	if(h1 < h2 && h1 < h3){
		*sensID = 1.0;
		*deg = DEG1;
		*magn = h1; // get adjacent distance of rover to wall
	}
	//if wall directly in front of robot and side sensors less than h2
	else if(h2 < h1 && h2 < h3){
		float m;
		float deg_m;
		//if next closest point is h1
		if(h1 < h3){
			*sensID = 1.5; // 2.0 - 0.5
			m = h2;
			deg_m = DEG1;
		}
		else{
			*sensID = 2.5; // 2.0 + 0.5
			m = h3;
			deg_m = DEG3;
		}
		*deg = deg_m - deg2rad(0.5);
		*magn = ((h2+m)/2); 
	}
	//if wall detected on right and wall on right is the closest wall
	else if(h3 < h1 && h3 < h2){
		*sensID = 3.0;
		*deg = DEG3;
		*magn = h3;
	}
	// enter here if general assumption is that sensors have the same relative distance from all walls, go straight!
	else{
		*sensID = 1.0;
		*deg = DEG1;
		*magn = target_wall_follow;
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
	
	// struct for the right motor
	struct MotorStruct motorL = {
	.dir = 1,
	.pwm_port = 'A', .in1_port = 'A', .in2_port = 'A', .encA_port = 'B', .encB_port = 'B',
	.pwm_pin = 4, .in1_pin = 5, .in2_pin = 6, .encA_pin = 4, .encB_pin = 5,
	.in1 = 1, .in2 = 0,
	.alt_func = 0x4,
	.pwm_tim = TIM14, .enc_tim = TIM3, // Assume {}_tim vars are TIM_Typedef pointers
	.currentSpeed = 0,
	.target = 100,
	};
	motorL_e = &motorL;
	
	// struct for the right motor
	struct MotorStruct motorR = {
	.dir = 1,
	.pwm_port = 'A', .in1_port = 'A', .in2_port = 'A', .encA_port = 'B', .encB_port = 'B',
	.pwm_pin = 2, .in1_pin = 8, .in2_pin = 9, .encA_pin = 10, .encB_pin = 11,
	.in1 = 1, .in2 = 0,
	.alt_func = 0,
	.pwm_tim = TIM15, .enc_tim = TIM2, // Assume {}_tim vars are TIM_Typedef pointers
	.currentSpeed = 0,
	.target = 100,
	};
	motorR_e = &motorR;
	
	// Produces an interrupt period of 0.056s or 18Hz
	uint32_t psc = 15;
	uint32_t arr = 27778; 
	
	// Setup GPIO stuff
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN;
	pwm_init(&motorL, &motorR, RCC_APB1ENR_TIM14EN, RCC_APB2ENR_TIM15EN, 1, 0); // TIM14 on APB1, and TIM15 on APB2
	encoder_init(&motorL, &motorR, psc, arr, RCC_APB1ENR_TIM3EN, RCC_APB1ENR_TIM2EN, 1, 0); // Setup ch1,2 on TIM3 and setup ch3,4 on TIM2
	
	setPIDFunc(0); // Set PID to go "straight"
	// End Moder Setup
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
