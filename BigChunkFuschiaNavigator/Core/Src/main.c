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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f0xx.h"
#define ARM_MATH_CM4


#include "IR.h"

void GPIO_Init(void);
void TIM2_us_Delay(uint32_t delay);

uint32_t data;
double time, dist;
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

/* USER CODE END PV */
void ADCStartUp(int adcChannel);
void ADCDisable();

void SystemClock_Config(void);





/**
  * @brief  The application entry point.
  * @retval int
  */
int main(){
	
		__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	
	
	
	initLEDS();
	initIRSensors();
	
	ADCInitSingleConversion();
	
	
	
			//ADCStartUp(10);
			//ADC1->CHSELR |= (1<<13);
			//ADC1->CHSELR |= (1<<1);
	
	int data = 0;
			int threshold1 = 40;
			int threshold2 = 60;
			int threshold3 = 80;
			int threshold4 = 105;

	int ADC_Result[] = {0,0,0};
			volatile int counter = 0;
			volatile int newCount = 0;
	
	int resultCount[] = {0,0,0};
	
			while (1)
		{
			ADCChangeChannel(10);
			ADCStartSingleConversion();
			ADCSingleConversion();
			ADC_Result[0] = ADC1->DR;
			
			ADCChangeChannel(13);
			ADCStartSingleConversion();
			ADCSingleConversion();
			ADC_Result[1] = ADC1->DR;
			
			ADCChangeChannel(1);
			ADCStartSingleConversion();
			ADCSingleConversion();
			ADC_Result[2] = ADC1->DR;
			
			//ADCSingleConversion();
			//ADC_Result[1] = ADC1->DR;
			
		//	ADCSingleConversion();
		//	ADC_Result[2] = ADC1->DR;
			if(ADC_Result[0] > threshold2)
         {
             GPIOC->ODR |= (1<<6);
         }
         else
         {
             GPIOC->ODR &= ~(1<<6);
         }
         
         if(ADC_Result[1] > threshold2)
         {
             GPIOC->ODR |= (1<<7);
         }
         else
         {
             GPIOC->ODR &= ~(1<<7);
         }
         
         if(ADC_Result[2] > threshold2)
         {
             GPIOC->ODR |= (1<<8);
         }
         else
         {
             GPIOC->ODR &= ~(1<<8);
         }  
			
			/*while(counter < 50000)
         {
             counter++;
                    
         }*/
				 GPIOC->ODR ^= (1<<9);
				 counter = 0;
		}
			
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


void GPIO_Init() {
	RCC->AHBENR |= 1<<17; //enable GPIOA clock
	GPIOA->MODER |= 1<<10; //set PA5 to output mode
	
	GPIOA->MODER &= ~(0x00003000); // set PA6 to input mode
}


void TIM2_us_Delay(uint32_t delay) {
	RCC->APB1ENR |= 1;
	TIM2->ARR = (int)(delay/0.0625);
	TIM2->CNT = 0;
	TIM2->CR1 |= 1;
	while(!(TIM2->SR & TIM_SR_UIF)) {}
	TIM2->SR &= ~(0x0001);
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
