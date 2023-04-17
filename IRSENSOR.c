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

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(){
	
		__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	
	//Initialize the LED pins to output:
	
	GPIOC->MODER |= (1<<12 | 1<<14 | 1<<16 | 1<<18);
	
	
	//Select a GPIO pin to use as the ADC input
	
	//PC0 -> Additional function ADC_IN10
	

	//MODER -> ANALOG Mode = 11
	//MODER[1:0] = [1:0]
	GPIOC->MODER |= (1<<0 | 1<<1);
	//PUPDR -> no pull up pull down -> 00
	//PUPDR[1:0] = [1:0]
	GPIOC->PUPDR &= ~(1<<0 | 1<<1);
	
	
	
	//*******************************************************************************************
	//							Connect the output (center pin) of a potentiometer to the 									*		
	//	input pin - the other two pins of the potentiometer should be connected to 3V and GND		*
	//*******************************************************************************************
	
	
	
	//enable the ADC1 in the RCC peripheral 
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; 
	
		//ADEN bit 0 -> set to 1 to enable
	//ADC1->CR |= (1<<0);
	
	//configure the ADC to 8-bit resolution, continuous conversion mode, 
	//   hardware triggers disabled (software trigger only)
			//CFGR1[4:3] = RES[1:0]
			// 10 -- 8 bits
			//CFGR1[13] = CONT[0] -> set to 1 for continuous
	ADC1->CFGR1 &= ~(1<<4 | 1<<3);
	ADC1->CFGR1 |= (1<<4);
	ADC1->CFGR1 |= (1<<13);
	
	
	//		CFGR1[11:10] = EXTEN[1:0] -- hardware disabled = 00
	ADC1->CFGR1 &= ~(1<<10 | 1<<11);
	
	//select/enable the input pin's channel for ADC conversion
		//channel selec 10 (using ADCIN10)
		//CHSELR bit 10
	ADC1->CHSELR |= (1<<10);
	
	
	//perform a self-calibration, enable, and start the ADC
	
			//1. Ensure that ADEN = 0 and DMAEN = 0.
			// ADEN is bit 0 in the CR register				DMAEN is bit 0 in the CFGR1 register
			ADC1->CR &= ~(1<<0);
			ADC1->CFGR1 &= ~(1<<0);
			
			//2. Set ADCAL = 1.
				//ADCAL is bit 31 in the CR register
			ADC1->CR |= (1<<31);
			//3. Wait until ADCAL = 0.
			while(1)
			{
				if(!(ADC1->CR >> 31))
				{
					break;
				}
			}
			ADC1->CR |= (1<<0);
			
			//4. The calibration factor can be read from bits 6:0 of ADC_DR.
			
	//enable, and start the ADC ->>>
			
			
			//Follow this procedure to enable the ADC:
			//1. Clear the ADRDY bit in ADC_ISR register by programming this bit to 1. (bit 0 in ISR)
			ADC1->ISR |= (1<<0);
			
			//2. Set ADEN = 1 in the ADC_CR register.
			ADC1->CR |= (1<<0);
			
			//3. Wait until ADRDY = 1 in the ADC_ISR register and continue to write ADEN = 1
				/*Wait until ADRDY = 1 in the ADC_ISR register and continue to write ADEN = 1
				(ADRDY is set after the ADC startup time). This can be handled by interrupt if the
				interrupt is enabled by setting the ADRDYIE bit in the ADC_IER register. */
			
			while(1)
			{
				if(ADC1->ISR & 1)
				{
					break;
				}
			}
			ADC1->CR |= (1<<2);
			
	
	
	int data = 0;
			int threshold1 = 60;
			int threshold2 = 70;
			int threshold3 = 80;
			int threshold4 = 100;

			
			while (1)
		{
		
		
		//DAC->DHR8R1 = triangle_table[counter];
		//counter++;
		//if(counter > 31)
		//{
		//	counter = 0;
		//}
		//HAL_Delay(1);
    // in the main loop, read the ADC data register and turn on/off LEDs depending on the value
		//use four increasing threshold values, each LED should have a minimum ADC value/voltage to turn on
		//DR[15:7]
		
		data = ADC1->DR;
		//GPIOC->ODR |= (1<<6);
		if(ADC1->DR > threshold4)
		{
			GPIOC->ODR |= (1<<6 | 1<<7 | 1<<8 | 1<<9);
		}
		else if(ADC1->DR > threshold3)
		{
			GPIOC->ODR |= (1<<6 | 1<<7 | 1<<8 );
			GPIOC->ODR &= ~(1<<9);
		}
		else if(ADC1->DR > threshold2)
		{
			GPIOC->ODR |= (1<<6 | 1<<7);
			GPIOC->ODR &= ~(1<<9 | 1<<8);
		}
		else if(ADC1->DR > threshold1)
		{
			GPIOC->ODR |= (1<<6);
			GPIOC->ODR &= ~(1<<9 | 1<<8 | 1<<7);
		}
		else
		{
			GPIOC->ODR &= ~(1<<9 | 1<<8 | 1<<7 | 1<<6);
			
		} 
		//As the voltage on the input pin increases, the LEDs should light one-by-one
		
		//if the pin voltage decreases below the threshold for a LED it should turn off
		
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
