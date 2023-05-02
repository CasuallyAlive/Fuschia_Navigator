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
#include "IR.h"

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
void TransmitChar(char dataToTransmit);
void TransmitString(char strToTransmit[]);

//void ReadRegister(void);
void ledCommand(char contents[]); 
void USART3_4_IRQHandler(void);
void NumberSwitch (int LED, char number);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int NEW_DATA = 0;
int CHARS_RECEIVED = 0;
char REGISTER_CONTENTS[2];
extern const uint8_t IDLE_STATE;
extern const uint8_t FOLLOW_WALL;
extern const uint8_t GO_FORWARD;
extern const uint8_t MANUAL;
extern const uint8_t STOP;
volatile uint8_t state; 
 
//for use with IR SENSORS
	int a;
	int b;
	int c;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN ;		//Enables the GPIOC peripheral clock
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN ;		//Enables the GPIOA peripheral clock
	GPIOC->MODER |= GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1 ; //Set PC4 (USART3_TX) and PC5 (USART3_RX) to alternate mode
	GPIOC->AFR[0] |= (1 << 16) | (1 <<20); //ENABLE AFRL for PC 4, 5
	GPIOC->MODER |= GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0 | 
	GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0; //Turn on LEDS
	GPIOC->ODR |= (1<<9);  //Set LED9 to high
	
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN ;
	 
	USART3->BRR = HAL_RCC_GetHCLKFreq()/9600; //set usart baud rate
	USART3->CR1 |= USART_CR1_TE; //enable TX
	USART3->CR1 |= USART_CR1_RE; //enable RX
	USART3->CR1 |= USART_CR1_RXNEIE; //enable RXNEIE 
	USART3->CR1 |= USART_CR1_UE; //Enable the USART
	
	NVIC_EnableIRQ(USART3_4_IRQn);
	NVIC_SetPriority(USART3_4_IRQn,1);
	
	
	//IR SENSOR SETUP
	initLEDS();
	initIRSensors();
	state = 0;
	
	ADCInitSingleConversion();
	
			int threshold1 = 40;
			int threshold2 = 20;
			int threshold3 = 20;
			int threshold4 = 105;
			
			
	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	TransmitString("Enter two bit command: ");
  while (1)
  { 
		
		
		 if (NEW_DATA){
			 ledCommand(REGISTER_CONTENTS);
			 NEW_DATA = 0;
			 TransmitString("Enter two bit command:");
		 }
		 
		   a = getSensorData(1);
			 b = getSensorData(2);
			 c = getSensorData(3);
		 
		if(a < threshold2)
		 {
				 GPIOC->ODR |= (1<<6);
		 }
		 else
		 {
				 GPIOC->ODR &= ~(1<<6);
		 }
		 
		 if(b < threshold2)
		 {
				 GPIOC->ODR |= (1<< 7);
		 }
		 else
		 {
				 GPIOC->ODR &= ~(1<<7);
		 }
		 
		 if(c < threshold2)
		 {
				 GPIOC->ODR |= (1<<8);
		 }
		 else
		 {
				 GPIOC->ODR &= ~(1<<8);
		 }  
		 //idle
		 if(state == 0)
		 {
			 //motors stop
		 }
		 //follow wall
		  if(state == 1)
		 {
			 
		 }
		 //go straight
		  if(state == 2)
		 {
			 //m1 = m2
		 }
		 //manual control
		  if(state == 3)
		 {
			 
		 }
		 //check threshold
		 
		/* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}
void TransmitChar(char dataToTransmit){
	
		while(!(USART3->ISR & USART_ISR_TXE)){
		
		}
	USART3->TDR = dataToTransmit;
}
void TransmitString(char strToTransmit[]){
	int i = 0;
	while(strToTransmit[i] != NULL){
		TransmitChar(strToTransmit[i]);
		i++;
	}
}

void ledCommand(char contents[]){
	switch(contents[0]){
		case 'w':
		case 'W':
			TransmitString("Go");
			NumberSwitch( 6 ,contents[1]);
		break;
		
		case 'a':
		case 'A':
			TransmitString("Left");
			NumberSwitch(9,contents[1]);
		break;
		
		case 's':
		case 'S':
			TransmitString("Reverse");
			NumberSwitch(7,contents[1]);
		break;
		
		case 'd':
		case 'D':
			TransmitString("Right");
			NumberSwitch(8,contents[1]);
		break;
		
		case 'g':
		case 'G':
			TransmitString("follow");
			NumberSwitch( 6 ,contents[1]);
		break;
		
		default:
			TransmitString("Not a valid command\r\n");
			 
	}
}

void NumberSwitch (int LED, char number){
	switch(number){
		
		case '0': 
			TransmitString(" striaght. \r\n");  
		//GPIOC->ODR &= ~(1<<LED);
		//motor 1 spin = motor 2 spin
			state = 2;
		break;
		
		case '1': 
			TransmitString(" backwords. \r\n");  
			//GPIOC->ODR &= ~(1<<LED);
			//motor 1 spin = motor 2 spin
			state = 3;
		break;
		
			case '2': 
			TransmitString(" wall. \r\n");  
		//GPIOC->ODR &= ~(1<<LED);
		//motor 1 spin = motor 2 spin
			state = 1;
		
		break;
		
		
		case '4': 
			TransmitString(" 45 degrees.  \r\n");
			//GPIOC->ODR ^= (1<<LED);		
			//motor 1 spin = -motor 2 spin
			//or motor 2 spin = -motor 1 spin
			state = 3;
		break;
		
		case '9': 
			TransmitString(" 90 degrees.  \r\n");
			//GPIOC->ODR ^= (1<<LED);
			//motor 1 spin = -motor 2 spin
			//or motor 2 spin = -motor 1 spin
			state = 3;
		break;
		 
		
		default:
			TransmitString("-0 = forward, 1 = backward, 4 = 45 degree turn, 9 = 90 degree turn \r\n");
	}
}
	
void USART3_4_IRQHandler(void){
	
	REGISTER_CONTENTS[CHARS_RECEIVED] = USART3->RDR;
	CHARS_RECEIVED++;
	if(CHARS_RECEIVED > 1){
		NEW_DATA = 1; // set flag to true
		CHARS_RECEIVED = 0;
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