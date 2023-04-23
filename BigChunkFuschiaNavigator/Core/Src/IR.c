#include "stm32f0xx.h"
#define ARM_MATH_CM4
#include "IR.h"

void initOutputs()
{
	//PA4, PA5, PA6			PC7 PC8 PC9				PA9 PA10 PA11
	GPIOA->MODER |= (1<<8 | 1<<10 | 1<< 12);
	
	GPIOC->MODER |= (1<<14 | 1<<16 | 1<<18);
	
	GPIOA->MODER |= (1<<9 | 1<<10 | 1<<11);
}


void initLEDS() {
	//Initialize the LED pins to output:
	GPIOC->MODER |= (1<<12 | 1<<14 | 1<<16 | 1<<18);
}

void initIRSensors() {
	//Select a GPIO pin to use as the ADC input
	
	//PC0 -> Additional function ADC_IN10
	//PC3 -> Additional function ADC_IN13
	//PA1 -> Additional function ADC_IN1
	

	//MODER -> ANALOG Mode = 11
	//MODER[1:0] = [1:0]
	GPIOC->MODER |= (1<<0 | 1<<1);
	//set PC1 and PC2 to analog mode
	GPIOC->MODER |= (1<<2 | 1<<3 | 1<<4 | 1<<5 | 1<<6 | 1<<7);
	GPIOA->MODER |= (1<<0 | 1<<1);
	
	//PUPDR -> no pull up pull down -> 00
	//PUPDR[1:0] = [1:0]
	GPIOC->PUPDR &= ~(1<<0 | 1<<1);
	//set PC1 and PC2 to no PUPD
	GPIOC->PUPDR &= ~(1<<2 | 1<<3 | 1<<4 | 1<<5 | 1<<6 | 1<<7);
	GPIOA->PUPDR &= ~(1<<0 | 1<<1);
}

void ADCChangeChannel(int channel)
{
	ADC1-> CHSELR &= ~(1<<10 | 1<<13 | 1<<1);
	ADC1-> CHSELR |= (1<<channel);
}

//currently uses one channel (10) for single conversion
void ADCInitSingleConversion()
{
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
	//lets do single value stuff
	//ADC1->CFGR1 |= (1<<13);
	ADC1->CFGR1 &= ~(1<<13); //set to single conversion
	ADC1->CFGR1 |= ADC_CFGR1_DMAEN | ADC_CFGR1_DMACFG;
	
	
	//		CFGR1[11:10] = EXTEN[1:0] -- hardware disabled = 00
	ADC1->CFGR1 &= ~(1<<10 | 1<<13);
	
	//select/enable the input pin's channel for ADC conversion
		//channel selec 10 (using ADCIN10)
		//CHSELR bit 10
	//ADC1->CHSELR |= (1<<10);
	//ADC1->CHSELR &= ~(1<<10 | 1<<11 | 1<<12);
	ADC1->CHSELR |= (1<<10);
	
	//perform a self-calibration, enable, and start the ADC
	
			//1. Ensure that ADEN = 0 and DMAEN = 0.
			// ADEN is bit 0 in the CR register				DMAEN is bit 0 in the CFGR1 register
			ADC1->CR &= ~(1<<0);
			ADC1->CFGR1 &= ~(1<<0);
			//ADC1->CFGR1 |= ADC_CFGR1_SCANDIR
			
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
	
}

void ADCStartSingleConversion()
{
		ADC1->CR |= ADC_CR_ADSTART; //start ADC conversion
}

void ADCSingleConversion()
{
	while ((ADC1->ISR & ADC_ISR_EOC) == 0) // Wait end of conversion 
		 {
		 // For robust implementation, add here time-out management 
		 }
}



void ADCDisable() {
	/* (1) Stop any ongoing conversion */
	/* (2) Wait until ADSTP is reset by hardware i.e. conversion is stopped */
	/* (3) Disable the ADC */
	/* (4) Wait until the ADC is fully disabled */
	ADC1->CR |= ADC_CR_ADSTP; /* (1) */
	while ((ADC1->CR & ADC_CR_ADSTP) != 0) /* (2) */
	{
	 /* For robust implementation, add here time-out management */
	}
	ADC1->CR |= ADC_CR_ADDIS; /* (3) */
	while ((ADC1->CR & ADC_CR_ADEN) != 0) /* (4) */
	{
	 //
	}
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