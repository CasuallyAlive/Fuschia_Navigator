#include "stmlibfuncs.h"

const uint8_t MODER_RESET_STATE = 0x0;
const uint8_t MODER_GEN_OUT = 0x1;
const uint8_t MODER_ALT_FUNC = 0x2;
const uint8_t MODER_ANALOG_MODE = 0x3;

// Sets pin to High or Low
void setPinState(char port, uint8_t pin, uint8_t out){
	void *ptr = getGPIOStruct(port);
	if(ptr == NULL){
		printf("Null pointer to GPIO Struct in setmoderBits\n");
		return;
	}
	if(out >= 0){
		((GPIO_TypeDef *)ptr)->ODR |= (1 << pin);
		return;
	}
		
	((GPIO_TypeDef *)ptr)->ODR &= ~(1 << pin);
}

// Mode should be a two bit number occupying bits [1:0]
void setModerBits(char port, uint8_t pin, uint8_t mode){
	if(mode >> 2 != 0){ // don't do anything if mode is not a 2 bit #
		printf("MODER mode must be two bits in length!");
		return;
	}
	
	uint8_t moder_pin_bit_start = 2*pin;
	void *ptr = getGPIOStruct(port);
	if(ptr == NULL){
		printf("Null pointer to GPIO Struct in setmoderBits\n");
		return;
	}
	
	((GPIO_TypeDef *)ptr)->MODER &= ~(0x3 << moder_pin_bit_start);
	((GPIO_TypeDef *)ptr)->MODER |= (mode << moder_pin_bit_start);
}	

// alt function should be a four bit number occupying bits [3:0]
void setAltFuncBits(char port, uint8_t pin, uint8_t alt_func){
	if(alt_func >> 4 == 0){
		printf("alt_func larger than 4 bits!");
		return;
	}
	
	void *ptr = getGPIOStruct(port);
	if(ptr == NULL){
		printf("Null pointer to GPIO Struct in setAltFuncBits\n");
		return;
	}
	
	uint8_t start = 0;
	if(pin < 8){ // use AFR low if pin < 8
		start = pin*4; // start of AFSEL{pin}
		
		((GPIO_TypeDef *)ptr)->AFR[0] %= ~(0xF << start); // shift 1111_2 by start bits and clear AFSEL{pin}
		((GPIO_TypeDef *)ptr)->AFR[0] |= (alt_func << start); // set AFR[0] to alt_func
		
		return;
	}

	start = (pin-8)*4; // start of AFSEL{pin}
		
	((GPIO_TypeDef *)ptr)->AFR[1] %= ~(0xF << start); // shift 1111_2 by start bits and clear AFSEL{pin}
	((GPIO_TypeDef *)ptr)->AFR[1] |= (alt_func << start); // set AFR[1] to alt_func
}

void* getGPIOStruct(char port){
	void *ptr = NULL;
	switch(tolower(port)){
			case 'a':
				ptr = ((void *)(GPIOA));
				break;
			case 'b':
				ptr = ((void *)(GPIOB));
				break;
			case 'c':
				ptr = ((void *)(GPIOC));
				break;
			case 'd':
				ptr = ((void *)(GPIOD));
				break;
			case 'e':
				ptr = ((void *)(GPIOE));
				break;
			case 'f':
				ptr = ((void *)(GPIOF));
				break;
			default:
				printf("Port for GPIO must be A-F!");
				break; //Should never happen
		}
	return ptr;
}