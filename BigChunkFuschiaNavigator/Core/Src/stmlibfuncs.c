#include "stmlibfuncs.h"

const uint8_t MODER_RESET_STATE = 0x0;
const uint8_t MODER_GEN_OUT = 0x1;
const uint8_t MODER_ALT_FUNC = 0x2;
const uint8_t MODER_ANALOG_MODE = 0x3;

// Sets pin to High or Low
void setPinState(void *gpioPTR, uint8_t pin, uint8_t out){

	if(gpioPTR == NULL){
		printf("Null pointer to GPIO Struct in setmoderBits\n");
		return;
	}
	if(out >= 0){
		((GPIO_TypeDef *)gpioPTR)->ODR |= (1 << pin);
		return;
	}
		
	((GPIO_TypeDef *)gpioPTR)->ODR &= ~(1 << pin);
}

// Mode should be a two bit number occupying bits [1:0]
void setModerBits(void *gpioPTR, uint8_t pin, uint8_t mode){
	if(mode >> 2 != 0){ // don't do anything if mode is not a 2 bit #
		printf("MODER mode must be two bits in length!");
		return;
	}
	
	uint8_t moder_pin_bit_start = 2*pin;

	if(gpioPTR == NULL){
		printf("Null pointer to GPIO Struct in setmoderBits\n");
		return;
	}
	
	((GPIO_TypeDef *)gpioPTR)->MODER &= ~(0x3 << moder_pin_bit_start);
	((GPIO_TypeDef *)gpioPTR)->MODER |= (mode << moder_pin_bit_start);
}	

// alt function should be a four bit number occupying bits [3:0]
void setAltFuncBits(void *gpioPTR, uint8_t pin, uint8_t alt_func){
	if(alt_func >> 4 != 0){
		printf("alt_func larger than 4 bits!");
		return;
	}
	
	if(gpioPTR == NULL){
		printf("Null pointer to GPIO Struct in setAltFuncBits\n");
		return;
	}
	
	uint8_t start = 0;
	if(pin < 8){ // use AFR low if pin < 8
		start = pin*4; // start of AFSEL{pin}
		
		((GPIO_TypeDef *)gpioPTR)->AFR[0] &= ~(0xF << start); // shift 1111_2 by start bits and clear AFSEL{pin}
		((GPIO_TypeDef *)gpioPTR)->AFR[0] |= (alt_func << start); // set AFR[0] to alt_func
		
		return;
	}

	start = (pin-8)*4; // start of AFSEL{pin}
		
	((GPIO_TypeDef *)gpioPTR)->AFR[1] &= ~(0xF << start); // shift 1111_2 by start bits and clear AFSEL{pin}
	((GPIO_TypeDef *)gpioPTR)->AFR[1] |= (alt_func << start); // set AFR[1] to alt_func
}

//GPIO_TypeDef* getGPIOStruct(char port){
//	if(port == 'a' || port == 'A'){
//		return GPIOA;
//	}
//	else if(port == 'b' || port == 'B'){
//		return GPIOB;
//	}
//	else if(port == 'c' || port == 'C'){
//		return GPIOC;
//	}
//	else if(port == 'd' || port == 'D'){
//		return GPIOD;
//	}
//	else if(port == 'e' || port == 'E'){
//		return GPIOE;
//	}
//	else if(port == 'f' || port == 'F'){
//		return GPIOF;
//	}
//	else{
//		return GPIOA;
//	}
//}