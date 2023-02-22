#include "exti.h"

static const unsigned char extiIrqn[] = {
	EXTI0_IRQn,     // 0
	EXTI1_IRQn,     // 1
	EXTI2_TSC_IRQn, // 2
	EXTI3_IRQn,     // 3
	EXTI4_IRQn,     // 4
	EXTI9_5_IRQn,   // 5
	EXTI9_5_IRQn,   // 6
	EXTI9_5_IRQn,   // 7
	EXTI9_5_IRQn,   // 8
	EXTI9_5_IRQn,   // 9
	EXTI15_10_IRQn, // 10
	EXTI15_10_IRQn, // 11
	EXTI15_10_IRQn, // 12
	EXTI15_10_IRQn, // 13
	EXTI15_10_IRQn, // 14
	EXTI15_10_IRQn  // 15
};

int attachInterrupt(GPIO_TypeDef *port, 
                    unsigned char numBit,
                    unsigned char mode)
{

	if(numBit > 15) return -1;
	//start (no nop, as there is enough code before SYSCFG access)
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	//config 
	int crField;
	if     (port == GPIOA) crField = 0;
	else if(port == GPIOB) crField = 1;
	else if(port == GPIOC) crField = 2;
	else if(port == GPIOD) crField = 3;
    //else if(port == GPIOE) crField = 4; /* no GPIOE on stm32F303 */
	else if(port == GPIOF) crField = 5;
	else return -2;
	//interrupt mask register
	EXTI->IMR  |= 1 << numBit;
	const int crId   = numBit >> 2;
	const int crOffset = ((numBit & 0x3)<<2);
	const int crMask = 0xF << crOffset;
	//raz previous CR config
	SYSCFG->EXTICR[crId] &= ~crMask;
	SYSCFG->EXTICR[crId] |= crField << crOffset;
	//mode (CHANGE is RISING and FALLING)
	if(mode & RISING) {
		EXTI->RTSR |= 1<<numBit;
	} else {
		EXTI->RTSR &= ~(1<<numBit);
	}
	if(mode & FALLING) {
		EXTI->FTSR |= 1<<numBit;
	} else {
		EXTI->FTSR &= ~(1<<numBit);
	}
	//enable IRQ
	NVIC_EnableIRQ(extiIrqn[numBit]);
	return 0;
}

