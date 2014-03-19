#include <stm32f30x.h>
//Blinking Led PC0

void delaybyms(unsigned int j);

int main( void ){
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // Enable GPIOC clock
	
	// PC[7:0] configuration
	//Bear in mind the hierarchy of operands
	//Reset all regiserts before asigning a value
	GPIOC->MODER = GPIOC->MODER & 0xFFFF0000 | 0x00005555; // 0b01: Output mode 
	GPIOC->OTYPER = GPIOC->OTYPER & 0xFFFFFF00; // 0b0 : PP (R)
	GPIOC->OSPEEDR = GPIOC->OSPEEDR & 0xFFFF0000 | 0x0000FFFF; // 0b11: 50MHz
	GPIOC->PUPDR = GPIOC->PUPDR & 0xFFFF0000; // 0b00: no PU/PD (R)
	
	
	for(;;){
		GPIOC->ODR = 0xFFFFFF01;
		delaybyms(500);
		GPIOC->ODR = 0xFFFFFF00;
		delaybyms(500);
		
	}
}

//delay function 
void delaybyms(unsigned int j){
	unsigned int k,l;
	for(k=0;k<j;k++)
		for(l=0;l<1427;l++);
}
