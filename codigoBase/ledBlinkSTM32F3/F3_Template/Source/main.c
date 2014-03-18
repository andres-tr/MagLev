#include <stm32f30x.h>

void delaybyms(unsigned int j);

int main( void ){
	unsigned char led_tab[] = {0x80,0x40,0x20,0x10,0x08,0x04,0x02,0x01,
0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80};
	
	char i = 0;
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // Enable GPIOC clock
	
	// PC[7:0] configuration
	GPIOC->MODER = GPIOC->MODER & 0xFFFF0000 | 0x00005555; // 0b01: Output
	GPIOC->OTYPER = GPIOC->OTYPER & 0xFFFFFF00; // 0b0 : PP (R)
	GPIOC->OSPEEDR = GPIOC->OSPEEDR & 0xFFFF0000 | 0x0000FFFF; // 0b11: 50MHz
	GPIOC->PUPDR = GPIOC->PUPDR & 0xFFFF0000; // 0b00: no PU/PD (R)
	
	for(;;){
		for (i = 0; i < 16; i++) {
		GPIOC->ODR = GPIOC->ODR & 0xFFFFFF00 | led_tab[i];
			delaybyms(1000);
			}
	}
}

void delaybyms(unsigned int j){
	unsigned int k,l;
}
