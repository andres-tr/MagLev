#include <stm32f30x.h>

void GPIO_Config(void) {
	// PC4 configuration (TX)
	RCC->AHBENR |= 1 << 19; // enable GPIOC clock
	GPIOC->MODER |= 2 << (4*2); // GPIO_Mode_AF
	GPIOC->OTYPER |= 1 << (4*1); // GPIO_OType_OD
	GPIOC->OSPEEDR |= 3 << (4*2); // GPIO_Speed_50MHz
  GPIOC->PUPDR &= ~(3 << (4*2)); // GPIO_PuPd_NOPULL
	GPIOC->AFR[0] |= 7 << (4*4); // AF7
	// PC5 configuration (RX)
	GPIOC->MODER |= 2 << (5*2); // GPIO_Mode_AF
	GPIOC->AFR[0] |= 7 << (5*4); // AF7
}

void USART1_Config(void){
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN; //EnableUSART1clock
	USART1->BRR = 72000000/115200;
	USART1->CR1 &= ~USART_CR1_OVER8; // Oversampling mode = 16
	USART1->CR1 &= ~USART_CR1_M; // Word length = 8 bits
	USART1->CR1 &= ~USART_CR1_PCE; // No parity
	USART1->CR1 |= USART_CR1_TE; // Transmitter enable
	USART1->CR1 |= USART_CR1_RE; // Receiver enable
	USART1->CR1 |= USART_CR1_UE; // USART enable
	USART1->CR2 &= ~(USART_CR2_STOP_1 | USART_CR2_STOP_0); //onestopbit
}

uint8_t SendChar (uint8_t ch) {
  while (!(USART1->ISR & USART_ISR_TXE));
  USART1->TDR = (ch & 0xFF);
  return (ch);
}

uint8_t GetChar (void){ 
	while (!(USART1->ISR & USART_ISR_RXNE));
	return ((uint8_t)(USART1->RDR & 0xFF)); 
}
	


int main( void ){
	GPIO_Config();
	USART1_Config();
	
	//Start your code HERE

	for(;;);
	
}
