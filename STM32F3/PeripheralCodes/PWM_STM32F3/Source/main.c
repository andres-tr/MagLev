#include <stm32f30x.h>
void TIM2_Config();


int main( void ){
	TIM2_Config();
	//Start your code HERE

	for(;;);
	
}


void TIM2_Config(void) {
	/*
	For PWM - PA1
	*/
	
	RCC->AHBENR|=1<<19;// enable Port C clock   slide 16 GPIOS PowerPoint
	RCC->APB1ENR|=RCC_APB1ENR_TIM3EN;// enable CLOCK of TIM3
	GPIOC->MODER|=2<<(8*2);//PC8 as alternate funtion mode 
	GPIOC->AFR[1]|=2;// Select AF2 for PC8  TIMER 3 CHANNEL 3
	
	TIM3->PSC=0; // Values for frecuency = 10 000 Hz
	TIM3->CCR3=700;
	TIM3->ARR=2465;
	TIM3->CCMR2|= TIM_CCMR2_OC3M_2|TIM_CCMR2_OC3M_1;
	TIM3->CCER|=TIM_CCER_CC3E;//Enable output compare
	TIM3->CR1|=TIM_CR1_CEN;//ENABLE TIMER
}

