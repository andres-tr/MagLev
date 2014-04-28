#include <stm32f30x.h>

int main( void )
{
	
	//Start your code HERE

	for(;;);
	
}

void TIM2_Config(  ){
    //-----------------------------------------------------PA1 Configuration------------------------------------------------------//
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // [ 17 ] IOPAEN = 1 I/O port A clock enabled.
    GPIOA->MODER |= 2 << ( 1*2 );// [ 3:2 ] MODER1[ 1:0 ] = 10b Alternate function on PIN A1
    GPIOA->OSPEEDR |= 3 << ( 1*2 ); // [ 3:2 ] OSPEEDR1[ 1:0 ] = 11b 50 MHz High-speed on PIN A1
    GPIOA->AFR[ 0 ] |= 1 << ( 1*4 ); // [ 7:4 ] AFRL1[ 3:0 ] = 0001b AF1( TIM2_CH2 ) on PIN A1    
    
    //-----------------------------------------------------TIM2 Configuration-----------------------------------------------------//
    
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // [ 0 ] TIM2EN = 1 TIM2 clock enabled
    TIM2->PSC = 0; // [ 15:0 ] PSC[ 15:0 ] = 0  Calculated prescaler = PSC + 1
    TIM2->ARR = (720000UL<<1); // [ 31:0 ] ARR[ 31:0 ] = 144800 TIM2 Up-Counts while <= to                                                                                          //   generated.
    TIM2->CCR2 = 108000; // [ 31:0 ] CCR2[ 31:0 ] = 10800 Calculated pulse-width of  20%. Same pattern as ARR. 
    TIM2->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2; // [ 24, 14:12 ] OC2M[ 3:0 ] = 0110b PWM Mode 1: Up-counter.
    CNT>=CCR2 -> OC2REF=0.
    TIM2->CCER |= TIM_CCER_CC2E; // [ 4 ] CC2E = 1 TIM2 channel 1 as output on // PIN A1.
    TIM2->CR1 |= TIM_CR1_CEN; // [ 0 ] CEN = 1 TIM2 Counter enabled.
}



void DAC1_Config(void){
    RCC->APB1ENR |= RCC_APB1ENR_DACEN; //enable DAC1 clock
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN; //enable GPIOA clock
    GPIOA->MODER |= 3<<(4*2); //PA4 Analog mode
    DAC->CR |= DAC_CR_EN1; //enable DAC
}