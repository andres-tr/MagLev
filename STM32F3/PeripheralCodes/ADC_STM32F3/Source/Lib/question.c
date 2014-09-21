#include <stm32f30x.h>

void conf( void )
{

	uint8_t i;	
	RCC->AHBENR |= (0x11 << 17);	
	GPIOA->MODER   &= 0xFFFFFFFC;	
	for( i=8; i<16; i++ )
	{	
		GPIOE->MODER   |= 1U << ( i*2 );
		GPIOE->OSPEEDR |= 3U << ( i*2 );
		GPIOE->PUPDR   |= 1U << ( i*2 );		
	}	
	
}

uint32_t button( void )
{
	
	uint32_t j;
	for(j=2500000;j>0;j--);
	return (GPIOA->IDR & 0x00000001);	
	
}

void A( void )
{

	GPIOE->ODR = 1 << 8;
	
}

void B( void )
{

	GPIOE->ODR = 1 << 9;
	
}

void C( void )
{

	GPIOE->ODR = 1 << 10;
	
}

void D( void )
{

	GPIOE->ODR = 1 << 11;
	
}

void E( void )
{

	GPIOE->ODR = 1 << 12;
	
}

void F( void )
{

	GPIOE->ODR = 1 << 13;
	
}

void G( void )
{

	GPIOE->ODR = 1 << 14;
	
}

void H( void )
{

	GPIOE->ODR = 1 << 15;
	
}
