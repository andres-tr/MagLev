#include <stm32f30x.h>

void Delay (uint32_t nTime);

uint16_t ADC1ConvertedValue = 0; 
uint16_t ADC1ConvertedVoltage = 0; 
uint16_t calibration_value = 0; 
volatile uint32_t TimingDelay = 0;

int main(void) {
	// At this stage the microcontroller clock tree is already configured
	RCC->CFGR2 |= RCC_CFGR2_ADCPRE12_DIV2; // Configure the ADC clock 
	RCC->AHBENR |= RCC_AHBENR_ADC12EN; //EnableADC1clock
	// Setup SysTick Timer for 1 µsec interrupts
	if (SysTick_Config(SystemCoreClock / 1000000)){
	// Capture error
	while (1){} 
	}

	// ADC Channel configuration PC1 in analog mode
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; //GPIOCPeriphclockenable 
	GPIOC->MODER |= 3 << (1*2); //ConfigureADCChannel7asanaloginput

	/* Calibration procedure */	
	ADC1->CR &= ~ADC_CR_ADVREGEN;
	ADC1->CR |= ADC_CR_ADVREGEN_0; // 01: ADC Voltage regulator enabled 
	Delay(10); // Insert delay equal to 10 µs
	ADC1->CR &= ~ADC_CR_ADCALDIF; //calibrationinSingle-endedinputsMode. 
	ADC1->CR |= ADC_CR_ADCAL; //StartADCcalibration
	// Read at 1 means that a calibration in progress.
	while (ADC1->CR & ADC_CR_ADCAL); //waituntilcalibrationdone 
	calibration_value = ADC1->CALFACT; //GetCalibrationValueADC1

	// ADC configuration	
	ADC1->CFGR |= ADC_CFGR_CONT; //ADC_ContinuousConvMode_Enable
	ADC1->CFGR &= ~ADC_CFGR_RES; //12-bitdataresolution
	ADC1->CFGR &= ~ADC_CFGR_ALIGN; //Rightdataalignment

	/* ADC1 regular channel7 configuration */	
	ADC1->SQR1 |= ADC_SQR1_SQ1_2 | ADC_SQR1_SQ1_1 | ADC_SQR1_SQ1_0; // SQ1 = 0x07, start converting ch7
	ADC1->SQR1 &= ~ADC_SQR1_L; // ADC regular channel sequence length = 0 => 1 conversion/sequence
	ADC1->SMPR1 |= ADC_SMPR1_SMP7_1 | ADC_SMPR1_SMP7_0; //=0x03=>samplingtime7.5ADCclockcycles
	ADC1->CR |= ADC_CR_ADEN; //EnableADC1
	while(!ADC1->ISR & ADC_ISR_ADRD); //waitforADRDY

	ADC1->CR |= ADC_CR_ADSTART; //StartADC1SoftwareConversion

	while (1) {
		while(!(ADC1->ISR & ADC_ISR_EOC)); // Test EOC flag
		ADC1ConvertedValue = ADC1->DR; //GetADC1converteddata
		ADC1ConvertedVoltage = (ADC1ConvertedValue * 3300)/(4096); //Computethevoltage
	} 
}

void SysTick_Handler(void){
	TimingDelay--; 
}

void Delay (uint32_t nTime){
  TimingDelay = nTime;
	while (TimingDelay !=0); 
}