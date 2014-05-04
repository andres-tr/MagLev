#include <stm32f30x.h>



uint16_t ADC1ConvertedValue = 0; 
uint16_t ADC1ConvertedVoltage = 0; 
uint16_t calibration_value = 0; 
volatile uint32_t TimingDelay = 0;

void Delay (uint32_t nTime);
void ADCConfig();
void readADC();
void TIM3_Config();
int flag = 0;
float corrienteSensor = 0;
float setPoint = 0.14;
float feedback = 0;
float error_actual = 0;
float u_actual = 0;
float u_pasado = 0;
float error_pasado = 0;
int dutyC;

int main(void) {
	ADCConfig();
	TIM3_Config();

	while (1){
		//Si la lectura del sensor entra en el rango se realiza lo siguiente
			//Traducir la referencia distancia (m) a corriente con la formula de la pendiente
			//Leer el sensor
			readADC();
		//2263 2863
			if(ADC1ConvertedVoltage > 2100 && ADC1ConvertedVoltage < 2900){
				flag = 0;
				//Convierto de voltaje del sensor pasando por el voltaje del micro a corriente
				feedback = -0.00093*ADC1ConvertedVoltage + 2.626;
				
				error_actual = feedback - setPoint; //Considerar signos por la ganancia negativa
				
				//u_actual = ecuacion en diferencias
					u_actual = (error_actual*259.8) - (error_pasado*2.126) - (u_pasado*0.4168);
				
				//u_anterior = u_actual
					u_pasado = u_actual;
				//e_anterior = e_actual
					error_pasado = error_actual;
				
//Investigar el signo de lasvariables por el tipo 
			
		
				//Convertir el dato de la ley de control (A) al equivalente en PWM con sus saturacion 
				if (u_actual <= 0){
					TIM3->CCR3 = 0;
				}else if(u_actual > 0.5){
					TIM3->CCR3 = 1024;
				}else{
					dutyC = (int)((2325.5*u_actual) + 99.683);
					//dutyC = ceil((2325.5*u_actual) + 99.683);
					TIM3->CCR3 = dutyC;
				}
				
		}else{
				flag = 1;
				TIM3->CCR3 = 0;
		}
	} 
}

void SysTick_Handler(void){
	TimingDelay--; 
}

void Delay (uint32_t nTime){
  TimingDelay = nTime;
	while (TimingDelay !=0); 
}
void ADCConfig(void){
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
}


void readADC(void){
	while(!(ADC1->ISR & ADC_ISR_EOC)); // Test EOC flag
		ADC1ConvertedValue = ADC1->DR; //GetADC1converteddata
		ADC1ConvertedVoltage = (ADC1ConvertedValue * 3300)/(4095); //Computethevoltage
}

void TIM3_Config(void) {
	/*
	For PWM - PA1
	*/
	
	RCC->AHBENR|=1<<19;// enable Port C clock   slide 16 GPIOS PowerPoint
	RCC->APB1ENR|=RCC_APB1ENR_TIM3EN;// enable CLOCK of TIM3
	GPIOC->MODER|=2<<(8*2);//PC8 as alternate funtion mode 
	GPIOC->AFR[1]|=2;// Select AF2 for PC8  TIMER 3 CHANNEL 3
	
	TIM3->PSC=0; // Values for frecuency = 10 000 Hz
	TIM3->CCR3=0;
	TIM3->ARR=2465;
	TIM3->CCMR2|= TIM_CCMR2_OC3M_2|TIM_CCMR2_OC3M_1;
	TIM3->CCER|=TIM_CCER_CC3E;//Enable output compare
	TIM3->CR1|=TIM_CR1_CEN;//ENABLE TIMER
}