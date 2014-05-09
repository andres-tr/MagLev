/**
  ******************************************************************************
  * @file    main.c
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    21-January-2013
  * @brief   Virtual Com Port Demo main file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include "hw_config.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_pwr.h"
#include "string.h"
#include "stdio.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Extern variables ----------------------------------------------------------*/
extern __IO uint8_t Receive_Buffer[64];
extern __IO  uint32_t Receive_length ;
extern __IO  uint32_t length ;
uint8_t Send_Buffer[64];
uint32_t packet_sent=1;
uint32_t packet_receive=1;

/* Control variables ----------------------------------------------------------*/
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
float setPoint = 0.49;
float feedback = 0;
float error_actual = 0;
float u_actual = 0;
float u_pasado = 0;
float error_pasado = 0;
int dutyC;
char buf[5];
float corriente = 0;

/* Private function prototypes -----------------------------------------------*/
/* Functions ---------------------------------------------------------*/
void Delay (uint32_t nTime);
void ADCConfig();
void readADC();
void TIM3_Config();

/*******************************************************************************
* Function Name  : main.
* Descriptioan    : Main routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int main(void)
{
  Set_System();
  Set_USBClock();
  USB_Interrupts_Config();
  USB_Init();
  ADCConfig();
	TIM3_Config();
	
  while (1)
  {
    if (bDeviceState == CONFIGURED)
    {
			/*
      CDC_Receive_DATA();
      //Check to see if we have data yet 
      if (Receive_length  != 0)
      {
        if (packet_sent == 1)
          CDC_Send_DATA ((unsigned char*)Receive_Buffer,Receive_length);
        Receive_length = 0;
      }
			*/
			
			
//___________________________ Controlador ___________________________ 
			//Si la lectura del sensor entra en el rango se realiza lo siguiente
			//Traducir la referencia distancia (m) a corriente con la formula de la pendiente
			//Leer el sensor
			readADC();
			//2263 2863
			if(ADC1ConvertedVoltage > 1900 && ADC1ConvertedVoltage < 2900){
				flag = 0;
				//Convierto de voltaje del sensor pasando por el voltaje del micro a corriente
				//feedback = -0.00093*ADC1ConvertedVoltage + 2.626; 
				feedback = -0.0009303822*ADC1ConvertedVoltage + 2.626037234; 
				//Punto suma
				error_actual = feedback - setPoint; //Considerar signos por la ganancia negativa
				
				//u_actual = ecuacion en diferencias
				u_actual = (error_actual*259.8) - (error_pasado*2.126) + (u_pasado*0.4168); //Ley de control numero 1
				//u_actual = (error_actual*459.8) - (error_pasado*384.6) + (u_pasado*0.2015); //Ley de control numero 2
				//u_actual = (error_actual*256.5) - (error_pasado*202.3) + (u_pasado*0.6211); //Ley de control numero 3
				//u_actual = (error_actual*13314.625) - (error_pasado*9310.2); //Ley de control numero 4
				//u_anterior = u_actual
				//u_pasado = u_actual;
				//e_anterior = e_actual
				
				if(error_actual > -0.05	&& error_actual < 0.05){
					corriente = (u_actual*0.0001) + setPoint;
				}else{
					corriente = (u_actual*0.1) + setPoint;
				}
				
				
				error_pasado = error_actual;
		
				//Convertir el dato de la ley de control (A) al equivalente en PWM con su saturacion 
				if (corriente <= 0.000001){
					TIM3->CCR3 = 0;
				}else if(corriente >.75){ //.7 //.65
					TIM3->CCR3 = 1797;
				}else{
					dutyC = (int)((2325.5*(corriente)) + 99.683);
					//dutyC = ceil((2325.5*u_actual) + 99.683);
					TIM3->CCR3 = dutyC;
				}
				
				sprintf(buf,"%d\n",ADC1ConvertedVoltage);
				CDC_Send_DATA((unsigned char*)&buf,5);
				
		}else{
				flag = 1;
				TIM3->CCR3 = 0;
				error_actual = 1.0000000;
				sprintf(buf,"%d\n",ADC1ConvertedVoltage);
				CDC_Send_DATA((unsigned char*)&buf,5);
		}
//___________________________ Controlador ___________________________ 
    }
  }
} 

#ifdef USE_FULL_ASSERT
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert_param error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert_param error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
}
#endif

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
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
