#include "main.h"
#include <stm32f30x.h>
#include <string.h>
#include <stdio.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Extern variables ----------------------------------------------------------*/

void Delay (uint32_t nTime);
void ADCConfig();
uint16_t readADC();


char output[5];
uint16_t ADC1ConvertedValue = 0; 
uint16_t ADC1ConvertedVoltage = 0; 
uint16_t calibration_value = 0; 
char buf[6];

//volatile uint32_t TimingDelay = 0;
extern __IO uint8_t Receive_Buffer[64];
extern __IO  uint32_t Receive_length ;
extern __IO  uint32_t length ;
uint8_t Send_Buffer[64];
uint32_t packet_sent=1;
uint32_t packet_receive=1;
volatile uint32_t TimingDelay = 0;


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : main.
* Descriptioan    : Main routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int main(void)
{
	//Send_Buffer[0] = (uint8_t ) "Hello";
	//length = 11;
  Set_System();
  Set_USBClock();
  USB_Interrupts_Config();
  USB_Init();
	ADCConfig();
  
  while (1)
  {
    if (bDeviceState == CONFIGURED)
    {
      //CDC_Receive_DATA();
			ADC1ConvertedVoltage = readADC();
			sprintf(output,"%4d\n",ADC1ConvertedVoltage);
			//CDC_Send_DATA ((unsigned char*)&ADC1ConvertedVoltage,2);
			CDC_Send_DATA ((unsigned char*)&output,5);
			
      /*Check to see if we have data yet */
      /*
			if (Receive_length  != 0)
      {
        if (packet_sent == 1)
          //CDC_Send_DATA ((unsigned char*)Receive_Buffer,Receive_length);
					ADC1ConvertedVoltage = readADC();
					//sprintf (buf, "%u", ADC1ConvertedVoltage);
					CDC_Send_DATA ((unsigned char*)&ADC1ConvertedVoltage,2);
					
					//CDC_Send_DATA(Send_Buffer[0],length);
        Receive_length = 0;
      }
			*/
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


void ADCConfig(){
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


void SysTick_Handler(void){
	TimingDelay--; 
}

void Delay (uint32_t nTime){
  TimingDelay = nTime;
	while (TimingDelay !=0); 
}



uint16_t readADC(){
	while(!(ADC1->ISR & ADC_ISR_EOC)); // Test EOC flag
	ADC1ConvertedValue = ADC1->DR; //GetADC1converteddata
	ADC1ConvertedVoltage = (ADC1ConvertedValue * 3300)/(4096); //Computethevoltage
	return ADC1ConvertedVoltage;
}