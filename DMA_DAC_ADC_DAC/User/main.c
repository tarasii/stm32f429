/**
 *	Keil project for DAC with DMA signal feature
 *
 *	Before you start, select your target, on the right of the "Load" button
 *
 *	@author		Tilen Majerle
 *	@email		tilen@majerle.eu
 *	@website	http://stm32f4-discovery.com
 *	@ide		Keil uVision 5
 *	@packs		STM32F4xx Keil packs version 2.2.0 or greater required
 *	@stdperiph	STM32F4xx Standard peripheral drivers version 1.4.0 or greater required
 */
/* Include core modules */
#include "stm32f4xx.h"
/* Include my libraries here */
#include "defines.h"
#include "tm_stm32f4_dac_signal.h"
#include "tm_stm32f4_delay.h"
#include "tm_stm32f4_disco.h"
#include "tm_stm32f4_adc.h"
#include <math.h>

#define adc_buff_len 512
#define dac_buff_len 32

uint32_t toggle_ms;

uint16_t adc_buff[adc_buff_len];
uint16_t dac_buff[dac_buff_len];

uint32_t uint32_time_diff(uint32_t now, uint32_t before)
{
  return (now >= before) ? (now - before) : (UINT32_MAX - before + now);
}

void ADC_InitADC() {
	ADC_InitTypeDef ADC_InitStruct;
	ADC_CommonInitTypeDef ADC_CommonInitStruct;
//	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	DMA_InitTypeDef DMA_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

  DMA_DeInit(DMA2_Stream0);
  
  DMA_StructInit(&DMA_InitStructure);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(ADC1->DR);	     		 
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&adc_buff;  
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;                         
  DMA_InitStructure.DMA_BufferSize = adc_buff_len;                     
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	     		 
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                    
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; 
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;	     	  
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                              
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;	                     	 
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream0, &DMA_InitStructure);		

  DMA_Cmd(DMA2_Stream0, ENABLE);

	/* Enable ADC clock */
	//RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
		
	/* Init ADC settings */
	ADC_InitStruct.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStruct.ADC_ExternalTrigConv = DISABLE;
	//ADC_InitStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
	ADC_InitStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	//ADC_InitStruct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_TRGO;
	ADC_InitStruct.ADC_NbrOfConversion = 1;
	ADC_InitStruct.ADC_ScanConvMode = ENABLE;
	ADC_InitStruct.ADC_Resolution = TM_ADC1_RESOLUTION;
	ADC_Init(ADC1, &ADC_InitStruct);
	
	ADC_CommonInitStruct.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
	ADC_CommonInitStruct.ADC_Mode = ADC_DualMode_AlterTrig;
	ADC_CommonInitStruct.ADC_Prescaler = ADC_Prescaler_Div4;
	ADC_CommonInitStruct.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStruct);
	
  ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_56Cycles);

	ADC_DiscModeCmd(ADC1, DISABLE);
	ADC_EOCOnEachRegularChannelCmd(ADC1, ENABLE);
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
	ADC_DMACmd(ADC1, ENABLE);
		
	/* Enable ADC */
	//ADC1->CR2 |= ADC_CR2_ADON;
  ADC_Cmd(ADC1, ENABLE);
	
//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
//		
//	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
//	TIM_TimeBaseStructure.TIM_Period = 525;
//	TIM_TimeBaseStructure.TIM_Prescaler = 0;
//	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV2;
//	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

//	TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update);
//	
//	TIM_Cmd(TIM2, ENABLE);	
}

void calc_sin(void){
	uint16_t i;
	double tmp;
	
	for(i=0;i<dac_buff_len;i++){
		//tmp = (1+sin(2*i*3.14/dac_buff_len))*4096/2;
		//dac_buff[i]=(uint16_t)tmp;
		tmp = sin(i*3.14/dac_buff_len)*4096;
		dac_buff[i]=(uint16_t)tmp;
	}
}

int main(void) {	

	SystemInit();
	
	TM_DELAY_Init();

	TM_DISCO_LedInit();
	
	calc_sin();
	
	TM_DAC_SIGNAL_Init(TM_DAC1, TIM4);
	TM_DAC_SIGNAL_Init(TM_DAC2, TIM5);
	
	/* Output predefined triangle signal with frequency of 5kHz */
	//TM_DAC_SIGNAL_SetSignal(TM_DAC1, TM_DAC_SIGNAL_Signal_Triangle, 5000);
	TM_DAC_SIGNAL_SetCustomSignal(TM_DAC1, adc_buff, adc_buff_len, 625);
	
	/* Output predefined square signal with frequency of 10kHz */
	//TM_DAC_SIGNAL_SetSignal(TM_DAC2, TM_DAC_SIGNAL_Signal_Sawtooth, 10000);
	TM_DAC_SIGNAL_SetCustomSignal(TM_DAC2, dac_buff, dac_buff_len, 5000);

	TM_GPIO_Init(GPIOA, GPIO_PIN_1, TM_GPIO_Mode_AN, TM_GPIO_OType_PP, TM_GPIO_PuPd_DOWN, TM_GPIO_Speed_Medium);
	
	ADC_InitADC();
	
	ADC_SoftwareStartConv(ADC1);
	
	
	while (1) {
		if (uint32_time_diff(TM_Time, toggle_ms) >= 1000) //1 sec delay
		{
			toggle_ms = TM_Time;		
			TM_DISCO_LedToggle(LED_GREEN);
		}
		
	}
}
