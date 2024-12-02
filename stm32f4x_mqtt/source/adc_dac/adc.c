/********************************************************************************/
/* adc.c                                                                        */
/* STM32F407ZGT6                                                                */
/* (Lee ChangWoo HL2IRW  hl2irw@daum.net 010-8573-6860)                 	*/
/* stm32f4x_test								*/
/********************************************************************************/
#include "../../hwdefs.h"
#include "../prototype.h"

#define ALPHA 				7			/* Q3: alpha = 0.875 */
#define ONE_MINUS_ALPHA			1			/* Q3: 1-alpha = 0.125 */
#define T_OFFSET			-10

volatile unsigned short adc_count,adc_tcount,adc_lcount;
volatile unsigned int adc_vsum,adc_tsum,adc_lsum;
volatile short adc_voltage1,adc_voltage2,adc_temperature;




int simple_iir (int orginal_value, int new_value)
{
      int filter_value,return_value;
      filter_value = (ALPHA * orginal_value) + (ONE_MINUS_ALPHA * new_value);
      return_value = (filter_value >> 3);
      return return_value;
}


unsigned short convert_voltage (unsigned short adc_data)
{
      unsigned int value;
      value = (unsigned int)((adc_data * 26406) >> 15);
      return (unsigned short)value;

}


short cal_temperature (unsigned short adc_data)
{
      unsigned int adcx;
      short result;
      double temperate;
      adcx = adc_data;
      temperate = (float)adcx * (3.3 / 4096);
      temperate = (temperate - 0.76) / 0.0025 + (25 + T_OFFSET);
      result = temperate * 100;
      return result;
}


void ADC_Config (void)
{
      ADC_CommonInitTypeDef ADC_CommonInitStructure;
      ADC_InitTypeDef ADC_InitStruct;
      RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,ENABLE);
      GPIO_Init_Pin(GPIOF,GPIO_Pin_7,GPIO_Speed_100MHz,GPIO_Mode_ADC);
      GPIO_Init_Pin(GPIOA,GPIO_Pin_5,GPIO_Speed_100MHz,GPIO_Mode_ADC);
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2 | RCC_APB2Periph_ADC3,ENABLE);
      RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);
      RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);
      RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC2,ENABLE);
      RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC2,DISABLE);
      RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC3,ENABLE);
      RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC3,DISABLE);
      ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
      ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
      ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
      ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
      ADC_CommonInit(&ADC_CommonInitStructure);
      ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;
      ADC_InitStruct.ADC_ScanConvMode = ENABLE;
      ADC_InitStruct.ADC_ContinuousConvMode = DISABLE;
      ADC_InitStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
      ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
      ADC_InitStruct.ADC_NbrOfConversion = 1;
      ADC_Init(ADC1,&ADC_InitStruct);
      ADC_Init(ADC2,&ADC_InitStruct);
      ADC_Init(ADC3,&ADC_InitStruct);
      ADC_TempSensorVrefintCmd(ENABLE);
      ADC_Cmd(ADC1,ENABLE);
      ADC_Cmd(ADC2,ENABLE);
      ADC_Cmd(ADC3,ENABLE);
      ADC_RegularChannelConfig(ADC1,ADC_Channel_16,1,ADC_SampleTime_15Cycles);
      ADC_RegularChannelConfig(ADC2,ADC_Channel_5,1,ADC_SampleTime_15Cycles);
      ADC_RegularChannelConfig(ADC3,ADC_Channel_5,1,ADC_SampleTime_15Cycles);
      ADC_SoftwareStartConv(ADC1);
      ADC_SoftwareStartConv(ADC2);
      ADC_SoftwareStartConv(ADC3);
}


void adc_process (void)
{
      if (ADC_GetFlagStatus(ADC3,ADC_FLAG_EOC)) {
         adc_vsum += ADC_GetConversionValue(ADC3);
         adc_count++;
         if (adc_count >= 100) {
      	    adc_count = 0;
            adc_voltage2 = simple_iir(adc_voltage2,convert_voltage(adc_vsum / 100));
            lcd_printf(1,5,"ADC3 CDS %4d mV ",adc_voltage2);
            adc_vsum = 0;
         }
         ADC_SoftwareStartConv(ADC3);
      }
      if (ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)) {
         adc_tsum += ADC_GetConversionValue(ADC1);
         adc_tcount++;
         if (adc_tcount >= 100) {
      	    adc_tcount = 0;
            adc_temperature = simple_iir(adc_temperature,cal_temperature(adc_tsum / 100));
            lcd_printf(1,4,"ADC1 Temperature %3d.%.2d ",adc_temperature / 100,adc_temperature % 100);
            adc_tsum = 0;
         }
         ADC_SoftwareStartConv(ADC1);
      }
      if (ADC_GetFlagStatus(ADC2,ADC_FLAG_EOC)) {
         adc_lsum += ADC_GetConversionValue(ADC2);
         adc_lcount++;
         if (adc_lcount >= 100) {
      	    adc_lcount = 0;
            adc_voltage1 = simple_iir(adc_voltage1,convert_voltage(adc_lsum / 100));
            lcd_printf(1,3,"ADC2 Volume %4d mV ",adc_voltage1);
            adc_lsum = 0;
         }
         ADC_SoftwareStartConv(ADC2);
      }
}