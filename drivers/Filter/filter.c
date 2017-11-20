
#include "filter.h"
#include "gpio.h"
//////////////////////////////////////////////////////////////////////////////////
//TXW不会写，百度无欺。	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途	   
//                                                                   修改人BY TXW								  
//////////////////////////////////////////////////////////////////////////////////
void CalAnalogRealDec(AnalogSampleBaseTypedef *pAnReDec)
{
	pAnReDec->AdcOrgAct = (int16_t)(pAnReDec->AdcOrg) - pAnReDec->AdcOffset; //请注意无符号数与有符号数运算的优先级问题（面试也会出现）
	pAnReDec->AdcRealDec = (int32_t)(pAnReDec->AdcOrgAct)*pAnReDec->k/(int32_t)(pAnReDec->scale);
}

void LowPassFilterProcess(AnalogProcessTypedef *pAdcIn)
{
	uint32_t adc_temp = 0;
	if(pAdcIn->lpf.Enable)
	{
		pAdcIn->lpf.Rin = *(pAdcIn->adcSample.pAdcOrg);
		//Yn=(1-A)Yn-1+A*Xn
		adc_temp = (65536 - pAdcIn->lpf.coffA)*(int32_t)pAdcIn->lpf.Rin + \
					pAdcIn->lpf.coffA *(int32_t)pAdcIn->lpf.Cout +pAdcIn->lpf.CoRemain;

		pAdcIn->lpf.Cout = ((adc_temp>>16)&0x00ff);
		pAdcIn->lpf.CoRemain = (adc_temp&0x00ff);

		pAdcIn->adcSample.AdcOrg = pAdcIn->lpf.Cout; //Yn-1
	}
	else
	{
		 pAdcIn->lpf.Rin = *(pAdcIn->adcSample.pAdcOrg);
	}
	CalAnalogRealDec(&pAdcIn->adcSample);
} 

DriverTypedef driver = 
{
    .ExAdcBuff[0].adcSample.pAdcOrg = (uint16_t *)&ADC1_2_DMA_DATA2.Split.Ain1V_Org,
    .ExAdcBuff[0].adcSample.AdcOffset = 2048,
    .ExAdcBuff[0].adcSample.scale = 1024,
    .ExAdcBuff[0].adcSample.k = 8192,
    .ExAdcBuff[0].lpf.coffA = 64261,
    .ExAdcBuff[0].lpf.Enable = 1,	
}; 	  
void ADC_IRQHandler(void)
{
	for(uint8_t i = 0; i < 5 ;++i)
	{
		ADC1_2_DMA_DATA2.Buff[i] = ADC1_2_DMA_DATA.Buff[i];
	}	
	ADC_SoftwareStartConv(ADC1);    //must first start adc1,then adc2, or the DMA data sequence will not right
    ADC_SoftwareStartConv(ADC2);   

	if(ADC_GetFlagStatus(ADC2, ADC_FLAG_JEOC) == SET)
    {
        _RUN_LED_ON;
        ADC_ClearFlag(ADC2, ADC_FLAG_JEOC);
        ADC_ClearITPendingBit(ADC2, ADC_IT_JEOC);
        ADC_ClearFlag(ADC1, ADC_FLAG_JEOC);
        ADC_ClearITPendingBit(ADC1, ADC_IT_JEOC);
        _RUN_LED_OFF;
        //get the adc value, current from -2048 to 2048
        driver.CurU.AdcOrg = ADC_GetInjectedConversionValue(ADC1,ADC_InjectedChannel_1);
        driver.CurU.AdcOrg = ADC_GetInjectedConversionValue(ADC1,ADC_InjectedChannel_2);

        _RUN_LED_ON;
        LowPassFilterProcess(&driver.ExAdcBuff[0]);
        LowPassFilterProcess(&driver.ExAdcBuff[1]);
        LowPassFilterProcess(&driver.ExAdcBuff[2]);
        LowPassFilterProcess(&driver.ExAdcBuff[3]);

        _RUN_LED_OFF;

    }

}
