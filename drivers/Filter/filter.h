#ifndef __FILTER_H
#define __FILTER_H
//////////////////////////////////////////////////////////////////////////////////
//TXW不会写，百度无欺。	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途	   
//                                                                   修改人BY TXW								  
//////////////////////////////////////////////////////////////////////////////////
#include "stm32f4xx.h"
#include "adc.h"
typedef struct 
{
	int8_t   Enable; //使能
	int16_t  coffA; //滤波系数
	int16_t  Rin;  //输入
	int16_t  Cout; //输出
	int16_t  CoRemain;//余数
}LowPassFilterTypedef;

typedef struct 
{
	 LowPassFilterTypedef lpf;
	 AnalogSampleBaseTypedef adcSample;
}AnalogProcessTypedef;

typedef struct 
{
	AnalogSampleBaseTypedef CurU;
	AnalogSampleBaseTypedef CurV;
	AnalogProcessTypedef    ExAdcBuff[4];
}DriverTypedef;
void CalAnalogRealDec(AnalogSampleBaseTypedef *pAnReDec);
void ADC_IRQHandler(void);
#endif
