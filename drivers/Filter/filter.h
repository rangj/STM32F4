#ifndef __FILTER_H
#define __FILTER_H
//////////////////////////////////////////////////////////////////////////////////
//TXW����д���ٶ����ۡ�	 
//������ֻ��ѧϰʹ�ã�δ���������ɣ��������������κ���;	   
//                                                                   �޸���BY TXW								  
//////////////////////////////////////////////////////////////////////////////////
#include "stm32f4xx.h"
#include "adc.h"
typedef struct 
{
	int8_t   Enable; //ʹ��
	int16_t  coffA; //�˲�ϵ��
	int16_t  Rin;  //����
	int16_t  Cout; //���
	int16_t  CoRemain;//����
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