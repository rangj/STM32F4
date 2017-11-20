/*
 * SR2D1_0V3_MCU_S1 -- Service robot 2In1 servo driver
 * (C)2016 Kinco Electric (Shenzhen) Ltd. -- All rights reserved
 */
#ifndef _ADC_H
#define _ADC_H

#include "stm32f4xx.h"

//////////////////////////////////////////////////////////////////////////////////
//TXW����д���ٶ����ۡ�	 
//������ֻ��ѧϰʹ�ã�δ���������ɣ��������������κ���;	   
//                                                                   �޸���BY TXW								  
//////////////////////////////////////////////////////////////////////////////////
typedef struct 
{
	uint16_t *pAdcOrg;
	uint16_t  AdcOrg;
	int16_t	  AdcOffset;
	int16_t   AdcOrgAct; //active data = AdcOrg - AdcOffset;
	int32_t   AdcRealDec;
	int16_t   scale;
	int16_t   k; 
}AnalogSampleBaseTypedef;

typedef union	  
{
    uint32_t  Buff[5];     
    struct
    {
        int16_t  DcbusV_Org; //DC Bus voltage adc original value
        int16_t  Temp1V_Org;  //temperature1 adc original value
        int16_t  DmCurrV_Org;  //dc motor current adc original value
        int16_t  Temp2V_Org;  //temperature2 current adc original value
        int16_t  BkcurV_Org; //brake current adc original value
        int16_t  DcbusV_Org2; //DC Bus voltage adc original value
        int16_t  Ain1V_Org;  //analog input 1 adc original value
        int16_t  Ain2V_Org;  //analog input 2 adc original value
        int16_t  Ain3V_Org;  //analog input 1 adc original value
        int16_t  Ain4V_Org;  //analog input 2 adc original value
    }Split;
}ADC1_2_DMA_BUFF_TYPEDEF;	   //������buf[5]���ոߵ�16bit�洢���е�adc����

extern  ADC1_2_DMA_BUFF_TYPEDEF ADC1_2_DMA_DATA;
extern  ADC1_2_DMA_BUFF_TYPEDEF ADC1_2_DMA_DATA2;
extern void AppADC_RegInjInit(void);  //init ADC1 and ADC2
#endif