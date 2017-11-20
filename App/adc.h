/*
 * SR2D1_0V3_MCU_S1 -- Service robot 2In1 servo driver
 * (C)2016 Kinco Electric (Shenzhen) Ltd. -- All rights reserved
 */
#ifndef _ADC_H
#define _ADC_H

typedef union
{
    INT32U  Buff[5];
    struct
    {
        INT16U  DcbusV_Org; //DC Bus voltage adc original value
        INT16U  Temp1V_Org;  //temperature1 adc original value
        INT16U  DmCurrV_Org;  //dc motor current adc original value
        INT16U  Temp2V_Org;  //temperature2 current adc original value
        INT16U  BkcurV_Org; //brake current adc original value
        INT16U  DcbusV_Org2; //DC Bus voltage adc original value
        INT16U  Ain1V_Org;  //analog input 1 adc original value
        INT16U  Ain2V_Org;  //analog input 2 adc original value
        INT16U  Ain3V_Org;  //analog input 1 adc original value
        INT16U  Ain4V_Org;  //analog input 2 adc original value
    }Split;
}ADC1_2_DMA_BUFF_TYPEDEF;

extern  ADC1_2_DMA_BUFF_TYPEDEF ADC1_2_DMA_DATA;
extern  ADC1_2_DMA_BUFF_TYPEDEF ADC1_2_DMA_DATA2;
extern void InitDualInjectADC(DriverStructTypedef* pDX);  //init ADC1 and ADC2
#endif
