/*
 * SR2D1_0V3_MCU_S1 -- Service robot 2In1 servo driver
 * (C)2016 Kinco Electric (Shenzhen) Ltd. -- All rights reserved
 */
#include "adc.h"

uint16_t testBuf[2];
ADC1_2_DMA_BUFF_TYPEDEF ADC1_2_DMA_DATA =
{
    .Split.Temp1V_Org = 1632,
    .Split.Temp2V_Org = 1632,
};
ADC1_2_DMA_BUFF_TYPEDEF ADC1_2_DMA_DATA2 =
{
    .Split.Temp1V_Org = 1632,
    .Split.Temp2V_Org = 1632,
};

//初始化规则+注入同步采样ADC，注入组用于Iu,Iv;规则用于DCbus....采样
//triggered by TIM1_Update.

void AppADC_RegInjInit(void)  //init ADC1
{
    ADC_InitTypeDef         ADC_InitStructure;
    ADC_CommonInitTypeDef   ADC_CommonInitStructure;    //for M4
    DMA_InitTypeDef         DMA_InitStructure;
    GPIO_InitTypeDef        GPIO_InitStructure;
    NVIC_InitTypeDef        NVIC_InitStructure;
    //uint8_t  i=0;

    // Enable ADC GPIO clocks
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    // Configure ADC1,2 Channel 0[PA0],ADC2 channel 1[PA1] pin as analog input,for driver1 U,V current sample
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    // Configure ADC1,2 chanel 8,9[PB0..1] pin as driver2 U and Temp2V
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    // Configure ADC1,2 Channel 1[PC1],ADC2 channel 2[PC2] pin as analog input,for driver2 U,V current sample
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_Init(GPIOC, &GPIO_InitStructure);


    //RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //set adc conversion clock:72MHz/6=12MHz
    // Enable ADC1.2 clocks
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
    // ADC Common Init
    ADC_CommonInitStructure.ADC_Mode = ADC_DualMode_RegSimult_InjecSimult;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;  //CLK = 84/4 = 21M (the ADC clk max 36Mhz@VDDA=3.3V)
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_2;//ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);

    //ADC1 Init
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; //can not ENABLE,or DMA will over run
    //ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_Rising;//no trigger in the master adc:ADC1.
    //ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_Ext_IT11;
    //ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1; //do not need the trip source
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion = 5;
    ADC_Init(ADC1, &ADC_InitStructure);
    // ADC2 Init, the parameters is same as ADC1
    ADC_Init(ADC2, &ADC_InitStructure);

    //ADC1,2 reggulation group channel 0..5 configuration
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4,  1, ADC_SampleTime_28Cycles);  //DCBUS_V = 0.06 * adcDCBUS_V
    ADC_RegularChannelConfig(ADC1, ADC_Channel_3,  2, ADC_SampleTime_28Cycles);  //DmCurrV
    ADC_RegularChannelConfig(ADC1, ADC_Channel_2,  3, ADC_SampleTime_28Cycles);  //BKCurr_V
    ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 4, ADC_SampleTime_28Cycles);  //MCU1_AIN1_V
    ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 5, ADC_SampleTime_28Cycles);  //MCU2_AIN1_V


    ADC_RegularChannelConfig(ADC2, ADC_Channel_14, 1, ADC_SampleTime_28Cycles);  //TEMP1_V
    ADC_RegularChannelConfig(ADC2, ADC_Channel_9,  2, ADC_SampleTime_28Cycles);  //TEMP2_V
    ADC_RegularChannelConfig(ADC2, ADC_Channel_4,  3, ADC_SampleTime_28Cycles);  //DCBUS_V
    ADC_RegularChannelConfig(ADC2, ADC_Channel_11, 4, ADC_SampleTime_28Cycles);  //MCU1_AIN2_V
    ADC_RegularChannelConfig(ADC2, ADC_Channel_13, 5, ADC_SampleTime_28Cycles);  //MCU2_AIN2_V

                                                                              //
    // ADC1 Injected channel configuration
    ADC_InjectedSequencerLengthConfig(ADC1, 2);     //must first write the length
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_1,  1, ADC_SampleTime_28Cycles); //U1_Curr   (must add the sample time or add the capacitor value)
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_8, 2, ADC_SampleTime_28Cycles); //U2_Curr
    ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_T1_TRGO);
    ADC_ExternalTrigInjectedConvEdgeConfig(ADC1, ADC_ExternalTrigInjecConvEdge_Rising);

    // ADC2 Injected channel configuration
    ADC_InjectedSequencerLengthConfig(ADC2, 2);     //must first write the length
    ADC_InjectedChannelConfig(ADC2, ADC_Channel_0,  1, ADC_SampleTime_28Cycles); //V1_Curr
    ADC_InjectedChannelConfig(ADC2, ADC_Channel_15, 2, ADC_SampleTime_28Cycles); //V2_Curr
    ADC_ExternalTrigInjectedConvConfig(ADC2, ADC_ExternalTrigInjecConv_T1_TRGO);
    ADC_ExternalTrigInjectedConvEdgeConfig(ADC2, ADC_ExternalTrigInjecConvEdge_Rising);

    //DMA request
    ADC_DMACmd(ADC1, ENABLE);
    ADC_DMACmd(ADC2, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);  //DMA clock should be opened also
    DMA_DeInit(DMA2_Stream0);
    //ADC1 RX DMA Configure
    DMA_InitStructure.DMA_Channel = DMA_Channel_0;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(ADC->CDR)); //must point to the common data register
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&(ADC1_2_DMA_DATA.Buff[0]); //ADC2 conversion result will put into the high 16 bit
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;  //DMA_DIR_PeripheralToMemory
    DMA_InitStructure.DMA_BufferSize = 5;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word; //here 16BIT + 16BIT
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; //must set to this mode
    DMA_InitStructure.DMA_Priority = DMA_Priority_Low;//DMA_Priority_High;
    //DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;//DMA_FIFOMode_Enable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream0, &DMA_InitStructure);
    //DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);
    ADC_MultiModeDMARequestAfterLastTransferCmd(ENABLE);
    DMA_Cmd(DMA2_Stream0, ENABLE);			 

    // Enable ADC1
    ADC_Cmd(ADC1, ENABLE);
    ADC_Cmd(ADC2, ENABLE);

    //after the TIM1 start
   /* do{													//会累加？
        if(ADC_GetFlagStatus(ADC2, ADC_FLAG_JEOC) == SET)	//ADC_FLAG_JEOC AD转换结束标志
        {
            ADC_ClearFlag(ADC2, ADC_FLAG_JEOC);			   //what 两次？
            ADC_ClearITPendingBit(ADC2, ADC_FLAG_JEOC);	   
            ADC_ClearFlag(ADC1, ADC_FLAG_JEOC);
            ADC_ClearITPendingBit(ADC1, ADC_FLAG_JEOC);
            i++;
        }
    }while(i<16);*/	

    //after 1ms get the u,v current adc offset value

    //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitStructure.NVIC_IRQChannel = ADC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    ADC_ClearFlag(ADC1, ADC_FLAG_JEOC|ADC_FLAG_OVR);
    ADC_ClearITPendingBit(ADC1, ADC_IT_JEOC|ADC_IT_OVR);
    ADC_ClearFlag(ADC2, ADC_FLAG_JEOC|ADC_FLAG_OVR);
    ADC_ClearITPendingBit(ADC2, ADC_IT_JEOC|ADC_IT_OVR);
    ADC_ITConfig(ADC2, ADC_IT_JEOC, ENABLE); //enable the end of coversion int,both of adc1 & adc2 is finished.

} 

