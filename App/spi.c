/*
 * SR2D1_0V3_MCU_S1 -- Service robot 2In1 servo driver
 * (C)2016 Kinco Electric (Shenzhen) Ltd. -- All rights reserved
 */
#include "includes.h"
//SPI3
INT16U  SPI3_DMA_RD_Buff[16];
INT16U  SPI3_DMA_TD_Buff[16];
INT16U  SPI3_DMA_RD_Buff2[16];
INT16U  SPI3_DMA_TD_Buff2[16];
void SPI3_Init(void)
{
    DMA_InitTypeDef     DMA_InitStructure;
    SPI_InitTypeDef     SPI_InitStructure;
    GPIO_InitTypeDef    GPIO_InitStructure;
    NVIC_InitTypeDef    NVIC_InitStructure;
    //分配SPI2管脚
    RCC_AHB1PeriphClockCmd(SPI3_PORT_RCC,ENABLE);
    //NSS pin control by software
    GPIO_InitStructure.GPIO_Pin = SPI3_SCK_PIN | SPI3_MOSI_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(SPI3_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = SPI3_MISO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(SPI3_PORT, &GPIO_InitStructure);

    //GPIO_PinAFConfig(SPI2_PORT, SPI2_NSS_PIN_SOURCE, GPIO_AF_SPI2);
    GPIO_PinAFConfig(SPI3_PORT, SPI3_SCK_PIN_SOURCE, GPIO_AF_SPI3);
    GPIO_PinAFConfig(SPI3_PORT, SPI3_MISO_PIN_SOURCE, GPIO_AF_SPI3);
    GPIO_PinAFConfig(SPI3_PORT, SPI3_MOSI_PIN_SOURCE, GPIO_AF_SPI3);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
    SPI_I2S_DeInit(SPI3);
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_Init(SPI3, &SPI_InitStructure);
    //SPI_SSOutputCmd(SPI2, ENABLE);

    SPI_Cmd(SPI3, ENABLE);

    //SPI3 DMA cofiguration
    //DMA 请求
    SPI_DMACmd(SPI3, SPI_I2S_DMAReq_Rx, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);  //记住DMA也要开时钟的
    DMA_DeInit(DMA1_Stream0);
    //SPI3 RX DMA Configure
    DMA_InitStructure.DMA_Channel = DMA_Channel_0;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(SPI3->DR));
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&SPI3_DMA_RD_Buff;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;  //DMA_DIR_PeripheralToMemory
    DMA_InitStructure.DMA_BufferSize = 16;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; //这里是half word
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//DMA_Mode_Circular;//DMA_Mode_Normal //这里必须设置为这个模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;//DMA_Priority_VeryHigh; //DMA_Priority_High;
    //DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream0, &DMA_InitStructure);
    DMA_ITConfig(DMA1_Stream0, DMA_IT_TC, ENABLE);
    DMA_ITConfig(DMA1_Stream0, DMA_IT_TE, DISABLE);
    DMA_ITConfig(DMA1_Stream0, DMA_IT_FE, DISABLE);
    DMA_ITConfig(DMA1_Stream0, DMA_IT_HT, DISABLE);
    /* DMA1_Stream0(SPI3_RX) enable */
    DMA_Cmd(DMA1_Stream0, ENABLE);

    //SPI_DMACmd(SPI3, SPI_I2S_DMAReq_Tx, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);  //记住DMA也要开时钟的
    DMA_DeInit(DMA1_Stream5);
    //SPI3 TX DMA Configure
    DMA_InitStructure.DMA_Channel = DMA_Channel_0;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(SPI3->DR));
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&SPI3_DMA_TD_Buff;
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;  //DMA_DIR_PeripheralToMemory
    DMA_InitStructure.DMA_BufferSize = 16;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; //这里是half word
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;//DMA_Mode_Normal;//DMA_Mode_Circular; //这里必须设置为这个模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    //DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream5, &DMA_InitStructure);
    DMA_ITConfig(DMA1_Stream5, DMA_IT_TC, DISABLE);
    DMA_ITConfig(DMA1_Stream5, DMA_IT_TE, DISABLE);
    DMA_ITConfig(DMA1_Stream5, DMA_IT_FE, DISABLE);
    DMA_ITConfig(DMA1_Stream5, DMA_IT_HT, DISABLE);
    /* DMA1_Stream5(SPI3_TX) enable */
    DMA_Cmd(DMA1_Stream5, ENABLE);  //好像必须先在这里使能一次，不知道为啥，奶奶的最近DMA问题多多20160615

    //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    //SPI RX DMA Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

}
INT32U      SensorData;
void DMA1_Stream0_IRQHandler(void)
{
    INT8U i;
    if(DMA_GetITStatus(DMA1_Stream0, DMA_IT_TCIF0) != RESET)
    {
        //put the SPI3_DMA_RD_Buff to the SPI3_DMA_RD_Buff2
        for(i=0; i<16; i++)
        {
            SPI3_DMA_RD_Buff2[i] = SPI3_DMA_RD_Buff[i];
        }
        _SPI3_CS_SET;
        DMA_Cmd(DMA1_Stream5, DISABLE);
        DMA_ClearFlag(DMA1_Stream5,DMA_FLAG_TCIF5|DMA_FLAG_HTIF5|DMA_FLAG_TEIF5|DMA_FLAG_DMEIF5|DMA_FLAG_FEIF5);
        SPI_DMACmd(SPI3, SPI_I2S_DMAReq_Tx, DISABLE);
        DMA_ClearITPendingBit(DMA1_Stream0, DMA_IT_TCIF0);
        DMA_ClearITPendingBit(DMA1_Stream0, DMA_IT_HTIF0);
    }
}

