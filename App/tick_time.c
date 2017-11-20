/*
 * SR2D1_0V3_MCU_S1 -- Service robot 2In1 servo driver
 * (C)2016 Kinco Electric (Shenzhen) Ltd. -- All rights reserved
 */
#include "includes.h"

//系统时钟配置，设计1ms产生一次中断
void  SysTickConfig(void)
{
    RCC_ClocksTypeDef  rcc_clocks;
    u32         cnts;
    RCC_GetClocksFreq(&rcc_clocks);
    cnts = (u32)rcc_clocks.HCLK_Frequency/1000;
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
    SysTick_Config(cnts);
}
void SysTick_Handler(void)
{
    //USART3 接收残帧处理
    if(DMA_GetCurrDataCounter(USART3_RX_DMA_STREAM) != USART3_DMA_RD_BUFF_SIZE)
    //if(DMA_GetCurrDataCounter(USART3_RX_DMA_STREAM) != 0)
    {
        SysService.ComMsg.Uart3RxTmOvCnt ++ ;
        if(SysService.ComMsg.Uart3RxTmOvCnt >= SysService.ComMsg.Uart3RxTmOvDef)
        {
            DMA_ITConfig(USART3_RX_DMA_STREAM, DMA_IT_TC, DISABLE);
            DMA_Cmd(USART3_RX_DMA_STREAM, DISABLE);//好奇怪，关闭DMA会触发传送完成标志位TCIF置1
            DMA_SetCurrDataCounter(USART3_RX_DMA_STREAM, USART3_DMA_RD_BUFF_SIZE);//必须先关闭DMA才能更改这个值
            DMA_ClearFlag(USART3_RX_DMA_STREAM, USART3_RX_DMA_FLAG_TCIF);        //YES,每次重置DMA DataCounter都会发生DMA传输完成标志位置位一次,所以就先来个清零行动
            DMA_Cmd(USART3_RX_DMA_STREAM, ENABLE);
            DMA_ITConfig(USART3_RX_DMA_STREAM, DMA_IT_TC, ENABLE);
            SysService.ComMsg.Uart3RxTmOvCnt = 0;
        }
    }
    else
    {
        SysService.ComMsg.Uart3RxTmOvCnt = 0;
    }

    //USART1 接收残帧处理
    if(DMA_GetCurrDataCounter(USART1_RX_DMA_STREAM) != SysService.Radar.FrameBytes)
    {
        SysService.ComMsg.Uart1RxTmOvCnt ++ ;
        if(SysService.ComMsg.Uart1RxTmOvCnt >= SysService.ComMsg.Uart1RxTmOvDef)
        {
            DMA_ITConfig(USART1_RX_DMA_STREAM, DMA_IT_TC, DISABLE);
            DMA_Cmd(USART1_RX_DMA_STREAM, DISABLE);
            DMA_SetCurrDataCounter(USART1_RX_DMA_STREAM, USART1_DMA_RD_BUFF_SIZE);
            DMA_ClearFlag(USART1_RX_DMA_STREAM, USART1_RX_DMA_FLAG_TCIF);        //YES,每次重置DMA DataCounter都会发生DMA传输完成标志位置位一次,所以就先来个清零行动
            DMA_Cmd(USART1_RX_DMA_STREAM, ENABLE);
            DMA_ITConfig(USART1_RX_DMA_STREAM, DMA_IT_TC, ENABLE);
            SysService.ComMsg.Uart1RxTmOvCnt = 0;
        }
    }
    else
    {
        SysService.ComMsg.Uart1RxTmOvCnt = 0;
    }

    //升降台校准过程计时
    if(DXC.PlatformInfo.CalCmd == 2)
    {
        DXC.PlatformInfo.CalOverTimeCnt ++;
    }
}
