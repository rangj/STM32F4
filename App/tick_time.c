/*
 * SR2D1_0V3_MCU_S1 -- Service robot 2In1 servo driver
 * (C)2016 Kinco Electric (Shenzhen) Ltd. -- All rights reserved
 */
#include "includes.h"

//ϵͳʱ�����ã����1ms����һ���ж�
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
    //USART3 ���ղ�֡����
    if(DMA_GetCurrDataCounter(USART3_RX_DMA_STREAM) != USART3_DMA_RD_BUFF_SIZE)
    //if(DMA_GetCurrDataCounter(USART3_RX_DMA_STREAM) != 0)
    {
        SysService.ComMsg.Uart3RxTmOvCnt ++ ;
        if(SysService.ComMsg.Uart3RxTmOvCnt >= SysService.ComMsg.Uart3RxTmOvDef)
        {
            DMA_ITConfig(USART3_RX_DMA_STREAM, DMA_IT_TC, DISABLE);
            DMA_Cmd(USART3_RX_DMA_STREAM, DISABLE);//����֣��ر�DMA�ᴥ��������ɱ�־λTCIF��1
            DMA_SetCurrDataCounter(USART3_RX_DMA_STREAM, USART3_DMA_RD_BUFF_SIZE);//�����ȹر�DMA���ܸ������ֵ
            DMA_ClearFlag(USART3_RX_DMA_STREAM, USART3_RX_DMA_FLAG_TCIF);        //YES,ÿ������DMA DataCounter���ᷢ��DMA������ɱ�־λ��λһ��,���Ծ������������ж�
            DMA_Cmd(USART3_RX_DMA_STREAM, ENABLE);
            DMA_ITConfig(USART3_RX_DMA_STREAM, DMA_IT_TC, ENABLE);
            SysService.ComMsg.Uart3RxTmOvCnt = 0;
        }
    }
    else
    {
        SysService.ComMsg.Uart3RxTmOvCnt = 0;
    }

    //USART1 ���ղ�֡����
    if(DMA_GetCurrDataCounter(USART1_RX_DMA_STREAM) != SysService.Radar.FrameBytes)
    {
        SysService.ComMsg.Uart1RxTmOvCnt ++ ;
        if(SysService.ComMsg.Uart1RxTmOvCnt >= SysService.ComMsg.Uart1RxTmOvDef)
        {
            DMA_ITConfig(USART1_RX_DMA_STREAM, DMA_IT_TC, DISABLE);
            DMA_Cmd(USART1_RX_DMA_STREAM, DISABLE);
            DMA_SetCurrDataCounter(USART1_RX_DMA_STREAM, USART1_DMA_RD_BUFF_SIZE);
            DMA_ClearFlag(USART1_RX_DMA_STREAM, USART1_RX_DMA_FLAG_TCIF);        //YES,ÿ������DMA DataCounter���ᷢ��DMA������ɱ�־λ��λһ��,���Ծ������������ж�
            DMA_Cmd(USART1_RX_DMA_STREAM, ENABLE);
            DMA_ITConfig(USART1_RX_DMA_STREAM, DMA_IT_TC, ENABLE);
            SysService.ComMsg.Uart1RxTmOvCnt = 0;
        }
    }
    else
    {
        SysService.ComMsg.Uart1RxTmOvCnt = 0;
    }

    //����̨У׼���̼�ʱ
    if(DXC.PlatformInfo.CalCmd == 2)
    {
        DXC.PlatformInfo.CalOverTimeCnt ++;
    }
}
