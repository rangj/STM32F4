/*
 * SR2D1_0V3_MCU_S1 -- Service robot 2In1 servo driver
 * (C)2016 Kinco Electric (Shenzhen) Ltd. -- All rights reserved
 */
#ifndef _UART_H
#define _UART_H

#define USART3_DMA_RD_BUFF_SIZE     10
#define USART3_DMA_TD_BUFF_SIZE     10
extern __IO uint8_t USART3_DMA_RD_Buff[USART3_DMA_RD_BUFF_SIZE];    //USART3 DMA接收缓冲区
extern __IO uint8_t USART3_DMA_TD_Buff[USART3_DMA_TD_BUFF_SIZE];    //USART3 DMA发送缓冲区
#define USART3_RX_QUEUE_MAX_SIZE  30
extern INT8U USART3_RX_QueueBuff[USART3_RX_QUEUE_MAX_SIZE];     //定义USART3_RX_Queue buff
extern ADT_QUEUE_TYPE5_DEF USART3_RX_Queue;                     //定义USART3_RX_Queue FIFO
#define USART3_TX_QUEUE_MAX_SIZE  30
extern INT8U USART3_TX_QueueBuff[USART3_TX_QUEUE_MAX_SIZE];     //定义USART3_TX_Queue buff
extern ADT_QUEUE_TYPE5_DEF USART3_TX_Queue;                     //定义USART3_TX_Queue FIFO
extern void USART3_Init(void);

//DMA redefine
#define USART3_RX_DMA_RCC           RCC_AHB1Periph_DMA1
#define USART3_TX_DMA_RCC           RCC_AHB1Periph_DMA1

#define USART3_RX_DMA_CHANNEL       DMA_Channel_4
#define USART3_RX_DMA_STREAM        DMA1_Stream1
#define USART3_TX_DMA_CHANNEL       DMA_Channel_4
#define USART3_TX_DMA_STREAM        DMA1_Stream3

#define USART3_RX_DMA_IT_TCIF       DMA_IT_TCIF1
#define USART3_TX_DMA_IT_TCIF       DMA_IT_TCIF3
#define USART3_RX_DMA_FLAG_TCIF     DMA_FLAG_TCIF1
#define USART3_TX_DMA_FLAG_TCIF     DMA_FLAG_TCIF3

#define USART3_RX_DMA_IRQN          DMA1_Stream1_IRQn
#define USART3_TX_DMA_IRQN          DMA1_Stream3_IRQn
#define USART3_RX_DMA_IRQHandler    DMA1_Stream1_IRQHandler
#define USART3_TX_DMA_IRQHandler    DMA1_Stream3_IRQHandler

//uart1
#define USART1_DMA_RD_BUFF_SIZE     8
#define USART1_DMA_TD_BUFF_SIZE     10
extern __IO uint8_t USART1_DMA_RD_Buff[USART3_DMA_RD_BUFF_SIZE];    //USART1 DMA接收缓冲区
extern __IO uint8_t USART1_DMA_TD_Buff[USART3_DMA_TD_BUFF_SIZE];    //USART1 DMA发送缓冲区
#define USART1_RX_QUEUE_MAX_SIZE  21
extern INT8U USART1_RX_QueueBuff[USART1_RX_QUEUE_MAX_SIZE];     //定义USART3_RX_Queue buff
extern ADT_QUEUE_TYPE5_DEF USART1_RX_Queue;                     //定义USART3_RX_Queue FIFO
extern void USART1_Init(void);

#define USART1_RX_DMA_RCC           RCC_AHB1Periph_DMA2
#define USART1_TX_DMA_RCC           RCC_AHB1Periph_DMA2

#define USART1_RX_DMA_CHANNEL       DMA_Channel_4
#define USART1_RX_DMA_STREAM        DMA2_Stream5
#define USART1_TX_DMA_CHANNEL       DMA_Channel_4
#define USART1_TX_DMA_STREAM        DMA2_Stream7

#define USART1_RX_DMA_IT_TCIF       DMA_IT_TCIF5
#define USART1_TX_DMA_IT_TCIF       DMA_IT_TCIF7
#define USART1_RX_DMA_FLAG_TCIF     DMA_FLAG_TCIF5
#define USART1_TX_DMA_FLAG_TCIF     DMA_FLAG_TCIF7

#define USART1_RX_DMA_IRQN          DMA2_Stream5_IRQn
#define USART1_TX_DMA_IRQN          DMA2_Stream7_IRQn
#define USART1_RX_DMA_IRQHandler    DMA2_Stream5_IRQHandler
#define USART1_TX_DMA_IRQHandler    DMA2_Stream7_IRQHandler

#endif
//
