/*
 * SR2D1_0V3_MCU_S1 -- Service robot 2In1 servo driver
 * (C)2016 Kinco Electric (Shenzhen) Ltd. -- All rights reserved
 */
#ifndef _CAN_H
#define _CAN_H

#define CAN2_RX_QUEUE_MAX_SIZE  36
extern INT8U CAN2_RX_QueueBuff[CAN2_RX_QUEUE_MAX_SIZE];      //定义CAN1_RX_Queue buff
extern ADT_QUEUE_TYPE5_DEF CAN2_RX_Queue;                      //定义CAN1_RX_Queue FIFO

extern CanTxMsg CAN2_TX;
extern CanRxMsg CAN2_RX;



extern void CAN2_Init(void);

#endif
