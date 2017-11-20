/*
 * SR2D1_0V3_MCU_S1 -- Service robot 2In1 servo driver
 * (C)2016 Kinco Electric (Shenzhen) Ltd. -- All rights reserved
 */
#ifndef _QUEUE_FUNCTION_H
#define _QUEUE_FUNCTION_H
/********************
*    结构体定义区   *
********************/
typedef struct
{
    INT16U Head;
    INT16U Tail;
    INT16U MaxSize;
    INT8U *pQueueBuff;
    INT16U QueueCount;
    INT8U EmptyFlag;
    INT8U FullFlag;
} ADT_QUEUE_TYPE5_DEF;
extern INT8U CreatQueueType5(ADT_QUEUE_TYPE5_DEF *pQueue,INT8U *pQBuff, INT16U Size);
extern INT8U IsQueueType5_Empty(ADT_QUEUE_TYPE5_DEF *pQueue);
extern INT8U IsQueueType5_Full(ADT_QUEUE_TYPE5_DEF *pQueue);
extern INT8U AddQueueType5_Item(ADT_QUEUE_TYPE5_DEF *pQueue, INT8U *pData, INT8U InNum);
extern INT8U GetQueueType5_Item(ADT_QUEUE_TYPE5_DEF *pQueue, INT8U *pData, INT8U OutNum);
extern INT8U GetTopQueueType5_Item(ADT_QUEUE_TYPE5_DEF *pQueue);
#endif
