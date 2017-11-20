/*
 * SR2D1_0V3_MCU_S1 -- Service robot 2In1 servo driver
 * (C)2016 Kinco Electric (Shenzhen) Ltd. -- All rights reserved
 */
/****************************************************************************************
* 文件名：QueueFunction.c
* 功能：显示队列相关函数,存储器队列相关函数
* 作者：JKS
* 日期：2012.11.26
* 备注：
****************************************************************************************/
#include "includes.h"
//================================================================================================
//==========================================================================================================
//  整合的FIFO队列方式5,对于同一类数据流,可以使用共同的代码进行FIFO
//==========================================================================================================
/***********************************************************
*   函数说明：创建一个新队列                              *
*   输入：                                                *
*   输出：                                                *
*   调用函数：无                                          *
***********************************************************/
INT8U CreatQueueType5(ADT_QUEUE_TYPE5_DEF *pQueue,INT8U *pQBuff, INT16U Size)
{
    INT8U i;
    pQueue->Head = 0;
    pQueue->Tail = 0;
    pQueue->QueueCount = 0;
    pQueue->MaxSize = Size;
    pQueue->pQueueBuff = pQBuff;
    pQueue->EmptyFlag = 0;
    pQueue->FullFlag = 0;
    return(0);
}
/***********************************************************
*   函数说明：队列判空函数                                 *
*   输入：    所定义的队列结构体变量                       *
*   输出：    队列是否为空                                 *
*   调用函数：无                                           *
***********************************************************/
INT8U IsQueueType5_Empty(ADT_QUEUE_TYPE5_DEF *pQueue)
{
    return (pQueue->Head == pQueue->Tail) && (pQueue->QueueCount == 0);  //环行队列判断，计数为0，头指针和尾指针相等，则为
}                                                                                           //队列空
/***********************************************************
*   函数说明：队列判满函数                                 *
*   输入：    所定义的队列结构体变量                       *
*   输出：    队列是否为空                                 *
*   调用函数：无                                           *
***********************************************************/
INT8U IsQueueType5_Full(ADT_QUEUE_TYPE5_DEF *pQueue)
{
    return (pQueue->Head == pQueue->Tail) && (pQueue->QueueCount != 0);//环行队列判断，计数为非0，头指针和尾指针相等，则为
}                                                                                           //队列满
/***********************************************************
*   函数说明：队列添加元素函数                             *
*   输入：    所定义的队列结构体变量,                      *
*             输入对象的首地址,个数,                       *
*
*   输出：    无                                           *
*   调用函数：无                                           *
***********************************************************/
INT8U AddQueueType5_Item(ADT_QUEUE_TYPE5_DEF *pQueue,INT8U *pData, INT8U InNum)
{
    INT8U i;
    if(IsQueueType5_Full(pQueue))
    {
        pQueue->FullFlag = 1;         //满标志位置位
    }
    if((pQueue->MaxSize - pQueue->QueueCount) < InNum)   //查看目前缓冲区还是否能够装得下本次要进的数据个数
    {
        return (1);
    }
    for(i=0; i<InNum; i++)
    {
        *((pQueue->pQueueBuff)+(pQueue->Tail)) = *(pData + i);
        pQueue->Tail++;
        if(pQueue->Tail >= pQueue->MaxSize)   //从头尾相连
        {
            pQueue->Tail = 0;
        }
        pQueue->QueueCount++;
    }
    return (0);
}
/***********************************************************
*   函数说明：队列输出一帧元素函,元素个数由之前进FIFO时候定义*
*   输入：    所定义的队列结构体变量                       *
*   输出：    队列的元素                                   *
*   调用函数：无                                           *
***********************************************************/
INT8U GetQueueType5_Item(ADT_QUEUE_TYPE5_DEF *pQueue, INT8U *pData, INT8U OutNum)
{
    INT8U i;
    for(i=0; i<OutNum; i++)
    {
        if((pQueue->Head == pQueue->Tail) && (pQueue->QueueCount == 0))   //判断是否为空
        {
            pQueue->EmptyFlag = 1;          //空标志位置位
            return (1);
        }
        else
        {
            *(pData + i) = *((pQueue->pQueueBuff)+(pQueue->Head));
            pQueue->Head++;
            if(pQueue->Head >= pQueue->MaxSize)
            {
                pQueue->Head = 0;
            }
            pQueue->QueueCount--;
        }
    }
    return (0);
}
/***********************************************************
*   函数说明：队列首个数据预览函数                         *
*   输入：    存放读取变量的空间指针                       *
*   输出：                                                 *
*   调用函数：无                                           *
***********************************************************/
INT8U GetTopQueueType5_Item(ADT_QUEUE_TYPE5_DEF *pQueue)
{
    if(IsQueueType5_Empty(pQueue))
    {
        // printf("队列为空"); //下溢,退出运行
        //  exit(1);
        pQueue->EmptyFlag = 1;
    }
    return  *((pQueue->pQueueBuff)+(pQueue->Head));
}
