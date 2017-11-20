/*
 * SR2D1_0V3_MCU_S1 -- Service robot 2In1 servo driver
 * (C)2016 Kinco Electric (Shenzhen) Ltd. -- All rights reserved
 */
/****************************************************************************************
* �ļ�����QueueFunction.c
* ���ܣ���ʾ������غ���,�洢��������غ���
* ���ߣ�JKS
* ���ڣ�2012.11.26
* ��ע��
****************************************************************************************/
#include "includes.h"
//================================================================================================
//==========================================================================================================
//  ���ϵ�FIFO���з�ʽ5,����ͬһ��������,����ʹ�ù�ͬ�Ĵ������FIFO
//==========================================================================================================
/***********************************************************
*   ����˵��������һ���¶���                              *
*   ���룺                                                *
*   �����                                                *
*   ���ú�������                                          *
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
*   ����˵���������пպ���                                 *
*   ���룺    ������Ķ��нṹ�����                       *
*   �����    �����Ƿ�Ϊ��                                 *
*   ���ú�������                                           *
***********************************************************/
INT8U IsQueueType5_Empty(ADT_QUEUE_TYPE5_DEF *pQueue)
{
    return (pQueue->Head == pQueue->Tail) && (pQueue->QueueCount == 0);  //���ж����жϣ�����Ϊ0��ͷָ���βָ����ȣ���Ϊ
}                                                                                           //���п�
/***********************************************************
*   ����˵����������������                                 *
*   ���룺    ������Ķ��нṹ�����                       *
*   �����    �����Ƿ�Ϊ��                                 *
*   ���ú�������                                           *
***********************************************************/
INT8U IsQueueType5_Full(ADT_QUEUE_TYPE5_DEF *pQueue)
{
    return (pQueue->Head == pQueue->Tail) && (pQueue->QueueCount != 0);//���ж����жϣ�����Ϊ��0��ͷָ���βָ����ȣ���Ϊ
}                                                                                           //������
/***********************************************************
*   ����˵�����������Ԫ�غ���                             *
*   ���룺    ������Ķ��нṹ�����,                      *
*             ���������׵�ַ,����,                       *
*
*   �����    ��                                           *
*   ���ú�������                                           *
***********************************************************/
INT8U AddQueueType5_Item(ADT_QUEUE_TYPE5_DEF *pQueue,INT8U *pData, INT8U InNum)
{
    INT8U i;
    if(IsQueueType5_Full(pQueue))
    {
        pQueue->FullFlag = 1;         //����־λ��λ
    }
    if((pQueue->MaxSize - pQueue->QueueCount) < InNum)   //�鿴Ŀǰ���������Ƿ��ܹ�װ���±���Ҫ�������ݸ���
    {
        return (1);
    }
    for(i=0; i<InNum; i++)
    {
        *((pQueue->pQueueBuff)+(pQueue->Tail)) = *(pData + i);
        pQueue->Tail++;
        if(pQueue->Tail >= pQueue->MaxSize)   //��ͷβ����
        {
            pQueue->Tail = 0;
        }
        pQueue->QueueCount++;
    }
    return (0);
}
/***********************************************************
*   ����˵�����������һ֡Ԫ�غ�,Ԫ�ظ�����֮ǰ��FIFOʱ����*
*   ���룺    ������Ķ��нṹ�����                       *
*   �����    ���е�Ԫ��                                   *
*   ���ú�������                                           *
***********************************************************/
INT8U GetQueueType5_Item(ADT_QUEUE_TYPE5_DEF *pQueue, INT8U *pData, INT8U OutNum)
{
    INT8U i;
    for(i=0; i<OutNum; i++)
    {
        if((pQueue->Head == pQueue->Tail) && (pQueue->QueueCount == 0))   //�ж��Ƿ�Ϊ��
        {
            pQueue->EmptyFlag = 1;          //�ձ�־λ��λ
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
*   ����˵���������׸�����Ԥ������                         *
*   ���룺    ��Ŷ�ȡ�����Ŀռ�ָ��                       *
*   �����                                                 *
*   ���ú�������                                           *
***********************************************************/
INT8U GetTopQueueType5_Item(ADT_QUEUE_TYPE5_DEF *pQueue)
{
    if(IsQueueType5_Empty(pQueue))
    {
        // printf("����Ϊ��"); //����,�˳�����
        //  exit(1);
        pQueue->EmptyFlag = 1;
    }
    return  *((pQueue->pQueueBuff)+(pQueue->Head));
}
