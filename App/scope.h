/*
 * SR2D1_0V3_MCU_S1 -- Service robot 2In1 servo driver
 * (C)2016 Kinco Electric (Shenzhen) Ltd. -- All rights reserved
 */
#ifndef _SCOPE_H
#define _SCOPE_H

#define SCP_LEN     8000        //ʾ�����洢���
typedef struct
{
    INT8U   Group;
    INT32U  OD1;   //ʾ����ָ��1,���ڲ�׽16λ������
    INT32U  OD2;   //ʾ����ָ��2,���ڲ�׽16λ������
    INT32U  OD3;   //ʾ����ָ��3,���ڲ�׽16λ������
    INT32U  OD4;   //ʾ����ָ��4,���ڲ�׽16λ������
    INT32U  OD5;   //ʾ����ָ��5,���ڲ�׽32λ������,��OD5�Ͳ�����OD1��OD2.
    INT32U  OD6;   //ʾ����ָ��6,���ڲ�׽32λ������,��OD6�Ͳ�����OD3��OD4.
    INT8U   OD1_ScaleFlag;  //OD1�߶ȱ任���
    INT8U   OD2_ScaleFlag;  //OD2�߶ȱ任���
    INT8U   OD3_ScaleFlag;  //OD3�߶ȱ任���
    INT8U   OD4_ScaleFlag;  //OD4�߶ȱ任���
    INT8U   OD5_ScaleFlag;  //OD5�߶ȱ任���
    INT8U   OD6_ScaleFlag;  //OD6�߶ȱ任���
    INT8U   OD1_MinusFlag;  //OD1���ű��
    INT8U   OD2_MinusFlag;  //OD2���ű��
    INT8U   OD3_MinusFlag;  //OD3���ű��
    INT8U   OD4_MinusFlag;  //OD4���ű��
    INT8U   OD5_MinusFlag;  //OD5���ű��
    INT8U   OD6_MinusFlag;  //OD6���ű��
    INT32U* OD1_SclPtr;     //OD1�߶ȱ任ֵָ��
    INT32U* OD2_SclPtr;     //OD2�߶ȱ任ֵָ��
    INT32U* OD3_SclPtr;     //OD3�߶ȱ任ֵָ��
    INT32U* OD4_SclPtr;     //OD4�߶ȱ任ֵָ��
    INT32U* OD5_SclPtr;     //OD5�߶ȱ任ֵָ��
    INT32U* OD6_SclPtr;     //OD6�߶ȱ任ֵָ��
    /*ʾ��������
    0...ʾ�������ڿ���״̬����ʱ���Զ�ȡʾ������׽������
    1...ʾ�������ڲ�׽�����¼�ǰһ�����ȵ����ݵ�״̬
    2...ʾ��������ʼ�ղ�׽���ݵ�״̬
    3...ʾ�����õ������źţ����ڲ�׽һ�����ȵ����ݣ���Ϻ󽫽������ģʽ */
    INT8U   Control;
    INT8U   ControlLast;
    INT16U  SamplePeriod;  //ʾ������������,62.5US�ı���
    INT16U  BfTrigLen;  //�����¼�ǰ��Ҫ�洢�����ݳ���
    INT16U  TotalLen;     //ʾ�����ܳ���
    INT16U  TrigEdge;  //ʾ�����������ؿ���0�½��ش���1�����ش���
    INT32U  TrigOD;   //ʾ������������,�ɴ˶����������ɼ���
    INT32S  TrigValue;  //ʾ������������������������Խ����ֵ��ʱ�򣬴�������
    INT32S  TrigValueBk;  //ʾ������������������������Խ����ֵ��ʱ�򣬴�������
    INT8U   TrigOD_DataType;    //����OD��������
    INT8U   TrigOD_ScaleFlag;   //����OD�߶ȱ任���
    INT8U   TrigOD_MinusFlag;   //����OD���ű��
    INT8U   InitReadData;       //ʾ��������,�ڶ�ȡʾ��������ǰ���ڳ�ʼ��ʾ����ָ��
    INT32U  *TrigOD_ScalePtr;   //����OD�߶ȱ任ָ��
    INT16U  *OD1_Ptr;
    INT16U  *OD2_Ptr;
    INT16U  *OD3_Ptr;
    INT16U  *OD4_Ptr;
    INT32U  *OD5_Ptr;
    INT32U  *OD6_Ptr;
    void    *TrigOD_Ptr;
    INT16U  *HeadPoint;
    INT16U  *TailPoint;
    INT16U  *SamplePtr;//�ɼ�ָ�룬�����ݷŵ�SCP_BUF�ж�Ӧ��λ�á�
    INT16U  Buff[SCP_LEN]; //this buf used for store the simple data,include OD1,OD2,OD3,OD4,OD5,OD6.
} SCP_Typedef;
extern  SCP_Typedef SCP;

extern void ScopeSampleDeal(void);
extern INT8U wSCP_OD1_Func(INT16U command, INT32U *pOD, void *bpOD, INT16U *rpOD);
extern INT8U wSCP_OD2_Func(INT16U command, INT32U *pOD, void *bpOD, INT16U *rpOD);
extern INT8U wSCP_OD3_Func(INT16U command, INT32U *pOD, void *bpOD, INT16U *rpOD);
extern INT8U wSCP_OD4_Func(INT16U command, INT32U *pOD, void *bpOD, INT16U *rpOD);
extern INT8U wSCP_OD5_Func(INT16U command, INT32U *pOD, void *bpOD, INT16U *rpOD);
extern INT8U wSCP_OD6_Func(INT16U command, INT32U *pOD, void *bpOD, INT16U *rpOD);
extern INT8U wSCP_TrigOD_Func(INT16U command, INT32U *pOD, void *bpOD, INT16U *rpOD);
extern INT8U wSCP_TrigValueFunc(INT16U command, INT32S *pOD, void *bpOD, void *rpOD);
#endif
