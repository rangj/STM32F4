/*
 * SR2D1_0V3_MCU_S1 -- Service robot 2In1 servo driver
 * (C)2016 Kinco Electric (Shenzhen) Ltd. -- All rights reserved
 */
#ifndef _SCOPE_H
#define _SCOPE_H

#define SCP_LEN     8000        //示波器存储深度
typedef struct
{
    INT8U   Group;
    INT32U  OD1;   //示波器指针1,用于捕捉16位的数据
    INT32U  OD2;   //示波器指针2,用于捕捉16位的数据
    INT32U  OD3;   //示波器指针3,用于捕捉16位的数据
    INT32U  OD4;   //示波器指针4,用于捕捉16位的数据
    INT32U  OD5;   //示波器指针5,用于捕捉32位的数据,用OD5就不能用OD1和OD2.
    INT32U  OD6;   //示波器指针6,用于捕捉32位的数据,用OD6就不能用OD3和OD4.
    INT8U   OD1_ScaleFlag;  //OD1尺度变换标记
    INT8U   OD2_ScaleFlag;  //OD2尺度变换标记
    INT8U   OD3_ScaleFlag;  //OD3尺度变换标记
    INT8U   OD4_ScaleFlag;  //OD4尺度变换标记
    INT8U   OD5_ScaleFlag;  //OD5尺度变换标记
    INT8U   OD6_ScaleFlag;  //OD6尺度变换标记
    INT8U   OD1_MinusFlag;  //OD1符号标记
    INT8U   OD2_MinusFlag;  //OD2符号标记
    INT8U   OD3_MinusFlag;  //OD3符号标记
    INT8U   OD4_MinusFlag;  //OD4符号标记
    INT8U   OD5_MinusFlag;  //OD5符号标记
    INT8U   OD6_MinusFlag;  //OD6符号标记
    INT32U* OD1_SclPtr;     //OD1尺度变换值指针
    INT32U* OD2_SclPtr;     //OD2尺度变换值指针
    INT32U* OD3_SclPtr;     //OD3尺度变换值指针
    INT32U* OD4_SclPtr;     //OD4尺度变换值指针
    INT32U* OD5_SclPtr;     //OD5尺度变换值指针
    INT32U* OD6_SclPtr;     //OD6尺度变换值指针
    /*示波器控制
    0...示波器处于空闲状态，此时可以读取示波器捕捉的数据
    1...示波器处于捕捉触发事件前一定长度的数据的状态
    2...示波器处于始终捕捉数据的状态
    3...示波器得到触发信号，正在捕捉一定长度的数据，完毕后将进入空闲模式 */
    INT8U   Control;
    INT8U   ControlLast;
    INT16U  SamplePeriod;  //示波器采样周期,62.5US的倍数
    INT16U  BfTrigLen;  //触发事件前需要存储的数据长度
    INT16U  TotalLen;     //示波器总长度
    INT16U  TrigEdge;  //示波器触发边沿控制0下降沿触发1上升沿触发
    INT32U  TrigOD;   //示波器触发对象,由此对象来触发采集。
    INT32S  TrigValue;  //示波器触发条件。当触发对象穿越整个值的时候，触发采样
    INT32S  TrigValueBk;  //示波器触发条件。当触发对象穿越整个值的时候，触发采样
    INT8U   TrigOD_DataType;    //触发OD数据类型
    INT8U   TrigOD_ScaleFlag;   //触发OD尺度变换标记
    INT8U   TrigOD_MinusFlag;   //触发OD负号标记
    INT8U   InitReadData;       //示波器数据,在读取示波器数据前用于初始化示波器指针
    INT32U  *TrigOD_ScalePtr;   //触发OD尺度变换指针
    INT16U  *OD1_Ptr;
    INT16U  *OD2_Ptr;
    INT16U  *OD3_Ptr;
    INT16U  *OD4_Ptr;
    INT32U  *OD5_Ptr;
    INT32U  *OD6_Ptr;
    void    *TrigOD_Ptr;
    INT16U  *HeadPoint;
    INT16U  *TailPoint;
    INT16U  *SamplePtr;//采集指针，将数据放到SCP_BUF中对应的位置。
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
