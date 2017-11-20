/*
 * SR2D1_0V3_MCU_S1 -- Service robot 2In1 servo driver
 * (C)2016 Kinco Electric (Shenzhen) Ltd. -- All rights reserved
 */
#include "includes.h"

//scope veriable
SCP_Typedef SCP =
{
    .SamplePeriod = 1,
    .BfTrigLen = 250,
    .TotalLen = 500
};
//SCP_Typedef HelloScope;
void ScopeSampleDeal(void)
{
    static INT16U  SampleCnt = 0; //��¼�ɼ�����
    static INT16U  SamplePeriodCnt = 0; //�ɼ������趨�����ٴ�pwm�жϣ��Ų���һ��
    //�����ݴ�SCP.TotalLen����ֹ�ڲɼ������У�SCP.TotalLen���޸ģ�Ӱ�������
    //ֻҪSCP.Control����1��Ҫ����βɼ���ɺ��޸�SCP.TotalLen������Ч��
    static INT16U  SCP_TotalLenShadow = 0;
    static INT8U  OneSampleLen = 0;//ÿһ�βɼ�Ҫռ���ٸ�SCP.Buff�еĿռ䣬���磺OD5��0D3����Ϊ0����ô��Ҫ2+1��3��SCP.Buff[]
    static INT8U FirstSimple = TRUE;//��һ�β����������úܶ����
    static INT8U FirstTrig  = TRUE; //�˱���Ƿ�ֹ��δ�����
    static INT8U TrigHappen = FALSE; //�Ƿ���������������Ƿ�����SampleCnt;��Ϊ�Զ�������û����ǰ��Ҫ��SampleCntһֱΪ0��
    static INT16U*  TrigPoint = NULL;//��¼��������SCP.Buff[]�еľ���λ�á�
    static UNION_DATA_DEF TrigOD_ValueLast = 0;
    static UNION_DATA_DEF TrigOD_ValueNow  = 0;
    INT8U   rising_edge_come = 0;
    INT8U   falling_edge_come = 0;
    UNION_DATA_STRUCT_DEF UnionDataStruct;
    UNION_DATA64_STRUCT_DEF un_temp64;
    if(SCP.Control > 0) //���βɼ����ɼ���ɺ�����SCP.Control=0
    {
        if(FirstSimple)
        {
            FirstSimple = FALSE;  //������λ
            FirstTrig = TRUE; //���õ�һ�δ���ʹ��
            TrigHappen = FALSE;  //�Ƿ񴥷�����Ҫ�ô˱��λ������SampleCnt�Ƿ�����
            OneSampleLen = 0;
            SampleCnt = 0;
            SCP.SamplePtr = SCP.Buff;
            SCP_TotalLenShadow = SCP.TotalLen;
            if(SCP.TrigOD != 0)
            {
                switch(SCP.TrigOD_DataType)
                {
                    case ODT16U:
                        TrigOD_ValueNow.Int16u = *((INT16U*)SCP.TrigOD_Ptr);
                        break;
                    case ODT16S:
                        TrigOD_ValueNow.Int16s = *((INT16S*)SCP.TrigOD_Ptr);
                        break;
                    case ODT32U:
                        TrigOD_ValueNow.Int32u = *((INT32U*)SCP.TrigOD_Ptr);
                        break;
                    case ODT32S:
                        TrigOD_ValueNow.Int32s = *((INT32S*)SCP.TrigOD_Ptr);
                        break;
                    case ODT08U:
                        TrigOD_ValueNow.Int8u = *((INT8U*)SCP.TrigOD_Ptr);
                        break;
                    case ODT08S:
                        TrigOD_ValueNow.Int8s = *((INT8S*)SCP.TrigOD_Ptr);
                        break;
                    default :
                        TrigOD_ValueNow.Int32s = 0;
                        break;
                }
                TrigOD_ValueLast.Int32s  = TrigOD_ValueNow.Int32s;
            }
            if(SCP.OD5 != 0)
            {
                OneSampleLen = OneSampleLen+2;
            }
            else
            {
                if(SCP.OD1 != 0)
                {
                    OneSampleLen++;
                }
                if(SCP.OD2 != 0)
                {
                    OneSampleLen++;
                }
            }
            if(SCP.OD6 != 0)
            {
                OneSampleLen = OneSampleLen+2;
            }
            else
            {
                if(SCP.OD3 != 0)
                {
                    OneSampleLen++;
                }
                if(SCP.OD4 != 0)
                {
                    OneSampleLen++;
                }
            }
        }
        SamplePeriodCnt++;
        if(SamplePeriodCnt >= SCP.SamplePeriod)
        {
            SamplePeriodCnt = 0;
            SampleCnt++;
            if(SCP.OD5 != 0)   // 32λ���ݲɼ�������ǾͲ��ܲɼ�16λ�������ˡ�
            {
                UnionDataStruct.Int32s = 0;
                if(SCP.OD5_ScaleFlag == 1) //��鱾�����Ƿ���Ҫ�߶ȱ任
                {
                    UnionDataStruct.Int32s = *SCP.OD5_Ptr;
                    if(SCP.OD5_MinusFlag == 0X80)   //����Ǹ���
                        un_temp64.Int64s = (INT64S)UnionDataStruct.Int32s;
                    else
                        un_temp64.Int64u = (INT64U)UnionDataStruct.Int32u;
                    un_temp64.Int64s = un_temp64.Int64s * 10000;
                    UnionDataStruct.Int32s = un_temp64.Int64s / (*SCP.OD5_SclPtr);
                }
                else
                {
                    UnionDataStruct.Int32s = *SCP.OD5_Ptr;
                }

                if(SCP.SamplePtr <= &SCP.Buff[SCP_LEN-2])  //û�е���buf����������ռ�
                {
                    //*(INT32U*)SCP.SamplePtr = *SCP.OD5_Ptr;
                    *(INT32U*)SCP.SamplePtr = UnionDataStruct.Int32s;
                    SCP.SamplePtr = SCP.SamplePtr + 2;
                    if(SCP.SamplePtr > &SCP.Buff[SCP_LEN-1])  //����buf�����һ���ռ䣬�򷵻�
                    {
                        SCP.SamplePtr = SCP.Buff;
                    }
                }
                else  //�������һ���ռ䣬��32λ���ݲ�ֳ�2��byte��bufβ����ͷ���ֱ�洢
                {
                    //*SCP.SamplePtr = *SCP.OD5_Ptr;
                    *SCP.SamplePtr = UnionDataStruct.Int32s;
                    SCP.SamplePtr = SCP.Buff;
                    //*SCP.SamplePtr = (*SCP.OD5_Ptr)>>16;
                    *SCP.SamplePtr = UnionDataStruct.Int32s >> 16;
                    SCP.SamplePtr++;
                }
            }
            else               //16λ���ݲɼ���
            {
                if(SCP.OD1 != 0)
                {
                    UnionDataStruct.Int32s = 0;
                    if(SCP.OD1_ScaleFlag == 1) //��鱾�����Ƿ���Ҫ�߶ȱ任
                    {
                        UnionDataStruct.Int16s = *SCP.OD1_Ptr;
                        if(SCP.OD1_MinusFlag == 0X80)   //����Ǹ���
                            un_temp64.Int64s = (INT64S)UnionDataStruct.Int16s;
                        else
                            un_temp64.Int64u = (INT64U)UnionDataStruct.Int16u;
                        un_temp64.Int64s = un_temp64.Int64s * 10000;
                        UnionDataStruct.Int32s = un_temp64.Int64s / (*SCP.OD1_SclPtr);
                    }
                    else
                    {
                        UnionDataStruct.Int16s = *SCP.OD1_Ptr;
                    }
                    //*SCP.SamplePtr = *SCP.OD1_Ptr;
                    *SCP.SamplePtr = UnionDataStruct.Int32s;
                    SCP.SamplePtr++;
                    if(SCP.SamplePtr > &SCP.Buff[SCP_LEN-1])  //����buf�����һ���ռ䣬�򷵻�
                    {
                        SCP.SamplePtr = SCP.Buff;
                    }
                }
                if(SCP.OD2 != 0)
                {
                    UnionDataStruct.Int32s = 0;
                    if(SCP.OD2_ScaleFlag == 1) //��鱾�����Ƿ���Ҫ�߶ȱ任
                    {
                        UnionDataStruct.Int16s = *SCP.OD2_Ptr;
                        if(SCP.OD2_MinusFlag == 0X80)   //����Ǹ���
                            un_temp64.Int64s = (INT64S)UnionDataStruct.Int16s;
                        else
                            un_temp64.Int64u = (INT64U)UnionDataStruct.Int16u;
                        un_temp64.Int64s = un_temp64.Int64s * 10000;
                        UnionDataStruct.Int32s = un_temp64.Int64s / (*SCP.OD2_SclPtr);
                    }
                    else
                    {
                        UnionDataStruct.Int16s = *SCP.OD2_Ptr;
                    }
                    //*SCP.SamplePtr = *SCP.OD2_Ptr;
                    *SCP.SamplePtr = UnionDataStruct.Int32s;
                    SCP.SamplePtr++;
                    if(SCP.SamplePtr > &SCP.Buff[SCP_LEN-1])  //����buf�����һ���ռ䣬�򷵻�
                    {
                        SCP.SamplePtr = SCP.Buff;
                    }
                }
            }
            if(SCP.OD6 != 0)
            {
                if(SCP.OD6_ScaleFlag == 1) //��鱾�����Ƿ���Ҫ�߶ȱ任
                {
                    UnionDataStruct.Int32s = *SCP.OD6_Ptr;
                    if(SCP.OD6_MinusFlag == 0X80)   //����Ǹ���
                        un_temp64.Int64s = (INT64S)UnionDataStruct.Int32s;
                    else
                        un_temp64.Int64u = (INT64U)UnionDataStruct.Int32s;
                    un_temp64.Int64s = un_temp64.Int64s * 10000;
                    UnionDataStruct.Int32s = un_temp64.Int64s / (*SCP.OD6_SclPtr);
                }
                else
                {
                    UnionDataStruct.Int32s = *SCP.OD6_Ptr;
                }
                if(SCP.SamplePtr <= &SCP.Buff[SCP_LEN-2])  //û�е���buf����������ռ�
                {
                    //*(INT32U*)SCP.SamplePtr = *SCP.OD6_Ptr;
                    *(INT32U*)SCP.SamplePtr = UnionDataStruct.Int32s;
                    SCP.SamplePtr = SCP.SamplePtr + 2;
                    if(SCP.SamplePtr > &SCP.Buff[SCP_LEN-1])  //����buf�����һ���ռ䣬�򷵻�
                    {
                        SCP.SamplePtr = SCP.Buff;
                    }
                }
                else  //�������һ���ռ䣬��32λ���ݲ�ֳ�2��byte��bufβ����ͷ���ֱ�洢
                {
                    //*SCP.SamplePtr = *SCP.OD6_Ptr;
                    *SCP.SamplePtr = UnionDataStruct.Int32s;
                    SCP.SamplePtr = SCP.Buff;
                    //*SCP.SamplePtr = (*SCP.OD6_Ptr)>>16;
                    *SCP.SamplePtr = UnionDataStruct.Int32s >> 16;
                    SCP.SamplePtr++;
                }
            }
            else
            {
                if(SCP.OD3 != 0)
                {
                    UnionDataStruct.Int32s = 0;
                    if(SCP.OD3_ScaleFlag == 1) //��鱾�����Ƿ���Ҫ�߶ȱ任
                    {
                        UnionDataStruct.Int16s = *SCP.OD3_Ptr;
                        if(SCP.OD3_MinusFlag == 0X80)   //����Ǹ���
                            un_temp64.Int64s = (INT64S)UnionDataStruct.Int16s;
                        else
                            un_temp64.Int64u = (INT64U)UnionDataStruct.Int16u;
                        un_temp64.Int64s = un_temp64.Int64s * 10000;
                        UnionDataStruct.Int32s = un_temp64.Int64s / (*SCP.OD3_SclPtr);
                    }
                    else
                    {
                        UnionDataStruct.Int16s = *SCP.OD3_Ptr;
                    }
                    //*SCP.SamplePtr = *SCP.OD3_Ptr;
                    *SCP.SamplePtr = UnionDataStruct.Int32s;
                    SCP.SamplePtr++;
                    if(SCP.SamplePtr > &SCP.Buff[SCP_LEN-1])  //����buf�����һ���ռ䣬�򷵻�
                    {
                        SCP.SamplePtr = SCP.Buff;
                    }
                }
                if(SCP.OD4 != 0)
                {
                    UnionDataStruct.Int32s = 0;
                    if(SCP.OD4_ScaleFlag == 1) //��鱾�����Ƿ���Ҫ�߶ȱ任
                    {
                        UnionDataStruct.Int16s = *SCP.OD4_Ptr;
                        if(SCP.OD2_MinusFlag == 0X80)   //����Ǹ���
                            un_temp64.Int64s = (INT64S)UnionDataStruct.Int16s;
                        else
                            un_temp64.Int64u = (INT64U)UnionDataStruct.Int16u;
                        un_temp64.Int64s = un_temp64.Int64s * 10000;
                        UnionDataStruct.Int32s = un_temp64.Int64s / (*SCP.OD4_SclPtr);
                    }
                    else
                    {
                        UnionDataStruct.Int16s = *SCP.OD4_Ptr;
                    }
                    //*SCP.SamplePtr = *SCP.OD4_Ptr;
                    *SCP.SamplePtr = UnionDataStruct.Int32s;
                    SCP.SamplePtr++;
                    if(SCP.SamplePtr > &SCP.Buff[SCP_LEN-1])  //����buf�����һ���ռ䣬�򷵻�
                    {
                        SCP.SamplePtr = SCP.Buff;
                    }
                }
            }
            if(SCP.TrigOD != 0) //�Զ��ɼ�
            {
                if(TrigHappen == FALSE)
                {
                    SampleCnt = 0;
                }
                switch(SCP.TrigOD_DataType)
                {
                    case ODT16U:
                        TrigOD_ValueNow.Int16u = *((INT16U*)SCP.TrigOD_Ptr);
                        if((TrigOD_ValueLast.Int16u<SCP.TrigValue)&&(TrigOD_ValueNow.Int16u>=SCP.TrigValue)&&FirstTrig)
                            rising_edge_come = TRUE;
                        if((TrigOD_ValueLast.Int16u>SCP.TrigValue)&&(TrigOD_ValueNow.Int16u<=SCP.TrigValue)&&FirstTrig)
                            falling_edge_come = TRUE;
                        break;
                    case ODT16S:
                        TrigOD_ValueNow.Int16s = *((INT16S*)SCP.TrigOD_Ptr);
                        if((TrigOD_ValueLast.Int16s<SCP.TrigValue)&&(TrigOD_ValueNow.Int16s>=SCP.TrigValue)&&FirstTrig)
                            rising_edge_come = TRUE;
                        if((TrigOD_ValueLast.Int16s>SCP.TrigValue)&&(TrigOD_ValueNow.Int16s<=SCP.TrigValue)&&FirstTrig)
                            falling_edge_come = TRUE;
                        break;
                    case ODT32U:
                        TrigOD_ValueNow.Int32u = *((INT32U*)SCP.TrigOD_Ptr);
                        if((TrigOD_ValueLast.Int32u<SCP.TrigValue)&&(TrigOD_ValueNow.Int32u>=SCP.TrigValue)&&FirstTrig)
                            rising_edge_come = TRUE;
                        if((TrigOD_ValueLast.Int32u>SCP.TrigValue)&&(TrigOD_ValueNow.Int32u<=SCP.TrigValue)&&FirstTrig)
                            falling_edge_come = TRUE;
                        break;
                    case ODT32S:
                        TrigOD_ValueNow.Int32s = *((INT32S*)SCP.TrigOD_Ptr);
                        if((TrigOD_ValueLast.Int32s<SCP.TrigValue)&&(TrigOD_ValueNow.Int32s>=SCP.TrigValue)&&FirstTrig)
                            rising_edge_come = TRUE;
                        if((TrigOD_ValueLast.Int32s>SCP.TrigValue)&&(TrigOD_ValueNow.Int32s<=SCP.TrigValue)&&FirstTrig)
                            falling_edge_come = TRUE;
                        break;
                    case ODT08U:
                        TrigOD_ValueNow.Int8u = *((INT8U*)SCP.TrigOD_Ptr);
                        if((TrigOD_ValueLast.Int8u<SCP.TrigValue)&&(TrigOD_ValueNow.Int8u>=SCP.TrigValue)&&FirstTrig)
                            rising_edge_come = 1;
                        if((TrigOD_ValueLast.Int8u>SCP.TrigValue)&&(TrigOD_ValueNow.Int8u<=SCP.TrigValue)&&FirstTrig)
                            falling_edge_come = TRUE;
                        break;
                    case ODT08S:
                        TrigOD_ValueNow.Int8s = *((INT8S*)SCP.TrigOD_Ptr);
                        if((TrigOD_ValueLast.Int8s<SCP.TrigValue)&&(TrigOD_ValueNow.Int8s>=SCP.TrigValue)&&FirstTrig)
                            rising_edge_come = TRUE;
                        if((TrigOD_ValueLast.Int8s>SCP.TrigValue)&&(TrigOD_ValueNow.Int8s<=SCP.TrigValue)&&FirstTrig)
                            falling_edge_come = TRUE;
                        break;
                    default :
                        TrigOD_ValueNow.Int32s = 0;
                        break;
                }
                if(SCP.TrigEdge > 0)//������
                {
                    //if((TrigOD_ValueLast<SCP.TrigValue)&&(TrigOD_ValueNow>=SCP.TrigValue)&&FirstTrig)//�õ�������,�����ǵ�һ�δ���
                    if(rising_edge_come)
                    {
                        FirstTrig = FALSE;
                        TrigHappen = TRUE;
                        SampleCnt = 0;
                        if(SCP.TotalLen > SCP.BfTrigLen)
                        {
                            SCP_TotalLenShadow = SCP.TotalLen - SCP.BfTrigLen;  //�������ɼ��ĸ���
                        }
                        else
                        {
                            SCP_TotalLenShadow = 0;
                        }
                        TrigPoint = SCP.SamplePtr; //��¼����ʱ������ָ��λ��SCP.Buff�еľ���λ�á�
                    }
                }
                else//�½���
                {
                    //if((TrigOD_ValueLast>SCP.TrigValue)&&(TrigOD_ValueNow<=SCP.TrigValue)&&FirstTrig)//�õ�������,�����ǵ�һ�δ���
                    if(falling_edge_come)
                    {
                        FirstTrig = FALSE;
                        TrigHappen = TRUE;
                        SampleCnt = 0;
                        if(SCP.TotalLen > SCP.BfTrigLen)
                        {
                            SCP_TotalLenShadow = SCP.TotalLen - SCP.BfTrigLen;  //�������ɼ��ĸ���
                        }
                        else
                        {
                            SCP_TotalLenShadow = 0;
                        }
                        TrigPoint = SCP.SamplePtr; //��¼����ʱ������ָ��λ��SCP.Buff�еľ���λ�á�
                    }
                }
                TrigOD_ValueLast  = TrigOD_ValueNow;
            }
            if(SampleCnt >= SCP_TotalLenShadow)  //�ɼ���ɣ�
            {
                if(SCP.TrigOD != 0) //�Զ��ɼ�
                {
                    if((TrigPoint - SCP.Buff) > OneSampleLen*SCP.BfTrigLen)
                    {
                        SCP.HeadPoint = TrigPoint - OneSampleLen*SCP.BfTrigLen;
                    }
                    else
                    {
                        SCP.HeadPoint = (&SCP.Buff[SCP_LEN-1])+1;
                        SCP.HeadPoint = SCP.HeadPoint - OneSampleLen*SCP.BfTrigLen;
                        SCP.HeadPoint = SCP.HeadPoint + (TrigPoint - SCP.Buff);
                    }
                    SCP.TailPoint = SCP.SamplePtr;
                }
                else
                {
                    SCP.HeadPoint = SCP.Buff;
                    SCP.TailPoint = SCP.SamplePtr;
                }
                SampleCnt = 0;
                SCP.SamplePtr = SCP.Buff;
                SCP_TotalLenShadow = SCP.TotalLen;
                SCP.Control = 0;
            }
        }
    }
    else
    {
        FirstSimple = TRUE;
    }
}

//scope function setting
///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
INT8U wSCP_OD1_Func(INT16U command, INT32U *pOD, void *bpOD, INT16U *rpOD)
{
    const OD_Attribute *npOD = NULL;
    npOD = FindOd(*rpOD, *pOD);
    if(npOD != NULL)
    {
        switch((npOD->DataType) & 0X0F)
        {
        case BYTE4:
            SCP.OD5 = SCP.OD1;  //ʹ��OD5����¼
            SCP.OD1 = 0;
            SCP.OD2 = 0;
            SCP.OD5_Ptr = (INT32U*)(npOD->pOD);
            if((npOD->Property) & SCL)
            {
                SCP.OD5_ScaleFlag = 1;
                SCP.OD5_SclPtr = (INT32U*)(npOD->bpOD);
                SCP.OD5_MinusFlag = (npOD->DataType) & 0x80;    //������־λ
            }
            else
            {
                SCP.OD5_ScaleFlag = 0;
                SCP.OD5_SclPtr = 0;
            }
            break;
        case BYTE2:
        case BYTE1:   // ����¼8λ������
            SCP.OD5 = 0;  //ʹ��OD1����¼������OD5�ļ�¼
            SCP.OD1_Ptr = (INT16U*)(npOD->pOD);
            if((npOD->Property) & SCL)
            {
                SCP.OD1_ScaleFlag = 1;
                SCP.OD1_SclPtr = (INT32U*)(npOD->bpOD);
                SCP.OD1_MinusFlag = (npOD->DataType) & 0x80;    //������־λ
            }
            else
            {
                SCP.OD1_ScaleFlag = 0;
                SCP.OD1_SclPtr = 0;
            }
            break;
        default:
            break;
        }
    }
    else
    {
        return (1);
    }
    return (0);
}
INT8U wSCP_OD2_Func(INT16U command, INT32U *pOD, void *bpOD, INT16U *rpOD)
{
    const OD_Attribute *npOD = NULL;
    npOD = FindOd(*rpOD, *pOD);
    if(npOD != NULL)
    {
        switch((npOD->DataType) & 0X0F)
        {
        case BYTE4:
            SCP.OD5 = SCP.OD2;  //ʹ��OD5����¼
            SCP.OD1 = 0;
            SCP.OD2 = 0;
            SCP.OD5_Ptr = (INT32U*)(npOD->pOD);
            if((npOD->Property) & SCL)
            {
                SCP.OD5_ScaleFlag = 1;
                SCP.OD5_SclPtr = (INT32U*)(npOD->bpOD);
                SCP.OD5_MinusFlag = (npOD->DataType) & 0x80;    //������־λ
            }
            else
            {
                SCP.OD5_ScaleFlag = 0;
                SCP.OD5_SclPtr = 0;
            }
            break;
        case BYTE2:
        case BYTE1:   // ����¼8λ������
            SCP.OD5 = 0;  //ʹ��OD1����¼������OD5�ļ�¼
            SCP.OD2_Ptr = (INT16U*)(npOD->pOD);
            if((npOD->Property) & SCL)
            {
                SCP.OD2_ScaleFlag = 1;
                SCP.OD2_SclPtr = (INT32U*)(npOD->bpOD);
                SCP.OD2_MinusFlag = (npOD->DataType) & 0x80;    //������־λ
            }
            else
            {
                SCP.OD2_ScaleFlag = 0;
                SCP.OD2_SclPtr = 0;
            }
            break;
        default:
            break;
        }
    }
    else
    {
        return (1);
    }
    return (0);
}
INT8U wSCP_OD3_Func(INT16U command, INT32U *pOD, void *bpOD, INT16U *rpOD)
{
    const OD_Attribute *npOD = NULL;
    npOD = FindOd(*rpOD, *pOD);
    if(npOD != NULL)
    {
        switch((npOD->DataType) & 0X0F)
        {
        case BYTE4:
            SCP.OD6 = SCP.OD3;  //ʹ��OD5����¼
            SCP.OD3 = 0;
            SCP.OD4 = 0;
            SCP.OD6_Ptr = (INT32U*)(npOD->pOD);
            if((npOD->Property) & SCL)
            {
                SCP.OD6_ScaleFlag = 1;
                SCP.OD6_SclPtr = (INT32U*)(npOD->bpOD);
                SCP.OD6_MinusFlag = (npOD->DataType) & 0x80;    //������־λ
            }
            else
            {
                SCP.OD6_ScaleFlag = 0;
                SCP.OD6_SclPtr = 0;
            }
            break;
        case BYTE2:
        case BYTE1:   // ����¼8λ������
            SCP.OD6 = 0;  //ʹ��OD1����¼������OD5�ļ�¼
            SCP.OD3_Ptr = (INT16U*)(npOD->pOD);
            if((npOD->Property) & SCL)
            {
                SCP.OD3_ScaleFlag = 1;
                SCP.OD3_SclPtr = (INT32U*)(npOD->bpOD);
                SCP.OD3_MinusFlag = (npOD->DataType) & 0x80;    //������־λ
            }
            else
            {
                SCP.OD3_ScaleFlag = 0;
                SCP.OD3_SclPtr = 0;
            }
            break;
        default:
            break;
        }
    }
    else
    {
        return (1);
    }
    return (0);
}
INT8U wSCP_OD4_Func(INT16U command, INT32U *pOD, void *bpOD, INT16U *rpOD)
{
    const OD_Attribute *npOD = NULL;
    npOD = FindOd(*rpOD, *pOD);
    if(npOD != NULL)
    {
        switch((npOD->DataType) & 0X0F)
        {
        case BYTE4:
            SCP.OD6 = SCP.OD4;  //ʹ��OD5����¼
            SCP.OD3 = 0;
            SCP.OD4 = 0;
            SCP.OD6_Ptr = (INT32U*)(npOD->pOD);
            if((npOD->Property) & SCL)
            {
                SCP.OD6_ScaleFlag = 1;
                SCP.OD6_SclPtr = (INT32U*)(npOD->bpOD);
                SCP.OD6_MinusFlag = (npOD->DataType) & 0x80;    //������־λ
            }
            else
            {
                SCP.OD6_ScaleFlag = 0;
                SCP.OD6_SclPtr = 0;
            }
            break;
        case BYTE2:
        case BYTE1:   // ����¼8λ������
            SCP.OD6 = 0;  //ʹ��OD1����¼������OD5�ļ�¼
            SCP.OD4_Ptr = (INT16U*)(npOD->pOD);
            if((npOD->Property) & SCL)
            {
                SCP.OD4_ScaleFlag = 1;
                SCP.OD4_SclPtr = (INT32U*)(npOD->bpOD);
                SCP.OD4_MinusFlag = (npOD->DataType) & 0x80;    //������־λ
            }
            else
            {
                SCP.OD4_ScaleFlag = 0;
                SCP.OD4_SclPtr = 0;
            }
            break;
        default:
            break;
        }
    }
    else
    {
        return (1);
    }
    return (0);
}
INT8U wSCP_OD5_Func(INT16U command, INT32U *pOD, void *bpOD, INT16U *rpOD)
{
    const OD_Attribute *npOD = NULL;
    npOD = FindOd(*rpOD, *pOD);
    if(npOD != NULL)
    {
        switch((npOD->DataType) & 0X0F)
        {
        case BYTE4:
            SCP.OD1 = 0;
            SCP.OD2 = 0;
            SCP.OD5_Ptr = (INT32U*)(npOD->pOD);
            break;
        case BYTE2:
        case BYTE1:   // ����¼8λ������
            SCP.OD1 = 0;
            SCP.OD2 = 0;
            SCP.OD5_Ptr = (INT32U*)(npOD->pOD);
            break;
        default:
            break;
        }

        if((npOD->Property) & SCL)
        {
            SCP.OD5_ScaleFlag = 1;
            SCP.OD5_SclPtr = (INT32U*)(npOD->bpOD);
            SCP.OD5_MinusFlag = (npOD->DataType) & 0x80;    //������־λ
        }
        else
        {
            SCP.OD5_ScaleFlag = 0;
            SCP.OD5_SclPtr = 0;
        }
    }
    else
    {
        return (1);
    }
    return (0);
}
INT8U wSCP_OD6_Func(INT16U command, INT32U *pOD, void *bpOD, INT16U *rpOD)
{
    const OD_Attribute *npOD = NULL;
    npOD = FindOd(*rpOD, *pOD);
    if(npOD != NULL)
    {
        switch((npOD->DataType) & 0X0F)
        {
        case BYTE4:
            SCP.OD1 = 0;
            SCP.OD2 = 0;
            SCP.OD6_Ptr = (INT32U*)(npOD->pOD);
            break;
        case BYTE2:
        case BYTE1:   // ����¼8λ������
            SCP.OD1 = 0;
            SCP.OD2 = 0;
            SCP.OD6_Ptr = (INT32U*)(npOD->pOD);
            break;
        default:
            break;
        }

        if((npOD->Property) & SCL)
        {
            SCP.OD6_ScaleFlag = 1;
            SCP.OD6_SclPtr = (INT32U*)(npOD->bpOD);
            SCP.OD5_MinusFlag = (npOD->DataType) & 0x80;    //������־λ
        }
        else
        {
            SCP.OD6_ScaleFlag = 0;
            SCP.OD6_SclPtr = 0;
        }

    }
    else
    {
        return (1);
    }
    return (0);
}
INT8U wSCP_TrigOD_Func(INT16U command, INT32U *pOD, void *bpOD, INT16U *rpOD)
{
    const OD_Attribute *npOD = NULL;
    npOD = FindOd(*rpOD, *pOD);
    if(npOD != NULL)
    {
        SCP.TrigOD_DataType = npOD->DataType;
        SCP.TrigOD_Ptr = npOD->pOD;
        SCP.OD5_MinusFlag = (npOD->DataType) & 0x80;    //������־λ
        if((npOD->Property) & SCL)
        {
            SCP.TrigOD_ScaleFlag = 1;
            SCP.TrigOD_ScalePtr = npOD->bpOD;
        }
        else
        {
            SCP.TrigOD_ScaleFlag = 0;
            SCP.TrigOD_ScalePtr = NULL;
        }
    }
    else
    {
        return (1);
    }
    return (0);
}

INT8U wSCP_TrigValueFunc(INT16U command, INT32S *pOD, void *bpOD, void *rpOD)
{
    UNION_DATA_STRUCT_DEF UnionDataStruct;
    UNION_DATA64_STRUCT_DEF un_temp64;

    if(SCP.TrigOD_ScaleFlag == 1)//Ϊ�߶ȱ任���Ĵ���
    {
        UnionDataStruct.Int32u = *pOD;
        switch(SCP.TrigOD_DataType)
        {
        case ODT16U:
            un_temp64.Int64u = (INT64U)UnionDataStruct.Int16u;
            break;
        case ODT16S:
            un_temp64.Int64s = (INT64S)UnionDataStruct.Int16s;
            break;
        case ODT08U:
            un_temp64.Int64u = (INT64U)UnionDataStruct.Int8u;
            break;
        case ODT08S:
            un_temp64.Int64s = (INT64U)UnionDataStruct.Int8s;
            break;
        case ODT32U:
            un_temp64.Int64u = (INT64U)UnionDataStruct.Int32u;
            break;
        case ODT32S:
            un_temp64.Int64s = (INT64U)UnionDataStruct.Int32s;
            break;
        default :
            break;
        }

        un_temp64.Int64s = un_temp64.Int64s * (*(SCP.TrigOD_ScalePtr));
        UnionDataStruct.Int32s = un_temp64.Int64s / 10000;
        SCP.TrigValue =  UnionDataStruct.Int32u;
    }
    else
    {
        SCP.TrigValue = *pOD;
    }
    return (0);
}
