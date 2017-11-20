/*
 * SR2D1_0V3_MCU_S1 -- Service robot 2In1 servo driver
 * (C)2016 Kinco Electric (Shenzhen) Ltd. -- All rights reserved
 */
#include "includes.h"
void ComEventProcessTask(void);     //UART3
void DriverFaultCheckAndProcess(DriverStructTypedef* pDX);
void OtherFaultCheckAndProcess(void);
void DCBUS_VoltCheckAndProcess(void);
void DinStateRead(void);
void DoutProcess(void);
void SaveConfigurationProcess(void);
void SteeringEngineCtrlProcee(void);
void DC_MotorProcess(void);
void PlatformProcess(void);

BIT32_STRUCT_DEF    Enc_Hall;

//�ϵ�ϵͳ��ʼ��
void SystemAppInit(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // 2 bits for pre-emption priority,2 bits for subpriority

    GpioInit();

    Enc1_PowerOn();
    Enc2_PowerOn();

    sEE_Init();
    PowerOnOD_Init();
    CreatQueueType5(&USART3_RX_Queue,USART3_RX_QueueBuff,USART3_RX_QUEUE_MAX_SIZE);
    CreatQueueType5(&USART3_TX_Queue,USART3_TX_QueueBuff,USART3_TX_QUEUE_MAX_SIZE);
    USART3_Init();      //��PC��ͨ��,������
    CreatQueueType5(&USART1_RX_Queue,USART1_RX_QueueBuff,USART1_RX_QUEUE_MAX_SIZE);
    USART1_Init();      //��PC��ͨ��,������
    LocalProperty.MyID = 0x01;
    CreatQueueType5(&CAN2_RX_Queue,CAN2_RX_QueueBuff,CAN2_RX_QUEUE_MAX_SIZE);

    USERCAN_Init();             //CAN2 initial
    InitTIM1();                 //for pwm
    InitTIM8();                 //for pwm
    InitTIM2();                 //for encode input
    InitTIM4();                 //for encode input
    InitTIM9();                 //for DC motor control
    TIM_Cmd(TIM8,ENABLE);
    TIM_Cmd(TIM1,ENABLE);

    AppADC_RegInjInit(&DX[D1]);
    SPI3_Init();
    GetHallOffset(&DX[D1].Fbk, &DX[D1].MotorPar);
    GetHallOffset(&DX[D2].Fbk, &DX[D2].MotorPar);
    InitLogicVoltErrInterrupt();
}

int main(void)
{
    INT32U  cnt=0;
    SysTickConfig();
    SystemAppInit();

    while(1)
    {
        ComEventProcessTask();      //UART3

//driver 1,2 common
        DinStateRead();     //read the din state
        DinFuncHandle();    //Din function handle
        SteeringEngineCtrlProcee(); //�������
        DCBUS_VoltCheckAndProcess();//��ѹ�ɼ�
        PlatformProcess();          //�����˿���
        DriverFaultCheckAndProcess(&DX[D1]);
        DriverFaultCheckAndProcess(&DX[D2]);
        OtherFaultCheckAndProcess();
        DoutProcess();
        SaveConfigurationProcess();
    }

}
void ComEventProcessTask(void)      //UART3
{
    INT8U i,temp8u;
    INT8U radar_temp[8];
    DMA_InitTypeDef DMA_InitStructure;

//usart3 sdo
    if(SysService.ComMsg.Uart3RxFnCnt != 0)
    {
        GetQueueType5_Item((ADT_QUEUE_TYPE5_DEF*)&USART3_RX_Queue, &SdoBuf.All[0], 10);
        SysService.ComMsg.Uart3RxFnCnt --;
        if(SdoBuf.All[0] == 0x7f)
        {
            SysState.ActiveId = IDX;
        }else if(SdoBuf.All[0] == LocalProperty.MyID)
        {
            SysState.ActiveId = ID1;
        }else if(SdoBuf.All[0] == (LocalProperty.MyID + 1))
        {
            SysState.ActiveId = ID2;
        }
        else
        {
            SysState.ActiveId = 0;
        }
        if(SysState.ActiveId != 0)
        {
            NewProcessOD(SysState.ActiveId);
            AddQueueType5_Item(&USART3_TX_Queue, &SdoBuf.All[0],10);
            //clear the id
            if(SysState.IdSpcManualSet == 0)    //auto updata the spc id
            {
                SysState.ActiveIdSpc = SysState.ActiveId;
            }
            SysState.ActiveIdReg = SysState.ActiveId;
        }
    }


    if(!IsQueueType5_Empty(&USART3_TX_Queue))   //���в�Ϊ��
    {
        if(DMA_GetCurrDataCounter(USART3_TX_DMA_STREAM) == 0)   //uart3 tx dma ����
        {
            GetQueueType5_Item((ADT_QUEUE_TYPE5_DEF*)&USART3_TX_Queue, &USART3_DMA_TD_Buff[0], 10);
            DMA_Cmd(USART3_TX_DMA_STREAM, DISABLE);
            DMA_SetCurrDataCounter(USART3_TX_DMA_STREAM, 10);
            DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF3|DMA_FLAG_HTIF3|DMA_FLAG_TEIF3|DMA_FLAG_DMEIF3|DMA_FLAG_FEIF3);  //must clear these flags before start another DMA transfer
            DMA_Cmd(USART3_TX_DMA_STREAM, ENABLE);
            USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);
        }
    }
    else
    {
        if(0xAA55 == SysService.SoftRestartFlag)    //����λMCUָ��
        {
            while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
            {
                ;    //�ȴ��������
            }
            __set_FAULTMASK(1);     //�ر������ж�
            NVIC_SystemReset();     //��λMCU
        }
    }



//can sdo
    if(SysService.ComMsg.CAN2_RxFnCnt != 0)
    {
        GetQueueType5_Item((ADT_QUEUE_TYPE5_DEF*)&CAN2_RX_Queue, &SdoBuf.All[0], 9);   //ָ������ѽ,yes!
        SysService.ComMsg.CAN2_RxFnCnt --;
        if(SdoBuf.All[0] == 0x7f)
        {
            SysState.ActiveId = IDX;
        }else if(SdoBuf.All[0] == LocalProperty.MyID)
        {
            SysState.ActiveId = ID1;
        }else if(SdoBuf.All[0] == (LocalProperty.MyID + 1))
        {
            SysState.ActiveId = ID2;
        }
        else
        {
            SysState.ActiveId = 0;
        }


        if(SysState.ActiveId != 0)
        {
        SdoBuf.Split.Check = -(SdoBuf.All[0] + SdoBuf.All[1] + SdoBuf.All[2]\
                               + SdoBuf.All[3] + SdoBuf.All[4] + SdoBuf.All[5]\
                               + SdoBuf.All[6] + SdoBuf.All[7] + SdoBuf.All[8]);
        NewProcessOD(SysState.ActiveId);
        CAN2_TX.StdId = 0x580 | SdoBuf.Split.ID; // ��Ӧ����
        CAN2_TX.ExtId=0;
        CAN2_TX.RTR=0;
        CAN2_TX.IDE=0;
        CAN2_TX.DLC=8;
        for(i=1; i<9; i++)
        {
            CAN2_TX.Data[i-1] = SdoBuf.All[i];
        }
        temp8u = CAN_Transmit(CAN2, &CAN2_TX);    //���Ͳ��õ������
//        while( CAN_TxStatus_Ok != CAN_TransmitStatus(CAN1, temp8u));  //�ȴ���������Ϣ�������
        if(0xAA55 == SysService.SoftRestartFlag)    //����λMCUָ��
        {
            __set_FAULTMASK(1);     //�ر������ж�
            NVIC_SystemReset();     //��λMCU
        }
        }
    }

    //usart1,��ȡ������ģ��
    if(SysService.ComMsg.Uart1RxFnCnt != 0)
    {
        GetQueueType5_Item((ADT_QUEUE_TYPE5_DEF*)&USART1_RX_Queue, &radar_temp[0], 8);    //ָ������ѽ,yes!
        SysService.ComMsg.Uart1RxFnCnt --;
        if((SysService.Radar.FrameProperty == radar_temp[1]) &&\
            (SysService.Radar.FrameTail == radar_temp[7])) //check the fram is ok?
        {
            if(SysService.Radar.FrameHead1 == radar_temp[0])    //channel ABCD
            {
                SysService.Radar.Data1.Split.Channel4 = radar_temp[3];   //channel 1
                SysService.Radar.Data1.Split.Channel3 = radar_temp[4];   //channel 2
                SysService.Radar.Data1.Split.Channel2 = radar_temp[5];   //channel 3
                SysService.Radar.Data1.Split.Channel1 = radar_temp[6];   //channel 4
            }
            else if (SysService.Radar.FrameHead2 == radar_temp[0])
            {
                SysService.Radar.Data2.Split.Channel8 = radar_temp[3];   //channel 5
                SysService.Radar.Data2.Split.Channel7 = radar_temp[4];   //channel 6
                SysService.Radar.Data2.Split.Channel6 = radar_temp[5];   //channel 7
                SysService.Radar.Data2.Split.Channel5 = radar_temp[6];   //channel 8
            }
        }
    }
}

void DriverFaultCheckAndProcess(DriverStructTypedef* pDX)
{
    INT8U driver_off_flag = 0;

    //calculate the device temperature
    pDX->TempBase.AdcOrg = *pDX->DriveInfo.TempDtPtr;
    CalAnalogRealDec(&pDX->TempBase);
    pDX->DriveInfo.TempDevice = pDX->TempBase.AdcRealDec - pDX->DriveInfo.TempDeviceOffset;
    if(pDX->DriveInfo.TempDevice > pDX->DriveInfo.TempErrPoint)
    {
        pDX->GbFlg.ErrState1 |= TEMP_OVER_ERR_CODE;
    }

    if((pDX->MotorPar.FB_Type) & ENC_ABZ_CHECK_CODE)    //if need check the enc abz
    {
        if((pDX->GbFlg.ErrState1 & ENC_ABZ_ERR_CODE) != ENC_ABZ_ERR_CODE)
        {
            if(_GET_ENC1_ABZ_ERR == ENC_ABZ_BIT_ERR)
            {
                pDX->GbFlg.ErrState1 |= ENC_ABZ_ERR_CODE;
            }
        }
    }
#if 0
    if((pDX->MotorPar.FB_Type) & ENC_UVW_CHECK_CODE)    //if need check the enc uvw
    {
        if((pDX->GbFlg.ErrState1 & ENC_UVW_ERR_CODE) != ENC_UVW_ERR_CODE)
        {
            if(_GET_ENC1_UVW_ERR == ENC_UVW_BIT_ERR)
            {
                pDX->GbFlg.ErrState1 |= ENC_UVW_ERR_CODE;
            }
        }
    }
#endif
    if((pDX->MotorPar.FB_Type) & ENC_COMU_CHECK_CODE)   //if comunication type encoder
    {
        if((pDX->GbFlg.ErrState1 & FIND_MOTOR_ERR_CODE) != FIND_MOTOR_ERR_CODE)
        {
            if(pDX->Fbk.ComuEncErrFlag == 1)
            {
                pDX->GbFlg.ErrState1 |= FIND_MOTOR_ERR_CODE;
            }
        }
    }

    //motor iit
    if(pDX->MotorIIt.RealIItDEC_D >= pDX->MotorIIt.UserSetII_DEC)
    {
        pDX->GbFlg.ErrState1 |= IIT_ERR_CODER;
    }

    //follow error
    if(pDX->PosLp.FollowErrMsg == 1)
    {
        pDX->GbFlg.ErrState1 |= FOLLOW_ERR_CODE;
    }

    pDX->GbFlg.ErrAct1 = (pDX->GbFlg.ErrState1 & pDX->GbFlg.ErrMask1);
    pDX->GbFlg.ErrPending1 = (pDX->GbFlg.ErrAct1 & (~pDX->GbFlg.ErrProcess1));   //��Ҫ����Ĵ���
    if(pDX->GbFlg.ErrPending1) //have error be process
    {
        if(pDX->GbFlg.ErrPending1 & ENC_ABZ_ERR_CODE)
        {
            pDX->GbFlg.ErrProcess1 |= ENC_ABZ_ERR_CODE;
            (pDX->pF_EncPwrOff)();
            pDX->GbFlg.Excitation = 0;
            driver_off_flag = 1;
        }
        if(pDX->GbFlg.ErrPending1 & ENC_UVW_ERR_CODE)
        {
            pDX->GbFlg.ErrProcess1 |= ENC_UVW_ERR_CODE;
            (pDX->pF_EncPwrOff)();
            pDX->GbFlg.Excitation = 0;
            driver_off_flag = 1;
        }
        if(pDX->GbFlg.ErrPending1 & FIND_MOTOR_ERR_CODE)
        {
            pDX->GbFlg.ErrProcess1 |= FIND_MOTOR_ERR_CODE;
            (pDX->pF_EncPwrOff)();
            pDX->GbFlg.Excitation = 0;
            driver_off_flag = 1;
        }
        if(pDX->GbFlg.ErrPending1 & DCBUS_LOW_ERR_CODE)
        {
            pDX->GbFlg.ErrProcess1 |= DCBUS_LOW_ERR_CODE;
            driver_off_flag = 1;
        }
        if(pDX->GbFlg.ErrPending1 & DCBUS_OVER_ERR_CODE)
        {
            pDX->GbFlg.ErrProcess1 |= DCBUS_OVER_ERR_CODE;
            driver_off_flag = 1;
        }
        if(pDX->GbFlg.ErrPending1 & IIT_ERR_CODER)
        {
            pDX->GbFlg.ErrProcess1 |= IIT_ERR_CODER;
            driver_off_flag = 1;
        }
        if(pDX->GbFlg.ErrPending1 & SHORT_CIRCUIT_CODE)
        {
            pDX->GbFlg.ErrProcess1 |= SHORT_CIRCUIT_CODE;
            driver_off_flag = 1;
        }
        if(pDX->GbFlg.ErrPending1 & TEMP_OVER_ERR_CODE)
        {
            pDX->GbFlg.ErrProcess1 |= TEMP_OVER_ERR_CODE;
            driver_off_flag = 1;
        }
        if(pDX->GbFlg.ErrPending1 & FOLLOW_ERR_CODE)
        {
            pDX->GbFlg.ErrProcess1 |= FOLLOW_ERR_CODE;
            driver_off_flag = 1;
        }

        if(driver_off_flag == 1)
        {
            (pDX->pF_DriverOff)();         //disable time pwm output
            pDX->GbFlg.CtrlWordAct = 0x06;
            pDX->GbFlg.CtrlWord = 0x06;
            pDX->GbFlg.CtrlModeAct = 0;
        }

    }
    if(pDX->GbFlg.ErrAct1)
    {
        pDX->DriveInfo.StateWord = DR_STATE_FAULT_BIT;
        (pDX->pF_DriverErrLedOn)();
    }
    else
    {
        pDX->DriveInfo.StateWord = DR_STATE_FAULT_CLR;
        (pDX->pF_DriverErrLedOff)();
    }
}

void OtherFaultCheckAndProcess(void)
{
    DXC.GbFlg.ErrAct1 = DX[0].GbFlg.ErrAct1 & DX[1].GbFlg.ErrAct1;
    DXC.GbFlg.ErrProcess1 = DX[0].GbFlg.ErrProcess1 & DX[1].GbFlg.ErrProcess1;
    DXC.GbFlg.ErrPending1 = DXC.GbFlg.ErrAct1 & (~DXC.GbFlg.ErrProcess1);
    if(DXC.GbFlg.ErrPending1)    //��������
    {
        if(DXC.GbFlg.ErrPending1 & DCMOTOR_ERR_CODE)
        {
            DX[0].GbFlg.ErrProcess1 |= DCMOTOR_ERR_CODE;
            DX[1].GbFlg.ErrProcess1 |= DCMOTOR_ERR_CODE;
            DC_MotorOff();
            DXC.DM_Ctrl.CtrlWord = 0x0006;
        }
        if(DXC.GbFlg.ErrPending1 & PLM_LIMIT_ERR_CODE)
        {
            DX[0].GbFlg.ErrProcess1 |= PLM_LIMIT_ERR_CODE;
            DX[1].GbFlg.ErrProcess1 |= PLM_LIMIT_ERR_CODE;
            DC_MotorOff();
            DXC.DM_Ctrl.CtrlWord = 0x0006;
        }
        if(DXC.GbFlg.ErrPending1 & CHOPING_R_ERR_CODE)
        {
            DX[0].GbFlg.ErrProcess1 |= CHOPING_R_ERR_CODE;
            DX[1].GbFlg.ErrProcess1 |= CHOPING_R_ERR_CODE;
        }
    }
    //��д�����������
    if((DXC.GbFlg.ErrAct1 & DCMOTOR_ERR_CODE) != DCMOTOR_ERR_CODE)
    {
        DX[0].GbFlg.ErrState1 &= (~DCMOTOR_ERR_CODE);
        DX[1].GbFlg.ErrState1 &= (~DCMOTOR_ERR_CODE);
    }
    if((DXC.GbFlg.ErrAct1 & CHOPING_R_ERR_CODE) != CHOPING_R_ERR_CODE)
    {
        DX[0].GbFlg.ErrState1 &= (~CHOPING_R_ERR_CODE);
        DX[1].GbFlg.ErrState1 &= (~CHOPING_R_ERR_CODE);
    }
    if((DXC.GbFlg.ErrAct1 & PLM_LIMIT_ERR_CODE) != PLM_LIMIT_ERR_CODE)
    {
        DX[0].GbFlg.ErrState1 &= (~PLM_LIMIT_ERR_CODE);
        DX[1].GbFlg.ErrState1 &= (~PLM_LIMIT_ERR_CODE);
    }

//���������źŲɼ�
    //�������
    if(DX[0].GbFlg.CtrlWordAct == 0x0F && SysService.SteEng[0].BitCtrl == 1)
        DX[0].GbFlg.ErrState1 |= STEENT_ERR_CODE;
    else
        DX[0].GbFlg.ErrState1 &= (~STEENT_ERR_CODE);

    if(DX[1].GbFlg.CtrlWordAct == 0x0F && SysService.SteEng[1].BitCtrl == 1)
        DX[1].GbFlg.ErrState1 |= STEENT_ERR_CODE;
    else
        DX[1].GbFlg.ErrState1 &= (~STEENT_ERR_CODE);


    //Эͬ�˶�����ʱ�������
    if(MOX.GbFlg.MotionCtrWordAct == 0x0f)
    {
        if((DX[0].DriveInfo.StateWord != 0) || (DX[1].DriveInfo.StateWord != 0))    //ֻҪ�и������,��ȫ������
        {
            MOX.GbFlg.MotionCtrWord = 0x06;
            MOX.GbFlg.MotionCtrWordAct = 0x06;
            wMotionControlWord(0, NULL, NULL, NULL);
        }
    }

    //DC motor iit error
    if(DXC.DM_IIt.RealIItDEC_D >= DXC.DM_IIt.UserSetII_DEC)
    {
        DX[0].GbFlg.ErrState1 |= DCMOTOR_ERR_CODE;   //for display
        DX[1].GbFlg.ErrState1 |= DCMOTOR_ERR_CODE;
    }
    if(DXC.BrakeIIt.RealIItDEC_D >= DXC.BrakeIIt.UserSetII_DEC)
    {
        DX[0].GbFlg.ErrState1 |= CHOPING_R_ERR_CODE;
        DX[1].GbFlg.ErrState1 |= CHOPING_R_ERR_CODE;
    }

    //ƽ̨������
    if(((DXC.PlatformInfo.CalCmd) & 0x03) == 0x00)   //������У׼ʱ�ǲ���������λ��
    {
    if(DXC.PlatformInfo.Pos.AllPulse.Int32s >= DXC.PlatformInfo.UpLimitDec) //uplimit
    {
        DXC.PlatformInfo.UpLimitFlag = 1;
        if((DXC.DM_Ctrl.CtrlWord & 0x0f) && DXC.DM_Ctrl.Direction == PLATFORM_UP_MOVE)
        {
            DX[0].GbFlg.ErrState1 |= PLM_LIMIT_ERR_CODE;
            DX[1].GbFlg.ErrState1 |= PLM_LIMIT_ERR_CODE;
        }
    }
    else
    {
        DXC.PlatformInfo.UpLimitFlag = 0;
    }

    if(DXC.PlatformInfo.Pos.AllPulse.Int32s <= DXC.PlatformInfo.DownLimitDec) //downlimit
    {
        DXC.PlatformInfo.DownLimitFlag = 1;
        if((DXC.DM_Ctrl.CtrlWord & 0x0f) && DXC.DM_Ctrl.Direction == PLATFORM_DOWM_MOVE)
        {
            DX[0].GbFlg.ErrState1 |= PLM_LIMIT_ERR_CODE;
            DX[1].GbFlg.ErrState1 |= PLM_LIMIT_ERR_CODE;
        }
    }
    else
    {
        DXC.PlatformInfo.DownLimitFlag = 0;
    }
    }
    else
    {
        DXC.PlatformInfo.UpLimitFlag = 0;
        DXC.PlatformInfo.DownLimitFlag = 0;
    }
}

void DCBUS_VoltCheckAndProcess(void)
{
    //calculate the DC bus real voltage
    DXC.DC_BusVolt.AdcOrg = ADC1_2_DMA_DATA2.Split.DcbusV_Org;
    CalAnalogRealDec(&DXC.DC_BusVolt);
    if(DXC.DC_BusVolt.AdcOrgAct > DXC.ComInfo.DC_BusOverDec)
    {
        DX[D1].GbFlg.ErrState1 |= DCBUS_OVER_ERR_CODE;
        DX[D2].GbFlg.ErrState1 |= DCBUS_OVER_ERR_CODE;
    }
    if(DX[D1].GbFlg.CtrlWordAct & 0x0f == 0x0f || DX[D2].GbFlg.CtrlWordAct & 0x0f == 0x0f )   //low voltage detect only at driver enable
    {
        if(DXC.DC_BusVolt.AdcOrgAct < DXC.ComInfo.DC_BusUnderDec)
        {
            DX[D1].GbFlg.ErrState1 |= DCBUS_LOW_ERR_CODE;
            DX[D2].GbFlg.ErrState1 |= DCBUS_LOW_ERR_CODE;
        }

    }
    DXC.ComInfo.RealDcBus = DXC.DC_BusVolt.AdcRealDec;    //for display,DriveInfo.RealDcBus is 16bit(Kinco)

}

void DinStateRead(void)
{
    BIT16_STRUCT_DEF din_temp;

    din_temp.All = GPIO_ReadInputData(GPIOD);

    DXC.Din.State.Bit.DIN1 = din_temp.Bit.bit4;
    DXC.Din.State.Bit.DIN2 = din_temp.Bit.bit7;
    DXC.Din.State.Bit.DIN3 = din_temp.Bit.bit10;
    DXC.Din.State.Bit.DIN4 = din_temp.Bit.bit11;
    DXC.Din.State.Bit.DIN5 = din_temp.Bit.bit12;
    DXC.Din.State.Bit.DIN6 = din_temp.Bit.bit13;
    DXC.Din.State.Bit.DIN7 = din_temp.Bit.bit14;
    DXC.Din.State.Bit.DIN8 = din_temp.Bit.bit15;

    DXC.Din.Virtual = (~(DXC.Din.Polarity ^  DXC.Din.State.All)) | DXC.Din.Simulate;

    //read AGV sensor digtial input
    SysService.AgvSnr.OrgData.All = *(SysService.AgvSnr.pValData);
    SysService.AgvSnr.InverData.All = ~(SysService.AgvSnr.OrgData.All);
}

void DoutProcess(void)
{
    DXC.Dout.Virtual  = DXC.Dout.Sys | DXC.Dout.Simulate ;
    DXC.Dout.State = (~(DXC.Dout.Virtual ^ DXC.Dout.Polarity)) & DXC.Dout.Mask;
    //*(DXC.Dout.pDoutData) = ~(DXC.Dout.State);
    if(DXC.Dout.State & 0x0001)
        _DOUT1_ON;
    else
        _DOUT1_OFF;
    if(DXC.Dout.State & 0x0002)
        _DOUT2_ON;
    else
        _DOUT2_OFF;
    if(DXC.Dout.State & 0x0004)
        _DOUT3_ON;
    else
        _DOUT3_OFF;
    if(DXC.Dout.State & 0x0008)
        _DOUT4_ON;
    else
        _DOUT4_OFF;
    if(DXC.Dout.State & 0x0010)
        _DOUT5_ON;
    else
        _DOUT5_OFF;
    if(DXC.Dout.State & 0x0020)
        _DOUT6_ON;
    else
        _DOUT6_OFF;
    if(DXC.Dout.State & 0x0040)
        _DOUT7_ON;
    else
        _DOUT7_OFF;
}

void SaveConfigurationProcess(void)
{
    INT8U   save_id;
    INT16U  save_type;

    save_id = SysState.ActiveId;

    if(SysService.DS.SaveComInfoCmd == 1)
    {
        save_type = S1;
        save_id = IDX;
        OD_DataSave(save_id, save_type);
        SysService.DS.SaveComInfoCmd = 0;
    }

    if(SysService.DS.SaveIoCfgCmd == 1)
    {
        save_type = S2;
        save_id = IDX;
        OD_DataSave(save_id, save_type);
        SysService.DS.SaveIoCfgCmd = 0;
    }

    if(SysService.DS.SaveDeviceCalibCmd == 1)
    {
        save_type = S3;
        OD_DataSave(save_id, save_type);
        SysService.DS.SaveDeviceCalibCmd = 0;
    }

    if(SysService.DS.SaveMotorPraCmd == 1)
    {
        save_type = S4;
        OD_DataSave(save_id, save_type);
        SysService.DS.SaveMotorPraCmd = 0;
    }

    if(SysService.DS.SaveCtrLpPraCmd == 1)
    {
        save_type = S5;
        OD_DataSave(save_id, save_type);
        SysService.DS.SaveCtrLpPraCmd = 0;
    }

    if(SysService.DS.SaveMotionCmd == 1)
    {
        save_type = S6;
        save_id = IDX;
        OD_DataSave(save_id, save_type);
        SysService.DS.SaveMotionCmd = 0;
    }

     if(SysService.DS.SaveStEngCmd == 1)
    {
        save_type = S7;
        save_id = IDX;
        OD_DataSave(save_id, save_type);
        SysService.DS.SaveStEngCmd = 0;
    }

    if(SysService.DS.SavePlatformCmd == 1)
    {
        save_type = S8;
        save_id = IDX;
        OD_DataSave(save_id, save_type);
        SysService.DS.SavePlatformCmd = 0;
    }
}


//�������
void SteeringEngineCtrlProcee(void)
{
    //���1
    if(SysService.SteEng[0].BitCtrl == 1)
    {
        if(DX[0].GbFlg.CtrlWordAct != 0x0F)
        {
            *SysService.SteEng[0].pValData = SysService.SteEng[0].LockVal;
        }
    }
    else
    {
        *SysService.SteEng[0].pValData = SysService.SteEng[0].UnLockVal;
    }
    //���2
    if(SysService.SteEng[1].BitCtrl == 1)
    {
        if(DX[1].GbFlg.CtrlWordAct != 0x0F)
        {
            *SysService.SteEng[1].pValData = SysService.SteEng[1].LockVal;
        }
    }
    else
    {
        *SysService.SteEng[1].pValData = SysService.SteEng[1].UnLockVal;
    }
}

//ƽ̨����
void PlatformProcess(void)
{
    GetPulseCount(&DXC.PlatformInfo.Pos);
    if(DXC.PlatformInfo.CalCmd == 1)    //��ʼУ׼
    {
        DXC.PlatformInfo.CalOverTimeCnt = 0;
        DXC.PlatformInfo.UpLimitFlag = 0;
        DXC.PlatformInfo.DownLimitFlag = 0;
        DXC.PlatformInfo.CalCmd = 2;    //У׼�����б��
        DXC.DM_Ctrl.CtrlWord = 0x1f;    //���·�����ԭ��
        wDC_MotorControlWord(0, &DXC.DM_Ctrl.CtrlWord, NULL, NULL);
    }
    else if(DXC.PlatformInfo.CalCmd == 2)   //Ѱ��ԭ�������
    {
        if(DXC.PlatformInfo.OriginDin != 0)  //����Ƿ��ҵ�ԭ����
        {
            DXC.DM_Ctrl.CtrlWord = 0x06;
            wDC_MotorControlWord(0, &DXC.DM_Ctrl.CtrlWord, NULL, NULL);
            DXC.PlatformInfo.Pos.AllPulse.Int32s = 0;   //����
            DXC.PlatformInfo.CalCmd = 0x60;    //У׼��ɱ��
            //Save the enc count offset
            sEE_WriteBuffer(&DXC.PlatformInfo.Pos.AllPulse.All8s[0], 160, 4);
        }
        else
        {
            if(DX[0].GbFlg.ErrState1 & DCMOTOR_ERR_CODE)//���������
                DXC.PlatformInfo.CalCmd = 0x80;    //У׼����
            if(DXC.PlatformInfo.CalOverTimeCnt > (1000*60*2))   //����2������
            {
                DXC.PlatformInfo.CalCmd = 0x70;    //У׼��ʱ
                DXC.DM_Ctrl.CtrlWord = 0x06;
                wDC_MotorControlWord(0, &DXC.DM_Ctrl.CtrlWord, NULL, NULL);
            }
        }
    }
}
