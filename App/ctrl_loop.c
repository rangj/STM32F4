/*
 * SR2D1_0V3_MCU_S1 -- Service robot 2In1 servo driver
 * (C)2016 Kinco Electric (Shenzhen) Ltd. -- All rights reserved
 */
#include "includes.h"
GlobleFlagTypedef       GbFlg;

void ADC_IRQHandler(void)
{
    ResultStatus rlt_status;
    INT8U   en_dx1_pos_loop_pi = 0;
    INT8U   en_dx1_spd_loop_pi = 0;
    INT8U   en_dx1_cur_loop_pi = 0;
    INT8U   en_dx2_pos_loop_pi = 0;
    INT8U   en_dx2_spd_loop_pi = 0;
    INT8U   en_dx2_cur_loop_pi = 0;
    INT32S  temp32s;
    INT32S  temp32u;
    INT64U  temp64u;
    INT64S  temp64s1;
    INT64S  temp64s2;
    INT8U   i;
    //debug
    DX[D1].GbFlg.TIM8_CntSyn = TIM8->CNT;
    DX[D1].GbFlg.TIM1_CntSyn = TIM1->CNT;

    _RUN_LED_OFF;

    //put the ADC data to buff2,不影响DMA
    for(i=0; i<5; i++)
    {
        ADC1_2_DMA_DATA2.Buff[i] = ADC1_2_DMA_DATA.Buff[i];
    }

    ADC_SoftwareStartConv(ADC1);    //must first start adc1,then adc2, or the DMA data sequence will not right
    ADC_SoftwareStartConv(ADC2);

    //DC motor on control
    DC_MotorControl();

    if(ADC_GetFlagStatus(ADC2, ADC_FLAG_JEOC) == SET)
    {
        _RUN_LED_ON;
        ADC_ClearFlag(ADC2, ADC_FLAG_JEOC);
        ADC_ClearITPendingBit(ADC2, ADC_IT_JEOC);
        ADC_ClearFlag(ADC1, ADC_FLAG_JEOC);
        ADC_ClearITPendingBit(ADC1, ADC_IT_JEOC);
        _RUN_LED_OFF;
        //get the adc value, current from -2048 to 2048
        DX[D1].CurU.AdcOrg = ADC_GetInjectedConversionValue(ADC1,ADC_InjectedChannel_1);
        DX[D2].CurU.AdcOrg = ADC_GetInjectedConversionValue(ADC1,ADC_InjectedChannel_2);
        DX[D1].CurV.AdcOrg = ADC_GetInjectedConversionValue(ADC2,ADC_InjectedChannel_1);//...66clk
        DX[D2].CurV.AdcOrg = ADC_GetInjectedConversionValue(ADC2,ADC_InjectedChannel_2);//...66clk
        _RUN_LED_ON;
//motion
        MotionCtrol();
        _RUN_LED_OFF;
//driver 2 cotrol
        LoopControl(&DX[D2]);
//driver 1 cotrol
        LoopControl(&DX[D1]);


//7777777777777777777777777777777777777777777777777
        _RUN_LED_ON;
        //calculate motor1 iit
        CalIItDec(&DX[D1].MotorIIt, DX[D1].Park.Iq);
        //calculate motor2 iit
        CalIItDec(&DX[D2].MotorIIt, DX[D2].Park.Iq);
        //calculate break  iit
        CalIItDec(&DXC.BrakeIIt, DXC.CurrBk.AdcRealDec);

        _RUN_LED_OFF;
        LowPass1FilterProcess(&DXC.ExtAi[0]);
        LowPass1FilterProcess(&DXC.ExtAi[1]);
        LowPass1FilterProcess(&DXC.ExtAi[2]);
        LowPass1FilterProcess(&DXC.ExtAi[3]);

        _RUN_LED_ON;
        //DC_MotorControl();
        ScopeSampleDeal();
        _RUN_LED_OFF;

    }

//get motor temperature
    GetMotorTemperature(&DX[D1]);
    GetMotorTemperature(&DX[D2]);
//put the SPI3_DMA_TD_Buff2 to SPI3_DMA_TD_Buff
    for(i=0; i<16; i++)
    {
        SPI3_DMA_TD_Buff[i] = SPI3_DMA_TD_Buff2[i];
    }
    _SPI3_CS_RESET;
    DMA1_Stream5->NDTR = 16;     //must be rewrite where DMA_Mode_Normal
    DMA_Cmd(DMA1_Stream5, ENABLE);

    SPI_DMACmd(SPI3, SPI_I2S_DMAReq_Tx, ENABLE);
    _RUN_LED_ON;
}

//Driver 1 Short current interrupt
void TIM1_BRK_TIM9_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM1,TIM_IT_Break) != RESET)
    {
        TIM_ClearFlag(TIM1, TIM_FLAG_Break);
        TIM_ClearITPendingBit(TIM1, TIM_FLAG_Break);
        Driver1_Off();          //disable time1 output
        DX[D1].GbFlg.CtrlWordAct = 0x00;
        DX[D1].GbFlg.CtrlWord = 0x00;
        DX[D1].GbFlg.ErrState1 |= 0x0080;
    }
    else
    {
        ;
    }

}

//Driver 2 Short current interrupt
void TIM8_BRK_TIM12_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM8,TIM_IT_Break) != RESET)
    {
        TIM_ClearFlag(TIM8, TIM_FLAG_Break);
        TIM_ClearITPendingBit(TIM8, TIM_FLAG_Break);
        Driver2_Off();  //disable time8 output
        DX[D2].GbFlg.CtrlWordAct = 0x00;
        DX[D2].GbFlg.CtrlWord = 0x00;
        DX[D2].GbFlg.ErrState1 |= 0x0080;
    }
    else
    {
        ;
    }
}

#if 1
//using print to  design loop cotrol
void LoopControl(DriverStructTypedef* pDX)
{
    ResultStatus rlt_status;
    INT8U   en_pos_loop_pi = 0;
    INT8U   en_spd_loop_pi = 0;
    INT8U   en_cur_loop_pi = 0;
    INT32S  temp32s;
    INT32S  temp32s1;
    INT32U  temp32u1,temp32u2;
    INT64U  temp64u;
    INT64S  temp64s1;
    INT64S  temp64s2;
    INT64U  temp64u1,temp64u2;

    CalAnalogRealDec(&pDX->CurU);
    pDX->Fbk.Iu = pDX->CurU.AdcRealDec;
    CalAnalogRealDec(&pDX->CurV);
    pDX->Fbk.Iv = pDX->CurV.AdcRealDec;

    ClarkTransform(&pDX->Clark, &pDX->Fbk);
    //get the position
    //GetPositionIncrementABZ(pDX->Fbk, pDX->QEI_TIMx);
    GetPositionComu(&pDX->Fbk);
    //calculate speed via 2 orde low pass filter
    CalFilterSpeed(&pDX->Fbk);
    //calculate speed unit:RPM
    CalSpeedRPM(&pDX->Fbk, pDX->MotorPar.FB_Resolution);

    if(((pDX->GbFlg.CtrlWordAct) & 0x0f) == 0x0f)
    {
        if(pDX->GbFlg.Excitation == 0)   //excitation not complete
        {
            switch(pDX->MotorPar.ExcitationMode)
            {
            case 0:
                pDX->Fbk.EleAngDec = SINE_TABLE_NUM - (SINE_TABLE_NUM/4);
                pDX->CurLp.CmdIqAct = pDX->MotorPar.ExcitationCurr;
                pDX->GbFlg.ExciCnt ++;
                if(pDX->GbFlg.ExciCnt >= (pDX->MotorPar.ExcitationTime*16))
                {
                    if(pDX->CurLp.RealIqPk > (pDX->MotorPar.ExcitationCurr/2))
                    {
                        pDX->GbFlg.Excitation = 1;   //
                        pDX->Fbk.EncOffset = pDX->Fbk.PosAbs;
                        pDX->Fbk.EncOffset16 = pDX->Fbk.QEI_CntNow;
                    }
                    else
                    {
                        ExcitationFaultProcess(pDX);
                    }
                }
            break;

            case 1:
                if(pDX->MotorPar.FB_Type == ENC_COMU_CHECK_CODE)    //magnet encoder
                {
                    pDX->Fbk.EncOffset = 0;
                    pDX->Fbk.EncOffset16 = 0;
                }
                else
                {
                    if(pDX->Fbk.FirstZ == 0)    //Z signal don't come
                    {
                        pDX->Fbk.EncOffset = pDX->Fbk.HallOffet;
                    }
                }

                pDX->GbFlg.Excitation = 1;
            break;

            default:break;

            }
        }
        en_cur_loop_pi = 1;     //enable current loop pi calculate
    }

    //calculate eleltronic angle
    if(pDX->GbFlg.Excitation == 1 && pDX->GbFlg.CtrlModeAct != 8)    //excitation complete
        CalEletronicAngleDec(&pDX->Fbk, &pDX->MotorPar);

    //mode control
    if(((pDX->GbFlg.CtrlWordAct) & 0x0f) == 0x0f)
    {
        en_cur_loop_pi = 1;     //enable current loop pi calculate
        if(pDX->GbFlg.Excitation == 1)   //excitation complete
        {
            //time base for enable calculate the speed loop, position loop
            if((pDX->GbFlg.PosLpTimeBaseCnt & 0x0f) == 0)       //enable the position loop calculate
                en_pos_loop_pi = 1;
            pDX->GbFlg.PosLpTimeBaseCnt ++;

            if((pDX->GbFlg.SpdLpTimeBaseCnt & 0x03) == 0)       //enable the speed loop calculate
                en_spd_loop_pi = 1;
            pDX->GbFlg.SpdLpTimeBaseCnt ++;

            switch(pDX->GbFlg.CtrlModeAct)
            {
            case 1:
                if(en_pos_loop_pi == 1) //position loop pi calculate
                {
                    pDX->PosLp.RemainPcn = pDX->PosLp.CmdCmdPos - pDX->PosLp.ProfilePcn;    //剩余里程
#if 1
                    pDX->GbFlg.QkStopAct = pDX->GbFlg.QkStopCmd | pDX->GbFlg.QkStopDin;
                    if(pDX->GbFlg.QkStopAct)
                    {
                        pDX->PosLp.ProfileDeceAct = pDX->PosLp.QkStopDeceAct;
                        if(pDX->PosLp.QkStopTrig == 0)
                        {
                            pDX->PosLp.QkStopTrig = 1;
                            temp64u = (INT64U)pDX->PosLp.ProfileVcn * (INT64U)pDX->PosLp.ProfileVcn;
                            temp64u = temp64u >> 15;
                            if(temp64u > 0x00000000ffffffff)    //当速度大于663rpm时将溢出
                            {
                                temp32u1 = temp64u >> 8; //避免溢出32位，移动8bit可以达到10000rpm以上
                                temp32u2 = temp32u1 / pDX->PosLp.ProfileDeceAct;//使用32位除法能节省好多时间
                                temp32u1 = temp32u1 - temp32u2 * pDX->PosLp.ProfileDeceAct; //得到余数
                                temp32u1 = temp32u1 * 256;
                                temp32u1 = temp32u1 / pDX->PosLp.ProfileDeceAct;
                                pDX->PosLp.ProfilePAdece = temp32u2 * 256 + temp32u1;
                            }
                            else
                            {
                                temp32u1 = temp64u;
                                pDX->PosLp.ProfilePAdece = temp32u1 / pDX->PosLp.ProfileDeceAct;//使用32位除法能节省好多时间
                            }
                            if(pDX->PosLp.ProfilePcn < pDX->PosLp.CmdCmdPos)
                            {
                                pDX->PosLp.CmdPos = pDX->PosLp.ProfilePcn + pDX->PosLp.ProfilePAdece;
                                if(pDX->PosLp.CmdPos > pDX->PosLp.CmdCmdPos)
                                    pDX->PosLp.CmdPos = pDX->PosLp.CmdCmdPos;
                            }
                            else if(pDX->PosLp.ProfilePcn > pDX->PosLp.CmdCmdPos)
                            {
                                pDX->PosLp.CmdPos = pDX->PosLp.ProfilePcn - pDX->PosLp.ProfilePAdece;
                                if(pDX->PosLp.CmdPos < pDX->PosLp.CmdCmdPos)
                                    pDX->PosLp.CmdPos = pDX->PosLp.CmdCmdPos;
                            }
                            else
                            {
                                pDX->PosLp.CmdPos = pDX->PosLp.CmdCmdPos;
                            }
                        }
                    }
                    else
                    {
                        pDX->PosLp.ProfileAcceAct = pDX->PosLp.ProfileAcceCmd;
                        pDX->PosLp.ProfileDeceAct = pDX->PosLp.ProfileDeceCmd;
                        pDX->PosLp.CmdPos = pDX->PosLp.CmdCmdPos;
                        pDX->PosLp.QkStopTrig = 0;
                    }
#endif

                    temp64u1 = (INT64U)pDX->PosLp.ProfileVcn * (INT64U)pDX->PosLp.ProfileVcn;//强制转换也会保留与扩展最高位的

                    if(pDX->PosLp.CmdPos > pDX->PosLp.ProfilePcn_1)
                    {
                        if(pDX->PosLp.ProfileVcn_1 < 0)
                        {
                            pDX->PosLp.ProfileVcn = ((INT32S)pDX->PosLp.ProfileAcceAct) + pDX->PosLp.ProfileVcn_1;
                        }
                        else
                        {
                        temp32s = pDX->PosLp.CmdPos - pDX->PosLp.ProfilePcn_1;
                        temp64u2 = (INT64U)temp32s * (INT64U)pDX->PosLp.ProfileDeceAct;
                        temp64u2 = temp64u2 * 32768;
                        if(temp64u2 > temp64u1)
                        //if((temp32s) > pDX->PosLp.ProfilePAdece)    //decelerate not start
                        {
                            if(pDX->PosLp.ProfileVcn < (INT32S)pDX->PosLp.CmdProfileSpeed)
                            {
                                pDX->PosLp.ProfileVcn = (INT32S)pDX->PosLp.ProfileAcceAct + pDX->PosLp.ProfileVcn_1;
                                if(pDX->PosLp.ProfileVcn > (INT32S)pDX->PosLp.CmdProfileSpeed)  //reach/bigger the CmdProfileSpeed
                                    pDX->PosLp.ProfileVcn = (INT32S)pDX->PosLp.CmdProfileSpeed;
                            }
                            else    //防止在一个行程未结束时，降低目标梯形速度值
                            {
                                pDX->PosLp.ProfileVcn = -(INT32S)pDX->PosLp.ProfileDeceAct + pDX->PosLp.ProfileVcn_1;
                                if(pDX->PosLp.ProfileVcn < (INT32S)pDX->PosLp.CmdProfileSpeed)  //reach/bigger the CmdProfileSpeed
                                    pDX->PosLp.ProfileVcn = (INT32S)pDX->PosLp.CmdProfileSpeed;
                            }
                        }
                        else
                        {
                            pDX->PosLp.ProfileVcn = pDX->PosLp.ProfileVcn_1 - (INT32S)pDX->PosLp.ProfileDeceAct;
                            if(pDX->PosLp.ProfileVcn < 0)  //reach 0
                                pDX->PosLp.ProfileVcn = 0;
                        }
                        }
                        temp32s = pDX->PosLp.ProfileVcn + pDX->PosLp.ProfileVcn_1;
                        temp32s = temp32s + pDX->PosLp.ProfilePcnFlt;
                        pDX->PosLp.ProfilePcnFlt = temp32s % 32768;
                        temp32s = temp32s / 32768;

                        pDX->PosLp.ProfilePcn = temp32s + pDX->PosLp.ProfilePcn_1;
                        pDX->PosLp.ProfileVcn_1 = pDX->PosLp.ProfileVcn;

                        if(pDX->PosLp.ProfilePcn >= pDX->PosLp.CmdPos)
                        {
                            pDX->PosLp.ProfilePcn = pDX->PosLp.CmdPos;
                            pDX->PosLp.CmdDeltaPosOut = pDX->PosLp.ProfilePcn - pDX->PosLp.ProfilePcn_1;
                            pDX->PosLp.ProfilePcnFlt = 0;
                            pDX->PosLp.ProfileVcn = 0;
                            pDX->PosLp.ProfileVcn_1 = 0;
                        }
                        else
                        {
                            pDX->PosLp.CmdDeltaPosOut = temp32s;
                        }
                        pDX->PosLp.ProfilePcn_1 = pDX->PosLp.ProfilePcn;
                    }
                    else if(pDX->PosLp.CmdPos < pDX->PosLp.ProfilePcn_1)
                    {
                        if(pDX->PosLp.ProfileVcn_1 > 0)
                        {
                            pDX->PosLp.ProfileVcn = -((INT32S)pDX->PosLp.ProfileAcceAct) + pDX->PosLp.ProfileVcn_1;
                        }
                        else
                        {
                        temp32s = -pDX->PosLp.CmdPos + pDX->PosLp.ProfilePcn_1;
                        temp64u2 = (INT64U)temp32s * (INT64U)pDX->PosLp.ProfileDeceAct;
                        temp64u2 = temp64u2 * 32768;
                        if(temp64u2 > temp64u1)
                        //if((temp32s) > pDX->PosLp.ProfilePAdece)    //decelerate not start
                        {
                            if(pDX->PosLp.ProfileVcn > -(INT32S)pDX->PosLp.CmdProfileSpeed)
                            {
                                pDX->PosLp.ProfileVcn = -(INT32S)pDX->PosLp.ProfileAcceAct + pDX->PosLp.ProfileVcn_1;
                                if(pDX->PosLp.ProfileVcn < -(INT32S)pDX->PosLp.CmdProfileSpeed)  //reach/bigger the CmdProfileSpeed
                                    pDX->PosLp.ProfileVcn = -(INT32S)pDX->PosLp.CmdProfileSpeed;
                            }
                            else    //防止在一个行程未结束时，降低目标梯形速度值
                            {
                                pDX->PosLp.ProfileVcn = (INT32S)pDX->PosLp.ProfileDeceAct + pDX->PosLp.ProfileVcn_1;
                                if(pDX->PosLp.ProfileVcn > -(INT32S)pDX->PosLp.CmdProfileSpeed)  //reach/bigger the CmdProfileSpeed
                                    pDX->PosLp.ProfileVcn = -(INT32S)pDX->PosLp.CmdProfileSpeed;
                            }
                        }
                        else
                        {
                            pDX->PosLp.ProfileVcn = pDX->PosLp.ProfileVcn_1 + (INT32S)pDX->PosLp.ProfileDeceAct;
                            if(pDX->PosLp.ProfileVcn > 0)  //reach 0
                                pDX->PosLp.ProfileVcn = 0;
                        }
                        }
                        temp32s = pDX->PosLp.ProfileVcn + pDX->PosLp.ProfileVcn_1;
                        temp32s = temp32s + pDX->PosLp.ProfilePcnFlt;
                        pDX->PosLp.ProfilePcnFlt = temp32s % 32768;
                        temp32s = temp32s / 32768;

                        pDX->PosLp.ProfilePcn = temp32s + pDX->PosLp.ProfilePcn_1;
                        pDX->PosLp.ProfileVcn_1 = pDX->PosLp.ProfileVcn;

                        if(pDX->PosLp.ProfilePcn <= pDX->PosLp.CmdPos)
                        {
                            pDX->PosLp.ProfilePcn = pDX->PosLp.CmdPos;
                            pDX->PosLp.CmdDeltaPosOut = pDX->PosLp.ProfilePcn - pDX->PosLp.ProfilePcn_1;
                            pDX->PosLp.ProfilePcnFlt = 0;
                            pDX->PosLp.ProfileVcn = 0;
                            pDX->PosLp.ProfileVcn_1 = pDX->PosLp.ProfileVcn;
                        }
                        else
                        {
                            pDX->PosLp.CmdDeltaPosOut = temp32s;
                        }
                        pDX->PosLp.ProfilePcn_1 = pDX->PosLp.ProfilePcn;
                    }
                    else
                    {
                        pDX->PosLp.ProfilePcnFlt = 0;
                        pDX->PosLp.ProfileVcn = 0;
                        pDX->PosLp.ProfileVcn_1 = 0;
                        pDX->PosLp.CmdDeltaPosOut = 0;
                    }

                    pDX->PosLp.CmdPosAct += pDX->PosLp.CmdDeltaPosOut;
                    pDX->PosLp.CmdPosAct += pDX->PosLp.CmdDeltaPosExt;
                    PositionLoopP(&pDX->PosLp, &pDX->Fbk, pDX->SpdLp.CmdSpeedMaxDec);

                    pDX->SpdLp.CmdSpeedDecAct = pDX->PosLp.OutCmdSpeed + pDX->PosLp.ProfileVcn + pDX->SpdLp.CmdSpeedDecExt;
                }
                if(en_spd_loop_pi == 1) //speed loop pi calculate
                {
                    LimitedMaxSpeed(&pDX->SpdLp);
                    SpeedLoopPI(&pDX->SpdLp, &pDX->Fbk, pDX->CurLp.CmdIqMax);
                    pDX->CurLp.CmdIqAct = pDX->SpdLp.OutCmdIq;
                }
            break;

            case 3:
                if(en_pos_loop_pi == 1)       //updata command speed
                {
                    QuiekStop(pDX);
                    pDX->SpdLp.DeltaCmdSpeed = pDX->SpdLp.CmdSpeedDec - pDX->Fbk.SpeedFilterDec;  //because kinco OD
                    if(pDX->SpdLp.DeltaCmdSpeed >= 0)        //正速度指令是加速，负速度指令时减速
                    {
                        if(pDX->SpdLp.CmdSpeedDecCal >= pDX->SpdLp.CmdSpeedDec)   //because kinco OD
                        {
                            pDX->SpdLp.CmdSpeedDecCal = pDX->SpdLp.CmdSpeedDec;
                        }
                        else
                        {
                            if(pDX->Fbk.SpeedFilterDec >= 0)
                                pDX->SpdLp.CmdSpeedDecCal += pDX->PosLp.ProfileAcceAct;
                            else
                                pDX->SpdLp.CmdSpeedDecCal += pDX->PosLp.ProfileDeceAct;
                        }
                    }
                    else                                                    //正速度指令是减速，负速度指令时加速
                    {
                        if(pDX->SpdLp.CmdSpeedDecCal <= pDX->SpdLp.CmdSpeedDec)   //because kinco OD
                        {
                            pDX->SpdLp.CmdSpeedDecCal = pDX->SpdLp.CmdSpeedDec;
                        }
                        else
                        {
                            if(pDX->Fbk.SpeedFilterDec >= 0)
                                pDX->SpdLp.CmdSpeedDecCal -= pDX->PosLp.ProfileDeceAct;
                            else
                                pDX->SpdLp.CmdSpeedDecCal -= pDX->PosLp.ProfileAcceAct;
                        }
                    }

                    pDX->SpdLp.CmdSpeedDecAct = pDX->PosLp.OutCmdSpeed + pDX->SpdLp.CmdSpeedDecCal + pDX->SpdLp.CmdSpeedDecExt;


                    temp32s = pDX->SpdLp.CmdSpeedDecCal + pDX->PosLp.CmdPosActRemain;
                    pDX->PosLp.CmdPosActRemain = temp32s%16380;
                    pDX->PosLp.CmdPosAct += (temp32s/16380);
                    pDX->PosLp.CmdPosAct += pDX->PosLp.CmdDeltaPosExt;
                    PositionLoopP(&pDX->PosLp, &pDX->Fbk, pDX->SpdLp.CmdSpeedMaxDec);
                }
                if(en_spd_loop_pi == 1)     //speed loop pi calculate
                {
                    LimitedMaxSpeed(&pDX->SpdLp);
                    SpeedLoopPI(&pDX->SpdLp, &pDX->Fbk, pDX->CurLp.CmdIqMax);
                    pDX->CurLp.CmdIqAct = pDX->SpdLp.OutCmdIq;
                }
            break;

            case -3:
                if(en_spd_loop_pi == 1)
                {
                    //pDX->SpdLp.CmdSpeedDec = pDX->SpdLp.CmdCmdSpeedDec;
                    QuiekStop(pDX);
                    pDX->SpdLp.CmdSpeedDecAct = pDX->SpdLp.CmdSpeedDec;
                    LimitedMaxSpeed(&pDX->SpdLp);
                    SpeedLoopPI(&pDX->SpdLp, &pDX->Fbk, pDX->CurLp.CmdIqMax);
                    pDX->CurLp.CmdIqAct = pDX->SpdLp.OutCmdIq;
                }
            break;

            case 4:
                //limited the speed
                if(en_spd_loop_pi == 1)
                {
                    if(pDX->CurLp.CmdIq >= 0)
                        pDX->SpdLp.CmdSpeedDecAct = pDX->SpdLp.CmdSpeedMaxDec;
                    else
                        pDX->SpdLp.CmdSpeedDecAct = -pDX->SpdLp.CmdSpeedMaxDec;
                    LimitedMaxSpeed(&pDX->SpdLp);
                    SpeedLoopPI(&pDX->SpdLp, &pDX->Fbk, pDX->CurLp.CmdIqMax);
                    if(pDX->CurLp.CmdIq >= 0)
                    {
                        if(pDX->SpdLp.OutCmdIq < (INT32S)pDX->CurLp.CmdIq)
                            if(pDX->SpdLp.OutCmdIq < 0)
                                pDX->CurLp.CmdIqAct = 0;
                            else
                                pDX->CurLp.CmdIqAct = pDX->SpdLp.OutCmdIq;
                        else
                            pDX->CurLp.CmdIqAct = pDX->CurLp.CmdIq;
                    }
                    else
                    {
                        if(pDX->SpdLp.OutCmdIq > (INT32S)pDX->CurLp.CmdIq)
                            if(pDX->SpdLp.OutCmdIq > 0)
                                pDX->CurLp.CmdIqAct = 0;
                            else
                                pDX->CurLp.CmdIqAct = pDX->SpdLp.OutCmdIq;
                        else
                            pDX->CurLp.CmdIqAct = pDX->CurLp.CmdIq;
                    }
                }

            break;

            case 40:
                pDX->CurLp.CmdIqAct = pDX->CurLp.CmdIq;
            break;

            case 0:
                pDX->CurLp.CmdIqAct = 0;
            break;

            case 8:
                pDX->CurLp.CmdIqAct = pDX->MotorPar.ExcitationCurr;
                pDX->SpdLp.CmdStepSpeedDecAct = pDX->SpdLp.CmdStepSpeedDecInt;
                pDX->Fbk.EleAngDec = (pDX->Fbk.EleAngDec + pDX->SpdLp.CmdStepSpeedDecInt) & 0x07ff;
            break;

            case 10:    //set magnet encoder zero
                pDX->CurLp.CmdIqAct = pDX->MotorPar.ExcitationCurr;
                pDX->Fbk.EleAngDec = pDX->Fbk.SetEleAngDec;
                if(pDX->Fbk.SetMagEncZeroTrig == 1)
                {
                    pDX->Fbk.SetMagEncZeroTrig = 2;
                    SPI3_DMA_TD_Buff2[0] = 12;
                }
                else if(pDX->Fbk.SetMagEncZeroTrig)
                {
                    (pDX->pF_DriverOff)();         //disable time pwm output
                    pDX->GbFlg.CtrlWord = 6;
                    pDX->GbFlg.CtrlWordAct = 6;
                    pDX->Fbk.SetMagEncZeroTrig = 0;
                    SPI3_DMA_TD_Buff2[0] = 0;
                }
                else
                {
                    SPI3_DMA_TD_Buff2[0] = 0;
                }
            break;
            default:
            break;
            }
        }
        else        //clear the time base counter
        {
            pDX->GbFlg.PosLpTimeBaseCnt = 0;
            pDX->GbFlg.SpdLpTimeBaseCnt = 0;
        }

    }

    //park transform
    //ParkTansform(&pDX->Park, &pDX->Clark, pDX->Fbk.EleAngDec);
    ParkTansform(&pDX->Park, &pDX->Clark, &pDX->Fbk);
    RealIqPeakHold(pDX);
    //calculate the current loop pi
    if(en_cur_loop_pi == 1)
    {
        CurrentLoopPI(&pDX->CurLp, &pDX->Park);


    ParkInvtTansform(&pDX->ParkInvt,&pDX->Park,&pDX->CurLp);
    rlt_status = Gen7SegSVPWM(&pDX->SVPWM_Info, &pDX->ParkInvt);
    }
    PWM_ValueUpdata(pDX->PWM_TIMx, &pDX->SVPWM_Info);
}
#endif


//the motion
MotionStructTypedef  MOX =
{
    .CarMchPar.Wheelbase = 591,
    .CarMchPar.WheelDia = 150,
    .CarMchPar.GearF = 39,
    .CarMchPar.GearD = 14,
    .MotionPar.Radius = 0xffffffff,
};

#if 1
void MotionCtrol(void)
{
    INT32S  temp32s;
    INT32S  temp32u;
    INT64U  temp64u;
    INT64U  temp64u1,temp64u2;

    if(MOX.GbFlg.MotionEnable == 1 && MOX.GbFlg.SwtichOnLock == 0)    //check
    {
        if(MOX.GbFlg.MotionCtrWordAct == 0x0f)
        {
            if(MOX.GbFlg.ParUpdataLock == 0)
            {
            switch(MOX.GbFlg.MotionModeAct)
            {
                case 33:
                    DX[0].SpdLp.CmdCmdSpeedDec = -MOX.MotionPar.CmdSpeedDec;
                    DX[1].SpdLp.CmdCmdSpeedDec = MOX.MotionPar.CmdSpeedDec;
                    DX[0].PosLp.ProfileAcceCmd = MOX.MotionPar.ProfileAcceCmd;
                    DX[1].PosLp.ProfileAcceCmd = MOX.MotionPar.ProfileAcceCmd;
                    DX[0].PosLp.ProfileDeceCmd = MOX.MotionPar.ProfileDeceCmd;
                    DX[1].PosLp.ProfileDeceCmd = MOX.MotionPar.ProfileDeceCmd;
                    DX[0].PosLp.QkStopDeceAct = MOX.MotionPar.CmdQkStopDece;
                    DX[1].PosLp.QkStopDeceAct = MOX.MotionPar.CmdQkStopDece;
                    MOX.TurnAngle.Enable = 1;
                break;
                case 11:
                    DX[0].PosLp.ProfileAcceCmd = MOX.MotionPar.ProfileAcceCmd;
                    DX[1].PosLp.ProfileAcceCmd = MOX.MotionPar.ProfileAcceCmd;
                    DX[0].PosLp.ProfileDeceCmd = MOX.MotionPar.ProfileDeceCmd;
                    DX[1].PosLp.ProfileDeceCmd = MOX.MotionPar.ProfileDeceCmd;
                    DX[0].PosLp.CmdProfileSpeed = MOX.MotionPar.CmdProfileSpeed;
                    DX[1].PosLp.CmdProfileSpeed = MOX.MotionPar.CmdProfileSpeed;
                    DX[0].PosLp.QkStopDeceAct = MOX.MotionPar.CmdQkStopDece;
                    DX[1].PosLp.QkStopDeceAct = MOX.MotionPar.CmdQkStopDece;
                    MOX.TurnAngle.Enable = 1;
                break;
                case 66:
                    if(MOX.GbFlg.RadiusUpdataTrig == 1)
                    {
                        if(MOX.GbFlg.RadiusUpdataMode == 1) //运行过程中半径更改了，先将双轴速度以之前的减速度减为0再按照新的加速度运行新的半径弧线
                        {
                            DX[0].SpdLp.CmdCmdSpeedDec = 0;
                            DX[1].SpdLp.CmdCmdSpeedDec = 0;
                            if(DX[0].SpdLp.CmdSpeedDecCal == 0 && DX[1].SpdLp.CmdSpeedDecCal == 0)
                            {
                                MOX.GbFlg.RadiusUpdataTrig = 0;
                            }
                        }
                        else if(MOX.GbFlg.RadiusUpdataMode == 0 || MOX.GbFlg.RadiusUpdataMode == 0xff) //运行过程中半径更改了，去本次和上次加减速最大值进行运转
                        {
                            DX[0].SpdLp.CmdCmdSpeedDec = MOX.MotionPar.CmdSpeedDec6622_1;
                            DX[1].SpdLp.CmdCmdSpeedDec = MOX.MotionPar.CmdSpeedDec6622_2;
                            if(DX[0].PosLp.ProfileAcceCmd < MOX.MotionPar.ProfileAcceCmd6622_1)
                                DX[0].PosLp.ProfileAcceCmd = MOX.MotionPar.ProfileAcceCmd6622_1;
                            if(DX[1].PosLp.ProfileAcceCmd < MOX.MotionPar.ProfileAcceCmd6622_2)
                                DX[1].PosLp.ProfileAcceCmd = MOX.MotionPar.ProfileAcceCmd6622_2;
                            if(DX[0].PosLp.ProfileDeceCmd < MOX.MotionPar.ProfileDeceCmd6622_1)
                                DX[0].PosLp.ProfileDeceCmd = MOX.MotionPar.ProfileDeceCmd6622_1;
                            if(DX[1].PosLp.ProfileDeceCmd < MOX.MotionPar.ProfileDeceCmd6622_2)
                                DX[1].PosLp.ProfileDeceCmd = MOX.MotionPar.ProfileDeceCmd6622_2;

                            if(DX[0].SpdLp.CmdSpeedDecCal == DX[0].SpdLp.CmdCmdSpeedDec && DX[1].SpdLp.CmdSpeedDecCal == DX[1].SpdLp.CmdCmdSpeedDec)
                            {
                                MOX.GbFlg.RadiusUpdataTrig = 0;
                            }
                        }

                    }
                    else
                    {
                        DX[0].SpdLp.CmdCmdSpeedDec = MOX.MotionPar.CmdSpeedDec6622_1;
                        DX[1].SpdLp.CmdCmdSpeedDec = MOX.MotionPar.CmdSpeedDec6622_2;
                        DX[0].PosLp.ProfileAcceCmd = MOX.MotionPar.ProfileAcceCmd6622_1;
                        DX[1].PosLp.ProfileAcceCmd = MOX.MotionPar.ProfileAcceCmd6622_2;
                        DX[0].PosLp.ProfileDeceCmd = MOX.MotionPar.ProfileDeceCmd6622_1;
                        DX[1].PosLp.ProfileDeceCmd = MOX.MotionPar.ProfileDeceCmd6622_2;
                    }
                    DX[0].PosLp.QkStopDeceAct = MOX.MotionPar.CmdQkStopDece6622_1;
                    DX[1].PosLp.QkStopDeceAct = MOX.MotionPar.CmdQkStopDece6622_2;
                    MOX.TurnAngle.Enable = 0;
                break;
                default : break;
            }
            }
        //turn angle
        if(MOX.TurnAngle.Enable == 1)
        {
        if((DX[0].GbFlg.PosLpTimeBaseCnt & 0x0f) == 0)  //必须使用这个条件，否则第一个轴会出现问题
        {
            MOX.TurnAngle.RemainPcn = MOX.TurnAngle.CmdCmdPos - MOX.TurnAngle.ProfilePcn;    //剩余里程
        if(DX[0].GbFlg.QkStopAct || DX[1].GbFlg.QkStopAct)
        {
            MOX.TurnAngle.ProfileDeceAct = DX[0].PosLp.QkStopDeceAct;
            if(MOX.TurnAngle.QkStopTrig == 0)
            {
                MOX.TurnAngle.QkStopTrig = 1;
#if 1
                temp64u = (INT64U)MOX.TurnAngle.ProfileVcn * (INT64U)MOX.TurnAngle.ProfileVcn;
                temp64u = temp64u >> 15;
                if(temp64u > 0x00000000ffffffff)    //当速度大于663rpm时将溢出
                {
                    temp32u = temp64u >> 8; //避免溢出32位，移动8bit可以达到10000rpm以上
                    temp32u = temp32u / MOX.TurnAngle.ProfileDeceAct;//使用32位除法能节省好多时间
                    MOX.TurnAngle.ProfilePAdece = temp32u * 256;
                }
                else
                {
                    temp32u = temp64u;
                    MOX.TurnAngle.ProfilePAdece = temp32u / MOX.TurnAngle.ProfileDeceAct;//使用32位除法能节省好多时间
                }
                if(MOX.TurnAngle.CmdPosAct < MOX.TurnAngle.CmdCmdPos)
                {
                    MOX.TurnAngle.CmdPos = MOX.TurnAngle.CmdPosAct + MOX.TurnAngle.ProfilePAdece;
                    if(MOX.TurnAngle.CmdPos > MOX.TurnAngle.CmdCmdPos)
                        MOX.TurnAngle.CmdPos = MOX.TurnAngle.CmdCmdPos;
                }
                else if(MOX.TurnAngle.CmdPosAct > MOX.TurnAngle.CmdCmdPos)
                {
                    MOX.TurnAngle.CmdPos = MOX.TurnAngle.CmdPosAct - MOX.TurnAngle.ProfilePAdece;
                    if(MOX.TurnAngle.CmdPos < MOX.TurnAngle.CmdCmdPos)
                        MOX.TurnAngle.CmdPos = MOX.TurnAngle.CmdCmdPos;
                }
                else
                {
                    MOX.TurnAngle.CmdPos = MOX.TurnAngle.CmdPosAct;
                }
#endif
            }
        }
        else
        {
            MOX.TurnAngle.ProfileAcceAct = MOX.TurnAngle.ProfileAcceCmd;
            MOX.TurnAngle.ProfileDeceAct = MOX.TurnAngle.ProfileDeceCmd;
            MOX.TurnAngle.CmdPos = MOX.TurnAngle.CmdCmdPos;
            MOX.TurnAngle.QkStopTrig = 0;
        }

        temp64u1 = (INT64U)MOX.TurnAngle.ProfileVcn * (INT64U)MOX.TurnAngle.ProfileVcn;//强制转换也会保留与扩展最高位的

        if(MOX.TurnAngle.CmdPos > MOX.TurnAngle.ProfilePcn_1)
        {
            if(MOX.TurnAngle.ProfileVcn_1 < 0)
            {
                //以最大的加减速度反转回来
                if(MOX.TurnAngle.ProfileAcceAct > MOX.TurnAngle.ProfileDeceAct)
                    MOX.TurnAngle.ProfileVcn = (INT32S)MOX.TurnAngle.ProfileAcceAct + MOX.TurnAngle.ProfileVcn_1;
                else
                    MOX.TurnAngle.ProfileVcn = (INT32S)MOX.TurnAngle.ProfileDeceAct + MOX.TurnAngle.ProfileVcn_1;
            }
            else
            {
            temp32s = MOX.TurnAngle.CmdPos - MOX.TurnAngle.ProfilePcn_1;
            temp64u2 = (INT64U)temp32s * (INT64U)MOX.TurnAngle.ProfileDeceAct;
            temp64u2 = temp64u2 * 32768;
            //if((temp32s) > MOX.TurnAngle.ProfilePAdece)    //decelerate not start
            if(temp64u2 > temp64u1)    //decelerate not start
            {
                if(MOX.TurnAngle.ProfileVcn < (INT32S)MOX.TurnAngle.CmdProfileSpeed)
                {
                    MOX.TurnAngle.ProfileVcn = (INT32S)MOX.TurnAngle.ProfileAcceAct + MOX.TurnAngle.ProfileVcn_1;
                    if(MOX.TurnAngle.ProfileVcn > (INT32S)MOX.TurnAngle.CmdProfileSpeed)  //reach/bigger the CmdProfileSpeed
                        MOX.TurnAngle.ProfileVcn = (INT32S)MOX.TurnAngle.CmdProfileSpeed;
                }
                else    //防止在一个行程未结束时，降低目标梯形速度值
                {
                    MOX.TurnAngle.ProfileVcn = -(INT32S)MOX.TurnAngle.ProfileDeceAct + MOX.TurnAngle.ProfileVcn_1;
                    if(MOX.TurnAngle.ProfileVcn < (INT32S)MOX.TurnAngle.CmdProfileSpeed)  //reach/bigger the CmdProfileSpeed
                        MOX.TurnAngle.ProfileVcn = (INT32S)MOX.TurnAngle.CmdProfileSpeed;
                }
            }
            else
            {
                MOX.TurnAngle.ProfileVcn = MOX.TurnAngle.ProfileVcn_1 - (INT32S)MOX.TurnAngle.ProfileDeceAct;
                if(MOX.TurnAngle.ProfileVcn < 0)  //reach 0
                    MOX.TurnAngle.ProfileVcn = 0;
            }
            }
            temp32s = MOX.TurnAngle.ProfileVcn + MOX.TurnAngle.ProfileVcn_1;
            temp32s = temp32s + MOX.TurnAngle.ProfilePcnFlt;
            _RUN_LED_OFF;
            MOX.TurnAngle.ProfilePcnFlt = temp32s % 32768;
            temp32s = temp32s / 32768;
            _RUN_LED_ON;
            MOX.TurnAngle.ProfilePcn = temp32s + MOX.TurnAngle.ProfilePcn_1;
            MOX.TurnAngle.ProfileVcn_1 = MOX.TurnAngle.ProfileVcn;

            if(MOX.TurnAngle.ProfilePcn >= MOX.TurnAngle.CmdPos)    //超过了
            {
                MOX.TurnAngle.ProfilePcn = MOX.TurnAngle.CmdPos;
                MOX.TurnAngle.CmdDeltaPosOut = MOX.TurnAngle.ProfilePcn - MOX.TurnAngle.ProfilePcn_1;
                MOX.TurnAngle.ProfilePcnFlt = 0;
                MOX.TurnAngle.ProfileVcn_1 = 0;
                MOX.TurnAngle.ProfileVcn = 0;
            }
            else
            {
                MOX.TurnAngle.CmdDeltaPosOut = temp32s; //增量输出，叠加到1,3模式的目标位置中
            }
            MOX.TurnAngle.CmdPosAct = MOX.TurnAngle.ProfilePcn;
            MOX.TurnAngle.ProfilePcn_1 = MOX.TurnAngle.ProfilePcn;
        }
        else if(MOX.TurnAngle.CmdPos < MOX.TurnAngle.ProfilePcn_1)
        {
            if(MOX.TurnAngle.ProfileVcn_1 > 0)
            {
                //以最大的加减速度反转回来
                if(MOX.TurnAngle.ProfileAcceAct > MOX.TurnAngle.ProfileDeceAct)
                    MOX.TurnAngle.ProfileVcn = -(INT32S)MOX.TurnAngle.ProfileAcceAct + MOX.TurnAngle.ProfileVcn_1;
                else
                    MOX.TurnAngle.ProfileVcn = -(INT32S)MOX.TurnAngle.ProfileDeceAct + MOX.TurnAngle.ProfileVcn_1;
            }
            else
            {
            temp32s = -MOX.TurnAngle.CmdPos + MOX.TurnAngle.ProfilePcn_1;
            temp64u2 = (INT64U)temp32s * (INT64U)MOX.TurnAngle.ProfileDeceAct;
            temp64u2 = temp64u2 * 32768;
            //if((temp32s) > MOX.TurnAngle.ProfilePAdece)    //decelerate not start
            if(temp64u2 > temp64u1)    //decelerate not start
            {
                if(MOX.TurnAngle.ProfileVcn > -(INT32S)MOX.TurnAngle.CmdProfileSpeed)
                {
                    MOX.TurnAngle.ProfileVcn = -(INT32S)MOX.TurnAngle.ProfileAcceAct + MOX.TurnAngle.ProfileVcn_1;
                    if(MOX.TurnAngle.ProfileVcn < -(INT32S)MOX.TurnAngle.CmdProfileSpeed)  //reach/bigger the CmdProfileSpeed
                        MOX.TurnAngle.ProfileVcn = -(INT32S)MOX.TurnAngle.CmdProfileSpeed;
                }
                else    //防止在一个行程未结束时，降低目标梯形速度值
                {
                    MOX.TurnAngle.ProfileVcn = (INT32S)MOX.TurnAngle.ProfileDeceAct + MOX.TurnAngle.ProfileVcn_1;
                    if(MOX.TurnAngle.ProfileVcn > -(INT32S)MOX.TurnAngle.CmdProfileSpeed)  //reach/bigger the CmdProfileSpeed
                        MOX.TurnAngle.ProfileVcn = -(INT32S)MOX.TurnAngle.CmdProfileSpeed;
                }
            }
            else
            {
                MOX.TurnAngle.ProfileVcn = MOX.TurnAngle.ProfileVcn_1 + (INT32S)MOX.TurnAngle.ProfileDeceAct;
                if(MOX.TurnAngle.ProfileVcn > 0)  //reach 0
                    MOX.TurnAngle.ProfileVcn = 0;
            }
            }
            temp32s = MOX.TurnAngle.ProfileVcn + MOX.TurnAngle.ProfileVcn_1;
            temp32s = temp32s + MOX.TurnAngle.ProfilePcnFlt;
            _RUN_LED_OFF;
            MOX.TurnAngle.ProfilePcnFlt = temp32s % 32768;
            temp32s = temp32s / 32768;
            _RUN_LED_ON;

            MOX.TurnAngle.ProfilePcn = temp32s + MOX.TurnAngle.ProfilePcn_1;
            MOX.TurnAngle.ProfileVcn_1 = MOX.TurnAngle.ProfileVcn;

            if(MOX.TurnAngle.ProfilePcn <= MOX.TurnAngle.CmdPos)    //超过了
            {
                MOX.TurnAngle.ProfilePcn = MOX.TurnAngle.CmdPos;
                MOX.TurnAngle.CmdDeltaPosOut = MOX.TurnAngle.ProfilePcn - MOX.TurnAngle.ProfilePcn_1;
                MOX.TurnAngle.ProfilePcnFlt = 0;
                MOX.TurnAngle.ProfileVcn_1 = 0;
                MOX.TurnAngle.ProfileVcn = 0;
            }
            else
            {
                MOX.TurnAngle.CmdDeltaPosOut = temp32s; //增量输出，叠加到1,3模式的目标位置中
            }
            MOX.TurnAngle.CmdPosAct = MOX.TurnAngle.ProfilePcn;
            MOX.TurnAngle.ProfilePcn_1 = MOX.TurnAngle.ProfilePcn;
        }
        else
        {
            MOX.TurnAngle.ProfilePcnFlt = 0;
            MOX.TurnAngle.ProfileVcn = 0;
            MOX.TurnAngle.ProfileVcn_1 = 0;
            MOX.TurnAngle.CmdDeltaPosOut = 0;
        }

        DX[0].SpdLp.CmdSpeedDecExt = MOX.TurnAngle.ProfileVcn;
        DX[1].SpdLp.CmdSpeedDecExt = MOX.TurnAngle.ProfileVcn;
        DX[0].PosLp.CmdDeltaPosExt = MOX.TurnAngle.CmdDeltaPosOut;
        DX[1].PosLp.CmdDeltaPosExt = MOX.TurnAngle.CmdDeltaPosOut;
        }
        }
        }
    }
}
#endif

//DC motor control
void DC_MotorControl(void)
{
    if(DXC.DM_Ctrl.EnableFlag == 1)
    {
        if(DXC.DM_Ctrl.OnTrig == 1)
        {
            DXC.DM_Ctrl.OnTrig = 0;
            DC_MotorOn();
        }
        else if(DXC.DM_Ctrl.OnTrig == 2)
        {
            DXC.DM_Ctrl.OnTrig = 0;
            DC_MotorOff();
        }
        if(DXC.DM_Ctrl.CtrlWord & 0x0f)
        {
            TIM_SetCounter(TIM9, DM_PWM_PERIOD/2);  //为了同步
        }
        DXC.CurrDm.AdcOrg = ADC1_2_DMA_DATA2.Split.DmCurrV_Org;
        CalAnalogRealDec(&DXC.CurrDm);
        //calculate dc motor iit
        CalIItDec(&DXC.DM_IIt, DXC.CurrDm.AdcRealDec);
    }

}

