#include "includes.h"
//开根号函数
INT16U isqrt32(INT32U x)
{
    INT32U m, y, b;
    m = 0x40000000;
    y = 0;
    while (m != 0)
    {
        b = y | m;
        y = y >> 1;
        if (x >= b)
        {
            x = x - b;
            y = y | m;
        }
        m >>= 2;
    }
    return y;
}
//计算时间天,时,分,秒
INT8U rOnlineTimeInfo(INT16U command, void *pOD, void *bpOD, void *rpOD)
{
    INT16U *pInt16u;
    INT8U  *pInt8u;
    rpOD = rpOD;
    switch(command)
    {
    case 0:
        pInt16u = pOD;
        *pInt16u = SysState.OnlineTime.Total.Total/86400;
        break;
    case 1:
        pInt8u = pOD;
        *pInt8u = (SysState.OnlineTime.Total.Total/3600)%24;
        break;
    case 2:
        pInt8u = pOD;
        *pInt8u = (SysState.OnlineTime.Total.Total/60)%60;
        break;
    case 3:
        pInt8u = pOD;
        *pInt8u = SysState.OnlineTime.Total.Total%60;
        break;
    }
    return (0);
}
//
//写默认值
INT8U wDefaultValueLoad(INT16U command, void *pOD, void *bpOD, void *rpOD)
{
    INT16U *pInt16u;
    INT16U i,flag_temp;
    UNION_OD_DATA_TYPE_DEF    OD_Type;
    pInt16u = pOD;
    bpOD = bpOD;
    rpOD = rpOD;
    if(((*pInt16u)&0xFF00) == 0xDF00) //检查标志位
    {
        flag_temp = (*pInt16u)&0x000F;
        flag_temp = flag_temp << 8;
        for(i=0; i<LocalProperty.OD_Sum; i++)
        {
            if(((OD_ADD[i].Property)&0X0F00) == flag_temp)    //搜索到需要load的对象,则进行加载(此处要注意呀,判断)
            {
                switch(OD_ADD[i].DataType)
                {
                case ODT16U:
                    OD_Type.pInt16u = OD_ADD[i].pOD;      //指向uInt数据类型
                    *OD_Type.pInt16u = OD_ADD[i].DefVal;
                    break;
                case ODT16S:
                    OD_Type.pInt16s = OD_ADD[i].pOD;      //指向uInt数据类型
                    *OD_Type.pInt16s = OD_ADD[i].DefVal;
                    break;
                case ODT08U:
                    OD_Type.pInt8u = OD_ADD[i].pOD;     //指向uChar数据类型
                    *OD_Type.pInt8u = OD_ADD[i].DefVal;
                    break;
                case ODT08S:
                    OD_Type.pInt8s = OD_ADD[i].pOD;     //指向uChar数据类型
                    *OD_Type.pInt8s = OD_ADD[i].DefVal;
                    break;
                case ODT32U:
                    OD_Type.pInt32s = OD_ADD[i].pOD;     //指向uLong数据类型
                    *OD_Type.pInt32s = OD_ADD[i].DefVal;
                    break;
                case ODT32S:
                    OD_Type.pInt32s = OD_ADD[i].pOD;      //指向Long数据类型
                    *OD_Type.pInt32s = OD_ADD[i].DefVal;
                    break;
                default :
                    break;
                }
            }
        }
        *pInt16u = 0;
    }
    return(0);
}

INT8U wCalActAcceAndDecelerate(INT16U command, INT16U *pOD, INT32U *bpOD, INT32U *rpOD)
{
    INT64U  temp64u;
    temp64u = (65536L*(INT64U)(*pOD)*(INT64U)(*rpOD))/4000000L;
    *bpOD = (INT16U)temp64u;
    return (0);
}

INT8U wCalActAcceletate(INT16U command, INT16U *pOD, void *bpOD, DriverStructTypedef *rpOD)
{
    INT64U  temp64u;
    temp64u = (65536L*(INT64U)(*pOD)*(INT64U)(rpOD->MotorPar.FB_Resolution))/4000000L;
    rpOD->PosLp.ProfileAcceCmd = (INT16U)temp64u;
    return (0);
}

INT8U wCalActDecelerate(INT16U command, INT16U *pOD, void *bpOD, DriverStructTypedef *rpOD)
{
    INT64U  temp64u;
    temp64u = (65536L*(INT64U)(*pOD)*(INT64U)(rpOD->MotorPar.FB_Resolution))/4000000L;
    rpOD->PosLp.ProfileDeceCmd = (INT16U)temp64u;
    return (0);
}

INT8U   wControlWord(INT16U command, INT16U *pOD, void *bpOD, DriverStructTypedef *rpOD)
{
    INT32U  temp32u = 72000;

    switch(*pOD)
    {
        case 0x0f:
            if(rpOD->GbFlg.CtrlWordAct != 0x0f)
            {
            if(rpOD->GbFlg.ErrAct1 & 0x4BFE)    //有些错误是不需要停车的
            {
                *pOD = 0x06;
            }
            else
            {
                if(rpOD->GbFlg.CtrlMode == 3 || rpOD->GbFlg.CtrlMode == 1)  //如果是3模式，那起始速度为当前实际速度，不管是什么速度
                {
                    rpOD->SpdLp.CmdSpeedDecCal = rpOD->Fbk.SpeedFilterDec;
                    rpOD->PosLp.OutCmdSpeed = 0;
                    rpOD->PosLp.CmdPosActRemain = 0;
                    rpOD->PosLp.CmdPosAct = rpOD->Fbk.PosAbs;

                    rpOD->PosLp.DownFlag = 0;
                    rpOD->PosLp.UpFlag = 0;
                    rpOD->PosLp.ProfilePcn = rpOD->Fbk.PosAbs;    //like offset
                    rpOD->PosLp.ProfilePcn_1 = rpOD->Fbk.PosAbs;    //like offset
                    rpOD->PosLp.ProfileVcn = rpOD->Fbk.SpeedFilterDec;
                    rpOD->PosLp.ProfileVcn_1 = rpOD->PosLp.ProfileVcn;
                    rpOD->PosLp.CmdDeltaPosExt = 0;
                    rpOD->SpdLp.CmdSpeedDecExt = 0;
                    rpOD->PosLp.QkStopTrig = 0;
                }
                ClrPosLoopPI(&rpOD->PosLp);
                ClrSpeedLoopPI(&rpOD->SpdLp);
                ClrCurrLoopPI(&rpOD->CurLp);
                (rpOD->pF_DriverOn)();
                rpOD->GbFlg.CtrlModeAct = rpOD->GbFlg.CtrlMode;
                rpOD->GbFlg.CtrlWordAct = *pOD;
            }
            }
        break;

        case 0x86:
            if(rpOD->GbFlg.ErrState1 & (ENC_ABZ_ERR_CODE | ENC_UVW_ERR_CODE | FIND_MOTOR_ERR_CODE))
            {
                __set_FAULTMASK(1);     //关闭所有中断
                //__set_PRIMASK(1);           //关闭中断
                ClrEncErrData(&rpOD->Fbk, rpOD->QEI_TIMx);

                //__set_PRIMASK(0);           //打开中断
                __set_FAULTMASK(0);
                (rpOD->pF_EncPwrOn)();
                while(temp32u) temp32u --;
                GetHallOffset(&rpOD->Fbk, &rpOD->MotorPar);    //reget hall offset
            }
            rpOD->Fbk.ComuEncErrCnt = 0;
            rpOD->GbFlg.ErrState1 = 0;
            rpOD->GbFlg.ErrProcess1 = 0;
            rpOD->GbFlg.ErrProcess2 = 0;
            rpOD->CurLp.RealIqPk = 0;
            rpOD->GbFlg.ExciCnt = 0;
            rpOD->PosLp.FollowErrMsg = 0;
            //rpOD->PosLp.FollowErr = 0;

        case 0x00:
        case 0x06:
        default:
            (rpOD->pF_DriverOff)();         //disable time pwm output
            rpOD->GbFlg.CtrlModeAct = 0;
            rpOD->GbFlg.CtrlWordAct = 0;
        break;
    }
    return(0);
}

INT8U   wCmdProfileSpeed(INT16U command, INT16U *pOD, void *bpOD, DriverStructTypedef *rpOD)
{
    rpOD->PosLp.ProfileVcn = rpOD->Fbk.SpeedFilterDec;
    rpOD->PosLp.ProfileVcn_1 = rpOD->PosLp.ProfileVcn;
    return(0);
}

INT8U   wCmdStepSpeed(INT16U command, INT16U *pOD, void *bpOD, DriverStructTypedef *rpOD)
{
    INT32S temp32s1;

    temp32s1 = rpOD->SpdLp.CmdCmdSpeedDec * rpOD->MotorPar.Poles;
    rpOD->SpdLp.CmdStepRemain = 128 * rpOD->MotorPar.FB_Resolution;

    rpOD->SpdLp.CmdStepSpeedDecInt = temp32s1 / rpOD->SpdLp.CmdStepRemain;
    rpOD->SpdLp.CmdStepSpeedDecFlt = temp32s1 % rpOD->SpdLp.CmdStepRemain;

    return(0);
}

//Fs=4000Hz,K=512
const INT32S SPD_LP_LF_COFF[][3] =
{
    { 11, -911, 410},    //100Hz
    { 16, -888, 392},    //120Hz
    { 21, -866, 375},    //140Hz
    { 27, -844, 359},    //160Hz
    { 33, -822, 343},    //180Hz
    { 40, -800, 328},    //200Hz
    { 48, -778, 314},    //220Hz
    { 55, -757, 300},    //240Hz
    { 64, -735, 287},    //260Hz
    { 73, -714, 275},    //280Hz
    { 82, -693, 263},    //300Hz
    { 90, -673, 251},    //320Hz
    {101, -652, 241},    //340Hz
    {110, -632, 230},    //360Hz
    {120, -612, 220},    //380Hz
    {130, -593, 211},    //400Hz
    {139, -574, 201},    //420Hz
    {150, -555, 193},    //440Hz
    {160, -536, 184},    //460Hz
    {171, -517, 176},    //480Hz
    {182, -499, 169},    //500Hz
    {192, -481, 161},    //520Hz
    {202, -464, 154},    //540Hz
    {213, -447, 148},    //560Hz
    {223, -430, 141},    //580Hz
    {234, -413, 135},    //600Hz
    {244, -397, 129},    //620Hz
    {255, -381, 124},    //640Hz
    {264, -366, 118},    //660Hz
    {275, -350, 113},    //680Hz
    {285, -335, 108},    //700Hz
    {294, -321, 103},    //720Hz
    {305, -306,  99},    //740Hz
    {315, -292,  95},    //760Hz
    {323, -279,  90},    //780Hz
    {333, -266,  87},    //800Hz
    {342, -253,  83},    //820Hz
    {351, -240,  79},    //840Hz
    {361, -227,  76},    //860Hz
    {369, -215,  72},    //880Hz
    {377, -204,  69},    //900Hz
    {386, -192,  66},    //920Hz
    {394, -181,  63},    //940Hz
    {403, -170,  61},    //960Hz
    {410, -160,  58},    //980Hz
    {418, -150,  56},    //1000Hz
    {425, -140,  53},    //1020Hz
    {433, -130,  51},    //1040Hz
    {440, -121,  49},    //1060Hz
    {446, -112,  46},    //1080Hz
    {453, -103,  44},    //1100Hz
    {460,  -95,  43},    //1120Hz
    {466,  -87,  41},    //1140Hz
    {472,  -79,  39},    //1160Hz
    {478,  -71,  37},    //1180Hz
    {484,  -64,  36},    //1200Hz
    {512,    0,   0},    //no filter
};
INT8U wSpeedLoopLF_Coff(INT16U command, INT8U *pOD, void *bpOD, DriverStructTypedef *rpOD)
{
    rpOD->Fbk.SpeedLF_A = SPD_LP_LF_COFF[*pOD][0];
    rpOD->Fbk.SpeedLF_B = SPD_LP_LF_COFF[*pOD][1];
    rpOD->Fbk.SpeedLF_C = SPD_LP_LF_COFF[*pOD][2];
    return(0);
}


INT8U   rRealIIt_D512(INT16U command, INT16U *pOD, void *bpOD, INT32U *rpOD)
{
    *pOD = (*rpOD)/512;
    return(0);
}

INT8U   rRealDM_IItmA(INT16U command, INT16U *pOD, void *bpOD, IItParameterTypedef *rpOD)
{
    *pOD = isqrt32(rpOD->RealIItDEC_D);
    return(0);
}

INT8U   wSetDC_BusUnderVolt(INT16U command, INT16U *pOD, void *bpOD, DriverStructComTypedef *rpOD)
{
    //DC_BusUnder = (DC_BusUnderDec)*K/Scale
    INT32S temp32s;
    temp32s = (INT32S)(*pOD)* rpOD->DC_BusVolt.Scale / rpOD->DC_BusVolt.K;
    rpOD->ComInfo.DC_BusUnder = (INT16U)temp32s;
    return(0);
}

INT8U   wSetDC_BusOverVolt(INT16U command, INT16U *pOD, void *bpOD, DriverStructComTypedef *rpOD)
{
    //DC_BusOver = (DC_BusOverDec)*K/Scale
    INT32S temp32s;
    temp32s = (INT32S)(*pOD)* rpOD->DC_BusVolt.Scale / rpOD->DC_BusVolt.K;
    rpOD->ComInfo.DC_BusOverDec = (INT16U)temp32s;
    return(0);
}

INT8U   wSetChopVolt(INT16U command, INT16U *pOD, void *bpOD, DriverStructComTypedef *rpOD)
{
    //ChopVolt = (ChopVoltDec)*K/Scale
    INT32S temp32s;
    temp32s = (INT32S)(*pOD)* rpOD->DC_BusVolt.Scale / rpOD->DC_BusVolt.K;
    rpOD->ComInfo.ChopVoltDec = (INT16U)temp32s;
    temp32s = (INT32S)(*pOD) - 3;
    temp32s = temp32s * rpOD->DC_BusVolt.Scale / rpOD->DC_BusVolt.K;
    rpOD->ComInfo.ChopVoltOffDec = (INT16U)temp32s;
    return(0);
}

INT8U   wCmdSpeedDec(INT16U command, INT32S *pOD, void *bpOD, INT32S *rpOD)
{
    if(*pOD > *rpOD)
    {
        *pOD = *rpOD;
    }
    else if(*pOD < (-(*rpOD)))
    {
       *pOD =  (-(*rpOD));
    }
    return(0);
}

INT8U   wCmdSpeedMaxRpm(INT16U command, INT16S *pOD, void *bpOD, DriverStructTypedef *rpOD)
{
    rpOD->SpdLp.CmdSpeedMaxDec = ((INT32S)(*pOD) * rpOD->MotorPar.FB_Resolution/1875) * 512;
    rpOD->SpdLp.CmdSpeedMaxDec1d1 = ((rpOD->SpdLp.CmdSpeedMaxDec)*110)/100;
    return(0);
}

INT8U   wSetMotorIIt_I(INT16U command, INT16U *pOD, void *bpOD, DriverStructTypedef *rpOD)
{
    INT32U temp32u;
    temp32u = (2048 * (*pOD))/ (rpOD->DriveInfo.I_Max);
    rpOD->MotorIIt.UserSetII_DEC = temp32u * temp32u;
    rpOD->MotorIIt.UserSetI = *pOD;
    return(0);
}

INT8U   wSetMotorIIt_T(INT16U command, INT16U *pOD, void *bpOD, DriverStructTypedef *rpOD)
{
    rpOD->MotorIIt.UserSetT_DEC = ((*pOD) * 256) * 16;
    rpOD->MotorIIt.UserSetT = *pOD;
    return(0);
}

INT8U   wSetDC_MotorIIt_I(INT16U command, INT16U *pOD, void *bpOD, IItParameterTypedef *rpOD)
{
    INT32U temp32u;
    temp32u = (*pOD);
    rpOD->UserSetII_DEC = temp32u * temp32u;
    return(0);
}

INT8U   wSetDC_MotorIIt_T(INT16U command, INT16U *pOD, void *bpOD, IItParameterTypedef *rpOD)
{
    rpOD->UserSetT_DEC = (*pOD) * 16000;
    return(0);
}

INT8U   wSingleDoutSimulate(INT16U command, INT8U *pOD, void *bpOD, INT16U *rpOD)
{
    if(*pOD == 1)
    {
        *rpOD = (*rpOD) | (0x0001<<command);
    }
    if(*pOD == 0)
    {
        *rpOD = (*rpOD) & (~(0x0001<<command));
    }
    return(0);
}

//motion
INT8U   wSetTurnAngle(INT16U command, INT32S *pOD, void *bpOD, MotionStructTypedef *rpOD)
{
    INT32S temp32s1;
    INT32S temp32s2;
    INT64S temp64s;
    temp32s1 = (INT32S)(*pOD) * rpOD->CarMchPar.Wheelbase * rpOD->CarMchPar.GearF;
    temp64s = (INT64S)DX[0].MotorPar.FB_Resolution * temp32s1;
    temp32s2 = (INT32S)rpOD->CarMchPar.GearD * rpOD->CarMchPar.WheelDia;
    temp64s = temp64s / temp32s2;
    rpOD->TurnAngle.CmdCmdPos += temp64s / (360);
    //rpOD->TurnAngle.TurnCmdTrig = 1;
    return(0);
}

INT8U   rRemainTurnAngle(INT16U command, INT32S *pOD, void *bpOD, MotionStructTypedef *rpOD)
{
    INT32S temp32s1;
    INT32S temp32s2;
    INT64S temp64s1,temp64s2;


    temp32s1 = (INT32S)rpOD->CarMchPar.GearD * rpOD->CarMchPar.WheelDia;
    temp64s1 = (INT64S)rpOD->TurnAngle.RemainPcn * temp32s1;
    temp64s1 = temp64s1 * 360;

    temp32s1 = rpOD->CarMchPar.Wheelbase * rpOD->CarMchPar.GearF;
    temp64s2 = (INT64S)DX[0].MotorPar.FB_Resolution * temp32s1;

    rpOD->TurnAngle.CmdAngle = temp64s1 / temp64s2;

    return(0);
}

INT8U   wSetPosLengthMm(INT16U command, INT32S *pOD, void *bpOD, MotionStructTypedef *rpOD)
{
    INT32S temp32s1;
    INT32S temp32s2;
    INT64S temp64s;
    //[(PosLengthMm/(pi*D))*GearF/GearD]*FB_Resolution
    temp32s1 = DX[0].MotorPar.FB_Resolution * rpOD->CarMchPar.GearF;
    temp64s = (INT64S)(*pOD) * temp32s1;
    temp64s = temp64s * 10000;

    temp32s2 = (INT32S)rpOD->CarMchPar.GearD * rpOD->CarMchPar.WheelDia;
    temp32s2 = temp32s2 * 31416;    //pi
    temp32s1 = temp64s / temp32s2;

    DX[0].PosLp.CmdCmdPos -= temp32s1;
    DX[1].PosLp.CmdCmdPos += temp32s1;

    return(0);
}

INT8U   rRemainPosLengthMm(INT16U command, INT32S *pOD, void *bpOD, DriverStructTypedef *rpOD)
{
    INT32S temp32s1;
    INT32S temp32s2;
    INT64S temp64s;
    //[(PosLengthMm/(pi*D))*GearF/GearD]*FB_Resolution
    temp32s1 = (INT32S)MOX.CarMchPar.GearD * MOX.CarMchPar.WheelDia;
    temp32s1 = temp32s1 * 31416;    //pi
    temp64s = (INT64S)DX[1].PosLp.RemainPcn * temp32s1;

    temp32s1 = DX[1].MotorPar.FB_Resolution * MOX.CarMchPar.GearF;

    temp64s = temp64s / temp32s1;

    MOX.PosLength.CmdPosMm = temp64s / 10000;
    return(0);
}

void ConversionSpeedMmps2Dec(INT32S *pOD, INT32S *rpOD)
{
    INT32S temp32s1,temp32s2;
    INT64S temp64s1,temp64s2;
    //[Dec]=512*40*FB_Resolution*GearF*[mm/s]/(3927*WheelDia*GearD)
    temp32s1 = DX[0].MotorPar.FB_Resolution * MOX.CarMchPar.GearF;
    temp64s1 = 512*40*(INT64S)temp32s1*(*pOD);

    temp32s1 = 3927 * MOX.CarMchPar.WheelDia * MOX.CarMchPar.GearD;

    *rpOD = temp64s1 / (INT64S)temp32s1;
}

void ConversionSpeedDec2Mmps(INT32S *pOD, INT32S *rpOD)
{
    INT32S temp32s1,temp32s2;
    INT64S temp64s1,temp64s2;
    //[mm/s]=3927*SpeedDec*WheelDia*GearD/(512*40*FB_Resolution*GearF)
    temp32s1 = 3927 * MOX.CarMchPar.WheelDia * MOX.CarMchPar.GearD;
    temp64s1 = (INT64S)(*pOD) * temp32s1;

    temp32s1 = DX[0].MotorPar.FB_Resolution * MOX.CarMchPar.GearF;
    temp64s2 = 512*40*(INT64S)temp32s1;

    *rpOD = temp64s1 / temp64s2;
}

INT8U   wSetMotionSpeed(INT16U command, INT32S *pOD, void *bpOD, void *rpOD)
{
    INT32S temp32s1,temp32s2;
    INT64S temp64s1,temp64s2;

    if(command == 0)    //DEC to mm/s
    {
        ConversionSpeedDec2Mmps(pOD, rpOD);
    }
    else if(command == 1) //mm/s to DEC
    {
        ConversionSpeedMmps2Dec(pOD, rpOD);
    }

    MOX.GbFlg.ParUpdataLock = 1;
    if(MOX.MotionPar.Radius == 0)   //半径为0时，前进后退
    {
        MOX.MotionPar.CmdSpeedDec6622_1 = -MOX.MotionPar.CmdSpeedDec;
        MOX.MotionPar.CmdSpeedDec6622_2 = MOX.MotionPar.CmdSpeedDec;
    }
    if(MOX.MotionPar.Radius == 1 || MOX.MotionPar.Radius == -1)     //原地打转
    {
        MOX.MotionPar.CmdSpeedDec6622_1 = -MOX.MotionPar.CmdSpeedDec;
        MOX.MotionPar.CmdSpeedDec6622_2 = -MOX.MotionPar.CmdSpeedDec;
    }
    if(MOX.MotionPar.Radius > 1)    //圆心在左轮左侧，目标速度规定给右轮速度
    {
        temp32s1 = MOX.MotionPar.Radius*2 - MOX.CarMchPar.Wheelbase;
        temp32s2 = MOX.MotionPar.Radius*2 + MOX.CarMchPar.Wheelbase;
        //计算目标速度
        MOX.MotionPar.CmdSpeedDec6622_1 = -MOX.MotionPar.CmdSpeedDec;
        temp64s1 = (INT64S)MOX.MotionPar.CmdSpeedDec * (INT64S)temp32s1;
        MOX.MotionPar.CmdSpeedDec6622_2 = temp64s1 / temp32s2;
    }
    if(MOX.MotionPar.Radius < -1)    //圆心在右轮右侧，目标速度规定给左轮速度
    {
        temp32s1 = MOX.MotionPar.Radius*2 + MOX.CarMchPar.Wheelbase;
        temp32s2 = MOX.MotionPar.Radius*2 - MOX.CarMchPar.Wheelbase;

        //计算目标速度
        MOX.MotionPar.CmdSpeedDec6622_2 = -MOX.MotionPar.CmdSpeedDec;
        temp64s1 = (INT64S)MOX.MotionPar.CmdSpeedDec * (INT64S)temp32s1;
        MOX.MotionPar.CmdSpeedDec6622_1 = temp64s1 / temp32s2;
    }
    MOX.GbFlg.ParUpdataLock = 0;
    return(0);
}

INT8U   wSetMotionProfileSpeed(INT16U command, INT32S *pOD, void *bpOD, void *rpOD)
{
    INT32S temp32s1,temp32s2;
    INT64S temp64s1,temp64s2;

    if(command == 0)    //DEC to mm/s
    {
        ConversionSpeedDec2Mmps(pOD, rpOD);
    }
    else if(command == 1) //mm/s to DEC
    {
        ConversionSpeedMmps2Dec(pOD, rpOD);
    }

    MOX.GbFlg.ParUpdataLock = 1;
    if(MOX.MotionPar.Radius == 0)   //半径为0时，前进后退
    {
        MOX.MotionPar.CmdProfileSpeed6622_1 = MOX.MotionPar.CmdProfileSpeed;
        MOX.MotionPar.CmdProfileSpeed6622_2 = MOX.MotionPar.CmdProfileSpeed;
    }
    if(MOX.MotionPar.Radius == 1 || MOX.MotionPar.Radius == -1)     //原地打转
    {
        MOX.MotionPar.CmdProfileSpeed6622_1 = MOX.MotionPar.CmdProfileSpeed;
        MOX.MotionPar.CmdProfileSpeed6622_2 = MOX.MotionPar.CmdProfileSpeed;
    }
    if(MOX.MotionPar.Radius > 1)    //圆心在左轮左侧，目标速度规定给右轮速度
    {
        temp32s1 = MOX.MotionPar.Radius*2 - MOX.CarMchPar.Wheelbase;
        temp32s2 = MOX.MotionPar.Radius*2 + MOX.CarMchPar.Wheelbase;

        //计算梯形速度
        MOX.MotionPar.CmdProfileSpeed6622_1 = MOX.MotionPar.CmdProfileSpeed;
        temp64s1 = (INT64S)MOX.MotionPar.CmdProfileSpeed * (INT64S)temp32s1;
        if(temp64s1 < 0)
            temp64s1 = -temp64s1;
        MOX.MotionPar.CmdProfileSpeed6622_2 = temp64s1 / temp32s2;
    }
    if(MOX.MotionPar.Radius < -1)    //圆心在右轮右侧，目标速度规定给左轮速度
    {
        temp32s1 = MOX.MotionPar.Radius*2 + MOX.CarMchPar.Wheelbase;
        temp32s2 = MOX.MotionPar.Radius*2 - MOX.CarMchPar.Wheelbase;

        //计算梯形速度
        MOX.MotionPar.CmdProfileSpeed6622_2 = MOX.MotionPar.CmdProfileSpeed;
        temp64s1 = (INT64S)MOX.MotionPar.CmdProfileSpeed * (INT64S)temp32s1;
        if(temp64s1 > 0)
            temp64s1 = -temp64s1;
        MOX.MotionPar.CmdProfileSpeed6622_1 = temp64s1 / temp32s2;
    }
    MOX.GbFlg.ParUpdataLock = 0;
    return(0);
}

INT8U   wSetMotionRadiusMm(INT16U command, INT32S *pOD, void *bpOD, void *rpOD)
{
    INT32S temp32s1,temp32s2;
    INT64S temp64s1,temp64s2;
    if(*pOD != MOX.MotionPar.Radius)
    {
    MOX.GbFlg.ParUpdataLock = 1;
    MOX.GbFlg.RadiusUpdataTrig = 1;
    MOX.MotionPar.Radius = *pOD;
    if(MOX.MotionPar.Radius == 0)   //半径为0时，前进后退
    {
        MOX.MotionPar.CmdSpeedDec6622_1 = -MOX.MotionPar.CmdSpeedDec;
        MOX.MotionPar.CmdSpeedDec6622_2 = MOX.MotionPar.CmdSpeedDec;
        MOX.MotionPar.CmdProfileSpeed6622_1 = MOX.MotionPar.CmdProfileSpeed;
        MOX.MotionPar.CmdProfileSpeed6622_2 = MOX.MotionPar.CmdProfileSpeed;
        MOX.MotionPar.ProfileAcceCmd6622_1 = MOX.MotionPar.ProfileAcceCmd;
        MOX.MotionPar.ProfileAcceCmd6622_2 = MOX.MotionPar.ProfileAcceCmd;
        MOX.MotionPar.ProfileDeceCmd6622_1 = MOX.MotionPar.ProfileDeceCmd;
        MOX.MotionPar.ProfileDeceCmd6622_2 = MOX.MotionPar.ProfileDeceCmd;
        MOX.MotionPar.CmdQkStopDece6622_1 = MOX.MotionPar.CmdQkStopDece;
        MOX.MotionPar.CmdQkStopDece6622_2 = MOX.MotionPar.CmdQkStopDece;
    }
    if(MOX.MotionPar.Radius == 1 || MOX.MotionPar.Radius == -1)     //原地打转
    {
        MOX.MotionPar.CmdSpeedDec6622_1 = -MOX.MotionPar.CmdSpeedDec;
        MOX.MotionPar.CmdSpeedDec6622_2 = -MOX.MotionPar.CmdSpeedDec;
        MOX.MotionPar.CmdProfileSpeed6622_1 = MOX.MotionPar.CmdProfileSpeed;
        MOX.MotionPar.CmdProfileSpeed6622_2 = MOX.MotionPar.CmdProfileSpeed;
        MOX.MotionPar.ProfileAcceCmd6622_1 = MOX.MotionPar.ProfileAcceCmd;
        MOX.MotionPar.ProfileAcceCmd6622_2 = MOX.MotionPar.ProfileAcceCmd;
        MOX.MotionPar.ProfileDeceCmd6622_1 = MOX.MotionPar.ProfileDeceCmd;
        MOX.MotionPar.ProfileDeceCmd6622_2 = MOX.MotionPar.ProfileDeceCmd;
        MOX.MotionPar.CmdQkStopDece6622_1 = MOX.MotionPar.CmdQkStopDece;
        MOX.MotionPar.CmdQkStopDece6622_2 = MOX.MotionPar.CmdQkStopDece;
    }
    if(MOX.MotionPar.Radius > 1)    //圆心在左轮左侧，目标速度规定给右轮速度
    {
        temp32s1 = MOX.MotionPar.Radius*2 - MOX.CarMchPar.Wheelbase;
        temp32s2 = MOX.MotionPar.Radius*2 + MOX.CarMchPar.Wheelbase;
        //计算目标速度
        MOX.MotionPar.CmdSpeedDec6622_1 = -MOX.MotionPar.CmdSpeedDec;
        temp64s1 = (INT64S)MOX.MotionPar.CmdSpeedDec * (INT64S)temp32s1;
        MOX.MotionPar.CmdSpeedDec6622_2 = temp64s1 / temp32s2;
        //计算梯形速度
        MOX.MotionPar.CmdProfileSpeed6622_1 = MOX.MotionPar.CmdProfileSpeed;
        temp64s1 = (INT64S)MOX.MotionPar.CmdProfileSpeed * (INT64S)temp32s1;
        if(temp64s1 < 0)
            temp64s1 = -temp64s1;
        MOX.MotionPar.CmdProfileSpeed6622_2 = temp64s1 / temp32s2;

        //计算加减速度
        MOX.MotionPar.ProfileAcceCmd6622_1 = MOX.MotionPar.ProfileAcceCmd;
        temp64s1 = (INT64S)MOX.MotionPar.ProfileAcceCmd * (INT64S)temp32s1;
        if(temp64s1 < 0)
            temp64s1 = -temp64s1;
        MOX.MotionPar.ProfileAcceCmd6622_2 = temp64s1 / temp32s2;

        MOX.MotionPar.ProfileDeceCmd6622_1 = MOX.MotionPar.ProfileDeceCmd;
        temp64s1 = (INT64S)MOX.MotionPar.ProfileDeceCmd * (INT64S)temp32s1;
        if(temp64s1 < 0)
            temp64s1 = -temp64s1;
        MOX.MotionPar.ProfileDeceCmd6622_2 = temp64s1 / temp32s2;

        //计算快速停止加速度
        MOX.MotionPar.CmdQkStopDece6622_1 = MOX.MotionPar.CmdQkStopDece;
        temp64s1 = (INT64S)MOX.MotionPar.CmdQkStopDece * (INT64S)temp32s1;
        if(temp64s1 < 0)
            temp64s1 = -temp64s1;
        MOX.MotionPar.CmdQkStopDece6622_2 = temp64s1 / temp32s2;
    }
    if(MOX.MotionPar.Radius < -1)    //圆心在右轮右侧，目标速度规定给左轮速度
    {
        temp32s1 = MOX.MotionPar.Radius*2 + MOX.CarMchPar.Wheelbase;
        temp32s2 = MOX.MotionPar.Radius*2 - MOX.CarMchPar.Wheelbase;

        //计算目标速度
        MOX.MotionPar.CmdSpeedDec6622_2 = -MOX.MotionPar.CmdSpeedDec;
        temp64s1 = (INT64S)MOX.MotionPar.CmdSpeedDec * (INT64S)temp32s1;
        MOX.MotionPar.CmdSpeedDec6622_1 = temp64s1 / temp32s2;
        //计算梯形速度
        MOX.MotionPar.CmdProfileSpeed6622_2 = MOX.MotionPar.CmdProfileSpeed;
        temp64s1 = (INT64S)MOX.MotionPar.CmdProfileSpeed * (INT64S)temp32s1;
        if(temp64s1 > 0)
            temp64s1 = -temp64s1;
        MOX.MotionPar.CmdProfileSpeed6622_1 = temp64s1 / temp32s2;

        //计算加减速度
        MOX.MotionPar.ProfileAcceCmd6622_2 = MOX.MotionPar.ProfileAcceCmd;
        temp64s1 = (INT64S)MOX.MotionPar.ProfileAcceCmd * (INT64S)temp32s1;
        if(temp64s1 > 0)
            temp64s1 = -temp64s1;
        MOX.MotionPar.ProfileAcceCmd6622_1 = temp64s1 / temp32s2;

        MOX.MotionPar.ProfileDeceCmd6622_2 = MOX.MotionPar.ProfileDeceCmd;
        temp64s1 = (INT64S)MOX.MotionPar.ProfileDeceCmd * (INT64S)temp32s1;
        if(temp64s1 > 0)
            temp64s1 = -temp64s1;
        MOX.MotionPar.ProfileDeceCmd6622_1 = temp64s1 / temp32s2;

        //计算快速停止加速度
        MOX.MotionPar.CmdQkStopDece6622_2 = MOX.MotionPar.CmdQkStopDece;
        temp64s1 = (INT64S)MOX.MotionPar.CmdQkStopDece * (INT64S)temp32s1;
        if(temp64s1 > 0)
            temp64s1 = -temp64s1;
        MOX.MotionPar.CmdQkStopDece6622_1 = temp64s1 / temp32s2;
    }
    MOX.GbFlg.ParUpdataLock = 0;
    }

    return(0);
}


INT8U   wMotionEnable(INT16U command, INT8U *pOD, void *bpOD, MotionStructTypedef *rpOD)
{
    if(*pOD == 1)
    {
        MOX.TurnAngle.ProfileVcn = 0;
        MOX.TurnAngle.ProfileVcn_1 = 0;
        MOX.TurnAngle.CmdDeltaPosOut = 0;
        MOX.TurnAngle.ProfilePcnFlt = 0;
        MOX.TurnAngle.CmdPos = MOX.TurnAngle.ProfilePcn_1;
        MOX.TurnAngle.CmdCmdPos = MOX.TurnAngle.CmdPos;
    }
    if(*pOD == 0)
    {
        DX[0].SpdLp.CmdCmdSpeedDec = 0;
        DX[1].SpdLp.CmdCmdSpeedDec = 0;
        DX[0].SpdLp.CmdSpeedDecExt = 0;
        DX[1].SpdLp.CmdSpeedDecExt = 0;
        DX[0].PosLp.CmdDeltaPosExt = 0;
        DX[1].PosLp.CmdDeltaPosExt = 0;
    }
}

INT8U   wMotionControlWord(INT16U command, INT32S *pOD, void *bpOD, MotionStructTypedef *rpOD)
{
    if(MOX.GbFlg.MotionEnable == 1)
    {
    switch(MOX.GbFlg.MotionCtrWord)
    {
        case 0x0f:
             if(MOX.GbFlg.MotionCtrWordAct != MOX.GbFlg.MotionCtrWord)
             {
                if(MOX.GbFlg.MotionMode == 33 || MOX.GbFlg.MotionMode == 66)
                {
                    MOX.GbFlg.SwtichOnLock = 1;
                    DX[D1].GbFlg.CtrlMode = 3;
                    DX[D2].GbFlg.CtrlMode = 3;
                }
                else if(MOX.GbFlg.MotionMode == 11)
                {
                    MOX.GbFlg.SwtichOnLock = 1;
                    DX[D1].GbFlg.CtrlMode = 1;
                    DX[D2].GbFlg.CtrlMode = 1;
                    DX[D1].PosLp.CmdCmdPos = DX[D1].Fbk.PosAbs;
                    DX[D2].PosLp.CmdCmdPos = DX[D2].Fbk.PosAbs;
                }

                if(MOX.GbFlg.SwtichOnLock == 1)
                {
                    if(SysService.SteEng[0].BitCtrl == 1 || SysService.SteEng[1].BitCtrl == 1)
                    {
                        MOX.GbFlg.MotionModeAct = 0;
                        MOX.GbFlg.MotionCtrWordAct = 0x06;
                        MOX.GbFlg.MotionCtrWord = 0x06;
                        DX[D1].GbFlg.CtrlWord = 0x06;
                        DX[D2].GbFlg.CtrlWord = 0x06;
                        wControlWord(0, &DX[D1].GbFlg.CtrlWord, NULL, &DX[D1]);
                        wControlWord(0, &DX[D2].GbFlg.CtrlWord, NULL, &DX[D2]);
                        DX[0].GbFlg.ErrState1 |= RESERVE_ERR_CODE;
                        DX[1].GbFlg.ErrState1 |= RESERVE_ERR_CODE;
                    }
                    else
                    {
                        //clear some
                        MOX.TurnAngle.ProfileVcn = 0;
                        MOX.TurnAngle.ProfileVcn_1 = 0;
                        MOX.TurnAngle.CmdDeltaPosOut = 0;
                        MOX.TurnAngle.ProfilePcnFlt = 0;
                        MOX.TurnAngle.CmdPos = MOX.TurnAngle.ProfilePcn_1;
                        MOX.TurnAngle.CmdCmdPos = MOX.TurnAngle.CmdPos;
                        MOX.TurnAngle.QkStopTrig = 0;

                        MOX.GbFlg.MotionModeAct = MOX.GbFlg.MotionMode;
                        MOX.GbFlg.MotionCtrWordAct = MOX.GbFlg.MotionCtrWord;
                        DX[D1].GbFlg.CtrlWord = 0x0f;
                        DX[D2].GbFlg.CtrlWord = 0x0f;
                        wControlWord(0, &DX[D1].GbFlg.CtrlWord, NULL, &DX[D1]);
                        wControlWord(0, &DX[D2].GbFlg.CtrlWord, NULL, &DX[D2]);
                        MOX.GbFlg.SwtichOnLock = 0;
                    }
                }
                MOX.GbFlg.RadiusUpdataTrig = 0;
            }
        break;
    case 0x86:
        DX[D1].GbFlg.CtrlWord = 0x86;
        DX[D2].GbFlg.CtrlWord = 0x86;
        MOX.GbFlg.MotionCtrWordAct = MOX.GbFlg.MotionCtrWord;
        wControlWord(0, &DX[D1].GbFlg.CtrlWord, NULL, &DX[D1]);
        wControlWord(0, &DX[D2].GbFlg.CtrlWord, NULL, &DX[D2]);

        break;
    case 0x00:
    case 0x06:
        DX[D1].GbFlg.CtrlWord = 0x06;
        DX[D2].GbFlg.CtrlWord = 0x06;
        MOX.GbFlg.MotionModeAct = 0;
        MOX.GbFlg.MotionCtrWordAct = MOX.GbFlg.MotionCtrWord;
        wControlWord(0, &DX[D1].GbFlg.CtrlWord, NULL, &DX[D1]);
        wControlWord(0, &DX[D2].GbFlg.CtrlWord, NULL, &DX[D2]);
    break;

    default:break;
    }
    }

    return(0);
}

INT8U   wDC_MotorControlWord(INT16U command, INT16U *pOD, void *bpOD, void *rpOD)
{
    switch(*pOD)
    {
    case 0x1f:
        if(DXC.PlatformInfo.DownLimitFlag == 0)
        {
            DXC.DM_Ctrl.Direction = 1;  //下降方向
            wDC_MotorDirectionAndSpeed(0, &DXC.DM_Ctrl.Direction, NULL, &DXC.DM_Ctrl);
            DX[0].GbFlg.ErrState1 &= (~(DCMOTOR_ERR_CODE|PLM_LIMIT_ERR_CODE));
            DX[0].GbFlg.ErrProcess1 &= (~(DCMOTOR_ERR_CODE|PLM_LIMIT_ERR_CODE));
            DX[0].GbFlg.ErrProcess2 &= (~(DCMOTOR_ERR_CODE|PLM_LIMIT_ERR_CODE));
            DX[1].GbFlg.ErrState1 &= (~(DCMOTOR_ERR_CODE|PLM_LIMIT_ERR_CODE));
            DX[1].GbFlg.ErrProcess1 &= (~(DCMOTOR_ERR_CODE|PLM_LIMIT_ERR_CODE));
            DX[1].GbFlg.ErrProcess2 &= (~(DCMOTOR_ERR_CODE|PLM_LIMIT_ERR_CODE));
            DXC.DM_Ctrl.OnTrig = 1;
        }
        else
        {
            *pOD = 0x06;
        }
    break;

    case 0x2f:
        if(DXC.PlatformInfo.UpLimitFlag == 0)
        {
            DXC.DM_Ctrl.Direction = 2;  //上升方向
            wDC_MotorDirectionAndSpeed(0, &DXC.DM_Ctrl.Direction, NULL, &DXC.DM_Ctrl);
            DX[0].GbFlg.ErrState1 &= (~(DCMOTOR_ERR_CODE|PLM_LIMIT_ERR_CODE));
            DX[0].GbFlg.ErrProcess1 &= (~(DCMOTOR_ERR_CODE|PLM_LIMIT_ERR_CODE));
            DX[0].GbFlg.ErrProcess2 &= (~(DCMOTOR_ERR_CODE|PLM_LIMIT_ERR_CODE));
            DX[1].GbFlg.ErrState1 &= (~(DCMOTOR_ERR_CODE|PLM_LIMIT_ERR_CODE));
            DX[1].GbFlg.ErrProcess1 &= (~(DCMOTOR_ERR_CODE|PLM_LIMIT_ERR_CODE));
            DX[1].GbFlg.ErrProcess2 &= (~(DCMOTOR_ERR_CODE|PLM_LIMIT_ERR_CODE));
            DXC.DM_Ctrl.OnTrig = 1;
        }
        else
        {
            *pOD = 0x06;
        }
    break;

    case 0x86:
        DX[0].GbFlg.ErrState1 &= (~(DCMOTOR_ERR_CODE|PLM_LIMIT_ERR_CODE));
        DX[0].GbFlg.ErrProcess1 &= (~(DCMOTOR_ERR_CODE|PLM_LIMIT_ERR_CODE));
        DX[0].GbFlg.ErrProcess2 &= (~(DCMOTOR_ERR_CODE|PLM_LIMIT_ERR_CODE));
        DX[1].GbFlg.ErrState1 &= (~(DCMOTOR_ERR_CODE|PLM_LIMIT_ERR_CODE));
        DX[1].GbFlg.ErrProcess1 &= (~(DCMOTOR_ERR_CODE|PLM_LIMIT_ERR_CODE));
        DX[1].GbFlg.ErrProcess2 &= (~(DCMOTOR_ERR_CODE|PLM_LIMIT_ERR_CODE));
    case 0x06:
    //break;
    default:
        DXC.DM_Ctrl.Direction = 0;  //不动
        DC_MotorOff();
    break;
    }
    return(0);
}

INT8U   wDC_MotorDirectionAndSpeed(INT16U command, void *pOD, void *bpOD, DcMotorCtrlTypedef *rpOD)
{
    INT8U temp8u;
    INT16U temp16u;
    switch(rpOD->Direction)
    {
    case 0: //不动
        TIM9->CCR1 = DM_PWM_PERIOD/2;
        TIM9->CCR2 = DM_PWM_PERIOD/2;
    break;

    case 1: //下降
        temp8u = rpOD->SpeedSelect;
        temp16u = rpOD->Speed[temp8u];
        TIM9->CCR1 = DM_PWM_PERIOD/2 + temp16u;
        TIM9->CCR2 = DM_PWM_PERIOD/2 - temp16u;
    break;

    case 2: //上升
        temp8u = rpOD->SpeedSelect;
        temp16u = rpOD->Speed[temp8u];
        TIM9->CCR1 = DM_PWM_PERIOD/2 - temp16u;
        TIM9->CCR2 = DM_PWM_PERIOD/2 + temp16u;
    break;

    default:break;
    }
    return(0);
}


INT8U   rPlatformHighAct(INT16U command, INT16S *pOD, void *bpOD, PlatformInfoTypedef *rpOD)
{
    INT32S temp32s;

    temp32s =  (rpOD->Pos.AllPulse.Int32s) * (INT32S)rpOD->Kn;
    temp32s = temp32s / rpOD->Kd;
    *pOD = temp32s + rpOD->MechOffsetMm;
    return(0);
}

INT8U   rPlatformDownLimitMm(INT16U command, INT16S *pOD, void *bpOD, PlatformInfoTypedef *rpOD)
{
    INT32S temp32s;

    temp32s =  (rpOD->DownLimitDec) * (INT32S)rpOD->Kn;
    temp32s = temp32s / rpOD->Kd;
    *pOD = temp32s + rpOD->MechOffsetMm;
    return(0);
}

INT8U   rPlatformUpLimitMm(INT16U command, INT16S *pOD, void *bpOD, PlatformInfoTypedef *rpOD)
{
    INT32S temp32s;

    temp32s =  (rpOD->UpLimitDec) * (INT32S)rpOD->Kn;
    temp32s = temp32s / rpOD->Kd;
    *pOD = temp32s + rpOD->MechOffsetMm;
    return(0);
}

INT8U   wPlatformDownLimitMm(INT16U command, INT16S *pOD, void *bpOD, PlatformInfoTypedef *rpOD)
{
    INT32S temp32s;
    temp32s = *pOD - rpOD->MechOffsetMm;
    temp32s = temp32s * rpOD->Kd;
    rpOD->DownLimitDec = temp32s / rpOD->Kn;
    return(0);
}

INT8U   wPlatformUpLimitMm(INT16U command, INT16S *pOD, void *bpOD, PlatformInfoTypedef *rpOD)
{
    INT32S temp32s;

    temp32s = *pOD - rpOD->MechOffsetMm;
    temp32s = temp32s * rpOD->Kd;
    rpOD->UpLimitDec = temp32s / rpOD->Kn;
    return(0);
}


INT8U   rAnalogInputRealVoltage(INT16U command, INT16S *pOD, void *bpOD, AnalogGroupBaseTypedef *rpOD)
{
    CalAnalogRealDec(rpOD);
    return(0);
}

void DinFuncQkStop(INT16U command)
{
    //这样做是为了避免多个IO口被配置同一个功能后导致处理出现抖动
    if(command != 0)
    {
        DX[0].GbFlg.QkStopDin |= command;
        DX[1].GbFlg.QkStopDin |= command;
    }
    else
    {
        DX[0].GbFlg.QkStopDin &= (~(0x0001<<DXC.Din.NowNum));
        DX[1].GbFlg.QkStopDin &= (~(0x0001<<DXC.Din.NowNum));
    }

}

void DinFuncPlatformOrg(INT16U command)
{
    //这样做是为了避免多个IO口被配置同一个功能后导致处理出现抖动
    if(command != 0)
    {
        DXC.PlatformInfo.OriginDin |= command;
    }
    else
    {
        DXC.PlatformInfo.OriginDin &= (~(0x0001<<DXC.Din.NowNum));
    }

}

const DinFuncAttribute DinFuncADD[] =
{
    //Index.......pFunc
    {0x1000,    DinFuncQkStop},
    {0x8100,    DinFuncPlatformOrg},    //平台原点
};

void DinFuncHandle(void)
{
    INT16U i,j,k;
    INT16U command = 0;
    //for(k=0; k<7; k++)
    {
    for(j=0; j<7; j++)
    {
        if(DXC.Din.FuncIndex[j] != 0)
        {
            DXC.Din.NowNum = j;
            command = DXC.Din.Virtual & (0x0001 << j);
            for(i=0; i<sizeof(DinFuncADD)/sizeof(*DinFuncADD); i++)     //搜索Function 列表
            {
                if(DXC.Din.FuncIndex[j] == DinFuncADD[i].Index)
                {

                    (DinFuncADD[i].pFunc)(command);
                }
            }
        }
    }
    }
}
