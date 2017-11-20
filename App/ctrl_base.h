/*
 * SR2D1_0V3_MCU_S1 -- Service robot 2In1 servo driver
 * (C)2016 Kinco Electric (Shenzhen) Ltd. -- All rights reserved
 */
#ifndef _CTRL_BASE_H
#define _CTRL_BASE_H
#define SINE_TABLE_NUM  2048
#define COSINE_TABLE_NUM    SINE_TABLE_NUM
#define SINE_COSINE_VALUE_MAX       16384U
#define K_UV_FACTOR     8192
#define SVPWM_OUT_POLARITY  0           //0:low level active; 1:high level active
#define CAL_SPEED_RPM_BASE_FCY      PWM_FCY
#define CAL_SPEED_RPM_CNT       ((200*CAL_SPEED_RPM_BASE_FCY)/1000)
#define CAL_SPEED_RPM_FTR       ((60*CAL_SPEED_RPM_BASE_FCY)/CAL_SPEED_RPM_CNT)
typedef enum {RLT_NO_ERR = 0, RLT_ERR = !RLT_NO_ERR} ResultStatus;

#define ENC_ABZ_ERR_CODE    0x0002
#define ENC_UVW_ERR_CODE    0x0004
#define TEMP_OVER_ERR_CODE  0X0010
#define DCBUS_OVER_ERR_CODE 0x0020
#define DCBUS_LOW_ERR_CODE  0x0040
#define SHORT_CIRCUIT_CODE  0x0080
#define CHOPING_R_ERR_CODE  0x0100
#define FOLLOW_ERR_CODE     0x0200
#define IIT_ERR_CODER       0x0800
#define FIND_MOTOR_ERR_CODE 0x4000
#define RESERVE_ERR_CODE    0x2000
#define STEENT_ERR_CODE     0x2000
#define DCMOTOR_ERR_CODE    0x1000
#define PLM_LIMIT_ERR_CODE  0x8000

#define ENC_ABZ_CHECK_CODE  0x0010
#define ENC_UVW_CHECK_CODE  0x0001
#define ENC_COMU_CHECK_CODE 0X0040
#define DR_STATE_FAULT_BIT  0x0008
#define DR_STATE_FAULT_CLR  0x0000

#define PLATFORM_DOWM_MOVE  0x0001
#define PLATFORM_UP_MOVE    0x0002

typedef union
{
    INT8U   All8u[4];
    INT8S   All8s[4];
    INT8U   Int8u;
    INT8S   Int8s;
    INT16U  Int16u;
    INT16S  Int16s;
    INT32U  Int32u;
    INT32S  Int32s;
    struct
    {
        INT16U Low16u;
        INT16U High16u;
    } Split;
} UNION_DATA_DEF;

typedef union
{
    INT8U   All8u[4];
    INT8S   All8s[4];
    INT8U   Int8u;
    INT8S   Int8s;
    INT16U  Int16u;
    INT16S  Int16s;
    INT32U  Int32u;
    INT32S  Int32s;
    struct
    {
        INT16U Low16u;
        INT16U High16u;
    } Split;
} UNION32_DATA_DEF;

typedef union
{
    INT8U   All8u[8];
    INT8S   All8s[8];
    INT8U   Int8u;
    INT8S   Int8s;
    INT16U  Int16u;
    INT16S  Int16s;
    INT32U  Int32u;
    INT32S  Int32s;
    INT64U  Int64u;
    INT64S  Int64s;
    struct
    {
        INT16U Low16u;
        INT16U Mid16u1;
        INT16U Mid16u2;
        INT16U High16u;
    } Split16;
} UNION64_DATA_DEF;

typedef struct
{
    INT16U* pAdcOrg;
    INT16U  AdcOrg;     //ADC original value
    INT16S  AdcOffset;  //offset
    INT16S  AdcOrgAct;  //active data = AdcOrg - AdcOffset;
    INT32S  AdcRealDec; //real ADC value dec
    INT16S  Scale;      //scale
    INT16S  K;
} AnalogGroupBaseTypedef;

typedef struct
{
    INT16S Ialpha;
    INT16S Ibeta;
} ClarkTypedef;  //clark正变换
typedef struct
{
    INT16S SinVal;
    INT16S CosVal;
    INT16S Iq;
    INT16S Id;
} ParkTypedef;  //park 正变换
typedef struct
{
    INT16S Valpha;
    INT16S Vbeta;
    INT32S VqLast;
    INT32S VdLast;
} ParkInvertTypedef;  //clark par 反变换
//电流环相关参数
typedef struct
{
    INT16S  DeltaIqNow;
    INT16S  DeltaIdNow;
    INT16S  DeltaIqLast;
    INT16S  DeltaIdLast;
    INT32S  VdKp;
    INT32S  VdKiSum;
    INT32S  VqKp;
    INT32S  VqKiSum;
    INT32S  Vd;  //注意要用32位，不然会溢出，试过的
    INT32S  Vq;
    INT16S  VdOut;  //注意要用32位，不然会溢出，试过的
    INT16S  VqOut;
    INT32S  VdLast;  //注意要用32位，不然会溢出，试过的
    INT32S  VqLast;
    INT32S  Qsum;
    INT32S  Dsum;
    INT32S  QsumMaxP;
    INT32S  QsumMaxN;
    INT32S  DsumMaxP;
    INT32S  DsumMaxN;
    INT32S  VqMaxOutP;  //must plus number
    INT32S  VqMaxOutN;  //must minus number
    INT32S  VdMaxOutP;  //must plus number
    INT32S  VdMaxOutN;  //must minus number
    INT16S  CmdIq;   //Iq command. 2048 means 15A
    INT16S  CmdId;   //Id command. 2048 means 15A
    INT16S  CmdIqFlt;
    INT16S  CmdIqAct;
    INT16S  CmdIdAct;
    INT16U  CmdIqMax;
    INT16S  RealIqPk;
    INT16S  RealIdPk;
    INT16U  Kcp;
    INT16U  Kci;
    INT8U   CalFlg;
} CurrLoopTypedef;
//速度环相关参数
typedef struct
{
    INT32S  SpeedOrgDec;
    INT32S  SpeedNowDec;
    INT32S  SpeedLastDec;
    INT32S  SpeedFilter;
    INT32S  CmdSpeedMaxDec;
    INT32S  CmdSpeedMaxDec1d1;  //for 4 mode limited speed
    INT16S  CmdSpeedMaxRpm;
    INT32S  CmdSpeedDec;
    INT32S  CmdCmdSpeedDec;
    INT32S  *pCmdSpeedDec;
    INT32S  CmdSpeedDecCal;     //由加减速计算所得速度
    INT32S  CmdSpeedDecExt;     //额外速度
    INT32S  CmdSpeedDecAct; //give the speed command from the speed loop.
    INT16S  CmdSpeedRpm;

    INT32S  CmdStepSpeedDecFlt;
    INT32S  CmdStepRemain;
    INT32S  CmdStepFltAdd;
    INT16S  CmdStepSpeedDecInt;
    INT16S  CmdStepSpeedDecAct;  //for the step mode

    INT32S  DeltaSpeedNowDec;
    INT32S  DeltaSpeedLastDec;
    INT32S  SpeedKiFltSum;
    INT32S  SpeedKpiFltSum;
    INT16U  Kvp;
    INT16U  Kvi;
    INT16U  Kvi32;
    INT16U  Kc;
    INT32S  SpeedKpKi;
    INT32S  SpeedKp;
    INT32S  SpeedKiSum;
    INT32S  SpeedKiSum32;
    INT32S  SpeedKiSumOut;
    INT32S  SpeedKiSumLimit;
    INT32S  SpeedKiFltSum32;
    INT32S  SpeedKi32Sum;
    INT8U   SampleCnt;
    INT8U   CalFlg;
    INT32S  SpeedCmdIq;
    INT32S  OutCmdIq;
    INT32S  OutCmdIqLimitP;
    INT32S  OutCmdIqLimitN;
    INT32S  OutCmdIqLast;
    INT32S  DeltaCmdSpeed;      //=CmdSpeedDec - SpeedFilterDec
    INT16U  FbkType;        //0:using SpeedFilter; 1:using SpeedOrg;
} SpeedLoopTypedef;
//extern SpeedLoopTypedef   SpdLp;
//位置环相关变量
typedef struct
{
    INT32S  NowPos;
    INT32S  MgtSnrPos;          //磁编码器位置
    INT32S  MgtSnrPosAbs;       //磁编码器单圈绝对位置
    INT32S  NowPosShadow;
    INT16S  NowAngleInt;
    INT32S  CmdCmdPos;
    INT32S  CmdPos;
    INT32S  CmdDeltaPosExt;
    INT32S  CmdPosAct;
    INT32S  CmdDeltaPosOut;
    INT32S  CmdPosActRemain;    //余数(%16384)
    INT32S  FollowErr;
    INT32U  MaxFollowErr;
    INT32S  OutCmdSpeed;
    INT16S  Kpp;
    INT16U  SampleCnt;
    INT16U  ProfileAcce16;      //简化加速度 2FF006 [rps/s]
    INT16U  ProfileDece16;      //简化减速度 2FF007 [rps/s]
    INT32U  ProfileAcceCmd;     //加速度
    INT32U  ProfileDeceCmd;     //减速度
    INT32U  ProfileAcceAct;     //有效的加速度
    INT32U  ProfileDeceAct;     //有效的减速度
    INT32U  QkStopDeceAct;      //急停减速度
    INT32U  CmdProfileSpeed;    //梯形速度指令
    INT32U  ProfileTn;
    INT32U  ProfileTn_1;
    INT32S  ProfilePcn;
    INT32S  ProfilePcn_1;
    INT32S  RemainPcn;          //剩余距离
    INT32S  ProfileVcn;
    INT32S  ProfileVcn_1;
    INT32S  ProfilePcnFlt;
    INT32U  ProfilePAdece;      //减速段所需的位置
    INT8U   CalFlg;
    INT8U   FollowErrMsg;
    INT8U   UpFlag;
    INT8U   DownFlag;
    INT8U   QkStopTrig;
} PositionLoopTypedef;
//extern PositionLoopTypedef    PosLp;
typedef struct
{
    INT16S  Vr1;
    INT16S  Vr2;
    INT16S  Vr3;
    INT16U  CCRU;
    INT16U  CCRV;
    INT16U  CCRW;
} SVPWM_InfoTypedef;
typedef struct
{
    INT16S  Iu; //current
    INT16S  Iv;
    INT16U  CalSpeedFilterCnt;
    INT16U  CalSpeedFilterInit;
    INT16U  CalSpeedRpmCnt;
    INT32S  SpeedFilterUsePosLast;
    INT32S  SpeedRpmUsePosLast;
    INT16S  SpeedOrgDec;    //unit:inc/0.25ms
    INT32S  SpeedFilterDec;
    INT32S  SpeedFilterOrg;
    INT32S  SpeedFilterNow;
    INT32S  SpeedFilterLast;
    INT16S  RealSpeedRpm;
    INT16S  RealSpeed0Rpm_0d01; //low speed
    INT8U   SpeedLF_N;
    INT16S  SpeedLF_A;
    INT16S  SpeedLF_B;
    INT16S  SpeedLF_C;
    INT16S  SpeedLF_FltLast;
    INT16S  SpeedLF_FltLastLast;
    INT32S  PosOnFirstZ;
    INT16U  FirstZ;
    INT16U  EleAngDec;
    INT16S  EleAngOffsetDec;
    INT32S  EncOffset;
    INT16U  EncOffset16;
    INT32S  PosAbs;
    INT32S  PosAbsLast;
    INT32S  PosAbs32;
    INT32S  PosAbsFlt;
    INT32S  PosMgtAbs;
    INT32S  HallOffet;
    BIT8_STRUCT_DEF   ENC_Hall;
    INT8U   (*pF_R_Hall)(void);
    INT16U  *pComuEncDt1;
    INT16U  *pComuEncDt2;
    INT16U  QEI_CntNow;
    INT16U  QEI_CntLast;
    INT32S  DeltaPos;
    INT16U  ComuEncErrCnt;
    INT8U   EncPwOnInitFlag;    //编码器上电标记
    INT8U   ComuEncErrFlag;
    INT16U  EncPwOnInitCnt;
    INT16U  ComuEncInitErrRcd;  //记录编码器上电初始化错误计数
    INT16U  SetEleAngDec;   //for mode 10
    INT8U   SetMagEncZeroTrig;
} FeedbackTypedef;

typedef struct
{
    INT8U   Group;                  //电机组
    INT16U  Type;                   //电机类型
    INT8U   FB_Type;                //反馈类型
    INT32U  FB_Resolution;          //反馈精度:电机编码器分辨率
    INT32U  FB_Scale;               //反馈尺度变换*10000/10000
    INT32U  FB_Period;              //反馈周期:Z信号检查编码器计数
    INT8U   Poles;                  //电机极对数
    INT8U   ExcitationMode;         //励磁模式
    INT16S  ExcitationCurr;
    INT16U  ExcitationTime;         //励磁时间
    INT16U  IIt_I;                  //电机I2t电流
    INT16U  IItFilter;              //电机过温保护时间
    INT16U  Imax;                   //电机最大电流
    INT16U  L;                      //相电感
    INT16U  R;                      //相电阻
    INT16U  Ke;
    INT16U  Kt;
    INT16U  Jr;
    INT16U  BrakeDutyCycle;
    INT16U  BrakeDelay;
    INT8U   RotateDir;
    INT16U  CurrBW;
    INT16U  UsingType;
    INT8U   WithBrake;
    INT16U  *pTemp;                 //温度数据指针
    INT16S  Temp;                   //电机温度
} MotorParameterTypedef;

//IIt参数结构体定义
typedef struct
{
    INT8U  EnableFlag;      //IIt开启标志位:0-关闭,1-开启
    INT16U UserSetI;        //用户设置电流参数
    INT16U UserSetT;        //用户设置时间参数
    INT32S UserSetII_DEC;   //MCU内部II DEC值,由UserSetI转换而来
    INT32S UserSetT_DEC;    //MCU内部T DEC值,由UserSetT转换而来
    INT16U RealIIt;         //容易读懂的值,单位为mA
    INT16U RealIItDEC_D512; //配合Kinco servo OD公式转换
    INT32S RealIItDEC_D;    //MCU内部处理用的II值商数部分
    INT32S RealIItDEC_R;    //MCU内部处理用的II值余数部分
} IItParameterTypedef;

//Drive common information
typedef struct
{
    INT16U  DC_BusUnder;    //母线低压报警点 unit：V
    INT16U  DC_BusUnderDec; //母线低压报警点 unit：dec
    INT16U  DC_BusOver;     //母线过压报警点 unit：V
    INT16U  DC_BusOverDec;  //母线过压报警点 unit：dec
    INT16U  ChopVolt;       //制动电压点 unit：V
    INT16U  ChopVoltDec;    //制动电压点 unit：dec
    INT16U  ChopVoltOffDec; //制动关闭电压点 unit：dec
    INT16S  TempErrPoint;   //温度报警点
    INT16U  TempErrPointDec;

    INT16S  RealDcBus;          //实际总线电压，单位：V
    INT16S  TempDevice;         //驱动器温度，单位：摄氏度
    INT16S  TempDeviceOffset;   //驱动器温度偏移，单位：摄氏度

    INT8U   QkStopMode;         //急停模式
}DriverComInfoTypedef;

//Drive information
typedef struct
{
    INT16U  I_Max;

    INT16U  StateWord;          //状态字 604100

    INT16U* TempDtPtr;       //温度数据指针
    INT16S  TempErrPoint;   //温度报警点
    INT16U  TempErrPointDec;
    INT16S  TempDevice;         //驱动器温度，单位：摄氏度
    INT16S  TempDeviceOffset;   //驱动器温度偏移，单位：摄氏度

}DriverInfoTypedef;
//全局标记
typedef struct
{
    INT8U   Excitation;         //励磁标志位
    INT8S   CtrlMode;           //控制模式
    INT8S   CtrlModeAct;        //有效控制模式
    INT8U   AutoSwitchOn;
    INT16U  CtrlWord;           //控制字
    INT16U  CtrlWordAct;        //有效控制字
    INT16U  RdMgtSnrEn;         //使能/关闭读写磁编码器
    INT32U  ExciCnt;
    INT16U  ErrState1;          //错误状态字1
    INT16U  ErrState2;          //错误状态字2
    INT16U  ErrAct1;            //真实错误状态字1
    INT16U  ErrAct2;            //真实错误状态字2
    INT16U  ErrMask1;
    INT16U  ErrMask2;
    INT16U  ErrPending1;
    INT16U  ErrProcess1;        //
    INT16U  ErrProcess2;        //
    INT16U  PosLpTimeBaseCnt;
    INT16U  SpdLpTimeBaseCnt;
    INT8U   Choping;            //制动标志位
    INT8U   StrRdMgtSnr;        //开始读磁编码器状态
    INT8U   QkStopDin;          //Din急停标记
    INT8U   QkStopCmd;          //急停指令
    INT8U   QkStopAct;          //急停
    INT8U   QkStopMode;         //急停模式
    INT16U  TIM1_CntSyn;        //debug
    INT16U  TIM8_CntSyn;        //debug
} GlobleFlagTypedef;

//../////////////////////////////////////////////////////////////////////
typedef struct
{
    AnalogGroupBaseTypedef TempBase;
    DriverInfoTypedef      DriveInfo;
    GlobleFlagTypedef      GbFlg;

    AnalogGroupBaseTypedef CurU;
    AnalogGroupBaseTypedef CurV;
    AnalogGroupBaseTypedef CurIsum;
    FeedbackTypedef        Fbk;

    ClarkTypedef           Clark;   //clark正变换
    ParkTypedef            Park;   //park 正变换
    ParkInvertTypedef      ParkInvt;//clark par 反变换

    CurrLoopTypedef        CurLp;//电流环相关参数
    SpeedLoopTypedef       SpdLp;//速度环相关参数
    PositionLoopTypedef    PosLp;//位置环相关变量

    SVPWM_InfoTypedef      SVPWM_Info;

    IItParameterTypedef    MotorIIt;    //motor current IIt
    IItParameterTypedef    DriveIIt;    //drive current IIt
    //IItParameterTypedef BrakeIIt;    //Brake current IIt
    MotorParameterTypedef  MotorPar;

    TIM_TypeDef*           PWM_TIMx;
    TIM_TypeDef*           QEI_TIMx;

    void (*pF_DriverOn)(void);
    void (*pF_DriverOff)(void);
    void (*pF_EncPwrOn)(void);
    void (*pF_EncPwrOff)(void);
    void (*pF_DriverErrLedOn)(void);
    void (*pF_DriverErrLedOff)(void);

}DriverStructTypedef;
extern DriverStructTypedef DX[2];

typedef union
{
    INT16U All;
    struct
    {
        INT16U DIN1: 1;
        INT16U DIN2: 1;
        INT16U DIN3: 1;
        INT16U DIN4: 1;
        INT16U DIN5: 1;
        INT16U DIN6: 1;
        INT16U DIN7: 1;
        INT16U DIN8: 1;
        INT16U DIN9: 1;
        INT16U DIN10: 1;
        INT16U DIN11: 1;
        INT16U DIN12: 1;
        INT16U DIN13: 1;
        INT16U DIN14: 1;
        INT16U DIN15: 1;
        INT16U DIN16: 1;
    } Bit;
} DIN_STRUCT_DEF;

typedef struct
{
    DIN_STRUCT_DEF State;
    INT16U  Simulate;
    INT16U  Polarity;
    INT16U  Virtual;    //State|Simulate
    INT16U  Sys;        //功能是否被激活标记
    INT16U  FuncIndex[8];
    INT8U   NowNum;
}DinGroupTypedef;

//Dout process define
typedef struct
{
    INT16U  Mask;
    INT16U  State;          //物理输出状态
    INT16U  Simulate;       //输出口模拟
    INT16U  Polarity;       //极性
    INT16U  Sys;            //功能是否被激活标记
    INT16U  Virtual;        //内部虚拟输出,对应各个定义的功能输出
    INT16U* pDoutData;      //实际输出指针,指向SPI缓冲区
    INT8U   SingleSim[8];   //单个输出口模拟
}DoutProcessTypedef;


typedef struct
{
    INT16U* pCntOrg;
    INT16U  CntNow;
    INT16U  CntLast;
    //INT32S  AllPulse;
    UNION32_DATA_DEF AllPulse;
}PulseCountTypedef;

//DC motor control typedef
typedef struct
{
    INT16U  Speed[4];
    INT16U  CtrlWord;
    INT16U  AdcOrgLast;
    INT8U   EnableFlag;     //DC motor 模块使能(存在)标记位
    INT8U   SpeedSelect;    //0:0速; 1:Speed[1]; 2:Speed[2]; 3:Speed[3]
    INT8U   Direction;      //0:不动; 1:上升; 2:下降
    INT8U   PwmOutXorNow;
    INT8U   PwmOutXorLast;
    INT8U   OnTrig;         //开关触发
}DcMotorCtrlTypedef;

//DXC GbFlg typedef
typedef struct
{
    INT16U  ErrState1;        //错误状态字1
    INT16U  ErrState2;        //错误状态字2
    INT16U  ErrAct1;          //错误状态字1
    INT16U  ErrAct2;          //错误状态字2
    INT16U  ErrMask1;
    INT16U  ErrMask2;
    INT16U  ErrPending1;
    INT16U  ErrProcess1;
    INT16U  ErrProcess2;
}DxcGbFlgTypedef;

typedef struct
{
    PulseCountTypedef Pos;
    INT16S  DownLimitMm;
    INT32S  DownLimitDec;
    INT16S  UpLimitMm;
    INT32S  UpLimitDec;
    INT32S  HighActDec;
    INT16S  HighActMm;        //y=k*(EncCnt-EncCntOffset) + MechOffsetMm
    INT32S  EncCntOffset;     //编码器偏移
    INT16S  MechOffsetMm;     //机械结构偏移
    INT16S  Kn;
    INT16S  Kd;
    INT16S  B;
    INT16S  X1;
    INT16S  Y1;
    INT16S  X2;
    INT16S  Y2;
    INT32U  CalOverTimeCnt;     //校准超时计数
    INT8U   CalCmd;
    INT8U   OriginDin;        //原点信号
    INT8U   UpLimitFlag;
    INT8U   DownLimitFlag;
}PlatformInfoTypedef;

typedef struct
{
    INT16U Enable;  //使能
    INT16U CfftA;   //滤波系数
    INT16U Rn;      //输入
    INT16U Cn;      //filter out
    INT16U CnRemain;//余数
}LowPass1FilterTypedef;

typedef struct
{
    LowPass1FilterTypedef Lp1f;
    AnalogGroupBaseTypedef AiBase;
}AnalogProcessTypedef;


typedef struct
{
    DriverComInfoTypedef ComInfo;
    AnalogGroupBaseTypedef CurrBk;
    AnalogGroupBaseTypedef DC_BusVolt;
    AnalogGroupBaseTypedef TempBase;
    AnalogGroupBaseTypedef CurrDm;
    IItParameterTypedef BrakeIIt;    //Brake current IIt
    IItParameterTypedef DM_IIt;    //DC motor current IIt
    DcMotorCtrlTypedef DM_Ctrl;
    DinGroupTypedef Din;
    DoutProcessTypedef Dout;
    //PulseCountTypedef PlatformPos;
    PlatformInfoTypedef PlatformInfo;
    PulseCountTypedef PulseInfo[2];
    AnalogProcessTypedef ExtAi[4];
    DxcGbFlgTypedef GbFlg;
}DriverStructComTypedef;
extern DriverStructComTypedef DXC;


//calculate the real current unit:dec
extern void CalAnalogRealDec(AnalogGroupBaseTypedef* pAlgGrpCurr);
//Clark transform
extern void ClarkTransform(ClarkTypedef* pClark, FeedbackTypedef* pFbk);
//Park tansform
//eAngle do not big than SINE_TABLE_NUM
//extern void ParkTansform(ParkTypedef* pPark, ClarkTypedef* pClark, INT16U eAngle);
extern void ParkTansform(ParkTypedef* pPark, ClarkTypedef* pClark, FeedbackTypedef* pFbk);
//inverse park transformation
extern void ParkInvtTansform(ParkInvertTypedef* pParkInvt,ParkTypedef* pPark,CurrLoopTypedef* pCurLp);
//7 segment SVPWM generator
extern ResultStatus Gen7SegSVPWM(SVPWM_InfoTypedef* pSVPWM_Info, ParkInvertTypedef* pParkInvt);
//PWM 输出比较值更新
extern void PWM_ValueUpdata(TIM_TypeDef* TIMx, SVPWM_InfoTypedef* pSVPWM_Info);
//get the position increment from ABZ encoder
extern void GetPositionIncrementABZ(FeedbackTypedef* pFbk, TIM_TypeDef* TIMx);
//get the pulse count
extern void GetPulseCount(PulseCountTypedef* pPCnt);
extern void GetPositionComu(FeedbackTypedef* pFbk);
//calculate the eletronic angle
extern void CalEletronicAngleDec(FeedbackTypedef* pFbk, MotorParameterTypedef* pMotorPar);
//calculate speed via 2 orde low pass filter
extern void CalFilterSpeed(FeedbackTypedef* pFbk);
//calculate speed unit:RPM
extern void CalSpeedRPM(FeedbackTypedef* pFbk, INT32S FB_Resolution);
//current loop PI adujust
extern void CurrentLoopPI(CurrLoopTypedef* pCurLp, ParkTypedef* pPark);
//speed loop PI adjust
extern void SpeedLoopPI(SpeedLoopTypedef* pSpdLp, FeedbackTypedef* pFbk, INT16U OutLimit);
//position loop P adjust
extern void PositionLoopP(PositionLoopTypedef* pPosLp, FeedbackTypedef* pFbk, INT32U OutLimit);
//calculate the IIt(DEC)
extern void CalIItDec(IItParameterTypedef* pIItPar, INT16S RealCurDec);

extern INT8U ReadEncHall_1_State(void);
extern INT8U ReadEncHall_2_State(void);
//clear filter speed data at encoder error
extern void ClrEncErrData(FeedbackTypedef* pFbk, TIM_TypeDef* TIMx);
//limited the max speed
extern void LimitedMaxSpeed(SpeedLoopTypedef* pSpdLp);
//quick stop
extern void QuiekStop(DriverStructTypedef* pDX);
//clear position loop
extern void ClrPosLoopPI(PositionLoopTypedef* pPosLp);
//clear speed loop PI value
extern void ClrSpeedLoopPI(SpeedLoopTypedef* pSpdLp);
//clear current loop PI value
extern void ClrCurrLoopPI(CurrLoopTypedef* pCurLp);
//driver 1 on / off
extern void Driver1_On(void);
extern void Driver1_Off(void);
//driver 2 on / off
extern void Driver2_On(void);
extern void Driver2_Off(void);

//driver enc 1 power on / off
extern void Enc1_PowerOn(void);
extern void Enc1_PowerOff(void);
//driver enc 2 power on / off
extern void Enc2_PowerOn(void);
extern void Enc2_PowerOff(void);
//driver 2 error led display
extern void Driver1_ErrLedOn(void);
extern void Driver1_ErrLedOff(void);
//driver 2 error led display
extern void Driver2_ErrLedOn(void);
extern void Driver2_ErrLedOff(void);

extern void GetMotorTemperature(DriverStructTypedef* pDX);
extern void RealIqPeakHold(DriverStructTypedef* pDX);
extern void ExcitationFaultProcess(DriverStructTypedef* pDX);

extern void LowPass1FilterProcess(AnalogProcessTypedef* pAi);
#endif
