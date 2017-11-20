/*
 * SR2D1_0V3_MCU_S1 -- Service robot 2In1 servo driver
 * (C)2016 Kinco Electric (Shenzhen) Ltd. -- All rights reserved
 */
/***************************************************
     本头文件定义那些会被多个文件调用的结构体,变量声明
****************************************************/
#ifndef _OD_LIB_H
#define _OD_LIB_H
#define FIRWARE_UPDATA_DATE     20150927

#define MASTER      0X01
#define SLAVER      0X02
#define NOW_MODE    SLAVER

#define NULL        0

#define ODT08U      0x01            //无符号字符型
#define ODT08S      0x81            //有符号字符型
#define ODT16U      0x02            //无符号整型
#define ODT16S      0x82            //有符号整型
#define ODT32U      0x04            //无符号长整型
#define ODT32S      0x84            //有符号长整型

#define BYTE1       0x01            //1个字节
#define BYTE2       0x02            //2个字节
#define BYTE4       0x04            //4个字节

#define ID_DOMAIN   0x000F
#define ID1         0x0001          //ID1
#define ID2         0x0002          //ID2
#define ID3         0x0003          //ID3
#define ID4         0x0004          //ID4
#define IDX         0x000F          //IDX
#define RO          0x0010          //只读
#define WO          0x0020          //只写
#define RW          0x0030          //可读写
/*
#define S1          0x0100          //保存类型1
#define S2          0x0200          //保存类型2
#define S4          0x0400          //保存类型3
#define S8          0x0800          //保存类型4
*/
#define SD_DOMAIN   0x0F00
#define S1          0x0100          //公共信息保存(通信接口设置等)
#define S2          0x0200          //接口配置保存(IO口功能配置,极性等)
#define S3          0x0300          //每轴驱动器校准参数保存
#define S4          0x0400          //每轴电机参数保存
#define S5          0x0500          //每轴控制环参数保存(模式,目标速度,Kpp,Kvp等)
#define S6          0x0600          //协同运动参数保存
#define S7          0x0700          //舵机参数保存
#define S8          0x0800          //平台参数保存
                                    //
#define IL          0X8000          //上电初始化
#define IFRW        0X4000          //上电执行一次读写函数(如果有)
#define SCL         0X2000          //变尺度变量标识
#define CDPC_O      0x1000          //表征变量回帧时候只使用CDPC格式

#define D1          0
#define D2          1

#define SERIOUS_ERR 0X40
#define NORMAL_ERR  0X00
#define KTP_PC      0X01
#define CD_PC       0X02
#define EEPROM_WRITE  0X10
#define EEPROM_READ   0X11

#define OD_DUP_ERR_CODE     0x0001

typedef struct
{
    /*  Index...Sub...DataType...OD_Point...Property...Max.....Min...DefVal...EromAddress...bOD_Print...rOD_Print...Command...*pF_R.....*pF_W...*/
    INT16U  Index;          //主索引
    INT8U   SubIndex;       //子索引
    INT8U   DataType;       //数据类型,表征变量数据类型
    void    *pOD;           //指向变量指针
    INT16U  Property;       //OD属性,指示初始化,保存,读,写,ID等
    INT32S  Max;            //上限值
    INT32S  Min;            //下限值
    INT32S  DefVal;         //默认值,一般是针对需要保存的参数才有默认值之说
    INT16U  EromAddress;    //存储,初始化读取EPPROM地址
    void    *bpOD;          //返回参数
    void    *rpOD;          //OD的关联参数,比如说设定电压:pOD是设定值,rpOD就是实际已设定到的值.
    //INT16U  *spOD;          //变尺度变量(10000),必须是16bit无符号变量
    INT16U  Command;        //写地址或指令,数据就是OD本身
    INT8U   (*pF_R)(INT16U Command,void *pOD, void *bpOD, void *rpOD);  //读函数,将读到结果赋给*pOD,*rpOD一般是做相应转换后的值(比如AD转换后对应实际电压值mV)
    INT8U   (*pF_W)(INT16U Command,void *pOD, void *bpOD, void *rpOD);  //写或执行函数(带有执行完后值传递bpOD,rpOD只是相关联,和错误状态返回)
} OD_Attribute;
typedef union
{
    INT8U All[10];          //一帧数据的缓冲区
    struct All_Split_Struct
    {
        INT8U ID;
        INT8U CMD;
        INT8U IndexL;
        INT8U IndexH;
        INT8U SubIndex;
        INT8U Data1;
        INT8U Data2;
        INT8U Data3;
        INT8U Data4;
        INT8U Check;
    } Split;
} SDO_BUFF_DEF;
typedef union
{
    INT32U  *pInt32u;
    INT32S  *pInt32s;
    INT16U  *pInt16u;
    INT16S  *pInt16s;
    INT8U   *pInt8u;
    INT8S   *pInt8s;
} UNION_OD_DATA_TYPE_DEF;
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
    struct  SplitStruct
    {
        INT16U Low16u;
        INT16U High16u;
    } Split;
} UNION_DATA_STRUCT_DEF;

typedef union
{
    INT64U  Int64u;
    INT64S  Int64s;
} UNION_DATA64_STRUCT_DEF;

typedef union
{
    INT8U All;
    struct
    {
        INT8U bit0: 1;
        INT8U bit1: 1;
        INT8U bit2: 1;
        INT8U bit3: 1;
        INT8U bit4: 1;
        INT8U bit5: 1;
        INT8U bit6: 1;
        INT8U bit7: 1;
    } Bit;
} BIT8_STRUCT_DEF;

typedef union
{
    INT16U All;
    struct
    {
        INT16U bit0: 1;
        INT16U bit1: 1;
        INT16U bit2: 1;
        INT16U bit3: 1;
        INT16U bit4: 1;
        INT16U bit5: 1;
        INT16U bit6: 1;
        INT16U bit7: 1;
        INT16U bit8: 1;
        INT16U bit9: 1;
        INT16U bit10: 1;
        INT16U bit11: 1;
        INT16U bit12: 1;
        INT16U bit13: 1;
        INT16U bit14: 1;
        INT16U bit15: 1;
    } Bit;
} BIT16_STRUCT_DEF;

typedef union
{
    INT32U All;
    struct
    {
        INT32U bit0: 1;
        INT32U bit1: 1;
        INT32U bit2: 1;
        INT32U bit3: 1;
        INT32U bit4: 1;
        INT32U bit5: 1;
        INT32U bit6: 1;
        INT32U bit7: 1;
        INT32U bit8: 1;
        INT32U bit9: 1;
        INT32U bit10: 1;
        INT32U bit11: 1;
        INT32U bit12: 1;
        INT32U bit13: 1;
        INT32U bit14: 1;
        INT32U bit15: 1;
        INT32U bit16: 1;
        INT32U bit17: 1;
        INT32U bit18: 1;
        INT32U bit19: 1;
        INT32U bit20: 1;
        INT32U bit21: 1;
        INT32U bit22: 1;
        INT32U bit23: 1;
        INT32U bit24: 1;
        INT32U bit25: 1;
        INT32U bit26: 1;
        INT32U bit27: 1;
        INT32U bit28: 1;
        INT32U bit29: 1;
        INT32U bit30: 1;
        INT32U bit31: 1;
    } Bit;
} BIT32_STRUCT_DEF;

typedef struct
{
    //INT8U NewErrFlag; //有新错误标记(标记是否被收录到历史错误中,1:还没有; 0:已收录并保存)
    INT8U NewErr;       //系统新错误状态
    INT8U LastErr[10];  //历史错误
} SYS_ERR_DEF;

//EEPROM
typedef struct
{
    INT16U StartAddr;   //读写的起始地址
    INT16U NumBytesRW;  //读写字节数
    INT16U CommandRW;   //读写指令
    INT8U DataBuff[32]; //读写数据缓冲区
} EEPROM_RW_DEF;

//本机属性相关参数
typedef struct
{
    INT8U   MyID;               //本机ID号
    INT8U   MyID_Reg;           //本机ID，可被修改的
    INT8U   NowMode;            //通讯传输模式
    INT8U   NowModeReg;         //通讯传输模式，可被修改的
    INT8U   RS232_1_Baudrate;   //now is uart3
    INT8U   RS232_2_Baudrate;   //now is uart1
    INT16U  OD_Sum;             //总OD个数
    INT32U  FirmwareVersion;    //固件版本日期
    INT32U  HardwareVersion;    //硬件版本 eg：H2V2
} LOCAL_PROPERTY_INFO_DEF;

#define APP_TIME_BASE_NUM   10

//save group define
typedef struct
{
    INT8U   SaveComInfoCmd;     //S1
    INT8U   SaveIoCfgCmd;       //S2
    INT8U   SaveDeviceCalibCmd; //S3
    INT8U   SaveMotorPraCmd;    //S4存储电机参数
    INT8U   SaveCtrLpPraCmd;    //S5存储控制环参数
    INT8U   SaveMotionCmd;      //S6
    INT8U   SaveStEngCmd;       //S7
    INT8U   SavePlatformCmd;    //S8
}DeviceSaveTypedef;

//radar data define
typedef struct
{
    union{
            INT32U  All;
            struct
            {
                INT8U Channel4;
                INT8U Channel3;
                INT8U Channel2;
                INT8U Channel1;
            }Split;
        }Data1;
    union{
            INT32U  All;
            struct
            {
                INT8U Channel8;
                INT8U Channel7;
                INT8U Channel6;
                INT8U Channel5;
            }Split;
        }Data2;

    INT8U FrameBytes;
    INT8U FrameHead1;
    INT8U FrameHead2;
    INT8U FrameProperty;
    INT8U FrameTail;
}RadarTypedef;

//steering engine control define
typedef struct
{
    INT8U   BitCtrl;        //舵机松紧控制;1:舵机锁住电机轴; 0:松开电机轴
    INT16U  LockVal;        //锁住值定义
    INT16U  UnLockVal;      //松开值定义
    INT16U* pValData;      //实际输出指针2,指向SPI缓冲区
}SteEngTypedef;

//AGV sensor signal input define
typedef struct
{
   union{
       INT16U  All;
       struct
        {
            INT8S Low8;
            INT8S High8;
        }Split;
    }OrgData;

    union{
        INT16U  All;
        struct
         {
            INT8S Low8;
            INT8S High8;
         }Split;
     }InverData;

    INT16U* pValData;       //指向SPI缓冲区
}AgvSensorTypedef;

//系统服务信息相关变量(标志位)
typedef struct
{
    INT16U  SaveFlag;           //保持标志位
    INT16U  SoftRestartFlag;    //软件重启标志位：0xaa55；
    INT16U  LoadDefVal;         //装载默认定义的参数,0xD100,0xD200,0xD400,0XD800
    INT16U  ErrClearFlag;       //错误复位标志位:0x86
    union App_Time_Base_Union   //应用事件时基计数结构体
    {
        INT16S Value[APP_TIME_BASE_NUM];
        struct App_Time_Base_Split
        {
            INT16U Led;
            INT16U Key;
            INT16U TotalRun;
            INT16U Cal;
            INT16U ErrCnt;
            INT16U ErrStep;
            INT16U ErrStart;
            INT16U LB_Cnt;
            INT16U LB;
        } Split;
    } AppTimeBaseCnt;

    //led和蜂鸣器控制状态状态机参数定义
    INT8U LB_NowState;
    INT16U LB_Ctl;

    //通信信息
    struct
    {
        INT8U   Uart3RxFnCnt;
        INT8U   Uart2RxFnCnt;
        INT8U   Uart1RxFnCnt;
        INT8U   CAN2_RxFnCnt;
        INT8U   Uart1RxTmOvCnt; //uart1 rx timeover count
        INT8U   Uart1RxTmOvDef; //uart1 rx timeover define
        INT8U   Uart3RxTmOvCnt; //uart3 rx timeover count
        INT8U   Uart3RxTmOvDef; //uart3 rx timeover define
    } ComMsg;
    //雷达数据
    RadarTypedef Radar;
    //舵机
    SteEngTypedef   SteEng[2];
    //AGV sensor
    AgvSensorTypedef AgvSnr;
    //存储
    DeviceSaveTypedef DS;
    //直流电机
    //DcMotorCtrlTypedef DM_Ctrl;
} SYS_SERVICE_INFO_DEF;

//系统状况相关变量
typedef struct
{
    INT8U   CPU_Use;            //CPU使用率
    INT8U   PowerDownMsg;       //
    INT8U   ErrorMsg;
    INT8U   HistoryErrPos;      //历史错误位置

    INT32U  ErrFlag;            //系统错误标记位
    INT32U    ErrFlagReg;       //系统错误标记位暂存
    INT32U  ErrCutoffMask;      //错误发生时候切断屏蔽位
    INT32U  ErrAlarmMask;       //错误发生时候声光报警屏蔽位
    INT16U  OD_DupNum;          //OD地址重复个数
    INT32U  OD_DupInfo[5];      //重复的OD信息,这里只给头5个记录空间
    INT16U  ActiveId;
    INT16U  ActiveIdReg;
    INT16U  ActiveIdSpc;        //search id for oscilloscope
    INT16U  IdSpcManualSet;     //if equal to 1,do not auto updata ActiveIdSpc

    INT16U  RelCntLockFlag;     //继电器控制时锁信号

    struct
    {
        union TotalOnlineTimeUnion
        {
            INT32U Total;       //总在线时间,单位:[S]
            INT8U  Spilt[4];
        } Total;
        INT16U Days;
        INT8U  Hours;
        INT8U  Minutes;
        INT8U  Seconds;
    } OnlineTime;

    union RedayUnion
    {
        INT32U All;
        struct
        {
            INT8U  PwrOnCalthFlg;   //上电Offset校准完毕标志位
            INT8U  SelfCheckFlg;    //上电自检结束标志位
        } Split;
    } Ready;

} SYS_STATE_INFO_DEF;

typedef struct
{
    GPIO_TypeDef  *DirGPIO;
    uint16_t      DirPinNum;
} DIRGPIO_STRUCT_DEF;


//历史错误记录结构体
#define HISTORY_ERR_NUM     8
typedef struct
{
    INT8U   Index;
    INT32U  ErrTime;
    INT32U  ErrFlag;
} HISTORY_ERR_DEF;

/*=====================================================================================================*/
typedef struct
{
    INT16U  OptEn;              //磁编码器操作使能
    INT16U  OptEnCmd;           //磁编码器操作使能
    INT8U   OptCnt;             //磁编码器操作计数器
    INT8U   Flag;               //标记

    INT8U   SpiTxNumFrm;        //spi发送帧数量
    INT8U   SpiRxNumFrm;        //spi接收帧数量
    INT16U  StrTrigComp;        //触发磁编码器发送命令时刻值
    INT16U  GetDataComp;        //获取磁编码器数据时刻值
    INT16U  SpiTxBuff[4];       //SPI发送缓冲器
    INT16U  SpiRxBuff[4];       //SPI接收缓冲器

    //每使能一次,操作一次的参数
    INT8U   OT_SpiFlg;          //标记是否是EEPROM类命令
    INT8U   OT_SpiTxOffset;     //发送偏移
    INT32U  OT_SpiCmdSlt;       //CDF选择
    INT8U   OT_SpiTxNumFrm;     //spi发送帧数量
    INT8U   OT_SpiRxNumFrm;     //spi接收帧数量
    INT16U  OT_StrTrigComp;     //触发磁编码器发送命令时刻值
    INT16U  OT_GetDataComp;     //获取磁编码器数据时刻值
    INT16U  OT_SpiTxBuff[4];    //SPI发送缓冲器
    INT16U  OT_SpiRxBuff[4];    //SPI接收缓冲器

    //eeprom
    INT8U   SpiTxMdt7_0;
    INT8U   SpiTxMdt15_8;
    INT8U   SpiTxMaddr7_0;

    INT8U   SpiRxMdt7_0;
    INT8U   SpiRxMdt15_8;
    INT8U   SpiRxMaddr7_0;

} MagnetSensorTypedef;
extern MagnetSensorTypedef MgtSnr;
typedef struct
{
    INT32U  CDF_Slt;
    INT16U  CDFx;
    INT16U  MDF0;
    INT16U  MDF1;
    INT16U  MDF2;
    INT8U   TxNumFrm;
    INT8U   RxNumFrm;
    INT16U  StrTrigComp;
    INT16U  GetDataComp;
    INT8U   SpiFlg;
} GetNickEncCmdTypedef;
extern const GetNickEncCmdTypedef       GNE_TB[];
/*=====================================================================================================*/
//通信方面相关变量
extern SDO_BUFF_DEF SdoBuf;
//本机属性相关变量定义
extern SYS_STATE_INFO_DEF   SysState;       //系统状态相关变量
extern SYS_SERVICE_INFO_DEF   SysService;     //系统服务相关变量
extern LOCAL_PROPERTY_INFO_DEF  LocalProperty;  //本机属性相关
extern SYS_ERR_DEF SysError;
extern EEPROM_RW_DEF EepromRW;
//OD列表变量
extern const OD_Attribute OD_ADD[];
//历史错误相关变量
extern HISTORY_ERR_DEF  HistoryErr[HISTORY_ERR_NUM];
//其他
extern UNION_DATA_STRUCT_DEF    SPI_TxData;
extern UNION_DATA_STRUCT_DEF    SPI_RxData;
extern const OD_Attribute* FindOd(INT16U ActiveId, INT32U OdAddress);
extern void NewProcessOD(INT16U ActiveId);
extern void ErrRecordImmediately(INT8U new_err);
extern void PowerOnOD_Init(void);
extern void OD_DataSave(INT8U save_id, INT16U save_type);
extern INT32U GetCompileDate(void);
extern void SDO_BlockProcess(INT32U SDO_BlockIndex);
#endif
