/*
 * SR2D1_0V3_MCU_S1 -- Service robot 2In1 servo driver
 * (C)2016 Kinco Electric (Shenzhen) Ltd. -- All rights reserved
 */
#include "includes.h"
//本机属性相关变量定义
SYS_STATE_INFO_DEF    SysState;       //系统状态相关变量
//系统服务相关变量
SYS_SERVICE_INFO_DEF    SysService =
{
    .ComMsg.Uart1RxTmOvDef = 10,
    .ComMsg.Uart3RxTmOvDef = 30,
    .SteEng[0].pValData = &SPI3_DMA_TD_Buff2[12],
    .SteEng[1].pValData = &SPI3_DMA_TD_Buff2[13],
    .AgvSnr.pValData = &SPI3_DMA_RD_Buff2[8],
};
LOCAL_PROPERTY_INFO_DEF   LocalProperty;  //本机属性相关
SYS_ERR_DEF SysError;
//通信方面相关变量
SDO_BUFF_DEF SdoBuf;       //定义接收缓冲区预处理
//eeprom相关信息,调试用而已
EEPROM_RW_DEF EepromRW;
//历史错误相关变量
HISTORY_ERR_DEF HistoryErr[HISTORY_ERR_NUM];
//其他
UNION_DATA_STRUCT_DEF   SPI_TxData;
UNION_DATA_STRUCT_DEF   SPI_RxData;
//函数声明

/*
typedef struct {
    //  Index...Sub...DataType...OD_Point...Property...Max.....Min...DefVal...EromAddress...bOD_Print...rOD_Print...Command...*pF_R.....*pF_W...//
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
    INT16U  Command;        //写地址或指令,数据就是OD本身
    INT8U   (*pF_R)(INT16U Command,void *pOD, void *bpOD, void *rpOD);  //读函数,将读到结果赋给*pOD,*rpOD一般是做相应转换后的值(比如AD转换后对应实际电压值mV)
    INT8U   (*pF_W)(INT16U Command,void *pOD, void *bpOD, void *rpOD);  //写或执行函数(带有执行完后值传递bpOD,rpOD只是相关联,和错误状态返回)
}OD_Attribute;*/
const OD_Attribute OD_ADD[] =
{
   /*Index.....Sub....DataType...OD_Point..............................Property....... Max.........Min.........DefVal......EromAddres......BackOD..........................rOD_Print.....................Command...*pF_R...........................*pF_W......................*/
    {0x2500,    0x01,   ODT08U, &LocalProperty.MyID,                    RO|IDX|IL,      0x7F,       0x00,       0x61,             16  ,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x100b,    0x00,   ODT08U, &LocalProperty.MyID_Reg,                RW|IDX|S1|IL,   0x7F,       0x00,       0x61,             16  ,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x2500,    0x03,   ODT08U, &LocalProperty.NowMode,                 RW|IDX|IL|CDPC_O,0x02,      0x01,       0X02,             18  ,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x2500,    0x04,   ODT08U, &LocalProperty.NowModeReg,              RW|IDX|S1|IL,   0x02,       0x01,       0x02,             18  ,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x2FE0,    0x01,   ODT08U, &LocalProperty.RS232_1_Baudrate,        RW|IDX|S1|IL,   0x07,       0x00,       0x07,             20  ,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x2FE0,    0x02,   ODT08U, &LocalProperty.RS232_2_Baudrate,        RW|IDX|S1|IL,   0x07,       0x00,       0x01,             21  ,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x2500,    0x5A,   ODT16U, &SysService.SaveFlag,                   RW,             0xFFFF,     0X00,       NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x2FFF,    0x00,   ODT16U, &SysService.SoftRestartFlag,            RW|IDX,         0xFFFF,     0x00,       NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x2FE5,    0x00,   ODT16U, &SysService.LoadDefVal,                 RW|IDX,         0xFFFF,     0x00,       NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           wDefaultValueLoad           },
    {0x2500,    0xF1,   ODT32U, &LocalProperty.FirmwareVersion,         RO|IDX,         0xFFFF,     0X00,       NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x2500,    0xF2,   ODT16U, &LocalProperty.OD_Sum,                  RO|IDX,         0xFFFF,     0X01,       NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x2500,    0xF3,   ODT32U, &LocalProperty.HardwareVersion,         RW|IDX|S1|IL,   0xFFFFFFFF, 0X00,       '2V0H',           24  ,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },

    {0x2530,    0x00,   ODT32U, &SysState.OnlineTime.Total.Total,       RO|IDX|IL,      0xffffffff, 0x00,       NULL,             32  ,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x2530,    0x01,   ODT16U, &SysState.OnlineTime.Days,              RO|IDX,         0xffff,     0x00,       NULL,             NULL,     NULL,                           NULL,                           0,      rOnlineTimeInfo,                NULL                        },
    {0x2530,    0x02,   ODT08U, &SysState.OnlineTime.Hours,             RO|IDX,         0xff,       0x00,       NULL,             NULL,     NULL,                           NULL,                           1,      rOnlineTimeInfo,                NULL                        },
    {0x2530,    0x03,   ODT08U, &SysState.OnlineTime.Minutes,           RO|IDX,         0xff,       0x00,       NULL,             NULL,     NULL,                           NULL,                           2,      rOnlineTimeInfo,                NULL                        },
    {0x2530,    0x04,   ODT08U, &SysState.OnlineTime.Seconds,           RO|IDX,         0xff,       0x00,       NULL,             NULL,     NULL,                           NULL,                           3,      rOnlineTimeInfo,                NULL                        },
    {0x2531,    0x00,   ODT16U, &SysState.OD_DupNum,                    RO|IDX,         0xffff,     0x00,       NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x2531,    0x01,   ODT32U, &SysState.OD_DupInfo[0],                RO|IDX,         0xffffffff, 0x00,       NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x2531,    0x02,   ODT32U, &SysState.OD_DupInfo[1],                RO|IDX,         0xffffffff, 0x00,       NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x2531,    0x03,   ODT32U, &SysState.OD_DupInfo[2],                RO|IDX,         0xffffffff, 0x00,       NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x2531,    0x04,   ODT32U, &SysState.OD_DupInfo[3],                RO|IDX,         0xffffffff, 0x00,       NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x2531,    0x05,   ODT32U, &SysState.OD_DupInfo[4],                RO|IDX,         0xffffffff, 0x00,       NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x2532,    0x00,   ODT32U, &SysState.ErrFlag,                      RW|IDX,         0xffffffff, 0x00,       NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x2532,    0x01,   ODT32U, &SysState.ErrFlagReg,                   RW|IDX,         0xffffffff, 0x00,       NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x2532,    0x10,   ODT08U, &SysState.HistoryErrPos,                RW|IDX|IL,      7,          0x00,       NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x2532,    0x80,   ODT32U, &SysState.ErrCutoffMask,                RW|IDX|S1|IL,   0xffffffff, 0x00,       0xffffffff,       96  ,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x2532,    0x81,   ODT32U, &SysState.ErrAlarmMask,                 RW|IDX|S1|IL,   0xffffffff, 0x00,       0xfff7ffff,       100 ,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },

    /*Index.....Sub....DataType...OD_Point..............................Property....... Max.........Min.........DefVal......EromAddres......BackOD..........................rOD_Print.....................Command...*pF_R...........................*pF_W......................*/
    {0x60F7,    0x12,   ODT16S, &DXC.ComInfo.RealDcBus,                 RO|IDX,         0xffff,     0x00,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
//    {0x60F7,    0x0B,   ODT16S, &DXC.ComInfo.TempDevice,                RO|IDX,         0xffff,     0x00,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
//    {0x60F7,    0x08,   ODT16S, &DXC.ComInfo.TempDeviceOffset,          RW|IDX,         200,        -100,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
//    {0x3000,    0x15,   ODT16U, &DXC.ComInfo.TempErrPoint,              RW|IDX,         500,        0x00,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x6510,    0x07,   ODT16U, &DXC.ComInfo.DC_BusUnder,               RW|IDX,         300,        0x00,       NULL,             NULL,     NULL,                           &DXC,                           0,      NULL,                           wSetDC_BusUnderVolt         },
    {0x6510,    0x09,   ODT16U, &DXC.ComInfo.DC_BusOver,                RW|IDX,         500,        0x00,       NULL,             NULL,     NULL,                           &DXC,                           0,      NULL,                           wSetDC_BusOverVolt          },
    {0x6510,    0x08,   ODT16U, &DXC.ComInfo.ChopVolt,                  RW|IDX,         500,        0x00,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           wSetChopVolt                },
    {0x2010,    0x0A,   ODT16U, &DXC.Din.State.All,                     RO|IDX,         500,        0x00,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x2010,    0x02,   ODT16U, &DXC.Din.Simulate,                      RW|IDX,         0xffff,        0x00,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x2010,    0x01,   ODT16U, &DXC.Din.Polarity,                      RW|IDX|S2|IL,   0xffff,        0x00,       0xfffe,           224,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x2010,    0x0B,   ODT16U, &DXC.Din.Virtual,                       RO|IDX,         0xffff,        0x00,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x2010,    0x03,   ODT16U, &DXC.Din.FuncIndex[0],                  RW|IDX|S2|IL,         0xffff,     0x00,       0x1000,             208,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x2010,    0x04,   ODT16U, &DXC.Din.FuncIndex[1],                  RW|IDX|S2|IL,         0xffff,     0x00,       0x8100,             210,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x2010,    0x05,   ODT16U, &DXC.Din.FuncIndex[2],                  RW|IDX|S2|IL,         0xffff,     0x00,       0x00,             212,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x2010,    0x06,   ODT16U, &DXC.Din.FuncIndex[3],                  RW|IDX|S2|IL,         0xffff,     0x00,       0x00,             214,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x2010,    0x07,   ODT16U, &DXC.Din.FuncIndex[4],                  RW|IDX|S2|IL,         0xffff,     0x00,       0x00,             216,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x2010,    0x08,   ODT16U, &DXC.Din.FuncIndex[5],                  RW|IDX|S2|IL,         0xffff,     0x00,       0x00,             218,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x2010,    0x09,   ODT16U, &DXC.Din.FuncIndex[6],                  RW|IDX|S2|IL,         0xffff,     0x00,       0x00,             220,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x2010,    0x1D,   ODT16U, &DXC.Din.FuncIndex[7],                  RW|IDX|S2|IL,         0xffff,     0x00,       0x00,             222,     NULL,                           NULL,                           0,      NULL,                           NULL                        },

//    {0x605A,    0x00,   ODT08U, &DXC.ComInfo.QkStopMode,                RW|IDX|S4|IL,         0xffff,     0x00,       0x01,             199,     NULL,                           NULL,                           0,      NULL,                           NULL                        },

    /*Index.....Sub....DataType...OD_Point..............................Property....... Max.........Min.........DefVal......EromAddres......BackOD..........................rOD_Print.....................Command...*pF_R...........................*pF_W......................*/
    {0x2010,    0x0D,   ODT16U, &DXC.Dout.Polarity,                     RW|IDX|S2|IL,   0xffff,     0x00,       0xffff,           226,      NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x2010,    0x16,   ODT16U, &DXC.Dout.Sys,                          RO|IDX,         500,        0x00,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x2010,    0x0E,   ODT16U, &DXC.Dout.Simulate,                     RW|IDX,         0xffff,     0x00,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x2010,    0x14,   ODT16U, &DXC.Dout.State,                        RO|IDX,         500,        0x00,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x2010,    0xE0,   ODT08U, &DXC.Dout.SingleSim[0],                 RW|IDX,         0xff,       0x00,       NULL,             NULL,     NULL,                           &DXC.Dout.Simulate,             0,      NULL,                           wSingleDoutSimulate         },
    {0x2010,    0xE1,   ODT08U, &DXC.Dout.SingleSim[1],                 RW|IDX,         0xff,       0x00,       NULL,             NULL,     NULL,                           &DXC.Dout.Simulate,             1,      NULL,                           wSingleDoutSimulate         },
    {0x2010,    0xE2,   ODT08U, &DXC.Dout.SingleSim[2],                 RW|IDX,         0xff,       0x00,       NULL,             NULL,     NULL,                           &DXC.Dout.Simulate,             2,      NULL,                           wSingleDoutSimulate         },
    {0x2010,    0xE3,   ODT08U, &DXC.Dout.SingleSim[3],                 RW|IDX,         0xff,       0x00,       NULL,             NULL,     NULL,                           &DXC.Dout.Simulate,             3,      NULL,                           wSingleDoutSimulate         },
    {0x2010,    0xE4,   ODT08U, &DXC.Dout.SingleSim[4],                 RW|IDX,         0xff,       0x00,       NULL,             NULL,     NULL,                           &DXC.Dout.Simulate,             4,      NULL,                           wSingleDoutSimulate         },
    {0x2010,    0xE5,   ODT08U, &DXC.Dout.SingleSim[5],                 RW|IDX,         0xff,       0x00,       NULL,             NULL,     NULL,                           &DXC.Dout.Simulate,             5,      NULL,                           wSingleDoutSimulate         },
    {0x2010,    0xE6,   ODT08U, &DXC.Dout.SingleSim[6],                 RW|IDX,         0xff,       0x00,       NULL,             NULL,     NULL,                           &DXC.Dout.Simulate,             6,      NULL,                           wSingleDoutSimulate         },
    {0x2010,    0xE7,   ODT08U, &DXC.Dout.SingleSim[7],                 RW|IDX,         0xff,       0x00,       NULL,             NULL,     NULL,                           &DXC.Dout.Simulate,             7,      NULL,                           wSingleDoutSimulate         },

    {0x4101,    0x01,   ODT16U, &DXC.DM_Ctrl.CtrlWord,                  RW|IDX,         0xffff,     0x00,       NULL,             NULL,     NULL,                           NULL,                             0,      NULL,                         wDC_MotorControlWord        },
    {0x4101,    0x02,   ODT08U, &DXC.DM_Ctrl.Direction,                 RW|IDX,         0x02,     0x00,       NULL,             NULL,     NULL,                             &DXC.DM_Ctrl,                     0,      NULL,                         wDC_MotorDirectionAndSpeed  },
    {0x4101,    0x03,   ODT08U, &DXC.DM_Ctrl.SpeedSelect,               RW|IDX,         0x03,     0x00,       NULL,             NULL,     NULL,                             &DXC.DM_Ctrl,                     0,      NULL,                         wDC_MotorDirectionAndSpeed  },

    {0x4101,    0x08,   ODT16U, &DXC.DM_Ctrl.Speed[1],                  RW|IDX,         5000,        0x00,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x4101,    0x09,   ODT16U, &DXC.DM_Ctrl.Speed[2],                  RW|IDX,         5000,        0x00,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x4101,    0x0A,   ODT16U, &DXC.DM_Ctrl.Speed[3],                  RW|IDX,         5000,        0x00,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },

    {0x4101,    0x10,   ODT16U, &DXC.DM_IIt.RealIIt,                    RO|IDX,         5000,        0x00,       NULL,             NULL,     NULL,                           &DXC.DM_IIt,                    0,      rRealDM_IItmA,                  NULL                        },
    {0x4101,    0x11,   ODT16U, &DXC.DM_IIt.UserSetI,                   RW|IDX|S8|IL,         0xffff,      0x00,       3000,             956,     NULL,                           &DXC.DM_IIt,                    0,      NULL,                          wSetDC_MotorIIt_I           },
    {0x4101,    0x12,   ODT16U, &DXC.DM_IIt.UserSetT,                   RW|IDX|S8|IL,         3600,        0x00,       3,                958,     NULL,                           &DXC.DM_IIt,                    0,      NULL,                          wSetDC_MotorIIt_T           },
    /*Index.....Sub....DataType...OD_Point..............................Property....... Max.........Min.........DefVal......EromAddres......BackOD..........................rOD_Print.....................Command...*pF_R...........................*pF_W......................*/
    {0x4101,    0x16,   ODT16S, &DXC.PlatformInfo.MechOffsetMm,         RW|IDX|S8|IL,    1000,        0,     715,                   974,     NULL,                           NULL,                          0,      NULL,                          NULL           },
    //{0x4101,    0x17,   ODT32S, &DXC.PlatformInfo.EncCntOffset,         RW|IDX|IL,    65535,       -65535,        300,                  976,     NULL,                           NULL,                          0,      NULL,                          NULL           },
    {0x4101,    0x18,   ODT16S, &DXC.PlatformInfo.DownLimitMm,          RW|IDX,        1800,        0,        10,                 NULL,     NULL,                           &DXC.PlatformInfo,            0,      rPlatformDownLimitMm,           wPlatformDownLimitMm           },
    {0x4101,    0x19,   ODT32S, &DXC.PlatformInfo.DownLimitDec,         RW|IDX|S8|IL,  20000,       -20000,     100,              960,     NULL,                           NULL,                          0,      NULL,                          NULL           },
    {0x4101,    0x20,   ODT16S, &DXC.PlatformInfo.UpLimitMm,            RW|IDX,        1800,        0,        120,                NULL,     NULL,                           &DXC.PlatformInfo,            0,      rPlatformUpLimitMm,           wPlatformUpLimitMm           },
    {0x4101,    0x21,   ODT32S, &DXC.PlatformInfo.UpLimitDec,           RW|IDX|S8|IL,   20000,       -20000,     5000,             964,     NULL,                           NULL,                          0,      NULL,                          NULL           },
    {0x4101,    0x22,   ODT16S, &DXC.PlatformInfo.HighActMm,            RO|IDX,         20000,       -20000,     5000,             NULL,     NULL,                           &DXC.PlatformInfo,             0,      rPlatformHighAct,                          NULL           },
    {0x4101,    0x23,   ODT32S, &DXC.PlatformInfo.Pos.AllPulse.Int32s,  RW|IDX|IL,      20000,       -20000,     5000,             160,      NULL,                           NULL,                          0,      NULL,                          NULL           },
    {0x4101,    0x30,   ODT08U, &DXC.PlatformInfo.CalCmd,               RW|IDX,         255,        0,           0,                NULL,     NULL,                          NULL,                           0,      NULL,                          NULL           },
    {0x4101,    0x31,   ODT16S, &DXC.PlatformInfo.Kn,                   RW|IDX|S8|IL,   32767,        -32768,    155,              968,     NULL,                           NULL,                          0,      NULL,                          NULL           },
    {0x4101,    0x32,   ODT16S, &DXC.PlatformInfo.Kd,                   RW|IDX|S8|IL,   32767,        -32768,    2731,             970,     NULL,                           NULL,                          0,      NULL,                          NULL           },

    {0x2501,    0x06,   ODT16U, &ADC1_2_DMA_DATA2.Split.Ain1V_Org,       RO|IDX,         NULL,       NULL,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x2501,    0x07,   ODT16U, &ADC1_2_DMA_DATA2.Split.Ain2V_Org,       RO|IDX,         NULL,       NULL,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x2501,    0x0E,   ODT16U, &ADC1_2_DMA_DATA2.Split.Ain3V_Org,       RO|IDX,         NULL,       NULL,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x2501,    0x0F,   ODT16U, &ADC1_2_DMA_DATA2.Split.Ain4V_Org,       RO|IDX,         NULL,       NULL,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },

    {0x2501,    0x61,   ODT16U, &DXC.ExtAi[0].AiBase.AdcOffset,         RW|IDX|S3|IL,      4095,       0,          2048,             128,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x2501,    0x62,   ODT16S, &DXC.ExtAi[0].AiBase.AdcOrgAct,         RO|IDX,         NULL,       NULL,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x2501,    0x63,   ODT16S, &DXC.ExtAi[0].AiBase.K,                 RW|IDX|S3|IL,      32767,     -32768,      4096,             130,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x2501,    0x64,   ODT16U, &DXC.ExtAi[0].Lp1f.CfftA,               RW|IDX|S3|IL,      65535,       0,         64261,            132,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x2501,    0x65,   ODT16U, &DXC.ExtAi[0].Lp1f.Cn,                  RO|IDX,         NULL,       NULL,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x2501,    0x6A,   ODT32S, &DXC.ExtAi[0].AiBase.AdcRealDec,        RO|IDX,         NULL,       NULL,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },

    {0x2501,    0x71,   ODT16U, &DXC.ExtAi[1].AiBase.AdcOffset,         RW|IDX|S3|IL,      4095,       0,          2048,             136,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x2501,    0x72,   ODT16S, &DXC.ExtAi[1].AiBase.AdcOrgAct,         RO|IDX,         NULL,       NULL,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x2501,    0x73,   ODT16S, &DXC.ExtAi[1].AiBase.K,                 RW|IDX|S3|IL,      32767,     -32768,      4096,             138,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x2501,    0x74,   ODT16U, &DXC.ExtAi[1].Lp1f.CfftA,               RW|IDX|S3|IL,      65535,       0,         64261,            140,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x2501,    0x75,   ODT16U, &DXC.ExtAi[1].Lp1f.Cn,                  RO|IDX,         NULL,       NULL,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x2501,    0x7A,   ODT32S, &DXC.ExtAi[1].AiBase.AdcRealDec,        RO|IDX,         NULL,       NULL,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },

    {0x2501,    0xE1,   ODT16U, &DXC.ExtAi[2].AiBase.AdcOffset,         RW|IDX|S3|IL,      4095,       0,          2048,             144,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x2501,    0xE2,   ODT16S, &DXC.ExtAi[2].AiBase.AdcOrgAct,         RO|IDX,         NULL,       NULL,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x2501,    0xE3,   ODT16S, &DXC.ExtAi[2].AiBase.K,                 RW|IDX|S3|IL,      32767,     -32768,      4096,             146,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x2501,    0xE4,   ODT16U, &DXC.ExtAi[2].Lp1f.CfftA,               RW|IDX|S3|IL,      65535,       0,         64261,            148,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x2501,    0xE5,   ODT16U, &DXC.ExtAi[2].Lp1f.Cn,                  RO|IDX,         NULL,       NULL,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x2501,    0xEA,   ODT32S, &DXC.ExtAi[2].AiBase.AdcRealDec,        RO|IDX,         NULL,       NULL,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },

    {0x2501,    0xF1,   ODT16U, &DXC.ExtAi[3].AiBase.AdcOffset,         RW|IDX|S3|IL,      4095,       0,          2048,             152,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x2501,    0xF2,   ODT16S, &DXC.ExtAi[3].AiBase.AdcOrgAct,         RO|IDX,         NULL,       NULL,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x2501,    0xF3,   ODT16S, &DXC.ExtAi[3].AiBase.K,                 RW|IDX|S3|IL,      32767,     -32768,      4096,             154,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x2501,    0xF4,   ODT16U, &DXC.ExtAi[3].Lp1f.CfftA,               RW|IDX|S3|IL,      65535,       0,         64261,            156,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x2501,    0xF5,   ODT16U, &DXC.ExtAi[3].Lp1f.Cn,                  RO|IDX,         NULL,       NULL,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x2501,    0xFA,   ODT32S, &DXC.ExtAi[3].AiBase.AdcRealDec,        RO|IDX,         NULL,       NULL,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },

    {0x4100,    0x01,   ODT08U, &SysService.SteEng[0].BitCtrl,          RW|IDX,               0xff,       0,        NULL,            NULL,    NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x4100,    0x02,   ODT16U, &SysService.SteEng[0].LockVal,          RW|IDX|S7|IL,         2000,       10,       1500,            200,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x4100,    0x03,   ODT16U, &SysService.SteEng[0].UnLockVal,        RW|IDX|S7|IL,         2000,       10,       500,             202,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x4100,    0x04,   ODT08U, &SysService.SteEng[1].BitCtrl,          RW|IDX,               0xff,       0,        NULL,            NULL,    NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x4100,    0x05,   ODT16U, &SysService.SteEng[1].LockVal,          RW|IDX|S7|IL,         2000,       10,       1500,            204,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x4100,    0x06,   ODT16U, &SysService.SteEng[1].UnLockVal,        RW|IDX|S7|IL,         2000,       10,       500,             206,     NULL,                           NULL,                           0,      NULL,                           NULL                        },

    {0x4100,    0x10,   ODT32U, &SysService.Radar.Data1.All,            RO|IDX,         NULL,       NULL,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x4100,    0x11,   ODT08U, &SysService.Radar.Data1.Split.Channel1, RO|IDX,         NULL,       NULL,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x4100,    0x12,   ODT08U, &SysService.Radar.Data1.Split.Channel2, RO|IDX,         NULL,       NULL,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x4100,    0x13,   ODT08U, &SysService.Radar.Data1.Split.Channel3, RO|IDX,         NULL,       NULL,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x4100,    0x14,   ODT08U, &SysService.Radar.Data1.Split.Channel4, RO|IDX,         NULL,       NULL,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x4100,    0x15,   ODT08U, &SysService.Radar.FrameBytes,               RW|IDX|S1|IL,   0xff,       0x00,       8,                192,      NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x4100,    0x16,   ODT08U, &SysService.Radar.FrameHead1,               RW|IDX|S1|IL,   0xff,       0x00,       0xFE,             193,      NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x4100,    0x17,   ODT08U, &SysService.Radar.FrameProperty,            RW|IDX|S1|IL,   0xff,       0x00,       0x55,             194,      NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x4100,    0x18,   ODT08U, &SysService.Radar.FrameTail,                RW|IDX|S1|IL,   0xff,       0x00,       0x16,             195,      NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x4100,    0x19,   ODT08U, &SysService.Radar.FrameHead2,               RW|IDX|S1|IL,   0xff,       0x00,       0xCB,             196,      NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x4100,    0x20,   ODT32U, &SysService.Radar.Data2.All,              RO|IDX,         NULL,       NULL,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x4100,    0x21,   ODT08U, &SysService.Radar.Data2.Split.Channel5,   RO|IDX,         NULL,       NULL,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x4100,    0x22,   ODT08U, &SysService.Radar.Data2.Split.Channel6,   RO|IDX,         NULL,       NULL,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x4100,    0x23,   ODT08U, &SysService.Radar.Data2.Split.Channel7,   RO|IDX,         NULL,       NULL,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x4100,    0x24,   ODT08U, &SysService.Radar.Data2.Split.Channel8,   RO|IDX,         NULL,       NULL,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },

    {0x4100,    0x81,   ODT08U, &SysService.AgvSnr.OrgData.Split.Low8,    RO|IDX,         NULL,       NULL,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x4100,    0x82,   ODT08U, &SysService.AgvSnr.OrgData.Split.High8,   RO|IDX,         NULL,       NULL,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x4100,    0x83,   ODT08U, &SysService.AgvSnr.InverData.Split.Low8,  RO|IDX,         NULL,       NULL,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x4100,    0x84,   ODT08U, &SysService.AgvSnr.InverData.Split.High8, RO|IDX,         NULL,       NULL,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },

    {0x2FE5,    0x01,   ODT08U, &SysService.DS.SaveComInfoCmd,                 RW|IDX,         0xff,       0x00,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x2FE5,    0x02,   ODT08U, &SysService.DS.SaveIoCfgCmd,                   RW|IDX,         0xff,       0x00,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x2FE5,    0x03,   ODT08U, &SysService.DS.SaveDeviceCalibCmd,             RW|IDX,         0xff,       0x00,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x2FE5,    0x04,   ODT08U, &SysService.DS.SaveMotorPraCmd,                RW|IDX,         0xff,       0x00,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x2FE5,    0x05,   ODT08U, &SysService.DS.SaveCtrLpPraCmd,                RW|IDX,         0xff,       0x00,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x2FE5,    0x06,   ODT08U, &SysService.DS.SaveMotionCmd,                  RW|IDX,         0xff,       0x00,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x2FE5,    0x07,   ODT08U, &SysService.DS.SaveStEngCmd,                   RW|IDX,         0xff,       0x00,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x2FE5,    0x08,   ODT08U, &SysService.DS.SavePlatformCmd,                RW|IDX,         0xff,       0x00,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },

    //motion OD
    /*Index.....Sub....DataType...OD_Point..............................Property....... Max.........Min.........DefVal......EromAddres......BackOD..........................rOD_Print.....................Command...*pF_R...........................*pF_W......................*/
    {0x2508,    0x04,   ODT32S, &MOX.TurnAngle.CmdAngle,              RW|IDX|IL,      2147483647, -2147483647,0,                NULL,     NULL,                           &MOX,                           0,      NULL,                           wSetTurnAngle               },
    {0x2508,    0x05,   ODT32S, &MOX.TurnAngle.CmdPos,                RO|IDX|IL,      NULL,       NULL,       0,                NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },

    {0x4300,    0x00,   ODT08U, &MOX.GbFlg.MotionEnable,                RW|IDX,         255,        0,          0,                NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                       },
    {0x4300,    0x01,   ODT08S, &MOX.GbFlg.MotionMode,                  RW|S6|IDX|IL,   127,        -128,       33,               768,      NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x4300,    0x02,   ODT16U, &MOX.GbFlg.MotionCtrWord,               RW|IDX,         0xFFFF,     0,          0,                NULL,     NULL,                           NULL,                        0,      NULL,                              wMotionControlWord          },
    {0x4300,    0x10,   ODT08U, &MOX.GbFlg.RadiusUpdataMode,            RW|IDX|IL|S6,   0xFF,       0,          0,                769,     NULL,                           NULL,                        0,      NULL,                              NULL                        },
    {0x4300,    0x81,   ODT08S, &MOX.GbFlg.MotionModeAct,               RO|IDX|IL,      127,        -128,       33,               768,      NULL,                           NULL,                           0,      NULL,                           NULL                        },

    {0x4301,    0x01,   ODT16U, &MOX.CarMchPar.Wheelbase,               RW|IDX|S6|IL,      0xFFFF,     0,          591,               776,     NULL,                           NULL,                        0,      NULL,                              NULL                     },
    {0x4301,    0x02,   ODT16U, &MOX.CarMchPar.WheelDia,                RW|IDX|S6|IL,      0xFFFF,     0,          150,               778,     NULL,                           NULL,                        0,      NULL,                              NULL                     },
    {0x4301,    0x03,   ODT16U, &MOX.CarMchPar.GearF,                   RW|IDX|S6|IL,      0xFFFF,     0,          39,                780,     NULL,                           NULL,                        0,      NULL,                              NULL                     },
    {0x4301,    0x04,   ODT16U, &MOX.CarMchPar.GearD,                   RW|IDX|S6|IL,      0xFFFF,     0,          14,                782,     NULL,                           NULL,                        0,      NULL,                              NULL                     },

    {0x4302,    0x01,   ODT32U, &MOX.MotionPar.ProfileAcceCmd,          RW|IDX|S6|SCL|IL,  21472000,    0,          1072,         784,      &DX[D1].MotorPar.FB_Scale,      NULL,                           0,      NULL,                           NULL                        },
    {0x4302,    0x02,   ODT32U, &MOX.MotionPar.ProfileDeceCmd,          RW|IDX|S6|SCL|IL,  21472000,    0,          3216,         788,      &DX[D1].MotorPar.FB_Scale,      NULL,                           0,      NULL,                           NULL                        },
    {0x4302,    0x03,   ODT32S, &MOX.MotionPar.CmdSpeedDec,             RW|IDX|S6|SCL|IL,  2147483647, -2147483647, 0,        792,      &DX[D1].MotorPar.FB_Scale,      &MOX.MotionPar.CmdSpeedMmps,        0,      NULL,                           wSetMotionSpeed               },
    {0x4302,    0x04,   ODT32S, &MOX.MotionPar.CmdSpeedMmps,            RW|IDX,        3000,       -3000,      0,        NULL,      NULL,                               &MOX.MotionPar.CmdSpeedDec,         1,      NULL,                           wSetMotionSpeed               },
    {0x4302,    0x05,   ODT32U, &MOX.MotionPar.CmdProfileSpeed,         RW|IDX|S6|SCL|IL,  2147483647,  0,          1789568,        796,      &DX[D1].MotorPar.FB_Scale, &MOX.MotionPar.CmdProfileSpeedMmps,                        0,      NULL,   wSetMotionProfileSpeed   },
    {0x4302,    0x06,   ODT32U, &MOX.MotionPar.CmdProfileSpeedMmps,     RW|IDX,        3000,       0,          0,        NULL,      NULL,      &MOX.MotionPar.CmdProfileSpeed,                        1,      NULL,                           wSetMotionProfileSpeed            },
    {0x4302,    0x0f,   ODT32U, &MOX.MotionPar.CmdQkStopDece,           RW|IDX|S6|SCL|IL,  21472000,    0,          107360,         812,       &DX[D1].MotorPar.FB_Scale,      NULL,                           0,      NULL,                           NULL                        },
    {0x4302,    0x10,   ODT32S, &MOX.MotionPar.CmdRadius,               RW|IDX|IFRW,   1000000,       -1000000,          0,        NULL,      NULL,      NULL,                        0,      NULL,                           wSetMotionRadiusMm            },

    {0x4303,    0x01,   ODT32S, &MOX.TurnAngle.CmdAngle,                RW|IDX,      360000, -360000,0,                NULL,     NULL,                           &MOX,                           0,      rRemainTurnAngle,                           wSetTurnAngle               },
    {0x4303,    0x02,   ODT32S, &MOX.TurnAngle.CmdPos,                  RO|IDX,      NULL,       NULL,       0,                NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x4303,    0x04,   ODT32U, &MOX.TurnAngle.ProfileAcceCmd,          RW|IDX|S6|SCL|IL,  21472000,    0,          322,            800,      &DX[D1].MotorPar.FB_Scale,      NULL,                           0,      NULL,                           NULL                        },
    {0x4303,    0x05,   ODT32U, &MOX.TurnAngle.ProfileDeceCmd,          RW|IDX|S6|SCL|IL,  21472000,    0,          322,            804,      &DX[D1].MotorPar.FB_Scale,      NULL,                           0,      NULL,                           NULL                        },
    {0x4303,    0x06,   ODT32U, &MOX.TurnAngle.CmdProfileSpeed,         RW|IDX|S6|SCL|IL,  2147483647,  0,          894784,          808,      &DX[D1].MotorPar.FB_Scale,      NULL,                        0,      NULL,                           NULL            },

    {0x4304,    0x01,   ODT32S, &MOX.PosLength.CmdPosMm,                RW|IDX,             50000000, -50000000, 0,       NULL,            NULL,                           &MOX,                 0,      rRemainPosLengthMm,              wSetPosLengthMm               },

    //示波器OD
    /*Index.....Sub....DataType...OD_Point..............................Property....... Max.........Min.........DefVal......EromAddres......BackOD..........................rOD_Print.....................Command...*pF_R...........................*pF_W......................*/
    {0x2300,    0x00,   ODT08U, &SCP.Group,                             RO|IDX|IL,      NULL,       NULL,       0,                NULL,     NULL,                           &SysState.ActiveIdSpc,          0,      NULL,                           NULL                        },
    {0x2300,    0x01,   ODT32U, &SCP.OD1,                               RW|IDX|IL,      0xffffffff, 0x00,       0,                NULL,     NULL,                           &SysState.ActiveIdSpc,          0,      NULL,                           wSCP_OD1_Func               },
    {0x2300,    0x02,   ODT32U, &SCP.OD2,                               RW|IDX|IL,      0xffffffff, 0x00,       0,                NULL,     NULL,                           &SysState.ActiveIdSpc,          0,      NULL,                           wSCP_OD2_Func               },
    {0x2300,    0x03,   ODT32U, &SCP.OD3,                               RW|IDX|IL,      0xffffffff, 0x00,       0,                NULL,     NULL,                           &SysState.ActiveIdSpc,          0,      NULL,                           wSCP_OD3_Func               },
    {0x2300,    0x04,   ODT32U, &SCP.OD4,                               RW|IDX|IL,      0xffffffff, 0x00,       0,                NULL,     NULL,                           &SysState.ActiveIdSpc,          0,      NULL,                           wSCP_OD4_Func               },
    {0x2300,    0x05,   ODT32U, &SCP.OD5,                               RW|IDX|IL,      0xffffffff, 0x00,       0,                NULL,     NULL,                           &SysState.ActiveIdSpc,          0,      NULL,                           wSCP_OD5_Func               },
    {0x2300,    0x06,   ODT32U, &SCP.OD6,                               RW|IDX|IL,      0xffffffff, 0x00,       0,                NULL,     NULL,                           &SysState.ActiveIdSpc,          0,      NULL,                           wSCP_OD6_Func               },
    {0x2300,    0x07,   ODT08U, &SCP.Control,                           RW|IDX|IL,      255,        0,          0,                NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x2300,    0x08,   ODT16U, &SCP.SamplePeriod,                      RW|IDX|IL,      0xffff,     1,          1,                NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x2300,    0x09,   ODT16U, &SCP.BfTrigLen,                         RW|IDX|IL,      4000,       0,          250,              NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x2300,    0x0A,   ODT16U, &SCP.TotalLen,                          RW|IDX|IL,      8000,       1,          500,              NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x2300,    0x0B,   ODT08U, &SCP.TrigEdge,                          RW|IDX|IL,      2,          0,          0,                NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x2300,    0x0C,   ODT32U, &SCP.TrigOD,                            RW|IDX|IL,      0xffffffff, 0x00,       0,                NULL,     NULL,                           &SysState.ActiveIdSpc,          0,      NULL,                           wSCP_TrigOD_Func            },
    {0x2300,    0x0D,   ODT32S, &SCP.TrigValueBk,                       RW|IDX|IL,      2147483647, -2147483647,0,                NULL,     NULL,                           NULL,                           0,      NULL,                           wSCP_TrigValueFunc          },
    {0x2300,    0x10,   ODT08U, &SCP.InitReadData,                      RW|IDX|IL,      255,        0,          0,                NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
//driver 1
    /*Index.....Sub....DataType...OD_Point..............................Property....... Max.........Min.........DefVal......EromAddres......BackOD..........................rOD_Print.....................Command...*pF_R...........................*pF_W......................*/
    {0x6041,    0x00,   ODT16U, &DX[D1].DriveInfo.StateWord,            RO|ID1,         0xffff,     0x00,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x60F7,    0x0B,   ODT16S, &DX[D1].DriveInfo.TempDevice,           RO|ID1,         0xffff,     0x00,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x60F7,    0x08,   ODT16S, &DX[D1].DriveInfo.TempDeviceOffset,     RW|ID1,         200,        -100,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x3000,    0x15,   ODT16U, &DX[D1].DriveInfo.TempErrPoint,         RW|ID1,         500,        0x00,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x2601,    0x00,   ODT16U, &DX[D1].GbFlg.ErrState1,                RO|ID1,         0xFFFF,     0X01,       NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x2602,    0x00,   ODT16U, &DX[D1].GbFlg.ErrState2,                RO|ID1,         0xFFFF,     0X01,       NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x2600,    0x00,   ODT16U, &DX[D1].GbFlg.ErrMask1,                 RW|ID1,         0xFFFF,     0X00,       NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x2605,    0x01,   ODT16U, &DX[D1].GbFlg.ErrMask1,                 RW|ID1,         0xFFFF,     0X00,       NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x2605,    0x04,   ODT16U, &DX[D1].GbFlg.ErrMask2,                 RW|ID1,         0xFFFF,     0X00,       NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },

    {0x605A,    0x00,   ODT08U, &DX[D1].GbFlg.QkStopMode,               RW|ID1|S5|IL,   0xff,     0x00,       0x01,               314,     NULL,                           NULL,                           0,      NULL,                           NULL                        },


    {0x4500,    0x01,   ODT16U, &DX[D1].GbFlg.TIM1_CntSyn,              RO|ID1,         0xFFFF,     0X00,       NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x4500,    0x02,   ODT16U, &DX[D1].GbFlg.TIM8_CntSyn,              RO|ID1,         0xFFFF,     0X00,       NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },

    //motor paramater od
    /*Index.....Sub....DataType...OD_Point..............................Property....... Max.........Min.........DefVal......EromAddres......BackOD..........................rOD_Print.....................Command...*pF_R...........................*pF_W......................*/
    {0x6410,    0x00,   ODT08U, &DX[D1].MotorPar.Group,                 RW|ID1|IL,          255,        0,          0,                NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x6410,    0x01,   ODT16U, &DX[D1].MotorPar.Type,                  RW|ID1|S4|IL,       65535,      0,          'SM',             256,      NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x6410,    0x02,   ODT08U, &DX[D1].MotorPar.FB_Type,               RW|ID1|S4|IL,       255,        0,          0x40,             258,      NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x6410,    0x03,   ODT32U, &DX[D1].MotorPar.FB_Resolution,         RW|ID1|S4|IL|SCL,   200000,     1,          65536,            260,      &DX[D1].MotorPar.FB_Scale,      NULL,                           0,      NULL,                           NULL                        },
    {0x6410,    0x04,   ODT32U, &DX[D1].MotorPar.FB_Period,             RW|ID1|S4|IL,       0xffffffff, 0,          2,                264,      NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x6410,    0x05,   ODT08U, &DX[D1].MotorPar.Poles,                 RW|ID1|S4|IL,       255,        0,          50,               268,      NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x6410,    0x06,   ODT08U, &DX[D1].MotorPar.ExcitationMode,        RW|ID1|S4|IL,       255,        0,          1,                269,      NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x6410,    0x07,   ODT16S, &DX[D1].MotorPar.ExcitationCurr,        RW|ID1|S4|IL,       2047,      -2048,       800,              270,      NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x6410,    0x08,   ODT16U, &DX[D1].MotorPar.ExcitationTime,        RW|ID1|S4|IL,       65535,      100,        1500,             272,      NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x6410,    0x09,   ODT16U, &DX[D1].MotorPar.IIt_I,                 RW|ID1|S4|IL|IFRW,  1000,       1,          82,               274,      NULL,                           &DX[D1],                        0,      NULL,                           wSetMotorIIt_I              },
    {0x6410,    0x0a,   ODT16U, &DX[D1].MotorPar.IItFilter,             RW|ID1|S4|IL|IFRW,  65535,      0,          234,              276,      NULL,                           &DX[D1],                        0,      NULL,                           wSetMotorIIt_T              },
    {0x6410,    0x0b,   ODT16U, &DX[D1].MotorPar.Imax,                  RW|ID1|S4|IL,       1000,       0,          82,               278,      NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x6410,    0x0c,   ODT16U, &DX[D1].MotorPar.L,                     RW|ID1|S4|IL,       1000,       0,          24,               280,      NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x6410,    0x0d,   ODT16U, &DX[D1].MotorPar.R,                     RW|ID1|S4|IL,       1000,       0,          7,                282,      NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x6410,    0x0e,   ODT16U, &DX[D1].MotorPar.Ke,                    RW|ID1|S4|IL,       1000,       0,          270,              284,      NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x6410,    0x0f,   ODT16U, &DX[D1].MotorPar.Kt,                    RW|ID1|S4|IL,       1000,       0,          40,               286,      NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x6410,    0x10,   ODT16U, &DX[D1].MotorPar.Jr,                    RW|ID1|S4|IL,       1000,       0,          5,                288,      NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x6410,    0x11,   ODT16U, &DX[D1].MotorPar.BrakeDutyCycle,        RW|ID1|S4|IL,       10000,      0,          2000,             290,      NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x6410,    0x12,   ODT16U, &DX[D1].MotorPar.BrakeDelay,            RW|ID1|S4|IL,       10000,      0,          500,              292,      NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x6410,    0x13,   ODT08U, &DX[D1].MotorPar.RotateDir,             RW|ID1|S4|IL,       255,        0,          0,                294,      NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x6410,    0x15,   ODT16U, &DX[D1].MotorPar.CurrBW,                RW|ID1|S4|IL,       65535,      0,          8000,             296,      NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x6410,    0x16,   ODT16U, &DX[D1].MotorPar.UsingType,             RO|ID1|IL,          65535,      0,          'SM',             256,      NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x6410,    0x17,   ODT08U, &DX[D1].MotorPar.WithBrake,             RW|ID1|S4|IL,       255,        0,          0,                295,      NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x6410,    0x19,   ODT16S, &DX[D1].MotorPar.Temp,                  RO|ID1,             255,        0,          0,                NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    //feedback gurop od
    /*Index.....Sub....DataType...OD_Point..............................Property....... Max.........Min.........DefVal......EromAddres......BackOD..........................rOD_Print.....................Command...*pF_R...........................*pF_W......................*/
    {0x6063,    0xff,   ODT32S, &DX[D1].Fbk.PosAbs,                     RO|ID1|SCL,     0xffffffff, 0,          10000,            NULL,     &DX[D1].MotorPar.FB_Scale,      NULL,                           0,      NULL,                           NULL                        },
    {0x6063,    0x00,   ODT32S, &DX[D1].Fbk.PosAbs32,                   RO|ID1,         0xffffffff, 0,          10000,            NULL,     NULL,      NULL,                           0,      NULL,                           NULL                        },
    {0x6078,    0x00,   ODT16S, &DX[D1].Park.Iq,                        RO|ID1,         0xffffffff, 0,          10000,            NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x60F9,    0x05,   ODT08U, &DX[D1].Fbk.SpeedLF_N,                  RW|ID1|S5|IL|IFRW, 44,         0,          10,               312,      NULL,                           &DX[D1],                        0,      NULL,                           wSpeedLoopLF_Coff           },
    {0x60F9,    0x16,   ODT16S, &DX[D1].Fbk.SpeedOrgDec,                RO|ID1|SCL,     NULL,       NULL,       NULL,             NULL,     &DX[D1].MotorPar.FB_Scale,      NULL,                           0,      NULL,                           NULL                        },
    {0x60F9,    0x18,   ODT16S, &DX[D1].Fbk.RealSpeedRpm,               RO|ID1,         NULL,       NULL,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x60F9,    0x19,   ODT16S, &DX[D1].Fbk.RealSpeed0Rpm_0d01,         RO|ID1,         NULL,       NULL,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x606C,    0x00,   ODT32S, &DX[D1].Fbk.SpeedFilterDec,             RO|ID1|SCL,     NULL,       NULL,       NULL,             NULL,     &DX[D1].MotorPar.FB_Scale,      NULL,                           0,      NULL,                           NULL                        },
    {0x250A,    0x07,   ODT32S, &DX[D1].Fbk.PosAbs,                     RO|ID1,         NULL,       NULL,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x4200,    0x00,   ODT16U, &DX[D1].Fbk.ComuEncErrCnt,              RW|ID1,         0xFFFF,     0X00,       NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x4200,    0x01,   ODT08U, &DX[D1].Fbk.SetMagEncZeroTrig,          RW|ID1,         0xFFFF,     0X00,       NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x4200,    0x02,   ODT16U, &DX[D1].Fbk.SetEleAngDec,               RW|ID1,         0xFFFF,     0X00,       NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x2507,    0x02,   ODT16S, &DX[D1].Fbk.EleAngOffsetDec,            RW|ID1,         512,        -512,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x60F6,    0x16,   ODT16U, &DX[D1].Fbk.EleAngDec,                  RO|ID1,         0xFFFF,     0X00,       NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x3010,    0x03,   ODT16U, &DX[D1].Fbk.QEI_CntNow,                 RO|ID1,         0xFFFF,     0X00,       NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    //position loop od
    /*Index.....Sub....DataType...OD_Point..............................Property....... Max.........Min.........DefVal......EromAddres......BackOD..........................rOD_Print.....................Command...*pF_R...........................*pF_W......................*/
    {0x607A,    0x00,   ODT32S, &DX[D1].PosLp.CmdCmdPos,                RW|ID1|SCL|IL,  2147483647, -2147483647,0,                NULL,     &DX[D1].MotorPar.FB_Scale,      NULL,                           0,      NULL,                           NULL                        },
    {0x60F4,    0x00,   ODT32S, &DX[D1].PosLp.FollowErr,                RO|ID1|SCL,             10000,      0,          30,             NULL,   &DX[D1].MotorPar.FB_Scale,      NULL,                           0,      NULL,                           NULL                        },
    {0x6065,    0x00,   ODT32U, &DX[D1].PosLp.MaxFollowErr,             RW|ID1|SCL|S5|IL,  2147483647,    0,        131072,            356,   &DX[D1].MotorPar.FB_Scale,      NULL,                           0,      NULL,                           NULL                        },
    {0x60FB,    0x01,   ODT16S, &DX[D1].PosLp.Kpp,                      RW|ID1|S5|IL,      10000,      0,          30,              320,      NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x60FB,    0x0A,   ODT32S, &DX[D1].PosLp.OutCmdSpeed,              RO|ID1|SCL,             10000,      0,          30,             NULL,   &DX[D1].MotorPar.FB_Scale,        NULL,                           0,      NULL,                           NULL                        },
    {0x2FF0,    0x06,   ODT16U, &DX[D1].PosLp.ProfileAcce16,            RW|ID1|S5|IL,      2000,        0,          3,              322,      NULL,                           &DX[D1],                        0,      NULL,                           wCalActAcceletate           },
    {0x2FF0,    0x07,   ODT16U, &DX[D1].PosLp.ProfileDece16,            RW|ID1|S5|IL,      2000,        0,          3,              324,      NULL,                           &DX[D1],                        0,      NULL,                           wCalActDecelerate           },
    {0x6081,    0x00,   ODT32U, &DX[D1].PosLp.CmdProfileSpeed,          RW|ID1|S5|SCL|IL,  2147483647, 0,          1789568,          326,      &DX[D1].MotorPar.FB_Scale,      &DX[D1],                        0,      NULL,                           wCmdProfileSpeed            },
    {0x6083,    0x00,   ODT32U, &DX[D1].PosLp.ProfileAcceCmd,           RW|ID1|S5|SCL|IL,  21472000,    0,          3216,            330,      &DX[D1].MotorPar.FB_Scale,      NULL,                           0,      NULL,                           NULL                        },
    {0x6084,    0x00,   ODT32U, &DX[D1].PosLp.ProfileDeceCmd,           RW|ID1|S5|SCL|IL,  21472000,    0,          3216,            334,      &DX[D1].MotorPar.FB_Scale,      NULL,                           0,      NULL,                           NULL                        },
    {0x605A,    0x01,   ODT32U, &DX[D1].PosLp.QkStopDeceAct,            RW|ID1|S5|SCL|IL,  21472000,    0,          107360,          316,      &DX[D1].MotorPar.FB_Scale,      NULL,                           0,      NULL,                           NULL                        },

    //speed loop od
    /*Index.....Sub....DataType...OD_Point..............................Property....... Max.........Min.........DefVal......EromAddres......BackOD..........................rOD_Print.....................Command...*pF_R...........................*pF_W......................*/
    {0x2FF0,    0x09,   ODT16S, &DX[D1].SpdLp.CmdSpeedRpm,              RW|ID1|IL,      10000,      -10000,     100,              NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x60FF,    0x00,   ODT32S, &DX[D1].SpdLp.CmdCmdSpeedDec,           RW|ID1|SCL|IL,  2147483647, -2147483647,0,          NULL,     &DX[D1].MotorPar.FB_Scale,      &DX[D1],                        0,      NULL,                           wCmdStepSpeed               },
    {0x60F9,    0x01,   ODT16U, &DX[D1].SpdLp.Kvp,                      RW|ID1|S5|IL,      10000,      0,          10,               338,      NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x60F9,    0x02,   ODT16U, &DX[D1].SpdLp.Kvi,                      RW|ID1|S5|IL,      100,        0,          30,               340,      NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x60F9,    0x08,   ODT32S, &DX[D1].SpdLp.SpeedKiSumLimit,          RW|ID1|S5|IL,      524288,        0,       291271,           352,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x60F9,    0x1E,   ODT32S, &DX[D1].SpdLp.OutCmdIq,                 RO|ID1,         0,          0,          0,                NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x607f,    0x00,   ODT32S, &DX[D1].SpdLp.CmdSpeedMaxDec,           RO|ID1|SCL,     0,          0,          0,                NULL,     &DX[D1].MotorPar.FB_Scale,      NULL,                           0,      NULL,                           NULL                        },
    {0x6080,    0x00,   ODT16S, &DX[D1].SpdLp.CmdSpeedMaxRpm,           RW|ID1|S5|IL|IFRW, 6000,       0,          500,             344,      NULL,                           &DX[D1],                        0,      NULL,                           wCmdSpeedMaxRpm             },
    {0x606B,    0x00,   ODT32S, &DX[D1].SpdLp.CmdSpeedDecAct,           RO|ID1|SCL,     0,          0,          0,                NULL,     &DX[D1].MotorPar.FB_Scale,      NULL,                           0,      NULL,                           NULL                        },
    {0x60F9,    0x1F,   ODT32S, &DX[D1].SpdLp.SpeedKiSum32,             RO|ID1,          524288,        0,       524288,           NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },

    //analog group od
    /*Index.....Sub....DataType...OD_Point..............................Property....... Max.........Min.........DefVal......EromAddres......BackOD..........................rOD_Print.....................Command...*pF_R...........................*pF_W......................*/
    {0x2501,    0x01,   ODT16U, &DX[D1].CurU.AdcOffset,                 RW|ID1,      4095,       0,          2047,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x2501,    0x02,   ODT16U, &DX[D1].CurV.AdcOffset,                 RW|ID1,      4095,       0,          2047,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x2501,    0x04,   ODT16U, &DX[D1].CurU.AdcOrg,                    RW|ID1,      4095,       0,          2047,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x2501,    0x05,   ODT16U, &DX[D1].CurV.AdcOrg,                    RW|ID1,      4095,       0,          2047,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },

    //factor group
    /*Index.....Sub....DataType...OD_Point..............................Property....... Max.........Min.........DefVal......EromAddres......BackOD..........................rOD_Print.....................Command...*pF_R...........................*pF_W......................*/
    {0x3000,    0x04,   ODT16S, &DX[D1].CurU.K,                         RW|ID1,      32767,      -32768,     8192,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x3000,    0x05,   ODT16S, &DX[D1].CurV.K,                         RW|ID1,      32767,      -32768,     8192,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },

    //current loop od
    /*Index.....Sub....DataType...OD_Point..............................Property....... Max.........Min.........DefVal......EromAddres......BackOD..........................rOD_Print.....................Command...*pF_R...........................*pF_W......................*/
    {0x6071,    0x00,   ODT16S, &DX[D1].CurLp.CmdIq,                    RW|ID1|IL,      2047,       -2047,      30,               NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x6073,    0x00,   ODT16U, &DX[D1].CurLp.CmdIqMax,                 RW|ID1|S5|IL,   65535,      0,          1706,             346,      NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x60F6,    0x08,   ODT16S, &DX[D1].CurLp.CmdIq,                    RW|ID1|IL,      2047,       -2047,      30,               NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x60F6,    0x09,   ODT16S, &DX[D1].CurLp.CmdIdAct,                 RW|ID1,         2047,       -2047,      0,                NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x60F6,    0x0C,   ODT16S, &DX[D1].CurLp.CmdIqAct,                 RW|ID1,         65535,      0,          100,              NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x60F6,    0x0D,   ODT16S, &DX[D1].CurLp.CmdIdAct,                 RW|ID1,         65535,      0,          100,              NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x60f6,    0x01,   ODT16U, &DX[D1].CurLp.Kcp,                      RW|ID1|S5|IL,   65535,      0,          1250,             348,      NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x60f6,    0x02,   ODT16U, &DX[D1].CurLp.Kci,                      RW|ID1|S5|IL,   65535,      0,          125,              350,      NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x60F6,    0x21,   ODT16S, &DX[D1].CurLp.VqOut,                    RO|ID1,         65535,      0,          100,              NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x60f6,    0x22,   ODT16U, &DX[D1].SVPWM_Info.CCRU,                RO|ID1,         65535,      0,          125,              NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x60f6,    0x23,   ODT16U, &DX[D1].SVPWM_Info.CCRV,                RO|ID1,         65535,      0,          125,              NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x60f6,    0x24,   ODT16U, &DX[D1].SVPWM_Info.CCRW,                RO|ID1,         65535,      0,          125,              NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },

    //globle flag od
    /*Index.....Sub....DataType...OD_Point..............................Property....... Max.........Min.........DefVal......EromAddres......BackOD..........................rOD_Print.....................Command...*pF_R...........................*pF_W......................*/
    {0x6060,    0x00,   ODT08S, &DX[D1].GbFlg.CtrlMode,                 RW|S5|ID1|IL,   127,        -128,       3,                313,      NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x6061,    0x00,   ODT08S, &DX[D1].GbFlg.CtrlModeAct,              RO|ID1,         127,        -128,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x6040,    0x00,   ODT16U, &DX[D1].GbFlg.CtrlWord,                 RW|ID1,         0xFFFF,     0,          0,                NULL,     NULL,                           &DX[D1],                        0,      NULL,                           wControlWord                },
    {0x2000,    0x00,   ODT08U, &DX[D1].GbFlg.AutoSwitchOn,             RW|ID1|IL,      255,        0,          0,                NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },

    //drive information od
    /*Index.....Sub....DataType...OD_Point..............................Property....... Max.........Min.........DefVal......EromAddres......BackOD..........................rOD_Print.....................Command...*pF_R...........................*pF_W......................*/
    {0x6510,    0x03,   ODT16U, &DX[D1].DriveInfo.I_Max,                RW|ID1,         1000,       10,         180,              NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },

    {0x60F6,    0x12,   ODT16U, &DX[D1].MotorIIt.RealIItDEC_D512,       RO|ID1,         1000,       10,         0,                NULL,     NULL,                           &DX[D1].MotorIIt.RealIItDEC_D,  0,      rRealIIt_D512,                  NULL                        },


//driver2
    /*Index.....Sub....DataType...OD_Point..............................Property....... Max.........Min.........DefVal......EromAddres......BackOD..........................rOD_Print.....................Command...*pF_R...........................*pF_W......................*/
    {0x6041,    0x00,   ODT16U, &DX[D2].DriveInfo.StateWord,            RO|ID2,         0xffff,     0x00,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x60F7,    0x0B,   ODT16S, &DX[D2].DriveInfo.TempDevice,           RO|ID2,         0xffff,     0x00,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x60F7,    0x08,   ODT16S, &DX[D2].DriveInfo.TempDeviceOffset,     RW|ID2,         200,        -100,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x3000,    0x15,   ODT16U, &DX[D2].DriveInfo.TempErrPoint,         RW|ID2,         500,        0x00,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x2601,    0x00,   ODT16U, &DX[D2].GbFlg.ErrState1,                RO|ID2,         0xFFFF,     0X01,       NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x2602,    0x00,   ODT16U, &DX[D2].GbFlg.ErrState2,                RO|ID2,         0xFFFF,     0X01,       NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x2600,    0x00,   ODT16U, &DX[D2].GbFlg.ErrMask1,                 RW|ID2,         0xFFFF,     0X00,       NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x2605,    0x01,   ODT16U, &DX[D2].GbFlg.ErrMask1,                 RW|ID2,         0xFFFF,     0X00,       NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x2605,    0x04,   ODT16U, &DX[D2].GbFlg.ErrMask2,                 RW|ID2,         0xFFFF,     0X00,       NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },

    {0x605A,    0x00,   ODT08U, &DX[D2].GbFlg.QkStopMode,               RW|ID2|S5|IL,   0xff,     0x00,       0x01,               570,     NULL,                           NULL,                           0,      NULL,                           NULL                        },

    //motor paramater od
    /*Index.....Sub....DataType...OD_Point..............................Property....... Max.........Min.........DefVal......EromAddres......BackOD..........................rOD_Print.....................Command...*pF_R...........................*pF_W......................*/
    {0x6410,    0x00,   ODT08U, &DX[D2].MotorPar.Group,                 RW|ID2|S4|IL,       255,        0,          0,                NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x6410,    0x01,   ODT16U, &DX[D2].MotorPar.Type,                  RW|ID2|S4|IL,       65535,      0,          'SM',             512,      NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x6410,    0x02,   ODT08U, &DX[D2].MotorPar.FB_Type,               RW|ID2|S4|IL,       255,        0,          0x40,             514,      NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x6410,    0x03,   ODT32U, &DX[D2].MotorPar.FB_Resolution,         RW|ID2|S4|IL|SCL,   200000,     1,          65536,            516,      &DX[D2].MotorPar.FB_Scale,      NULL,                           0,      NULL,                           NULL                        },
    {0x6410,    0x04,   ODT32U, &DX[D2].MotorPar.FB_Period,             RW|ID2|S4|IL,       0xffffffff, 0,          2,                520,      NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x6410,    0x05,   ODT08U, &DX[D2].MotorPar.Poles,                 RW|ID2|S4|IL,       255,        0,          50,               524,      NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x6410,    0x06,   ODT08U, &DX[D2].MotorPar.ExcitationMode,        RW|ID2|S4|IL,       255,        0,          1,                525,      NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x6410,    0x07,   ODT16S, &DX[D2].MotorPar.ExcitationCurr,        RW|ID2|S4|IL,       2047,       -2048,      800,              526,      NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x6410,    0x08,   ODT16U, &DX[D2].MotorPar.ExcitationTime,        RW|ID2|S4|IL,       65535,      100,        1500,             528,      NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x6410,    0x09,   ODT16U, &DX[D2].MotorPar.IIt_I,                 RW|ID2|S4|IL|IFRW,  1000,       1,          82,               530,      NULL,                           &DX[D2],                        0,      NULL,                           wSetMotorIIt_I              },
    {0x6410,    0x0a,   ODT16U, &DX[D2].MotorPar.IItFilter,             RW|ID2|S4|IL|IFRW,       65535,      0,          234,              532,      NULL,                           &DX[D2],                        0,      NULL,                           wSetMotorIIt_T              },
    {0x6410,    0x0b,   ODT16U, &DX[D2].MotorPar.Imax,                  RW|ID2|S4|IL,       1000,       0,          82,               534,      NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x6410,    0x0c,   ODT16U, &DX[D2].MotorPar.L,                     RW|ID2|S4|IL,       1000,       0,          24,               536,      NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x6410,    0x0d,   ODT16U, &DX[D2].MotorPar.R,                     RW|ID2|S4|IL,       1000,       0,          7,                538,      NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x6410,    0x0e,   ODT16U, &DX[D2].MotorPar.Ke,                    RW|ID2|S4|IL,       1000,       0,          270,              540,      NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x6410,    0x0f,   ODT16U, &DX[D2].MotorPar.Kt,                    RW|ID2|S4|IL,       1000,       0,          40,                542,      NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x6410,    0x10,   ODT16U, &DX[D2].MotorPar.Jr,                    RW|ID2|S4|IL,       1000,       0,          50,                544,      NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x6410,    0x11,   ODT16U, &DX[D2].MotorPar.BrakeDutyCycle,        RW|ID2|S4|IL,       10000,      0,          2000,             546,      NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x6410,    0x12,   ODT16U, &DX[D2].MotorPar.BrakeDelay,            RW|ID2|S4|IL,       10000,      0,          500,              548,      NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x6410,    0x13,   ODT08U, &DX[D2].MotorPar.RotateDir,             RW|ID2|S4|IL,       255,        0,          0,                550,      NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x6410,    0x15,   ODT16U, &DX[D2].MotorPar.CurrBW,                RW|ID2|S4|IL,       65535,      0,          8000,             552,      NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x6410,    0x16,   ODT16U, &DX[D2].MotorPar.UsingType,             RO|ID2|IL,          65535,      0,          'SM',             512,      NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x6410,    0x17,   ODT08U, &DX[D2].MotorPar.WithBrake,             RW|ID2|S4|IL,       255,        0,          0,                551,      NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x6410,    0x19,   ODT16S, &DX[D2].MotorPar.Temp,                  RO|ID2,             255,        0,          0,                NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },

    //feedback gurop od
    /*Index.....Sub....DataType...OD_Point..............................Property....... Max.........Min.........DefVal......EromAddres......BackOD..........................rOD_Print.....................Command...*pF_R...........................*pF_W......................*/
    {0x6063,    0xff,   ODT32S, &DX[D2].Fbk.PosAbs,                     RO|ID2|SCL,     0xffffffff, 0,          10000,            NULL,     &DX[D2].MotorPar.FB_Scale,      NULL,                           0,      NULL,                           NULL                        },
    {0x6063,    0x00,   ODT32S, &DX[D2].Fbk.PosAbs32,                   RO|ID2,         0xffffffff, 0,          10000,            NULL,     NULL,      NULL,                           0,      NULL,                           NULL                        },
    {0x6078,    0x00,   ODT16S, &DX[D2].Park.Iq,                        RO|ID2,         0xffffffff, 0,          10000,            NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x60F9,    0x05,   ODT08U, &DX[D2].Fbk.SpeedLF_N,                  RW|ID2|S5|IL|IFRW, 44,         0,          10,               568,     NULL,                           &DX[D2],                        0,      NULL,                           wSpeedLoopLF_Coff           },
    {0x60F9,    0x16,   ODT16S, &DX[D2].Fbk.SpeedOrgDec,                RO|ID2|SCL,     NULL,       NULL,       NULL,             NULL,     &DX[D2].MotorPar.FB_Scale,      NULL,                           0,      NULL,                           NULL                        },
    {0x60F9,    0x18,   ODT16S, &DX[D2].Fbk.RealSpeedRpm,               RO|ID2,         NULL,       NULL,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x60F9,    0x19,   ODT16S, &DX[D2].Fbk.RealSpeed0Rpm_0d01,         RO|ID2,         NULL,       NULL,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x606C,    0x00,   ODT32S, &DX[D2].Fbk.SpeedFilterDec,             RO|ID2|SCL,     NULL,       NULL,       NULL,             NULL,     &DX[D2].MotorPar.FB_Scale,      NULL,                           0,      NULL,                           NULL                        },
    {0x250A,    0x07,   ODT32S, &DX[D2].Fbk.PosAbs,                     RO|ID2,         NULL,       NULL,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x4200,    0x00,   ODT16U, &DX[D2].Fbk.ComuEncErrCnt,              RW|ID2,         0xFFFF,     0X00,       NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x4200,    0x01,   ODT08U, &DX[D2].Fbk.SetMagEncZeroTrig,          RW|ID2,         0xFFFF,     0X00,       NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x4200,    0x02,   ODT16U, &DX[D2].Fbk.SetEleAngDec,               RW|ID2,         0xFFFF,     0X00,       NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x2507,    0x02,   ODT16S, &DX[D2].Fbk.EleAngOffsetDec,            RW|ID2,         512,        -512,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x60F6,    0x16,   ODT16U, &DX[D2].Fbk.EleAngDec,                  RO|ID2,         0xFFFF,     0X00,       NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x3010,    0x03,   ODT16U, &DX[D2].Fbk.QEI_CntNow,                 RO|ID2,         0xFFFF,     0X00,       NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    //position loop od
    /*Index.....Sub....DataType...OD_Point..............................Property....... Max.........Min.........DefVal......EromAddres......BackOD..........................rOD_Print.....................Command...*pF_R...........................*pF_W......................*/
    {0x607A,    0x00,   ODT32S, &DX[D2].PosLp.CmdCmdPos,                RW|ID2|IL|SCL,  2147483647, -2147483647,0,                NULL,     &DX[D2].MotorPar.FB_Scale,      NULL,                           0,      NULL,                           NULL                        },
    {0x60F4,    0x00,   ODT32S, &DX[D2].PosLp.FollowErr,                RO|ID2|SCL,             10000,      0,          30,             NULL,   &DX[D2].MotorPar.FB_Scale,      NULL,                           0,      NULL,                           NULL                        },
    {0x6065,    0x00,   ODT32U, &DX[D2].PosLp.MaxFollowErr,             RW|ID2|SCL|S5|IL,  2147483647,    0,        131072,            612,   &DX[D2].MotorPar.FB_Scale,      NULL,                           0,      NULL,                           NULL                        },
    {0x60FB,    0x01,   ODT16S, &DX[D2].PosLp.Kpp,                      RW|ID2|S5|IL,      10000,      0,          30,              576,     NULL,                          NULL,                           0,      NULL,                           NULL                        },
    {0x60FB,    0x0A,   ODT32S, &DX[D2].PosLp.OutCmdSpeed,              RO|ID2|SCL,             10000,      0,          30,             NULL,   &DX[D2].MotorPar.FB_Scale,      NULL,                           0,      NULL,                           NULL                        },
    {0x2FF0,    0x06,   ODT16U, &DX[D2].PosLp.ProfileAcce16,            RW|ID2|S5|IL,      2000,        0,          3,              578,     NULL,                           &DX[D2],                        0,      NULL,                           wCalActAcceletate           },
    {0x2FF0,    0x07,   ODT16U, &DX[D2].PosLp.ProfileDece16,            RW|ID2|S5|IL,      2000,        0,          3,              580,     NULL,                           &DX[D2],                        0,      NULL,                           wCalActDecelerate           },
    {0x6081,    0x00,   ODT32U, &DX[D2].PosLp.CmdProfileSpeed,          RW|ID2|S5|IL|SCL,  2147483647, 0,          1789568,          582,     &DX[D2].MotorPar.FB_Scale,      &DX[D2],                        0,      NULL,                           wCmdProfileSpeed            },
    {0x6083,    0x00,   ODT32U, &DX[D2].PosLp.ProfileAcceCmd,           RW|ID2|S5|IL|SCL,  21472000,    0,          3216,           586,     &DX[D2].MotorPar.FB_Scale,      NULL,                           0,      NULL,                           NULL                        },
    {0x6084,    0x00,   ODT32U, &DX[D2].PosLp.ProfileDeceCmd,           RW|ID2|S5|IL|SCL,  21472000,    0,          3216,           590,     &DX[D2].MotorPar.FB_Scale,      NULL,                           0,      NULL,                           NULL                        },
    {0x605A,    0x01,   ODT32U, &DX[D2].PosLp.QkStopDeceAct,            RW|ID2|S5|SCL|IL,  21472000,    0,          107360,            572,      &DX[D2].MotorPar.FB_Scale,      NULL,                           0,      NULL,                           NULL                        },

    //speed loop od
    /*Index.....Sub....DataType...OD_Point..............................Property....... Max.........Min.........DefVal......EromAddres......BackOD..........................rOD_Print.....................Command...*pF_R...........................*pF_W......................*/
    {0x2FF0,    0x09,   ODT16S, &DX[D2].SpdLp.CmdSpeedRpm,              RW|ID2|IL,      10000,      -10000,     100,              NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x60FF,    0x00,   ODT32S, &DX[D2].SpdLp.CmdCmdSpeedDec,           RW|ID2|IL|SCL,  2147483647, -2147483647,0,          NULL,     &DX[D2].MotorPar.FB_Scale,      &DX[D2],                        0,      NULL,                           wCmdStepSpeed               },
    {0x60F9,    0x01,   ODT16U, &DX[D2].SpdLp.Kvp,                      RW|ID2|S5|IL,      1000,       0,          10,               594,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x60F9,    0x02,   ODT16U, &DX[D2].SpdLp.Kvi,                      RW|ID2|S5|IL,      100,        0,          30,                596,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x60F9,    0x08,   ODT32S, &DX[D2].SpdLp.SpeedKiSumLimit,          RW|ID2|S5|IL,      524288,        0,       291271,            608,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x60F9,    0x1E,   ODT32S, &DX[D2].SpdLp.OutCmdIq,                 RO|ID2,         0,          0,          0,                NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x607f,    0x00,   ODT32S, &DX[D2].SpdLp.CmdSpeedMaxDec,           RO|ID2|SCL,     0,          0,          0,                NULL,     &DX[D2].MotorPar.FB_Scale,      NULL,                           0,      NULL,                           NULL                        },
    {0x6080,    0x00,   ODT16S, &DX[D2].SpdLp.CmdSpeedMaxRpm,           RW|ID2|S5|IL|IFRW, 6000,    0,               500,       600,     NULL,     &DX[D2],                        0,                              NULL,                           wCmdSpeedMaxRpm             },
    {0x606B,    0x00,   ODT32S, &DX[D2].SpdLp.CmdSpeedDecAct,           RO|ID2|SCL,     0,          0,          0,                NULL,     &DX[D2].MotorPar.FB_Scale,      NULL,                           0,      NULL,                           NULL                        },
    {0x60F9,    0x1F,   ODT32S, &DX[D2].SpdLp.SpeedKiSum32,             RO|ID2,          524288,        0,       524288,           NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    //analog group od
    /*Index.....Sub....DataType...OD_Point..............................Property....... Max.........Min.........DefVal......EromAddres......BackOD..........................rOD_Print.....................Command...*pF_R...........................*pF_W......................*/
    {0x2501,    0x01,   ODT16U, &DX[D2].CurU.AdcOffset,                 RW|ID2,      4095,       0,          2047,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x2501,    0x02,   ODT16U, &DX[D2].CurV.AdcOffset,                 RW|ID2,      4095,       0,          2047,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x2501,    0x04,   ODT16U, &DX[D2].CurU.AdcOrg,                    RW|ID2,      4095,       0,          2047,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x2501,    0x05,   ODT16U, &DX[D2].CurV.AdcOrg,                    RW|ID2,      4095,       0,          2047,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },

    //factor group
    /*Index.....Sub....DataType...OD_Point..............................Property....... Max.........Min.........DefVal......EromAddres......BackOD..........................rOD_Print.....................Command...*pF_R...........................*pF_W......................*/
    {0x3000,    0x04,   ODT16S, &DX[D2].CurU.K,                         RW|ID2,      32767,      -32768,     8192,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x3000,    0x05,   ODT16S, &DX[D2].CurV.K,                         RW|ID2,      32767,      -32768,     8192,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },

    //current loop od
    /*Index.....Sub....DataType...OD_Point..............................Property....... Max.........Min.........DefVal......EromAddres......BackOD..........................rOD_Print.....................Command...*pF_R...........................*pF_W......................*/
    {0x6071,    0x00,   ODT16S, &DX[D2].CurLp.CmdIq,                    RW|ID2|IL,      2047,       -2047,      30,               NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x6073,    0x00,   ODT16U, &DX[D2].CurLp.CmdIqMax,                 RW|ID2|S5|IL,   65535,      0,          1706,              602,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x60F6,    0x08,   ODT16S, &DX[D2].CurLp.CmdIq,                    RW|ID2|IL,      2047,       -2047,      30,               NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x60F6,    0x09,   ODT16S, &DX[D2].CurLp.CmdIdAct,                 RW|ID2,         2047,       -2047,      0,                NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x60F6,    0x0C,   ODT16S, &DX[D2].CurLp.CmdIqAct,                 RW|ID2,         65535,      0,          100,              NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x60F6,    0x0D,   ODT16S, &DX[D2].CurLp.CmdIdAct,                 RW|ID2,         65535,      0,          100,              NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x60f6,    0x01,   ODT16U, &DX[D2].CurLp.Kcp,                      RW|ID2|S5|IL,   65535,      0,          1250,              604,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x60f6,    0x02,   ODT16U, &DX[D2].CurLp.Kci,                      RW|ID2|S5|IL,   65535,      0,          125,               606,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x60F6,    0x21,   ODT16S, &DX[D2].CurLp.VqOut,                    RO|ID2,         65535,      0,          100,              NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x60f6,    0x22,   ODT16U, &DX[D2].SVPWM_Info.CCRU,                RO|ID2,         65535,      0,          125,              NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x60f6,    0x23,   ODT16U, &DX[D2].SVPWM_Info.CCRV,                RO|ID2,         65535,      0,          125,              NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x60f6,    0x24,   ODT16U, &DX[D2].SVPWM_Info.CCRW,                RO|ID2,         65535,      0,          125,              NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },

    //globle flag od
    /*Index.....Sub....DataType...OD_Point..............................Property....... Max.........Min.........DefVal......EromAddres......BackOD..........................rOD_Print.....................Command...*pF_R...........................*pF_W......................*/
    {0x6060,    0x00,   ODT08S, &DX[D2].GbFlg.CtrlMode,                 RW|S5|ID2|IL,   127,        -128,       3,                569,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x6061,    0x00,   ODT08S, &DX[D2].GbFlg.CtrlModeAct,              RO|ID2,         127,        -128,       NULL,             NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },
    {0x6040,    0x00,   ODT16U, &DX[D2].GbFlg.CtrlWord,                 RW|ID2,         0xFFFF,     0,          NULL,             NULL,     NULL,                           &DX[D2],                        0,      NULL,                           wControlWord                },
    {0x2000,    0x00,   ODT08U, &DX[D2].GbFlg.AutoSwitchOn,             RW|ID2|IL,      255,        0,          0,                NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },

    //drive information od
    /*Index.....Sub....DataType...OD_Point..............................Property....... Max.........Min.........DefVal......EromAddres......BackOD..........................rOD_Print.....................Command...*pF_R...........................*pF_W......................*/
    {0x6510,    0x03,   ODT16U, &DX[D2].DriveInfo.I_Max,                RW|ID2,         1000,       10,         180,              NULL,     NULL,                           NULL,                           0,      NULL,                           NULL                        },

    {0x60F6,    0x12,   ODT16U, &DX[D2].MotorIIt.RealIItDEC_D512,       RO|ID2,         1000,       10,         0,                NULL,     NULL,                           &DX[D2].MotorIIt.RealIItDEC_D,  0,      rRealIIt_D512,                  NULL                        },

    //eeprom debug od
    /*Index.....Sub....DataType...OD_Point..............................Property....... Max.........Min.........DefVal......EromAddres......BackOD..........................rOD_Print.....................Command...*pF_R...........................*pF_W......................*/
    {0x5000,    0x00,   ODT16U, &EepromRW.StartAddr,                    RW,             2047,       0x0000,   NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x5000,    0x01,   ODT16U, &EepromRW.NumBytesRW,                   RW,             32,         0x0000,     NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x5000,    0x02,   ODT16U, &EepromRW.CommandRW,                    RW,             0xFF,       0x00,       NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x5001,    0x00,   ODT08U, &EepromRW.DataBuff[0],                  RW,             0xff,       0x0000,     NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x5001,    0x01,   ODT08U, &EepromRW.DataBuff[1],                  RW,             0xff,       0x0000,     NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x5001,    0x02,   ODT08U, &EepromRW.DataBuff[2],                  RW,             0xff,       0x0000,     NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x5001,    0x03,   ODT08U, &EepromRW.DataBuff[3],                  RW,             0xff,       0x0000,     NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x5001,    0x04,   ODT08U, &EepromRW.DataBuff[4],                  RW,             0xff,       0x0000,     NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x5001,    0x05,   ODT08U, &EepromRW.DataBuff[5],                  RW,             0xff,       0x0000,     NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x5001,    0x06,   ODT08U, &EepromRW.DataBuff[6],                  RW,             0xff,       0x0000,     NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x5001,    0x07,   ODT08U, &EepromRW.DataBuff[7],                  RW,             0xff,       0x0000,     NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x5001,    0x08,   ODT08U, &EepromRW.DataBuff[8],                  RW,             0xff,       0x0000,     NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x5001,    0x09,   ODT08U, &EepromRW.DataBuff[9],                  RW,             0xff,       0x0000,     NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x5001,    0x0A,   ODT08U, &EepromRW.DataBuff[10],                 RW,             0xff,       0x0000,     NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x5001,    0x0B,   ODT08U, &EepromRW.DataBuff[11],                 RW,             0xff,       0x0000,     NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x5001,    0x0C,   ODT08U, &EepromRW.DataBuff[12],                 RW,             0xff,       0x0000,     NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x5001,    0x0D,   ODT08U, &EepromRW.DataBuff[13],                 RW,             0xff,       0x0000,     NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x5001,    0x0E,   ODT08U, &EepromRW.DataBuff[14],                 RW,             0xff,       0x0000,     NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x5001,    0x0F,   ODT08U, &EepromRW.DataBuff[15],                 RW,             0xff,       0x0000,     NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x5001,    0x10,   ODT08U, &EepromRW.DataBuff[16],                 RW,             0xff,       0x0000,     NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x5001,    0x11,   ODT08U, &EepromRW.DataBuff[17],                 RW,             0xff,       0x0000,     NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x5001,    0x12,   ODT08U, &EepromRW.DataBuff[18],                 RW,             0xff,       0x0000,     NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x5001,    0x13,   ODT08U, &EepromRW.DataBuff[19],                 RW,             0xff,       0x0000,     NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x5001,    0x14,   ODT08U, &EepromRW.DataBuff[20],                 RW,             0xff,       0x0000,     NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x5001,    0x15,   ODT08U, &EepromRW.DataBuff[21],                 RW,             0xff,       0x0000,     NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x5001,    0x16,   ODT08U, &EepromRW.DataBuff[22],                 RW,             0xff,       0x0000,     NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x5001,    0x17,   ODT08U, &EepromRW.DataBuff[23],                 RW,             0xff,       0x0000,     NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x5001,    0x18,   ODT08U, &EepromRW.DataBuff[24],                 RW,             0xff,       0x0000,     NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x5001,    0x19,   ODT08U, &EepromRW.DataBuff[25],                 RW,             0xff,       0x0000,     NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x5001,    0x1A,   ODT08U, &EepromRW.DataBuff[26],                 RW,             0xff,       0x0000,     NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x5001,    0x1B,   ODT08U, &EepromRW.DataBuff[27],                 RW,             0xff,       0x0000,     NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x5001,    0x1C,   ODT08U, &EepromRW.DataBuff[28],                 RW,             0xff,       0x0000,     NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x5001,    0x1D,   ODT08U, &EepromRW.DataBuff[29],                 RW,             0xff,       0x0000,     NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x5001,    0x1E,   ODT08U, &EepromRW.DataBuff[30],                 RW,             0xff,       0x0000,     NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },
    {0x5001,    0x1F,   ODT08U, &EepromRW.DataBuff[31],                 RW,             0xff,       0x0000,     NULL,             NULL,     NULL,                           NULL,                           NULL,   NULL,                           NULL                        },

};
//
INT16U OD_Sum = sizeof(OD_ADD)/sizeof(*OD_ADD);

//搜索和执行OD任务,本处理最快用时大约25uS,最长大约为230uS,取决于OD在哪个位置
void NewProcessOD(INT16U ActiveId)
{
    static INT32U LastOD;             //最后一个OD索引记录
    const OD_Attribute *npOD = NULL;
    const OD_Attribute *spOD = &OD_ADD[0];
    UNION_OD_DATA_TYPE_DEF    OD_Type;
    UNION_DATA_STRUCT_DEF UnionDataStruct;
    UNION_DATA64_STRUCT_DEF un_temp64;
    INT16U i, active_id_temp;
    INT8U temp8u,temp8;
    INT8S temp8s;
    INT16U temp16u;
    INT16S temp16s;
    SysError.NewErr = 0;        //一定要先清除功能函数返回的错误
    //数据一进来就先计算check是否正确,
    temp8 = -(SdoBuf.All[0] + SdoBuf.All[1] + SdoBuf.All[2]\
              + SdoBuf.All[3] + SdoBuf.All[4] + SdoBuf.All[5]\
              + SdoBuf.All[6] + SdoBuf.All[7] + SdoBuf.All[8]);
    if(temp8 != SdoBuf.Split.Check)       //如果check值不对,那说明数据传输过程中出错了,回发出错信息
    {
        SdoBuf.Split.CMD = 0x80;    //返回错误标识
    }
    else      //check正确,搜索ID并执行相应的动作
    {
        if((SdoBuf.Split.CMD == 0x60) || (SdoBuf.Split.CMD == 0x70))       //SDO block transit
        {
            SDO_BlockProcess(LastOD);
        }
        else
        {
            UnionDataStruct.All8u[0] = SdoBuf.Split.IndexL;      //将OD的Index放在联合体中
            UnionDataStruct.All8u[1] = SdoBuf.Split.IndexH;
            for(i=0; i<sizeof(OD_ADD)/sizeof(*OD_ADD); i++)     //搜索OD
            {
                active_id_temp = spOD->Property & IDX;
                if(((active_id_temp == IDX) || (active_id_temp == ActiveId)) && spOD->Index == UnionDataStruct.Int16u  && spOD->SubIndex == SdoBuf.Split.SubIndex)  //搜索到OD,将OD首地址付给npOD
                {
                    npOD = spOD;
                    break;
                }
                else
                {
                    spOD++;
                }
            }
            if(npOD != NULL)    //假如存在这个OD
            {
                if(SdoBuf.Split.CMD == 0x40)        //读对象
                {
                    LastOD = ((INT32U)(SdoBuf.Split.IndexH)<<16) + ((INT32U)(SdoBuf.Split.IndexL)<<8) + (INT32U)(SdoBuf.Split.SubIndex);
                    UnionDataStruct.Int32u = 0;
                    if(npOD->pF_R != NULL)          //存在需要现场获取的标示,
                    {
                        SysError.NewErr = (npOD->pF_R)(npOD->Command,npOD->pOD,npOD->bpOD,npOD->rpOD);
                    }
                    switch(npOD->DataType)
                    {
                    case ODT16U:
                        OD_Type.pInt16u = npOD->pOD;      //指向uInt数据类型
                        UnionDataStruct.Int16u =   *OD_Type.pInt16u;
                        SdoBuf.Split.CMD = 0X4B;    //存在2个字节数据
                        if((npOD->Property)&SCL) //检查本变量是否需要尺度变换
                            un_temp64.Int64u = (INT64U)UnionDataStruct.Int16u;
                        break;
                    case ODT16S:
                        OD_Type.pInt16s = npOD->pOD;      //指向uInt数据类型
                        UnionDataStruct.Int16s =   *OD_Type.pInt16s;
                        SdoBuf.Split.CMD = 0X4B;    //存在2个字节数据
                        if((npOD->Property)&SCL) //检查本变量是否需要尺度变换
                            un_temp64.Int64s = (INT64S)UnionDataStruct.Int16s;
                        break;
                    case ODT08U:
                        OD_Type.pInt8u = npOD->pOD;     //指向uChar数据类型
                        UnionDataStruct.Int8u    =   *OD_Type.pInt8u;
                        SdoBuf.Split.CMD = 0X4F;    //存在1个字节数据
                        if((npOD->Property)&SCL) //检查本变量是否需要尺度变换
                            un_temp64.Int64u = (INT64U)UnionDataStruct.Int8u;
                        break;
                    case ODT08S:
                        OD_Type.pInt8s = npOD->pOD;     //指向uChar数据类型
                        UnionDataStruct.Int8s    =   *OD_Type.pInt8s;
                        SdoBuf.Split.CMD = 0X4F;    //存在1个字节数据
                        if((npOD->Property)&SCL) //检查本变量是否需要尺度变换
                            un_temp64.Int64s = (INT64U)UnionDataStruct.Int8s;
                        break;
                    case ODT32U:
                        OD_Type.pInt32u = npOD->pOD;     //指向uLong数据类型
                        UnionDataStruct.Int32u    =   *OD_Type.pInt32u;
                        SdoBuf.Split.CMD = 0X43;    //存在4个字节数据
                        if((npOD->Property)&SCL) //检查本变量是否需要尺度变换
                            un_temp64.Int64u = (INT64U)UnionDataStruct.Int32u;
                        break;
                    case ODT32S:
                        OD_Type.pInt32s = npOD->pOD;     //指向uLong数据类型
                        UnionDataStruct.Int32s    =   *OD_Type.pInt32s;
                        SdoBuf.Split.CMD = 0X43;    //存在4个字节数据
                        if((npOD->Property)&SCL) //检查本变量是否需要尺度变换
                            un_temp64.Int64s = (INT64U)UnionDataStruct.Int32s;
                        break;
                    default :
                        break;
                    }
                    if((npOD->Property)&SCL) //检查本变量是否需要尺度变换
                    {
                        un_temp64.Int64s = un_temp64.Int64s * 10000;
                        UnionDataStruct.Int32s = un_temp64.Int64s / (*((INT32U*)(npOD->bpOD)));
                    }
                    //返回值
                    SdoBuf.Split.Data1 = UnionDataStruct.All8u[0];
                    SdoBuf.Split.Data2 = UnionDataStruct.All8u[1];
                    SdoBuf.Split.Data3 = UnionDataStruct.All8u[2];
                    SdoBuf.Split.Data4 = UnionDataStruct.All8u[3];
                }
                else if((SdoBuf.Split.CMD == 0x2f) || (SdoBuf.Split.CMD == 0x2b) || (SdoBuf.Split.CMD == 0x23))       //写对象
                {
                    LastOD = ((INT32U)(SdoBuf.Split.IndexH)<<16) + ((INT32U)(SdoBuf.Split.IndexL)<<8) + (INT32U)(SdoBuf.Split.SubIndex);
                    if((npOD->Property)&WO) //检测一下这个OD是否可写,注意啦:这里不能写成if(npOD->Property&WO)
                    {
                        UnionDataStruct.All8u[0] = SdoBuf.Split.Data1; //将要写入的数据放在联合体中
                        UnionDataStruct.All8u[1] = SdoBuf.Split.Data2;
                        UnionDataStruct.All8u[2] = SdoBuf.Split.Data3;
                        UnionDataStruct.All8u[3] = SdoBuf.Split.Data4;
                        //为尺度变换做的处理和正负数处理(不同字节数的要全部转换成32bit的)
                        switch(npOD->DataType)
                        {
                        case ODT16U:
                            temp16u = UnionDataStruct.Int16u;
                            UnionDataStruct.Int32u = (INT32U)temp16u;    //20160624 Liangshun
                            if((npOD->Property)&SCL) //检查本变量是否需要尺度变换
                            {
                                un_temp64.Int64u = (INT64U)temp16u;
                            }
                            break;
                        case ODT16S:
                            temp16s = UnionDataStruct.Int16s;
                            UnionDataStruct.Int32s = (INT32S)temp16s;
                            if((npOD->Property)&SCL) //检查本变量是否需要尺度变换
                            {
                                un_temp64.Int64s = (INT64S)temp16s;
                            }
                            break;
                        case ODT08U:
                            temp8u = UnionDataStruct.Int8u;
                            UnionDataStruct.Int32u = (INT32U)temp8u;
                            if((npOD->Property)&SCL) //检查本变量是否需要尺度变换
                            {
                                un_temp64.Int64u = (INT64U)temp8u;
                            }
                            break;
                        case ODT08S:
                            temp8s = UnionDataStruct.Int8s;
                            UnionDataStruct.Int32s = (INT32S)temp8s;
                            if((npOD->Property)&SCL) //检查本变量是否需要尺度变换
                            {
                                un_temp64.Int64s = (INT64S)temp8s;
                            }
                            break;
                        case ODT32U:
                            if((npOD->Property)&SCL) //检查本变量是否需要尺度变换
                                un_temp64.Int64u = (INT64U)UnionDataStruct.Int32u;
                            break;
                        case ODT32S:
                            if((npOD->Property)&SCL) //检查本变量是否需要尺度变换
                                un_temp64.Int64s = (INT64S)UnionDataStruct.Int32s;
                            break;
                        default :
                            break;
                        }
                        if((npOD->Property)&SCL) //检查本变量是否需要尺度变换
                        {
                            un_temp64.Int64s = un_temp64.Int64s * (*((INT32U*)(npOD->bpOD)));
                            UnionDataStruct.Int32s = un_temp64.Int64s / 10000;
                        }
                        if((npOD->DataType)&0x80)  //判断有符号数,还是无符号数,进行不同的处理,要不然会出错的
                        {
                            if(UnionDataStruct.Int32s > npOD->Max)//判断数据是否超过最大值
                            {
                                UnionDataStruct.Int32s = npOD->Max;
                            }
                            else if(UnionDataStruct.Int32s < npOD->Min)//判断数据是否低于最小值
                            {
                                UnionDataStruct.Int32s = npOD->Min;
                            }
                        }
                        else    //无符号数
                        {
                            if(UnionDataStruct.Int32u > (INT32U)npOD->Max)//判断数据是否超过最大值
                            {
                                UnionDataStruct.Int32u = (INT32U)npOD->Max;
                            }
                            else if(UnionDataStruct.Int32u < (INT32U)npOD->Min)//判断数据是否低于最小值
                            {
                                UnionDataStruct.Int32u = (INT32U)npOD->Min;
                            }
                        }
                        switch(npOD->DataType)
                        {
                        case ODT16U:
                            OD_Type.pInt16u = npOD->pOD;      //指向uInt数据类型
                            *OD_Type.pInt16u = UnionDataStruct.Int16u;
                            break;
                        case ODT16S:
                            OD_Type.pInt16s = npOD->pOD;      //指向uInt数据类型
                            *OD_Type.pInt16s = UnionDataStruct.Int16s;
                            break;
                        case ODT08U:
                            OD_Type.pInt8u = npOD->pOD;     //指向uChar数据类型
                            *OD_Type.pInt8u = UnionDataStruct.Int8u;
                            break;
                        case ODT08S:
                            OD_Type.pInt8s = npOD->pOD;     //指向uChar数据类型
                            *OD_Type.pInt8s = UnionDataStruct.Int8s;  //2012.09.24 dyc
                            break;
                        case ODT32U:
                            OD_Type.pInt32u = npOD->pOD;     //指向uLong数据类型
                            *OD_Type.pInt32u = UnionDataStruct.Int32u;
                            break;
                        case ODT32S:
                            OD_Type.pInt32s = npOD->pOD;     //指向uLong数据类型
                            *OD_Type.pInt32s = UnionDataStruct.Int32s;
                            break;
                        default :
                            break;
                        }
                        if(npOD->pF_W != NULL)
                        {
                            SysError.NewErr = (npOD->pF_W)(npOD->Command, npOD->pOD, npOD->bpOD, npOD->rpOD);
                        }
                        //if(NULL != npOD->bpOD)      //如果存在返回参数,返回参数数据类型跟OD一样(注意啦),那将返回数据放进data1~data4
                        //{
                        //    UnionDataStruct.Int32s = (*(INT32S*)(npOD->bpOD));       //得到返回值
                        //}
                        SdoBuf.Split.CMD = 0x60;    //本操作成功,后面还会检测SysError.NewErr状态,属于函数功能返回的错误状态
                        if((npOD->Property)&SCL) //检查本变量是否需要尺度变换
                        {
                            un_temp64.Int64s = (INT64S)UnionDataStruct.Int32s * 10000;
                            UnionDataStruct.Int32s = un_temp64.Int64s / (*((INT32U*)(npOD->bpOD)));
                        }
                        //返回值
                        SdoBuf.Split.Data1 = UnionDataStruct.All8u[0];
                        SdoBuf.Split.Data2 = UnionDataStruct.All8u[1];
                        SdoBuf.Split.Data3 = UnionDataStruct.All8u[2];
                        SdoBuf.Split.Data4 = UnionDataStruct.All8u[3];
                    }
                    else            //本OD不可写
                    {
                        SdoBuf.Split.CMD = 0x80;    //返回错误标识:
                    }
                }
                else        //CMD错误
                {
                    SdoBuf.Split.CMD = 0x80;    //返回错误标识:
                }
            }
            else    //不存在这个OD
            {
                SdoBuf.Split.CMD = 0x80;    //返回错误标识
            }
        }
    }
    //检测函数功能返回的
    if(SysError.NewErr) //调用错误记录功能函数时候,有错误也将CMD改了,不管什么模式下
    {
        ErrRecordImmediately(SysError.NewErr);
        SdoBuf.Split.CMD = 0x80;    //返回错误标识
    }
    //KTP-PC模式
    if((LocalProperty.NowMode == KTP_PC) && (0 == (CDPC_O&(npOD->Property))))     //如果上位机使用KTP_PC且本变量不受特殊限制,折将CMD返回值修改掉
    {
        //如果CMD,对象读写属性出错也返回0x80,这时不影响KTP_PC上位机错误解析,因为KTP_PC上位机错误解析忽略最高位.
        SdoBuf.Split.CMD = (SdoBuf.Split.CMD&0x80) | SysError.NewErr;
    }
    //重新计算check
    SdoBuf.Split.Check = -(SdoBuf.All[0] + SdoBuf.All[1] + SdoBuf.All[2]\
                            + SdoBuf.All[3] + SdoBuf.All[4] + SdoBuf.All[5]\
                            + SdoBuf.All[6] + SdoBuf.All[7] + SdoBuf.All[8]);
}
//上电初始化加载
void PowerOnOD_Init(void)
{
    INT16U i,j;
    UNION_OD_DATA_TYPE_DEF    OD_Type;
    UNION_DATA_STRUCT_DEF UnionDataStruct;
    INT16U ReadNum = 0;
    const OD_Attribute *npOD = &OD_ADD[0];
    //固定数值初始化
    LocalProperty.OD_Sum = sizeof(OD_ADD)/sizeof(*OD_ADD);
    LocalProperty.FirmwareVersion = GetCompileDate();
    //检验一下OD是否有重复的,避免OD多了不小心给安排了同样的Index和SubIndex
/*    for(j=0; j<sizeof(OD_ADD)/sizeof(*OD_ADD)-1; j++)
    {
        for(i=j+1; i<sizeof(OD_ADD)/sizeof(*OD_ADD); i++)
        {
            if(((OD_ADD[j].Index) == (OD_ADD[i].Index)))
            {
                if((OD_ADD[j].SubIndex) == (OD_ADD[i].SubIndex))  //有重复地址的OD
                {
                    if(SysState.OD_DupNum < 5)  //记录空间只有5个,所以要限制一下
                    {
                        SysState.OD_DupInfo[SysState.OD_DupNum] = ((INT32U)(OD_ADD[i].Index)<<8) + OD_ADD[i].SubIndex ;
                    }
                    SysState.OD_DupNum ++;
                }
            }
        }
    }
    if(SysState.OD_DupNum) //有OD地址重复
    {
        SysState.ErrFlag = SysState.ErrFlag | OD_DUP_ERR_CODE;
    }
*/
    //从EEPROM读取数值
    for(i=0; i<sizeof(OD_ADD)/sizeof(*OD_ADD); i++)     //搜索OD
    {
        if((OD_ADD[i].Property)&IL)
        {
            if(OD_ADD[i].EromAddress == NULL)   //如果没有分配EEPROM地址,那就直接加载DefValue值
            {
                UnionDataStruct.Int32s = OD_ADD[i].DefVal;
            }
            else
            {
                ReadNum = (OD_ADD[i].DataType)&0x0f;
                sEE_ReadBuffer(&UnionDataStruct.All8u[0], OD_ADD[i].EromAddress, &ReadNum);
                while(ReadNum)
                {
                    ;    //这里要插入超时处理
                }
            }
            //赋值处理
            switch(OD_ADD[i].DataType)
            {
            case ODT16U:
                OD_Type.pInt16u = OD_ADD[i].pOD;      //指向uInt数据类型
                *OD_Type.pInt16u = UnionDataStruct.Int16u;
                break;
            case ODT16S:
                OD_Type.pInt16s = OD_ADD[i].pOD;      //指向uInt数据类型
                *OD_Type.pInt16s = UnionDataStruct.Int16s;
                break;
            case ODT08U:
                OD_Type.pInt8u = OD_ADD[i].pOD;     //指向uChar数据类型
                *OD_Type.pInt8u = UnionDataStruct.Int8u;
                break;
            case ODT08S:
                OD_Type.pInt8s = OD_ADD[i].pOD;     //指向uChar数据类型
                *OD_Type.pInt8s = UnionDataStruct.Int8s;
                break;
            case ODT32U:
                OD_Type.pInt32u = OD_ADD[i].pOD;     //指向uLong数据类型
                *OD_Type.pInt32u = UnionDataStruct.Int32u;
                break;
            case ODT32S:
                OD_Type.pInt32s = OD_ADD[i].pOD;      //指向Long数据类型
                *OD_Type.pInt32s = UnionDataStruct.Int32s;
                break;
            default :
                break;
            }
        }
    }
    //执行一次读写函数
    for(i=0; i<sizeof(OD_ADD)/sizeof(*OD_ADD); i++)     //搜索OD
    {
        if((npOD->Property)&IFRW)
        {
            if(npOD->pF_R != NULL)          //存在需要现场获取的标示,
            {
                (npOD->pF_R)(npOD->Command,npOD->pOD,npOD->bpOD,npOD->rpOD);
            }
            if(npOD->pF_W != NULL)
            {
                (npOD->pF_W)(npOD->Command, npOD->pOD, npOD->bpOD, npOD->rpOD);
            }
        }
        npOD++;
    }
}
//数据保存
void    OD_DataSave(INT8U save_id, INT16U save_type)
{
    INT16U i;
    UNION_OD_DATA_TYPE_DEF  OD_Type;
    UNION_DATA_STRUCT_DEF UnionDataWrite;
    UNION_DATA_STRUCT_DEF UnionDataRead;
    INT16U read_num;
    if(save_type & SD_DOMAIN)           //判断是否存在此保存指令
    {
        for(i=0; i<sizeof(OD_ADD)/sizeof(*OD_ADD); i++)     //搜索OD
        {
            if((((OD_ADD[i].Property) & ID_DOMAIN) == save_id) && (((OD_ADD[i].Property) & SD_DOMAIN) == save_type))    //搜索到需要保存的OD,则进行保存(此处要注意呀,判断)
            {
                UnionDataWrite.Int32s = 0;  //清零
                switch(OD_ADD[i].DataType)
                {
                case ODT08U:
                    OD_Type.pInt8u = OD_ADD[i].pOD;
                    UnionDataWrite.Int8u = *OD_Type.pInt8u;
                    break;
                case ODT16U:
                    OD_Type.pInt16u = OD_ADD[i].pOD;
                    UnionDataWrite.Int16u = *OD_Type.pInt16u;
                    break;
                case ODT32U:
                    OD_Type.pInt32u = OD_ADD[i].pOD;
                    UnionDataWrite.Int32u = *OD_Type.pInt32u;
                    break;
                case ODT08S:
                    OD_Type.pInt8s = OD_ADD[i].pOD;
                    UnionDataWrite.Int8s = *OD_Type.pInt8s;
                    break;
                case ODT16S:
                    OD_Type.pInt16s = OD_ADD[i].pOD;
                    UnionDataWrite.Int16s = *OD_Type.pInt16s;
                    break;
                case ODT32S:
                    OD_Type.pInt32s = OD_ADD[i].pOD;
                    UnionDataWrite.Int32s = *OD_Type.pInt32s;
                    break;
                default:
                    break;
                }
                read_num = (OD_ADD[i].DataType)&0x0f;
                UnionDataRead.Int32s = 0;   //清零
                sEE_ReadBuffer(&UnionDataRead.All8u[0], OD_ADD[i].EromAddress, &read_num);
                while(read_num)
                {
                    ;    //这里要插入超时处理
                }
                if(UnionDataRead.Int32s != UnionDataWrite.Int32s)
                {
                    sEE_WriteBuffer(&UnionDataWrite.All8u[0], OD_ADD[i].EromAddress, (OD_ADD[i].DataType)&0x0f);
                    while((*sEEDataWritePointer) != 0); //这里要插入超时处理
                }
            }
        }
    }
}
//错误立即记录
void ErrRecordImmediately(INT8U new_err)
{
    INT8U i;
    for(i=9; i>=1; i--)
    {
        SysError.LastErr[i] = SysError.LastErr[i-1];        //移动错误
    }
    SysError.LastErr[0] = new_err;
    SysService.SaveFlag = S4;
}
//获取编译日期
INT32U GetCompileDate(void)
{
    const INT8U Month_Table[12][3]= {"Jan","Feb","Mar","Apr","May","Jun","Jul","Aug","Sep","Oct","Nov","Dec"};
    INT8U i,j;
    INT8U *_date;
    INT8U mon,day; //sec,min,hour;
    INT16U year;
    //固定数值初始化
    _date = __DATE__;   //读电脑当前日期,格式eg: Mar 15 2013
    for(i=0; i<12; i++)
    {
        if((Month_Table[i][0] == _date[0]) && (Month_Table[i][1] == _date[1]) && (Month_Table[i][2] == _date[2]))
        {
            mon = i+1;    //得到月份
        }
    }
    if(_date[4]==' ')//得到日期
    {
        day=_date[5]-'0';      //-'0'操作将字符型转换为整型,参考ASCII码的转换，eg '7'-'0' =7
    }
    else
    {
        day=10*(_date[4]-'0')+_date[5]-'0';
    }
    year=1000*(_date[7]-'0')+100*(_date[8]-'0')+10*(_date[9]-'0')+_date[10]-'0';  //得到年份
    return (year*10000 + mon*100 + day);    //返回当前日期,格式为一个32bit整型数 eg:20130515
}
//寻找OD
const OD_Attribute* FindOd(INT16U ActiveId, INT32U OdAddress)
{
    const OD_Attribute *npOD = NULL;
    const OD_Attribute *spOD = &OD_ADD[0];
    INT16U  i,active_id_temp;
    INT16U index,sub_index;
    index = (OdAddress)>>16;
    sub_index = ((OdAddress)>>8) & 0x000000ff;
    for(i=0; i<sizeof(OD_ADD)/sizeof(*OD_ADD); i++)     //搜索OD
    {
        //使用指针,执行时间降低20%左右
        active_id_temp = spOD->Property & IDX;
        if(((active_id_temp == IDX) || (active_id_temp == ActiveId)) && spOD->Index == index  && spOD->SubIndex == sub_index)  //搜索到OD,将OD首地址付给npOD
        {
            npOD = spOD;
            break;
        }
        else
        {
            spOD++;
        }
    }
    return(npOD);
}
void SDO_BlockProcess(INT32U SDO_BlockIndex)
{
    static INT8U*  SCP_UploadPtr = (INT8U*)SCP.Buff;
    static INT16U  SCP_DataRemain = 0;
    static INT8U   FirstTxFlg = TRUE;
    INT8U tp = 0;
    INT8U i = 0;
    if(SDO_BlockIndex == 0x230010) //read osc data.
    {
        if(FirstTxFlg)
        {
            FirstTxFlg = FALSE;
            if(SCP.HeadPoint < SCP.TailPoint)
            {
                SCP_DataRemain = (INT8U*)SCP.TailPoint - (INT8U*)SCP.HeadPoint;
            }
            else
            {
                SCP_DataRemain = ((INT8U*)SCP.TailPoint - (INT8U*)SCP.Buff)+((INT8U*)(&SCP.Buff[SCP_LEN-1]+1)-(INT8U*)SCP.HeadPoint);
            }
            SCP_UploadPtr = (INT8U*)SCP.HeadPoint;
        }
        if(SCP_DataRemain > 7)
        {
            SCP_DataRemain = SCP_DataRemain - 7;
            for(i=0; i<7; i++)
            {
                SdoBuf.All[i+2] = *SCP_UploadPtr;
                SCP_UploadPtr++;
                if(SCP_UploadPtr >= ((INT8U*)(&SCP.Buff[SCP_LEN-1]+1)))
                {
                    SCP_UploadPtr = (INT8U*)SCP.Buff;
                }
            }
            //pc send 0x60,driver ansawer 0x00,pc send 0x70,driver ansawer 0x10
            SdoBuf.Split.CMD = (SdoBuf.Split.CMD) & 0x10;
        }
        else   //last frame send to pc
        {
            //pc send 0x60,driver ansawer 0x00,pc send 0x70,driver ansawer 0x10
            SdoBuf.Split.CMD = ((SdoBuf.Split.CMD) & 0x10) + ((7-SCP_DataRemain)<<1) + 1;
            i=2;
            while(SCP_DataRemain > 0)
            {
                SdoBuf.All[i] = *SCP_UploadPtr;
                SCP_UploadPtr++;
                if(SCP_UploadPtr >= (INT8U*)(&SCP.Buff[SCP_LEN-1]+1))
                {
                    SCP_UploadPtr = (INT8U*)SCP.Buff;
                }
                i++;
                SCP_DataRemain--;
            }
            SCP_UploadPtr = (INT8U*)SCP.Buff;  //发送指针指向buf的起始地方。
            FirstTxFlg = TRUE; //此次传输完成，可以开始下一次传输。
        }
    }
}

