/*
 * SR2D1_0V3_MCU_S1 -- Service robot 2In1 servo driver
 * (C)2016 Kinco Electric (Shenzhen) Ltd. -- All rights reserved
 */
/***************************************************
     ��ͷ�ļ�������Щ�ᱻ����ļ����õĽṹ��,��������
****************************************************/
#ifndef _OD_LIB_H
#define _OD_LIB_H
#define FIRWARE_UPDATA_DATE     20150927

#define MASTER      0X01
#define SLAVER      0X02
#define NOW_MODE    SLAVER

#define NULL        0

#define ODT08U      0x01            //�޷����ַ���
#define ODT08S      0x81            //�з����ַ���
#define ODT16U      0x02            //�޷�������
#define ODT16S      0x82            //�з�������
#define ODT32U      0x04            //�޷��ų�����
#define ODT32S      0x84            //�з��ų�����

#define BYTE1       0x01            //1���ֽ�
#define BYTE2       0x02            //2���ֽ�
#define BYTE4       0x04            //4���ֽ�

#define ID_DOMAIN   0x000F
#define ID1         0x0001          //ID1
#define ID2         0x0002          //ID2
#define ID3         0x0003          //ID3
#define ID4         0x0004          //ID4
#define IDX         0x000F          //IDX
#define RO          0x0010          //ֻ��
#define WO          0x0020          //ֻд
#define RW          0x0030          //�ɶ�д
/*
#define S1          0x0100          //��������1
#define S2          0x0200          //��������2
#define S4          0x0400          //��������3
#define S8          0x0800          //��������4
*/
#define SD_DOMAIN   0x0F00
#define S1          0x0100          //������Ϣ����(ͨ�Žӿ����õ�)
#define S2          0x0200          //�ӿ����ñ���(IO�ڹ�������,���Ե�)
#define S3          0x0300          //ÿ��������У׼��������
#define S4          0x0400          //ÿ������������
#define S5          0x0500          //ÿ����ƻ���������(ģʽ,Ŀ���ٶ�,Kpp,Kvp��)
#define S6          0x0600          //Эͬ�˶���������
#define S7          0x0700          //�����������
#define S8          0x0800          //ƽ̨��������
                                    //
#define IL          0X8000          //�ϵ��ʼ��
#define IFRW        0X4000          //�ϵ�ִ��һ�ζ�д����(�����)
#define SCL         0X2000          //��߶ȱ�����ʶ
#define CDPC_O      0x1000          //����������֡ʱ��ֻʹ��CDPC��ʽ

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
    INT16U  Index;          //������
    INT8U   SubIndex;       //������
    INT8U   DataType;       //��������,����������������
    void    *pOD;           //ָ�����ָ��
    INT16U  Property;       //OD����,ָʾ��ʼ��,����,��,д,ID��
    INT32S  Max;            //����ֵ
    INT32S  Min;            //����ֵ
    INT32S  DefVal;         //Ĭ��ֵ,һ���������Ҫ����Ĳ�������Ĭ��ֵ֮˵
    INT16U  EromAddress;    //�洢,��ʼ����ȡEPPROM��ַ
    void    *bpOD;          //���ز���
    void    *rpOD;          //OD�Ĺ�������,����˵�趨��ѹ:pOD���趨ֵ,rpOD����ʵ�����趨����ֵ.
    //INT16U  *spOD;          //��߶ȱ���(10000),������16bit�޷��ű���
    INT16U  Command;        //д��ַ��ָ��,���ݾ���OD����
    INT8U   (*pF_R)(INT16U Command,void *pOD, void *bpOD, void *rpOD);  //������,�������������*pOD,*rpODһ��������Ӧת�����ֵ(����ADת�����Ӧʵ�ʵ�ѹֵmV)
    INT8U   (*pF_W)(INT16U Command,void *pOD, void *bpOD, void *rpOD);  //д��ִ�к���(����ִ�����ֵ����bpOD,rpODֻ�������,�ʹ���״̬����)
} OD_Attribute;
typedef union
{
    INT8U All[10];          //һ֡���ݵĻ�����
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
    //INT8U NewErrFlag; //���´�����(����Ƿ���¼����ʷ������,1:��û��; 0:����¼������)
    INT8U NewErr;       //ϵͳ�´���״̬
    INT8U LastErr[10];  //��ʷ����
} SYS_ERR_DEF;

//EEPROM
typedef struct
{
    INT16U StartAddr;   //��д����ʼ��ַ
    INT16U NumBytesRW;  //��д�ֽ���
    INT16U CommandRW;   //��дָ��
    INT8U DataBuff[32]; //��д���ݻ�����
} EEPROM_RW_DEF;

//����������ز���
typedef struct
{
    INT8U   MyID;               //����ID��
    INT8U   MyID_Reg;           //����ID���ɱ��޸ĵ�
    INT8U   NowMode;            //ͨѶ����ģʽ
    INT8U   NowModeReg;         //ͨѶ����ģʽ���ɱ��޸ĵ�
    INT8U   RS232_1_Baudrate;   //now is uart3
    INT8U   RS232_2_Baudrate;   //now is uart1
    INT16U  OD_Sum;             //��OD����
    INT32U  FirmwareVersion;    //�̼��汾����
    INT32U  HardwareVersion;    //Ӳ���汾 eg��H2V2
} LOCAL_PROPERTY_INFO_DEF;

#define APP_TIME_BASE_NUM   10

//save group define
typedef struct
{
    INT8U   SaveComInfoCmd;     //S1
    INT8U   SaveIoCfgCmd;       //S2
    INT8U   SaveDeviceCalibCmd; //S3
    INT8U   SaveMotorPraCmd;    //S4�洢�������
    INT8U   SaveCtrLpPraCmd;    //S5�洢���ƻ�����
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
    INT8U   BitCtrl;        //����ɽ�����;1:�����ס�����; 0:�ɿ������
    INT16U  LockVal;        //��סֵ����
    INT16U  UnLockVal;      //�ɿ�ֵ����
    INT16U* pValData;      //ʵ�����ָ��2,ָ��SPI������
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

    INT16U* pValData;       //ָ��SPI������
}AgvSensorTypedef;

//ϵͳ������Ϣ��ر���(��־λ)
typedef struct
{
    INT16U  SaveFlag;           //���ֱ�־λ
    INT16U  SoftRestartFlag;    //���������־λ��0xaa55��
    INT16U  LoadDefVal;         //װ��Ĭ�϶���Ĳ���,0xD100,0xD200,0xD400,0XD800
    INT16U  ErrClearFlag;       //����λ��־λ:0x86
    union App_Time_Base_Union   //Ӧ���¼�ʱ�������ṹ��
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

    //led�ͷ���������״̬״̬����������
    INT8U LB_NowState;
    INT16U LB_Ctl;

    //ͨ����Ϣ
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
    //�״�����
    RadarTypedef Radar;
    //���
    SteEngTypedef   SteEng[2];
    //AGV sensor
    AgvSensorTypedef AgvSnr;
    //�洢
    DeviceSaveTypedef DS;
    //ֱ�����
    //DcMotorCtrlTypedef DM_Ctrl;
} SYS_SERVICE_INFO_DEF;

//ϵͳ״����ر���
typedef struct
{
    INT8U   CPU_Use;            //CPUʹ����
    INT8U   PowerDownMsg;       //
    INT8U   ErrorMsg;
    INT8U   HistoryErrPos;      //��ʷ����λ��

    INT32U  ErrFlag;            //ϵͳ������λ
    INT32U    ErrFlagReg;       //ϵͳ������λ�ݴ�
    INT32U  ErrCutoffMask;      //������ʱ���ж�����λ
    INT32U  ErrAlarmMask;       //������ʱ�����ⱨ������λ
    INT16U  OD_DupNum;          //OD��ַ�ظ�����
    INT32U  OD_DupInfo[5];      //�ظ���OD��Ϣ,����ֻ��ͷ5����¼�ռ�
    INT16U  ActiveId;
    INT16U  ActiveIdReg;
    INT16U  ActiveIdSpc;        //search id for oscilloscope
    INT16U  IdSpcManualSet;     //if equal to 1,do not auto updata ActiveIdSpc

    INT16U  RelCntLockFlag;     //�̵�������ʱ���ź�

    struct
    {
        union TotalOnlineTimeUnion
        {
            INT32U Total;       //������ʱ��,��λ:[S]
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
            INT8U  PwrOnCalthFlg;   //�ϵ�OffsetУ׼��ϱ�־λ
            INT8U  SelfCheckFlg;    //�ϵ��Լ������־λ
        } Split;
    } Ready;

} SYS_STATE_INFO_DEF;

typedef struct
{
    GPIO_TypeDef  *DirGPIO;
    uint16_t      DirPinNum;
} DIRGPIO_STRUCT_DEF;


//��ʷ�����¼�ṹ��
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
    INT16U  OptEn;              //�ű���������ʹ��
    INT16U  OptEnCmd;           //�ű���������ʹ��
    INT8U   OptCnt;             //�ű���������������
    INT8U   Flag;               //���

    INT8U   SpiTxNumFrm;        //spi����֡����
    INT8U   SpiRxNumFrm;        //spi����֡����
    INT16U  StrTrigComp;        //�����ű�������������ʱ��ֵ
    INT16U  GetDataComp;        //��ȡ�ű���������ʱ��ֵ
    INT16U  SpiTxBuff[4];       //SPI���ͻ�����
    INT16U  SpiRxBuff[4];       //SPI���ջ�����

    //ÿʹ��һ��,����һ�εĲ���
    INT8U   OT_SpiFlg;          //����Ƿ���EEPROM������
    INT8U   OT_SpiTxOffset;     //����ƫ��
    INT32U  OT_SpiCmdSlt;       //CDFѡ��
    INT8U   OT_SpiTxNumFrm;     //spi����֡����
    INT8U   OT_SpiRxNumFrm;     //spi����֡����
    INT16U  OT_StrTrigComp;     //�����ű�������������ʱ��ֵ
    INT16U  OT_GetDataComp;     //��ȡ�ű���������ʱ��ֵ
    INT16U  OT_SpiTxBuff[4];    //SPI���ͻ�����
    INT16U  OT_SpiRxBuff[4];    //SPI���ջ�����

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
//ͨ�ŷ�����ر���
extern SDO_BUFF_DEF SdoBuf;
//����������ر�������
extern SYS_STATE_INFO_DEF   SysState;       //ϵͳ״̬��ر���
extern SYS_SERVICE_INFO_DEF   SysService;     //ϵͳ������ر���
extern LOCAL_PROPERTY_INFO_DEF  LocalProperty;  //�����������
extern SYS_ERR_DEF SysError;
extern EEPROM_RW_DEF EepromRW;
//OD�б����
extern const OD_Attribute OD_ADD[];
//��ʷ������ر���
extern HISTORY_ERR_DEF  HistoryErr[HISTORY_ERR_NUM];
//����
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
