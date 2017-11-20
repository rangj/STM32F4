/*
 * SR2D1_0V3_MCU_S1 -- Service robot 2In1 servo driver
 * (C)2016 Kinco Electric (Shenzhen) Ltd. -- All rights reserved
 */
#ifndef _Work_H
#define _Work_H

typedef struct
{   //Index.......Function
    INT16U  Index;   //索引
    void (*pFunc)(INT16U command);  //功能函数
} DinFuncAttribute;
extern const DinFuncAttribute DinFuncADD[];

void DinFuncHandle(void);


extern  INT16U  isqrt32(INT32U x);
extern  INT8U   rOnlineTimeInfo(INT16U command, void *pOD, void *bpOD, void *rpOD);
extern  INT8U   wDefaultValueLoad(INT16U command, void *pOD, void *bpOD, void *rpOD);
extern  INT8U   wControlWord(INT16U command, INT16U *pOD, void *bpOD, DriverStructTypedef *rpOD);
extern  INT8U   wCmdProfileSpeed(INT16U command, INT16U *pOD, void *bpOD, DriverStructTypedef *rpOD);
extern  INT8U   wSpeedLoopLF_Coff(INT16U command, INT8U *pOD, void *bpOD, DriverStructTypedef *rpOD);
extern  INT8U   wOT_SpiTxBuff(INT16U command, INT32U *pOD, void *bpOD, void *rpOD);
extern  INT8U   wCalActAcceAndDecelerate(INT16U command, INT16U *pOD, INT32U *bpOD, INT32U *rpOD);
extern  INT8U   wCalActAcceletate(INT16U command, INT16U *pOD, void *bpOD, DriverStructTypedef *rpOD);
extern  INT8U   wCalActDecelerate(INT16U command, INT16U *pOD, void *bpOD, DriverStructTypedef *rpOD);
extern  INT8U   rRealIIt_D512(INT16U command, INT16U *pOD, void *bpOD, INT32U *rpOD);
extern  INT8U   rRealDM_IItmA(INT16U command, INT16U *pOD, void *bpOD, IItParameterTypedef *rpOD);
extern  INT8U   wSetDC_BusUnderVolt(INT16U command, INT16U *pOD, void *bpOD, DriverStructComTypedef *rpOD);
extern  INT8U   wSetDC_BusOverVolt(INT16U command, INT16U *pOD, void *bpOD, DriverStructComTypedef *rpOD);
extern  INT8U   wSetChopVolt(INT16U command, INT16U *pOD, void *bpOD, DriverStructComTypedef *rpOD);
extern  INT8U   wCmdSpeedMaxRpm(INT16U command, INT16S *pOD, void *bpOD, DriverStructTypedef *rpOD);
extern  INT8U   wSetMotorIIt_I(INT16U command, INT16U *pOD, void *bpOD, DriverStructTypedef *rpOD);
extern  INT8U   wSetMotorIIt_T(INT16U command, INT16U *pOD, void *bpOD, DriverStructTypedef *rpOD);
extern  INT8U   wSetDC_MotorIIt_I(INT16U command, INT16U *pOD, void *bpOD, IItParameterTypedef *rpOD);
extern  INT8U   wSetDC_MotorIIt_T(INT16U command, INT16U *pOD, void *bpOD, IItParameterTypedef *rpOD);
extern  INT8U   wCmdStepSpeed(INT16U command, INT16U *pOD, void *bpOD, DriverStructTypedef *rpOD);

extern  INT8U   wMotionEnable(INT16U command, INT8U *pOD, void *bpOD, MotionStructTypedef *rpOD);
extern  INT8U   wSetTurnAngle(INT16U command, INT32S *pOD, void *bpOD, MotionStructTypedef *rpOD);
extern  INT8U   rRemainTurnAngle(INT16U command, INT32S *pOD, void *bpOD, MotionStructTypedef *rpOD);
extern  INT8U   wSetPosLengthMm(INT16U command, INT32S *pOD, void *bpOD, MotionStructTypedef *rpOD);
extern  INT8U   rRemainPosLengthMm(INT16U command, INT32S *pOD, void *bpOD, DriverStructTypedef *rpOD);
extern  INT8U   wSetMotionSpeed(INT16U command, INT32S *pOD, void *bpOD, void *rpOD);
extern  INT8U   wSetMotionProfileSpeed(INT16U command, INT32S *pOD, void *bpOD, void *rpOD);
extern  INT8U   wSetMotionRadiusMm(INT16U command, INT32S *pOD, void *bpOD, void *rpOD);
extern  INT8U   wMotionControlWord(INT16U command, INT32S *pOD, void *bpOD, MotionStructTypedef *rpOD);

extern  INT8U   wDC_MotorControlWord(INT16U command, INT16U *pOD, void *bpOD, void *rpOD);
extern  INT8U   wDC_MotorDirectionAndSpeed(INT16U command, void *pOD, void *bpOD, DcMotorCtrlTypedef *rpOD);
extern  INT8U   wPlatformCalibration(INT16U command, INT8U *pOD, void *bpOD, PlatformInfoTypedef *rpOD);
extern  INT8U   rPlatformHighAct(INT16U command, INT16S *pOD, void *bpOD, PlatformInfoTypedef *rpOD);
extern  INT8U   rPlatformDownLimitMm(INT16U command, INT16S *pOD, void *bpOD, PlatformInfoTypedef *rpOD);
extern  INT8U   rPlatformUpLimitMm(INT16U command, INT16S *pOD, void *bpOD, PlatformInfoTypedef *rpOD);
extern  INT8U   wPlatformDownLimitMm(INT16U command, INT16S *pOD, void *bpOD, PlatformInfoTypedef *rpOD);
extern  INT8U   wPlatformUpLimitMm(INT16U command, INT16S *pOD, void *bpOD, PlatformInfoTypedef *rpOD);

extern  INT8U   wSingleDoutSimulate(INT16U command, INT8U *pOD, void *bpOD, INT16U *rpOD);
extern  INT8U   rAnalogInputRealVoltage(INT16U command, INT16S *pOD, void *bpOD, AnalogGroupBaseTypedef *rpOD);
#endif
