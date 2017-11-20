#ifndef _CTRL_LOOP_H
#define _CTRL_LOOP_H

typedef struct
{
    INT16U  Wheelbase;      //小车两轮轴距,单位mm
    INT16U  WheelDia;       //小车轮子直径,单位mm
    INT16U  GearF;          //减速箱齿轮分子
    INT16U  GearD;          //减速箱齿轮分母
}CarMachineParStructTypedef;

typedef struct
{
    INT8U  Enable;
    INT8U  QkStopTrig;
    INT32S CmdAngle;
    INT32S CmdSpeed;
    INT32S CmdPosOut;
    INT32S CmdPos;
    INT32S CmdCmdPos;
    INT32S CmdPosAct;
    INT32S CmdDeltaPosOut;
    INT32U  ProfileAcceCmd;     //加速度
    INT32U  ProfileDeceCmd;     //减速度
    INT32U  ProfileAcceAct;     //有效的加速度
    INT32U  ProfileDeceAct;     //有效的减速度
    INT32U  CmdProfileSpeed;    //梯形速度指令
    INT32S  ProfilePcn;
    INT32S  ProfilePcn_1;
    INT32S  RemainPcn;    //剩余未走完里程
    INT32S  ProfileVcn;
    INT32S  ProfileVcn_1;
    INT32S  ProfilePcnFlt;
    INT32U  ProfilePAdece;      //减速段所需的位置

}TurnAngleStructTypedef;
/*
 * SR2D1_0V3_MCU_S1 -- Service robot 2In1 servo driver
 * (C)2016 Kinco Electric (Shenzhen) Ltd. -- All rights reserved
 */
typedef struct
{
    INT32S  CmdPosMm;    //定长设置,单位mm
    INT32S  RemainPosMm;    //剩余未走完里程
}MotionPosLenthTypedef;

typedef struct
{
    INT8U   MotionEnable;   //协同运动使能1:使能
    INT8S   MotionMode;     //协同运动模式
    INT8S   MotionModeAct;  //协同运动有效模式
    INT8U   RadiusUpdataTrig;//半径更改标志
    INT8U   RadiusUpdataMode;//半径更新运行模式
    INT16U  MotionCtrWord;  //协同运动控制字
    INT16U  MotionCtrWordAct;  //协同运动有效控制字
    INT8U   SwtichOnLock;    //协同运动双轴同时上电锁标记，避免第一个轴命令写入后刚好被中断,
    INT8U   ParUpdataLock;  //协同参数更新锁标记,避免未同时更新
}MotionGlobleFlagTypedef;

typedef struct
{
    INT32U  ProfileAcceCmd;     //加速度
    INT32U  ProfileDeceCmd;     //减速度
    INT32U  ProfileAcceCmd6622_1;   //22,66模式下加速度
    INT32U  ProfileAcceCmd6622_2;   //22,66模式下加速度
    INT32U  ProfileDeceCmd6622_1;   //22,66模式减速度
    INT32U  ProfileDeceCmd6622_2;   //22,66模式减速度
    INT32S  CmdRadius;             //22，66模式下圆弧半径
    INT32S  Radius;             //22，66模式下圆弧半径
    INT32S  CmdSpeedDec;            //速度指令
    INT32S  CmdSpeedMmps;           //速度指令
    INT32U  CmdProfileSpeed;        //梯形速度指令
    INT32U  CmdProfileSpeedMmps;    //梯形速度指令
    INT32S  CmdSpeedDec6622_1;      //22,66模式下速度
    INT32S  CmdSpeedDec6622_2;      //22,66模式速度
    INT32U  CmdProfileSpeed6622_1;   //22,66模式下速度
    INT32U  CmdProfileSpeed6622_2;   //22,66模式速度
    INT32U  CmdQkStopDece;          //快速停止减速度
    INT32U  CmdQkStopDece6622_1;   //22,66模式快速停止减速度
    INT32U  CmdQkStopDece6622_2;   //22,66模式快速停止减速度

}MotionParTypedef;

typedef struct
{
    CarMachineParStructTypedef CarMchPar;
    MotionPosLenthTypedef PosLength;
    TurnAngleStructTypedef TurnAngle;
    MotionGlobleFlagTypedef GbFlg;
    MotionParTypedef MotionPar;
}MotionStructTypedef;
extern MotionStructTypedef  MOX;

//using print to  design loop cotrol
extern void LoopControl(DriverStructTypedef* pDX);
extern void MotionCtrol(void);
//DC motor control
extern void DC_MotorControl(void);
#endif
