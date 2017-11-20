#ifndef _CTRL_LOOP_H
#define _CTRL_LOOP_H

typedef struct
{
    INT16U  Wheelbase;      //С���������,��λmm
    INT16U  WheelDia;       //С������ֱ��,��λmm
    INT16U  GearF;          //��������ַ���
    INT16U  GearD;          //��������ַ�ĸ
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
    INT32U  ProfileAcceCmd;     //���ٶ�
    INT32U  ProfileDeceCmd;     //���ٶ�
    INT32U  ProfileAcceAct;     //��Ч�ļ��ٶ�
    INT32U  ProfileDeceAct;     //��Ч�ļ��ٶ�
    INT32U  CmdProfileSpeed;    //�����ٶ�ָ��
    INT32S  ProfilePcn;
    INT32S  ProfilePcn_1;
    INT32S  RemainPcn;    //ʣ��δ�������
    INT32S  ProfileVcn;
    INT32S  ProfileVcn_1;
    INT32S  ProfilePcnFlt;
    INT32U  ProfilePAdece;      //���ٶ������λ��

}TurnAngleStructTypedef;
/*
 * SR2D1_0V3_MCU_S1 -- Service robot 2In1 servo driver
 * (C)2016 Kinco Electric (Shenzhen) Ltd. -- All rights reserved
 */
typedef struct
{
    INT32S  CmdPosMm;    //��������,��λmm
    INT32S  RemainPosMm;    //ʣ��δ�������
}MotionPosLenthTypedef;

typedef struct
{
    INT8U   MotionEnable;   //Эͬ�˶�ʹ��1:ʹ��
    INT8S   MotionMode;     //Эͬ�˶�ģʽ
    INT8S   MotionModeAct;  //Эͬ�˶���Чģʽ
    INT8U   RadiusUpdataTrig;//�뾶���ı�־
    INT8U   RadiusUpdataMode;//�뾶��������ģʽ
    INT16U  MotionCtrWord;  //Эͬ�˶�������
    INT16U  MotionCtrWordAct;  //Эͬ�˶���Ч������
    INT8U   SwtichOnLock;    //Эͬ�˶�˫��ͬʱ�ϵ�����ǣ������һ��������д���պñ��ж�,
    INT8U   ParUpdataLock;  //Эͬ�������������,����δͬʱ����
}MotionGlobleFlagTypedef;

typedef struct
{
    INT32U  ProfileAcceCmd;     //���ٶ�
    INT32U  ProfileDeceCmd;     //���ٶ�
    INT32U  ProfileAcceCmd6622_1;   //22,66ģʽ�¼��ٶ�
    INT32U  ProfileAcceCmd6622_2;   //22,66ģʽ�¼��ٶ�
    INT32U  ProfileDeceCmd6622_1;   //22,66ģʽ���ٶ�
    INT32U  ProfileDeceCmd6622_2;   //22,66ģʽ���ٶ�
    INT32S  CmdRadius;             //22��66ģʽ��Բ���뾶
    INT32S  Radius;             //22��66ģʽ��Բ���뾶
    INT32S  CmdSpeedDec;            //�ٶ�ָ��
    INT32S  CmdSpeedMmps;           //�ٶ�ָ��
    INT32U  CmdProfileSpeed;        //�����ٶ�ָ��
    INT32U  CmdProfileSpeedMmps;    //�����ٶ�ָ��
    INT32S  CmdSpeedDec6622_1;      //22,66ģʽ���ٶ�
    INT32S  CmdSpeedDec6622_2;      //22,66ģʽ�ٶ�
    INT32U  CmdProfileSpeed6622_1;   //22,66ģʽ���ٶ�
    INT32U  CmdProfileSpeed6622_2;   //22,66ģʽ�ٶ�
    INT32U  CmdQkStopDece;          //����ֹͣ���ٶ�
    INT32U  CmdQkStopDece6622_1;   //22,66ģʽ����ֹͣ���ٶ�
    INT32U  CmdQkStopDece6622_2;   //22,66ģʽ����ֹͣ���ٶ�

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
