/*
 * SR2D1_0V3_MCU_S1 -- Service robot 2In1 servo driver
 * (C)2016 Kinco Electric (Shenzhen) Ltd. -- All rights reserved
 */
#ifndef  _TIME_H
#define  _TIME_H
#define PWM_FCY         16000               //FCY = 16KHz, PWM frequncy must > 1.3KHz, for TIM1.ARR <= 0XFFFF
#define PWM_PERIOD (168000000/PWM_FCY/2)
#define DM_PWM_FCY      16000               //FCY = 16KHz, PWM frequncy must > 1.3KHz, for TIM1.ARR <= 0XFFFF
#define DM_PWM_PERIOD (168000000/DM_PWM_FCY)
extern void InitTIM4(void);
extern void InitTIM1(void);
extern void InitTIM3(void);
extern void InitTIM2(void);
extern void InitTIM8(void);
extern void InitTIM9(void);
#endif
//
