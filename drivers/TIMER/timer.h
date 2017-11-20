#ifndef _TIMER_H
#define _TIMER_H
#include "stm32f4xx.h"
//////////////////////////////////////////////////////////////////////////////////
//TXW����д���ٶ����ۡ�	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;	   
//                                                                   �޸���BY TXW								  
//////////////////////////////////////////////////////////////////////////////////
#define ENC1_A_PORT		GPIOA
#define ENC1_A_PIN		GPIO_Pin_5
#define ENC1_A_AF		GPIO_PinSource5

#define ENC1_B_PORT		GPIOB
#define ENC1_B_PIN		GPIO_Pin_3
#define ENC1_B_AF		GPIO_PinSource3

#define ENC2_A_PORT		GPIOB
#define ENC2_A_PIN		GPIO_Pin_6
#define ENC2_A_AF		GPIO_PinSource6

#define ENC2_B_PORT		GPIOB
#define ENC2_B_PIN		GPIO_Pin_7
#define ENC2_B_AF		GPIO_PinSource7

#define DM_PWM_FCY      16000               //DC_Motor FCY = 16KHz, PWM frequncy must > 1.3KHz, for TIM1.ARR <= 0XFFFF
#define DM_PWM_PERIOD (168000000/DM_PWM_FCY)

#define PWM_PERIOD	  (168000000/DM_PWM_FCY)//Motor FCY = 16KHz, PWM frequncy must > 1.3KHz, for TIM1.ARR <= 0XFFFF

void TIM2_ENC1_Init(void);
void TIM4_ENC2_Init(void);
void TIM9_DC_MOTOR_Init(void);
void DC_MotorOn(void);
void TIM1_Driver1_Init(void);
void TIM3_Int_Init(u16 arr,u16 psc);


#endif
