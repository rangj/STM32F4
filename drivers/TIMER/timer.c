#include "timer.h"


/********************************************************************
*�������ƣ� void TIM2_ENC1_Init(void)
*�������ܣ�	���ö�ʱ��������
*���������	none
*���ز�����	��
*������ע�� 
*********************************************************************/
void TIM2_ENC1_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;	

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA |RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	GPIO_PinAFConfig(ENC1_A_PORT, ENC1_A_AF, GPIO_AF_TIM2);
    GPIO_PinAFConfig(ENC1_B_PORT, ENC1_B_AF, GPIO_AF_TIM2);

	// Configure TIM2 as alternate function
    GPIO_InitStructure.GPIO_Pin = ENC1_A_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(ENC1_A_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = ENC1_B_PIN;
    GPIO_Init(ENC1_B_PORT, &GPIO_InitStructure);

 	// Time base configuration
    //TIM_TimeBaseStructure.TIM_Period = 9999;
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_CounterModeConfig(TIM2, TIM_CounterMode_Up);

	//TIM_ETRConfig(TIM4,TIM_ExtTRGPSC_OFF,TIM_ExtTRGPolarity_NonInverted,4);
    TIM_EncoderInterfaceConfig(TIM2,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);
    //TIM_EncoderInterfaceConfig(TIM2,TIM_EncoderMode_TI1,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 4;
    TIM_ICInit(TIM2, &TIM_ICInitStructure);

	TIM_SetCounter(TIM2, 0);
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    TIM_Cmd(TIM2, ENABLE);
}
/********************************************************************
*�������ƣ� void TIM4_ENC2_Init(void)
*�������ܣ�	���ö�ʱ��������
*���������	none
*���ز�����	��
*������ע�� 
*********************************************************************/
void TIM4_ENC2_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;	

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	GPIO_PinAFConfig(ENC2_A_PORT, ENC2_A_AF, GPIO_AF_TIM4);
    GPIO_PinAFConfig(ENC2_B_PORT, ENC2_B_AF, GPIO_AF_TIM4);

	// Configure TIM2 as alternate function
    GPIO_InitStructure.GPIO_Pin = ENC2_A_PIN |ENC2_B_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(ENC2_A_PORT, &GPIO_InitStructure);

 	// Time base configuration
    //TIM_TimeBaseStructure.TIM_Period = 9999;
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
    TIM_CounterModeConfig(TIM4, TIM_CounterMode_Up);

	//TIM_ETRConfig(TIM4,TIM_ExtTRGPSC_OFF,TIM_ExtTRGPolarity_NonInverted,4);
    TIM_EncoderInterfaceConfig(TIM4,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);
    //TIM_EncoderInterfaceConfig(TIM2,TIM_EncoderMode_TI1,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 4;
    TIM_ICInit(TIM4, &TIM_ICInitStructure);

	TIM_SetCounter(TIM4, 0);
    TIM_ClearFlag(TIM4, TIM_FLAG_Update);
    TIM_Cmd(TIM4, ENABLE);
}
/********************************************************************
*�������ƣ� void TIM9_DC_MOTOR_Init(void)
*�������ܣ�	����DC Motor PWM
*���������	none
*���ز�����	��
*������ע�� 
*********************************************************************/
void TIM9_DC_MOTOR_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE); //���� 168M (2>1?)

	GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_TIM9);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource6, GPIO_AF_TIM9);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

	TIM_TimeBaseStructure.TIM_Period = DM_PWM_PERIOD;  //168000000/10500=16000Hz pwm;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;       //����Ƶ
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //
    TIM_TimeBaseInit(TIM9, &TIM_TimeBaseStructure);	//TIM9_CounterMode only one mode: edge-aligned mode and upcount

	//����оƬ�ߵ�ƽ��Ч
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;  //CNT<PULSE ��ʱ��Ϊ�ߵ�ƽ��
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCNPolarity_High;

    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;

	TIM_OCInitStructure.TIM_Pulse = DM_PWM_PERIOD/2;  //50% PWM
    TIM_OC1Init(TIM9, &TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_Pulse = DM_PWM_PERIOD/2;  //50% PWM
    TIM_OC2Init(TIM9, &TIM_OCInitStructure);
    TIM_SetCounter(TIM9, DM_PWM_PERIOD/2);
}

//dc motor on / off
void DC_MotorOn(void)
{
    TIM_SetCounter(TIM9, DM_PWM_PERIOD/2);
    TIM_Cmd(TIM9, ENABLE);
	TIM9 -> CCR1 = DM_PWM_PERIOD/2 +3000;
	TIM9 -> CCR2 = DM_PWM_PERIOD/2 -3000;

}
/********************************************************************
*�������ƣ� void TIM1_Driver1_Init(void)
*�������ܣ�	����Driver1 PWM
*���������	none
*���ز�����	��
*������ע�� 
*********************************************************************/
void TIM1_Driver1_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    TIM_BDTRInitTypeDef TIM_BDTRInitStructure;	  // ������ɲ����������
    NVIC_InitTypeDef  NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); //168M

    // Connect PE15 to TIM1_BKIN
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource15, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource8, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource10, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource12, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_15;         //TIM1_BKIN
    GPIO_Init(GPIOE, &GPIO_InitStructure);

	TIM_TimeBaseStructure.TIM_Period = PWM_PERIOD;  //168000000/pwm_fcy;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;       //����Ƶ
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //
    //TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned3;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 1;//see stm32f4 manuel page 305 shows the meaning
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;  //CNT<PULSE ��ʱ��Ϊ�ߵ�ƽ��
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    //����оƬ�ߵ�ƽ��Ч
    //TIM1_CHxN:�������ű�,TIM1_CHx�������ű�,
    //(PWM_PERIOD - SVPWM���)��ΪCCRxֵ
    //���������������űۿ�ͨ�����ĵ�(TIM1_UP_IRQn)
    //TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;  //CNT<PULSE ��ʱ��Ϊ�ߵ�ƽ��
    //TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    //TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_Pulse = PWM_PERIOD/2;  //50% PWM
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_Pulse = PWM_PERIOD/2;  //50% PWM
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_Pulse = PWM_PERIOD/2;  //50% PWM
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_Pulse = PWM_PERIOD/2;  //50% PWM
    TIM_OC4Init(TIM1, &TIM_OCInitStructure);

	// Automatic Output enable, Break, dead time and lock configuration
    TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable; // don't know what does this use for
    TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
    TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1;
    TIM_BDTRInitStructure.TIM_DeadTime = 126;  //750ns, 126/168=0.75us
    TIM_BDTRInitStructure.TIM_Break = TIM_Break_Enable;
    TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_Low;
    TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;
    TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_TIM9_IRQn;//TIM1_BRK_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

	TIM_ClearFlag(TIM1, TIM_FLAG_Break);    //Clear the Beak interrupt flag
    TIM_ClearITPendingBit(TIM1, TIM_FLAG_Break);
    TIM_ITConfig(TIM1,TIM_IT_Break,ENABLE);
    TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Update);//used for ADC sample,but here only read adc

	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;//TIM1_UP_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

}
/********************************************************************
*�������ƣ� void TIM3_Int_Init(u16 arr,u16 psc)
*�������ܣ�	��ʱ��3��ʼ��
*���������	u16 arr,u16 psc 
*���ز�����	��
*������ע�� Tout=((arr+1)*(psc+1))/Ft us. Ft��������Ϊ84MHZ
*********************************************************************/
void TIM3_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	
  	TIM_TimeBaseInitStructure.TIM_Period = arr; 	//�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);//��ʼ��TIM3
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //����ʱ��3�����ж�
	TIM_Cmd(TIM3,ENABLE); //ʹ�ܶ�ʱ��3
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn; //��ʱ��3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}

/********************************************************************
*�������ƣ�void TIM3_IRQHandler(void) 
*�������ܣ���ʱ��3�жϷ�����	
*�����������
*���ز�������
*������ע��CAN������Ϣ��������Ƕ��������жϺ�������
*********************************************************************/
void TIM3_IRQHandler(void)
{

	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) //����ж�
	{
			  ;
	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //����жϱ�־λ
}
/*
void TIM1_UP_IRQHandler(void)
{
    if(TIM_GetFlagStatus(TIM1, TIM_FLAG_Update) != RESET)
    {
       // ScopeSampleDeal();
        //TIM_ClearFlag(TIM1, TIM_FLAG_Update);   //Clear the Updata interrupt flag(must)
       // TIM_ClearITPendingBit(TIM1, TIM_FLAG_Update);
	   ;
    }
}  */



