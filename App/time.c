/*
 * SR2D1_0V3_MCU_S1 -- Service robot 2In1 servo driver
 * (C)2016 Kinco Electric (Shenzhen) Ltd. -- All rights reserved
 */
#include "includes.h"

//***************************************************************
// Function    : InitTIM2
// Description : init timer2 for driver1 QEI
//***************************************************************
void InitTIM2(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    //GPIO configer for timer2
    // Enable GPIO clock
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    // Enable Timer2 clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    // Connect PA5 to TIM2_CH1
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_TIM2);
    // Connect PB3 to TIM2_CH2
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_TIM2);
    // Configure TIM2 as alternate function
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
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
//***************************************************************
// Function    : InitTIM4
// Description : init timer4 for driver2  QEI
//***************************************************************
void InitTIM4(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    //GPIO configer for timer1
    // Enable GPIO clock
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    // Enable Timer3 clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    // Connect PC6 to TIM3_CH1
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);
    // Connect PE11 to TIM4_CH2
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);
    // Configure TIM4 as alternate function
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
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
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 4;
    TIM_ICInit(TIM4, &TIM_ICInitStructure);
    TIM_SetCounter(TIM4, 0);
    TIM_ClearFlag(TIM4, TIM_FLAG_Update);
    TIM_Cmd(TIM4, ENABLE);

}

//***************************************************************
// Function    : InitTIM9
// Description : for DC motor control PWM
//***************************************************************
void InitTIM9(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    //GPIO configer for timer1
    // Enable GPIO clock
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    // Enable Timer9 clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
    // Connect PE5 to TIM9_CH1
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_TIM9);
    // Connect PE6 to TIM9_CH2
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource6, GPIO_AF_TIM9);
    // Configure TIM9 as alternate function
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

        // Time base configuration
    TIM_TimeBaseStructure.TIM_Period = DM_PWM_PERIOD;  //168000000/10500=16000Hz pwm;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;       //不分频
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //
    //TIM9_CounterMode only one mode: edge-aligned mode and upcount
    TIM_TimeBaseInit(TIM9, &TIM_TimeBaseStructure);
    //驱动芯片高电平有效
    //TIM1_CHxN:控制下桥臂,TIM1_CHx控制上桥臂,
    //电流采样点在下桥臂开通的中心点(TIM1_UP_IRQn)
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;  //CNT<PULSE 的时候，为高电平，
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCNPolarity_High;

    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;

    TIM_OCInitStructure.TIM_Pulse = DM_PWM_PERIOD/2;  //50% PWM
    TIM_OC1Init(TIM9, &TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_Pulse = DM_PWM_PERIOD/2;  //50% PWM
    TIM_OC2Init(TIM9, &TIM_OCInitStructure);
    TIM_SetCounter(TIM9, DM_PWM_PERIOD/2);
}

//***************************************************************
// Function    : InitTIM3
// Description : init timer3 for A/B P/D CW/CCW count
//***************************************************************
void InitTIM3(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    //GPIO configer for timer1
    // Enable GPIO clock
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    // Enable Timer3 clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    // Connect PC6 to TIM3_CH1
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);
    // Connect PE11 to TIM4_CH2
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM3);
    // Configure TIM4 as alternate function
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    // Time base configuration
    //TIM_TimeBaseStructure.TIM_Period = 9999;
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    TIM_CounterModeConfig(TIM3, TIM_CounterMode_Up);
    TIM_EncoderInterfaceConfig(TIM3,TIM_EncoderMode_TI12,TIM_ICPolarity_BothEdge,TIM_ICPolarity_Rising);
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 4;
    TIM_ICInit(TIM3, &TIM_ICInitStructure);
    TIM_SetCounter(TIM3, 0);
    TIM_ClearFlag(TIM3, TIM_FLAG_Update);
    TIM_Cmd(TIM3, ENABLE);
}


//***************************************************************
// Function    : InitTIM1
// Description : init timer1 produce complementary pwm
//***************************************************************
void InitTIM1(void)
{
    //GPIO assign for timer1, pulse will output from pc6
    GPIO_InitTypeDef  GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;
    //GPIO configer for timer1
    // Enable GPIO clock
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    // Connect PE15 to TIM1_BKIN
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource15, GPIO_AF_TIM1);
    // Connect PE8 to TIM1_CH1N
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource8, GPIO_AF_TIM1);
    // Connect PE9 to TIM1_CH1
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);
    // Connect PE10 to TIM1_CH2N
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource10, GPIO_AF_TIM1);
    // Connect PE11 to TIM1_CH2
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);
    // Connect PE12 to TIM1_CH3N
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource12, GPIO_AF_TIM1);
    // Connect PE13 to TIM1_CH3
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);
    // Connect PC9 to TIM1_CH4 for DCmotor pwm2
    //GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM8);

    // Configure TIM1 as alternate function
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10\
                                   | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_15;         //TIM1_BKIN
    GPIO_Init(GPIOE, &GPIO_InitStructure);
    // Enable Timer1 clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);//  high level 20000,low level 100
    // Time base configuration
    TIM_TimeBaseStructure.TIM_Period = PWM_PERIOD;  //168000000/pwm_fcy;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;       //不分频
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //
    //TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned3;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 1;//stm32f4 manuel page 305 shows the meaning
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
    // Channel 1, 2 and 3 Configuration in PWM mode
    //驱动芯片高电平有效
    //TIM1_CHxN:控制上桥臂,TIM1_CHx控制下桥臂,
    //(PWM_PERIOD - SVPWM输出)作为CCRx值
    //电流采样点在下桥臂开通的中心点(TIM1_UP_IRQn)
    //TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;  //CNT<PULSE 的时候，为高电平，
    //TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    //TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
    //驱动芯片低电平有效
    //TIM1_CHxN控制下桥臂,TIM1_CHx控制上桥臂,
    //(PWM_PERIOD - SVPWM输出)作为CCRx值
    //电流采样点在下桥臂开通的中心点(TIM1_UP_IRQn)
    //TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;  //CNT<PULSE 的时候，为低电平
    //TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    //TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
    //驱动芯片高电平有效
    //TIM1_CHxN:控制下桥臂,TIM1_CHx控制上桥臂,
    //(PWM_PERIOD - SVPWM输出)作为CCRx值
    //电流采样点在下桥臂开通的中心点(TIM1_UP_IRQn)
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;  //CNT<PULSE 的时候，为高电平，
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    //驱动芯片高电平有效
    //TIM1_CHxN:控制上桥臂,TIM1_CHx控制下桥臂,
    //(PWM_PERIOD - SVPWM输出)作为CCRx值
    //电流采样点在下桥臂开通的中心点(TIM1_UP_IRQn)
    //TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;  //CNT<PULSE 的时候，为高电平，
    //TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    //TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
    //TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    //TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Set;
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
    //Enable TIM1_BKIN interrupt,and set it to the highest priority
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_TIM9_IRQn;//TIM1_BRK_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    TIM_ClearFlag(TIM1, TIM_FLAG_Break);    //Clear the Beak interrupt flag
    TIM_ClearITPendingBit(TIM1, TIM_FLAG_Break);
    TIM_ITConfig(TIM1,TIM_IT_Break,ENABLE);
    TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Update);//used for ADC sample,but here only read adc
    //Enable the TIM1 Updata interrupt:let the Priority is the highest
    //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    //NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;//TIM1_UP_IRQn;
    //NVIC_InitStructure.NVIC_IRQChannel = TIM1_TRG_COM_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    //TIM_ITConfig(TIM1,TIM_IT_CC4,ENABLE);
    //TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE);
    //TIM_ITConfig(TIM1,TIM_IT_Trigger,ENABLE);
    //TIM_CtrlPWMOutputs(TIM1, ENABLE);   //debug output
    //TIM_Cmd(TIM1,ENABLE);  //enable the timer1
}

//***************************************************************
// Function    : InitTIM1
// Description : init timer1 produce complementary pwm
//***************************************************************
void InitTIM8(void)
{
    //GPIO assign for timer1, pulse will output from pc6
    GPIO_InitTypeDef  GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;
    //GPIO configer for timer1
    // Enable GPIO clock
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    // Connect PA6 to TIM8_BKIN
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM8);
    // Connect PA7 to TIM8_CH1N
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM8);
    // Connect PC6 to TIM8_CH1
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM8);
    // Connect PB14 to TIM8_CH2N
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_TIM8);
    // Connect PC7 to TIM8_CH2
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM8);
    // Connect PB15 to TIM8_CH3N
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_TIM8);
    // Connect PC8 to TIM8_CH3
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM8);

    // Configure TIM1 as alternate function
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;         //TIM1_BKIN
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    // Enable Timer1 clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);//  high level 20000,low level 100
    // Time base configuration
    TIM_TimeBaseStructure.TIM_Period = PWM_PERIOD;  //168000000/pwm_fcy;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;       //不分频
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //
    //TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned3;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 1;//stm32f4 manuel page 305 shows the meaning
    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
    // Channel 1, 2 and 3 Configuration in PWM mode
    //驱动芯片高电平有效
    //TIM1_CHxN:控制上桥臂,TIM1_CHx控制下桥臂,
    //(PWM_PERIOD - SVPWM输出)作为CCRx值
    //电流采样点在下桥臂开通的中心点(TIM1_UP_IRQn)
    //TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;  //CNT<PULSE 的时候，为高电平，
    //TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    //TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
    //驱动芯片低电平有效
    //TIM1_CHxN控制下桥臂,TIM1_CHx控制上桥臂,
    //(PWM_PERIOD - SVPWM输出)作为CCRx值
    //电流采样点在下桥臂开通的中心点(TIM1_UP_IRQn)
    //TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;  //CNT<PULSE 的时候，为低电平
    //TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    //TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
    //驱动芯片高电平有效
    //TIM1_CHxN:控制下桥臂,TIM1_CHx控制上桥臂,
    //(PWM_PERIOD - SVPWM输出)作为CCRx值
    //电流采样点在下桥臂开通的中心点(TIM1_UP_IRQn)
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;  //CNT<PULSE 的时候，为高电平，
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    //驱动芯片高电平有效
    //TIM1_CHxN:控制上桥臂,TIM1_CHx控制下桥臂,
    //(PWM_PERIOD - SVPWM输出)作为CCRx值
    //电流采样点在下桥臂开通的中心点(TIM1_UP_IRQn)
    //TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;  //CNT<PULSE 的时候，为高电平，
    //TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    //TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
    //TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    //TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_Pulse = PWM_PERIOD/2;  //50% PWM
    TIM_OC1Init(TIM8, &TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_Pulse = PWM_PERIOD/2;  //50% PWM
    TIM_OC2Init(TIM8, &TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_Pulse = PWM_PERIOD/2;  //50% PWM
    TIM_OC3Init(TIM8, &TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_Pulse = PWM_PERIOD/2;  //50% PWM
    TIM_OC4Init(TIM8, &TIM_OCInitStructure);

    // Automatic Output enable, Break, dead time and lock configuration
    TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable; // don't know what does this use for
    TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
    TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1;
    TIM_BDTRInitStructure.TIM_DeadTime = 126;  //750ns, 126/168=0.75us
    TIM_BDTRInitStructure.TIM_Break = TIM_Break_Enable;
    TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_Low;
    TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;
    TIM_BDTRConfig(TIM8, &TIM_BDTRInitStructure);
    //Enable TIM1_BKIN interrupt,and set it to the highest priority
    NVIC_InitStructure.NVIC_IRQChannel = TIM8_BRK_TIM12_IRQn;//TIM1_BRK_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    TIM_ClearFlag(TIM8, TIM_FLAG_Break);    //Clear the Beak interrupt flag
    TIM_ClearITPendingBit(TIM8, TIM_FLAG_Break);
    TIM_ITConfig(TIM8,TIM_IT_Break,ENABLE);
    //TIM_SelectOutputTrigger(TIM8, TIM_TRGOSource_Update);//used for ADC sample,but here only read adc
    //Enable the TIM1 Updata interrupt:let the Priority is the highest
    //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    //NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannel = TIM8_UP_TIM13_IRQn;//TIM1_UP_IRQn;
    //NVIC_InitStructure.NVIC_IRQChannel = TIM1_TRG_COM_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    //TIM_ITConfig(TIM1,TIM_IT_CC4,ENABLE);
    //TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE);
    //TIM_ITConfig(TIM1,TIM_IT_Trigger,ENABLE);
    //TIM_CtrlPWMOutputs(TIM8, ENABLE);   //debug output
    //TIM_Cmd(TIM8,ENABLE);  //enable the timer8
}


void TIM1_UP_IRQHandler(void)
{
    if(TIM_GetFlagStatus(TIM1, TIM_FLAG_Update) != RESET)
    {
        ScopeSampleDeal();
        TIM_ClearFlag(TIM1, TIM_FLAG_Update);   //Clear the Updata interrupt flag(must)
        TIM_ClearITPendingBit(TIM1, TIM_FLAG_Update);
    }
}

