/*
 * SR2D1_0V3_MCU_S1 -- Service robot 2In1 servo driver
 * (C)2016 Kinco Electric (Shenzhen) Ltd. -- All rights reserved
 */
#include "includes.h"

const UNION_APP_GPIO_INIT_TYPEDEF  AppGpioInitTab =
{
    .Split.ErrLed.Port = ERR_LED_PORT,
    .Split.ErrLed.Pin = ERR_LED_PIN,
    .Split.ErrLed.Mode = GPIO_Mode_OUT,
    .Split.ErrLed.Speed = GPIO_Speed_50MHz,
    .Split.ErrLed.Otype = GPIO_OType_PP,
    .Split.ErrLed.Pud = GPIO_PuPd_NOPULL,
    .Split.ErrLed.InitStatus = Bit_RESET,

    .Split.RunLed.Port = RUN_LED_PORT,
    .Split.RunLed.Pin = RUN_LED_PIN,
    .Split.RunLed.Mode = GPIO_Mode_OUT,
    .Split.RunLed.Speed = GPIO_Speed_50MHz,
    .Split.RunLed.Otype = GPIO_OType_PP,
    .Split.RunLed.Pud = GPIO_PuPd_NOPULL,
    .Split.RunLed.InitStatus = Bit_RESET,

    .Split.DefLed.Port = DEF_LED_PORT,
    .Split.DefLed.Pin = DEF_LED_PIN,
    .Split.DefLed.Mode = GPIO_Mode_OUT,
    .Split.DefLed.Speed = GPIO_Speed_50MHz,
    .Split.DefLed.Otype = GPIO_OType_PP,
    .Split.DefLed.Pud = GPIO_PuPd_NOPULL,
    .Split.DefLed.InitStatus = Bit_RESET,

    .Split.Encp1C.Port = ENCP1_C_PORT,
    .Split.Encp1C.Pin = ENCP1_C_PIN,
    .Split.Encp1C.Mode = GPIO_Mode_OUT,
    .Split.Encp1C.Speed = GPIO_Speed_2MHz,
    .Split.Encp1C.Otype = GPIO_OType_PP,
    .Split.Encp1C.Pud = GPIO_PuPd_NOPULL,
    .Split.Encp1C.InitStatus = Bit_SET, //power off at initialize

    .Split.Enc1Z.Port = ENC1_Z_PORT,
    .Split.Enc1Z.Pin = ENC1_Z_PIN,
    .Split.Enc1Z.Mode = GPIO_Mode_IN,
    .Split.Enc1Z.Speed = GPIO_Speed_50MHz,
    .Split.Enc1Z.Pud = GPIO_PuPd_DOWN,

    .Split.Enc1AbzErr.Port = ENC1_ABZ_ERR_PORT,
    .Split.Enc1AbzErr.Pin = ENC1_ABZ_ERR_PIN,
    .Split.Enc1AbzErr.Mode = GPIO_Mode_IN,
    .Split.Enc1AbzErr.Speed = GPIO_Speed_50MHz,
    .Split.Enc1AbzErr.Pud = GPIO_PuPd_DOWN,

    .Split.Encp2C.Port = ENCP2_C_PORT,
    .Split.Encp2C.Pin = ENCP2_C_PIN,
    .Split.Encp2C.Mode = GPIO_Mode_OUT,
    .Split.Encp2C.Speed = GPIO_Speed_50MHz,
    .Split.Encp2C.Otype = GPIO_OType_PP,
    .Split.Encp2C.Pud = GPIO_PuPd_NOPULL,
    .Split.Encp2C.InitStatus = Bit_SET, //power off at initialize

    .Split.Enc2Z.Port = ENC2_Z_PORT,
    .Split.Enc2Z.Pin = ENC2_Z_PIN,
    .Split.Enc2Z.Mode = GPIO_Mode_IN,
    .Split.Enc2Z.Speed = GPIO_Speed_50MHz,
    .Split.Enc2Z.Pud = GPIO_PuPd_DOWN,

    .Split.Enc2AbzErr.Port = ENC2_ABZ_ERR_PORT,
    .Split.Enc2AbzErr.Pin = ENC2_ABZ_ERR_PIN,
    .Split.Enc2AbzErr.Mode = GPIO_Mode_IN,
    .Split.Enc2AbzErr.Speed = GPIO_Speed_50MHz,
    .Split.Enc2AbzErr.Pud = GPIO_PuPd_DOWN,

    .Split.LvErr.Port = LV_ERR_PORT,
    .Split.LvErr.Pin = LV_ERR_PIN,
    .Split.LvErr.Mode = GPIO_Mode_IN,
    .Split.LvErr.Speed = GPIO_Speed_50MHz,
    .Split.LvErr.Pud = GPIO_PuPd_DOWN,

    .Split.BrakeC.Port = BRAKE_C_PORT,
    .Split.BrakeC.Pin = BRAKE_C_PIN,
    .Split.BrakeC.Mode = GPIO_Mode_OUT,
    .Split.BrakeC.Speed = GPIO_Speed_50MHz,
    .Split.BrakeC.Otype = GPIO_OType_PP,
    .Split.BrakeC.Pud = GPIO_PuPd_DOWN,
    .Split.BrakeC.InitStatus = Bit_RESET,

    .Split.Din1.Port = DIN1_PORT,
    .Split.Din1.Pin = DIN1_PIN,
    .Split.Din1.Mode = GPIO_Mode_IN,
    .Split.Din1.Speed = GPIO_Speed_50MHz,
    .Split.Din1.Pud = GPIO_PuPd_UP,

    .Split.Din2.Port = DIN2_PORT,
    .Split.Din2.Pin = DIN2_PIN,
    .Split.Din2.Mode = GPIO_Mode_IN,
    .Split.Din2.Speed = GPIO_Speed_50MHz,
    .Split.Din2.Pud = GPIO_PuPd_UP,

    .Split.Din3.Port = DIN3_PORT,
    .Split.Din3.Pin = DIN3_PIN,
    .Split.Din3.Mode = GPIO_Mode_IN,
    .Split.Din3.Speed = GPIO_Speed_50MHz,
    .Split.Din3.Pud = GPIO_PuPd_UP,

    .Split.Din4.Port = DIN4_PORT,
    .Split.Din4.Pin = DIN4_PIN,
    .Split.Din4.Mode = GPIO_Mode_IN,
    .Split.Din4.Speed = GPIO_Speed_50MHz,
    .Split.Din4.Pud = GPIO_PuPd_UP,

    .Split.Din5.Port = DIN5_PORT,
    .Split.Din5.Pin = DIN5_PIN,
    .Split.Din5.Mode = GPIO_Mode_IN,
    .Split.Din5.Speed = GPIO_Speed_50MHz,
    .Split.Din5.Pud = GPIO_PuPd_UP,

    .Split.Din6.Port = DIN6_PORT,
    .Split.Din6.Pin = DIN6_PIN,
    .Split.Din6.Mode = GPIO_Mode_IN,
    .Split.Din6.Speed = GPIO_Speed_50MHz,
    .Split.Din6.Pud = GPIO_PuPd_UP,

    .Split.Din7.Port = DIN7_PORT,
    .Split.Din7.Pin = DIN7_PIN,
    .Split.Din7.Mode = GPIO_Mode_IN,
    .Split.Din7.Speed = GPIO_Speed_50MHz,
    .Split.Din7.Pud = GPIO_PuPd_UP,

    .Split.Din8.Port = DIN8_PORT,
    .Split.Din8.Pin = DIN8_PIN,
    .Split.Din8.Mode = GPIO_Mode_IN,
    .Split.Din8.Speed = GPIO_Speed_50MHz,
    .Split.Din8.Pud = GPIO_PuPd_UP,

    .Split.Dout1.Port = DOUT1_PORT,
    .Split.Dout1.Pin = DOUT1_PIN,
    .Split.Dout1.Mode = GPIO_Mode_OUT,
    .Split.Dout1.Speed = GPIO_Speed_50MHz,
    .Split.Dout1.Otype = GPIO_OType_PP,
    .Split.Dout1.Pud = GPIO_PuPd_DOWN,
    .Split.Dout1.InitStatus = Bit_RESET,

    .Split.Dout2.Port = DOUT2_PORT,
    .Split.Dout2.Pin = DOUT2_PIN,
    .Split.Dout2.Mode = GPIO_Mode_OUT,
    .Split.Dout2.Speed = GPIO_Speed_50MHz,
    .Split.Dout2.Otype = GPIO_OType_PP,
    .Split.Dout2.Pud = GPIO_PuPd_DOWN,
    .Split.Dout2.InitStatus = Bit_RESET,

    .Split.Dout3.Port = DOUT3_PORT,
    .Split.Dout3.Pin = DOUT3_PIN,
    .Split.Dout3.Mode = GPIO_Mode_OUT,
    .Split.Dout3.Speed = GPIO_Speed_50MHz,
    .Split.Dout3.Otype = GPIO_OType_PP,
    .Split.Dout3.Pud = GPIO_PuPd_DOWN,
    .Split.Dout3.InitStatus = Bit_RESET,

    .Split.Dout4.Port = DOUT4_PORT,
    .Split.Dout4.Pin = DOUT4_PIN,
    .Split.Dout4.Mode = GPIO_Mode_OUT,
    .Split.Dout4.Speed = GPIO_Speed_50MHz,
    .Split.Dout4.Otype = GPIO_OType_PP,
    .Split.Dout4.Pud = GPIO_PuPd_DOWN,
    .Split.Dout4.InitStatus = Bit_RESET,

    .Split.Dout5.Port = DOUT5_PORT,
    .Split.Dout5.Pin = DOUT5_PIN,
    .Split.Dout5.Mode = GPIO_Mode_OUT,
    .Split.Dout5.Speed = GPIO_Speed_50MHz,
    .Split.Dout5.Otype = GPIO_OType_PP,
    .Split.Dout5.Pud = GPIO_PuPd_DOWN,
    .Split.Dout5.InitStatus = Bit_RESET,

    .Split.Dout6.Port = DOUT6_PORT,
    .Split.Dout6.Pin = DOUT6_PIN,
    .Split.Dout6.Mode = GPIO_Mode_OUT,
    .Split.Dout6.Speed = GPIO_Speed_50MHz,
    .Split.Dout6.Otype = GPIO_OType_PP,
    .Split.Dout6.Pud = GPIO_PuPd_DOWN,
    .Split.Dout6.InitStatus = Bit_RESET,

    .Split.Dout7.Port = DOUT7_PORT,
    .Split.Dout7.Pin = DOUT7_PIN,
    .Split.Dout7.Mode = GPIO_Mode_OUT,
    .Split.Dout7.Speed = GPIO_Speed_50MHz,
    .Split.Dout7.Otype = GPIO_OType_PP,
    .Split.Dout7.Pud = GPIO_PuPd_DOWN,
    .Split.Dout7.InitStatus = Bit_RESET,

    .Split.BellC.Port = BELL_C_PORT,
    .Split.BellC.Pin = BELL_C_PIN,
    .Split.BellC.Mode = GPIO_Mode_OUT,
    .Split.BellC.Speed = GPIO_Speed_50MHz,
    .Split.BellC.Otype = GPIO_OType_PP,
    .Split.BellC.Pud = GPIO_PuPd_DOWN,
    .Split.BellC.InitStatus = Bit_RESET,

    .Split.Rs485rde.Port = RS485_DRE_PORT,
    .Split.Rs485rde.Pin = RS485_DRE_PIN,
    .Split.Rs485rde.Mode = GPIO_Mode_OUT,
    .Split.Rs485rde.Speed = GPIO_Speed_50MHz,
    .Split.Rs485rde.Otype = GPIO_OType_PP,
    .Split.Rs485rde.Pud = GPIO_PuPd_DOWN,
    .Split.Rs485rde.InitStatus = Bit_RESET,

    .Split.SPI3_CS.Port = SPI3_CS_PORT,
    .Split.SPI3_CS.Pin = SPI3_CS_PIN,
    .Split.SPI3_CS.Mode = GPIO_Mode_OUT,
    .Split.SPI3_CS.Speed = GPIO_Speed_100MHz,
    .Split.SPI3_CS.Otype = GPIO_OType_PP,
    .Split.SPI3_CS.Pud = GPIO_PuPd_UP,
    .Split.SPI3_CS.InitStatus = Bit_SET,

    .Split.DM_SDin.Port = DM_SD_PORT,
    .Split.DM_SDin.Pin = DM_SD_PIN,
    .Split.DM_SDin.Mode = GPIO_Mode_IN,
    .Split.DM_SDin.Speed = GPIO_Speed_50MHz,
    .Split.DM_SDin.Pud = GPIO_PuPd_UP,

};

void GpioInit(void)
{
    GPIO_InitTypeDef    GPIO_InitStructure;
    INT8U   i;
    /* GPIO Periph clocks enable */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

    for(i=0; i<MAX_APP_GPIO_NUM; i++)
    {
        if((AppGpioInitTab.AppGpioPro[i].Port != (GPIO_TypeDef*)0x00000000) && (AppGpioInitTab.AppGpioPro[i].Port != (GPIO_TypeDef*)0xffffffff))
        {
            GPIO_InitStructure.GPIO_Pin = AppGpioInitTab.AppGpioPro[i].Pin;
            GPIO_InitStructure.GPIO_Mode = AppGpioInitTab.AppGpioPro[i].Mode;
            GPIO_InitStructure.GPIO_Speed = AppGpioInitTab.AppGpioPro[i].Speed;
            GPIO_InitStructure.GPIO_OType = AppGpioInitTab.AppGpioPro[i].Otype;
            GPIO_InitStructure.GPIO_PuPd = AppGpioInitTab.AppGpioPro[i].Pud;
            GPIO_Init(AppGpioInitTab.AppGpioPro[i].Port, &GPIO_InitStructure);
            if(((AppGpioInitTab.AppGpioPro[i].Mode) & GPIO_Mode_OUT) || \
               ((AppGpioInitTab.AppGpioPro[i].Mode) & GPIO_Mode_AF))
            {
                if(AppGpioInitTab.AppGpioPro[i].InitStatus == Bit_SET)
                    GPIO_SetBits(AppGpioInitTab.AppGpioPro[i].Port, AppGpioInitTab.AppGpioPro[i].Pin);
                else
                    GPIO_ResetBits(AppGpioInitTab.AppGpioPro[i].Port, AppGpioInitTab.AppGpioPro[i].Pin);
            }
        }
    }

    //extern gpio input interrupt port initialize for ENC1Z,ZNC2Z
    EXTI_InitTypeDef    EXTI_InitStructure;
    NVIC_InitTypeDef    NVIC_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);    //enable the syscfg clock
    if((DX[0].MotorPar.FB_Type) & ENC_ABZ_CHECK_CODE)
    {
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource0); //for enc1z
    /* Configure EXTI Line0 */
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;    //rising edge
    EXTI_ClearITPendingBit(EXTI_Line0);
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* Enable and set EXTI Line0 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    }
    if((DX[0].MotorPar.FB_Type) & ENC_ABZ_CHECK_CODE)
    {
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource1);     //for enc2z
    /* Configure EXTI Line1 */
    EXTI_InitStructure.EXTI_Line = EXTI_Line1;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;    //rising edge
    EXTI_ClearITPendingBit(EXTI_Line1);
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* Enable and set EXTI Line1 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    }
}

void InitLogicVoltErrInterrupt(void)
{
    EXTI_InitTypeDef    EXTI_InitStructure;
    NVIC_InitTypeDef    NVIC_InitStructure;
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource2);     //for LV_ERR
    /* Configure EXTI Line2 */
    EXTI_InitStructure.EXTI_Line = EXTI_Line2;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;    //rising edge
    EXTI_ClearITPendingBit(EXTI_Line2);
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* Enable and set EXTI Line2 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

}

//Driver 1 Encoder index signal interrupt
void EXTI0_IRQHandler(void)
{
    INT16U  temp16u;

    if(EXTI_GetITStatus(EXTI_Line0) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line0);
        if(_GET_ENC1_Z_STATUS == SET)            //if GPIOB pin4 interrupt
        {
            if(DX[D1].Fbk.FirstZ == 0) //first capture the encoder Z signal
            {
                temp16u = TIM2->CNT;
                //Fbk1.FirstZ = 1;
                DX[D1].Fbk.PosOnFirstZ = ((DX[D1].Fbk.PosAbs)&0xffff0000)|((INT32S)temp16u);
                //debug
                if(DX[D1].MotorPar.ExcitationMode != 0)
                    DX[D1].Fbk.EncOffset = DX[D1].Fbk.PosOnFirstZ;
                DX[D1].Fbk.FirstZ = 1;
            }
        }
    }
}

//Driver 1 Encoder index signal interrupt
void EXTI1_IRQHandler(void)
{
    INT16U  temp16u;

    if(EXTI_GetITStatus(EXTI_Line1) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line1);
        if(_GET_ENC2_Z_STATUS == SET)            //if GPIOB pin4 interrupt
        {
            if(DX[D2].Fbk.FirstZ == 0) //first capture the encoder Z signal
            {
                temp16u = TIM4->CNT;
                //Fbk1.FirstZ = 1;
                DX[D2].Fbk.PosOnFirstZ = ((DX[D2].Fbk.PosAbs)&0xffff0000)|((INT32S)temp16u);
                //debug
                if(DX[D2].MotorPar.ExcitationMode != 0)
                    DX[D2].Fbk.EncOffset = DX[D2].Fbk.PosOnFirstZ;
                DX[D2].Fbk.FirstZ = 1;
            }
        }
    }
}

//logic voltage signal interrupt
void EXTI2_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line2) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line2);
        //save the platform A/B position information
        sEE_WriteBuffer(&DXC.PlatformInfo.Pos.AllPulse.All8s[0], 160, 4);
        while((*sEEDataWritePointer) != 0); //这里要插入超时处理
    }
}
