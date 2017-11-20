/*
 * SR2D1_0V3_MCU_S1 -- Service robot 2In1 servo driver
 * (C)2016 Kinco Electric (Shenzhen) Ltd. -- All rights reserved
 */
#ifndef _GPIO_H
#define _GPIO_H

#define MAX_APP_GPIO_NUM    79

#define ERR_LED_PORT    GPIOC
#define ERR_LED_PIN     GPIO_Pin_9      //对应原理图中的MCU1_LEDB(LED3)
#define RUN_LED_PORT    GPIOB
#define RUN_LED_PIN     GPIO_Pin_8      //对应原理图中的MCU1_LEDG(LED2)
#define DEF_LED_PORT    GPIOE
#define DEF_LED_PIN     GPIO_Pin_14     //对应原理图中的MCU1_LEDR(LED1)

#define GPIO_ToggleBits(PORT,PIN)   GPIO_WriteBit(PORT, PIN, !GPIO_ReadOutputDataBit(PORT, PIN))

#define _RUN_LED_ON     GPIO_SetBits(RUN_LED_PORT, RUN_LED_PIN)     //RUN灯亮
#define _RUN_LED_OFF    GPIO_ResetBits(RUN_LED_PORT, RUN_LED_PIN)   //RUN灯灭
#define _ERR_LED_ON     GPIO_SetBits(ERR_LED_PORT, ERR_LED_PIN)     //ERR灯亮
#define _ERR_LED_OFF    GPIO_ResetBits(ERR_LED_PORT, ERR_LED_PIN)   //ERR灯灭
#define _DEF_LED_ON     GPIO_SetBits(DEF_LED_PORT, DEF_LED_PIN)
#define _DEF_LED_OFF    GPIO_ResetBits(DEF_LED_PORT, DEF_LED_PIN)
#define _DR1_ERR_LED_ON     GPIO_SetBits(ERR_LED_PORT, ERR_LED_PIN)     //ERR灯亮
#define _DR1_ERR_LED_OFF    GPIO_ResetBits(ERR_LED_PORT, ERR_LED_PIN)   //ERR灯灭
#define _DR2_ERR_LED_ON     GPIO_SetBits(DEF_LED_PORT, DEF_LED_PIN)
#define _DR2_ERR_LED_OFF    GPIO_ResetBits(DEF_LED_PORT, DEF_LED_PIN)

//encoder 1 port define
#define ENCP1_C_PORT     GPIOC                                   //encoder power control port
#define ENCP1_C_PIN      GPIO_Pin_15                              //encoder power control pin
#define _ENC1_POWER_OFF  GPIO_SetBits(ENCP1_C_PORT, ENCP1_C_PIN)   //encoder power off
#define _ENC1_POWER_ON   GPIO_ResetBits(ENCP1_C_PORT, ENCP1_C_PIN) //encoder power on
//#define _ENC1_POWER_OFF  ENCP1_C_PORT->BSRRH = ENCP1_C_PIN // GPIO_SetBits(ENCP1_C_PORT, ENCP1_C_PIN)   //encoder power off
//#define _ENC1_POWER_ON   ENCP1_C_PORT->BSRRL = ENCP1_C_PIN //GPIO_ResetBits(ENCP1_C_PORT, ENCP1_C_PIN) //encoder power on

//#define ENC_SEL_C_PORT  GPIOC
//#define ENC_SEL_C_PIN   GPIO_Pin_9
//#define _ENC_SEL_DIF    GPIO_SetBits(ENC_SEL_C_PORT,ENC_SEL_C_PIN)      //select the difference tpye encoder
//#define _ENC_SEL_TTL    GPIO_ResetBits(ENC_SEL_C_PORT,ENC_SEL_C_PIN)    //select the TTL type encoder

//#define ENC_TTL_DIR_PORT    GPIOC
//#define ENC_TTL_DIR_PIN     GPIO_Pin_8
//#define _ENC_TTL_DIR_OUT    GPIO_SetBits(ENC_TTL_DIR_PORT,ENC_TTL_DIR_PIN)
//#define _ENC_TTL_DIR_IN     GPIO_ResetBits(ENC_TTL_DIR_PORT,ENC_TTL_DIR_PIN)

#define ENC1_Z_PORT          GPIOE
#define ENC1_Z_PIN           GPIO_Pin_0
#define _GET_ENC1_Z_STATUS   GPIO_ReadInputDataBit(ENC1_Z_PORT, ENC1_Z_PIN)

#define ENC1_ABZ_ERR_PORT    GPIOE
#define ENC1_ABZ_ERR_PIN     GPIO_Pin_4
#define _GET_ENC1_ABZ_ERR    GPIO_ReadInputDataBit(ENC1_ABZ_ERR_PORT, ENC1_ABZ_ERR_PIN)

//#define ENC1_UVW_ERR_PORT    GPIOE
//#define ENC1_UVW_ERR_PIN     GPIO_Pin_3
//#define _GET_ENC1_UVW_ERR    GPIO_ReadInputDataBit(ENC1_UVW_ERR_PORT, ENC1_UVW_ERR_PIN)

//#define ENC1_U_PORT          GPIOE
//#define ENC1_U_PIN           GPIO_Pin_1
//#define _GET_ENC1_U_STATUS   GPIO_ReadInputDataBit(ENC1_U_PORT, ENC1_U_PIN)

//#define ENC1_V_PORT          GPIOE
//#define ENC1_V_PIN           GPIO_Pin_2
//#define _GET_ENC1_V_STATUS   GPIO_ReadInputDataBit(ENC1_V_PORT, ENC1_V_PIN)

//#define ENC1_W_PORT          GPIOE
//#define ENC1_W_PIN           GPIO_Pin_3
//#define _GET_ENC1_W_STATUS   GPIO_ReadInputDataBit(ENC1_W_PORT, ENC1_W_PIN)

//encoder 2 port define
#define ENCP2_C_PORT     GPIOD                                   //encoder power control port
#define ENCP2_C_PIN      GPIO_Pin_0                              //encoder power control pin
#define _ENC2_POWER_OFF  GPIO_SetBits(ENCP2_C_PORT, ENCP2_C_PIN)   //encoder power off
#define _ENC2_POWER_ON   GPIO_ResetBits(ENCP2_C_PORT, ENCP2_C_PIN) //encoder power on

//#define ENC_SEL_C_PORT  GPIOC
//#define ENC_SEL_C_PIN   GPIO_Pin_9
//#define _ENC_SEL_DIF    GPIO_SetBits(ENC_SEL_C_PORT,ENC_SEL_C_PIN)      //select the difference tpye encoder
//#define _ENC_SEL_TTL    GPIO_ResetBits(ENC_SEL_C_PORT,ENC_SEL_C_PIN)    //select the TTL type encoder

//#define ENC_TTL_DIR_PORT    GPIOC
//#define ENC_TTL_DIR_PIN     GPIO_Pin_8
//#define _ENC_TTL_DIR_OUT    GPIO_SetBits(ENC_TTL_DIR_PORT,ENC_TTL_DIR_PIN)
//#define _ENC_TTL_DIR_IN     GPIO_ResetBits(ENC_TTL_DIR_PORT,ENC_TTL_DIR_PIN)

#define ENC2_Z_PORT          GPIOD
#define ENC2_Z_PIN           GPIO_Pin_1
#define _GET_ENC2_Z_STATUS   GPIO_ReadInputDataBit(ENC2_Z_PORT, ENC2_Z_PIN)

#define ENC2_ABZ_ERR_PORT    GPIOC
#define ENC2_ABZ_ERR_PIN     GPIO_Pin_13
#define _GET_ENC2_ABZ_ERR    GPIO_ReadInputDataBit(ENC2_ABZ_ERR_PORT, ENC2_ABZ_ERR_PIN)

//#define ENC2_UVW_ERR_PORT    GPIOA
//#define ENC2_UVW_ERR_PIN     GPIO_Pin_15
//#define _GET_ENC2_UVW_ERR    GPIO_ReadInputDataBit(ENC2_UVW_ERR_PORT, ENC2_UVW_ERR_PIN)

//#define ENC2_U_PORT          GPIOC
//#define ENC2_U_PIN           GPIO_Pin_13
//#define _GET_ENC2_U_STATUS   GPIO_ReadInputDataBit(ENC2_U_PORT, ENC2_U_PIN)

//#define ENC2_V_PORT          GPIOC
//#define ENC2_V_PIN           GPIO_Pin_14
//#define _GET_ENC2_V_STATUS   GPIO_ReadInputDataBit(ENC2_V_PORT, ENC2_V_PIN)

//#define ENC2_W_PORT          GPIOC
//#define ENC2_W_PIN           GPIO_Pin_15
//#define _GET_ENC2_W_STATUS   GPIO_ReadInputDataBit(ENC2_W_PORT, ENC2_W_PIN)


#define LV_ERR_PORT         GPIOB
#define LV_ERR_PIN          GPIO_Pin_2
#define _GET_LV_ERR         GPIO_ReadInputDataBit(LV_ERR_PORT, LV_ERR_PIN)

//#define DRIVE1_C_PORT        GPIOA
//#define DRIVE1_C_PIN         GPIO_Pin_8
#define _DRIVE1_ON          TIM_CtrlPWMOutputs(TIM1, ENABLE)   //enable time8 output
#define _DRIVE1_OFF         TIM_CtrlPWMOutputs(TIM1, DISABLE)   //enable time8 output

//#define DRIVE2_C_PORT        GPIOE
//#define DRIVE2_C_PIN         GPIO_Pin_7
#define _DRIVE2_ON          TIM_CtrlPWMOutputs(TIM8, ENABLE)   //enable time8 output
#define _DRIVE2_OFF         TIM_CtrlPWMOutputs(TIM8, DISABLE)   //disable time8 output

#define BRAKE_C_PORT        GPIOD
#define BRAKE_C_PIN         GPIO_Pin_2
#define _BRAKE_OFF          GPIO_SetBits(BRAKE_C_PORT,BRAKE_C_PIN)
#define _BRAKE_ON           GPIO_ResetBits(BRAKE_C_PORT,BRAKE_C_PIN)

#define DIN1_PORT           GPIOD
#define DIN1_PIN            GPIO_Pin_4
#define _GET_DIN1_STATUS    GPIO_ReadInputDataBit(DIN1_PORT, DIN1_PIN)

#define DIN2_PORT           GPIOC
#define DIN2_PIN            GPIO_Pin_7
#define _GET_DIN2_STATUS    GPIO_ReadInputDataBit(DIN2_PORT, DIN2_PIN)

#define DIN3_PORT           GPIOD
#define DIN3_PIN            GPIO_Pin_10
#define _GET_DIN3_STATUS    GPIO_ReadInputDataBit(DIN3_PORT, DIN3_PIN)

#define DIN4_PORT           GPIOC
#define DIN4_PIN            GPIO_Pin_11
#define _GET_DIN4_STATUS    GPIO_ReadInputDataBit(DIN4_PORT, DIN4_PIN)

#define DIN5_PORT           GPIOD
#define DIN5_PIN            GPIO_Pin_12
#define _GET_DIN5_STATUS    GPIO_ReadInputDataBit(DIN5_PORT, DIN5_PIN)

#define DIN6_PORT           GPIOC
#define DIN6_PIN            GPIO_Pin_13
#define _GET_DIN6_STATUS    GPIO_ReadInputDataBit(DIN6_PORT, DIN6_PIN)

#define DIN7_PORT           GPIOD
#define DIN7_PIN            GPIO_Pin_14
#define _GET_DIN7_STATUS    GPIO_ReadInputDataBit(DIN7_PORT, DIN7_PIN)

#define DIN8_PORT           GPIOD
#define DIN8_PIN            GPIO_Pin_15
#define _GET_DIN8_STATUS    GPIO_ReadInputDataBit(DIN8_PORT, DIN8_PIN)

//Dout1~7
#define DOUT1_PORT          GPIOB
#define DOUT1_PIN           GPIO_Pin_4
#define _DOUT1_ON           GPIO_SetBits(DOUT1_PORT,DOUT1_PIN)
#define _DOUT1_OFF          GPIO_ResetBits(DOUT1_PORT,DOUT1_PIN)

#define DOUT2_PORT          GPIOB
#define DOUT2_PIN           GPIO_Pin_5
#define _DOUT2_ON           GPIO_SetBits(DOUT2_PORT,DOUT2_PIN)
#define _DOUT2_OFF          GPIO_ResetBits(DOUT2_PORT,DOUT2_PIN)

#define DOUT3_PORT          GPIOE
#define DOUT3_PIN           GPIO_Pin_3
#define _DOUT3_ON           GPIO_SetBits(DOUT3_PORT,DOUT3_PIN)
#define _DOUT3_OFF          GPIO_ResetBits(DOUT3_PORT,DOUT3_PIN)

#define DOUT4_PORT          GPIOD
#define DOUT4_PIN           GPIO_Pin_3
#define _DOUT4_ON           GPIO_SetBits(DOUT4_PORT,DOUT4_PIN)
#define _DOUT4_OFF          GPIO_ResetBits(DOUT4_PORT,DOUT4_PIN)

#define DOUT5_PORT          GPIOE
#define DOUT5_PIN           GPIO_Pin_1
#define _DOUT5_ON           GPIO_SetBits(DOUT5_PORT,DOUT5_PIN)
#define _DOUT5_OFF          GPIO_ResetBits(DOUT5_PORT,DOUT5_PIN)

#define DOUT6_PORT          GPIOB
#define DOUT6_PIN           GPIO_Pin_9
#define _DOUT6_ON           GPIO_SetBits(DOUT6_PORT,DOUT6_PIN)
#define _DOUT6_OFF          GPIO_ResetBits(DOUT6_PORT,DOUT6_PIN)

#define DOUT7_PORT          GPIOA
#define DOUT7_PIN           GPIO_Pin_15
#define _DOUT7_ON           GPIO_SetBits(DOUT7_PORT,DOUT7_PIN)
#define _DOUT7_OFF          GPIO_ResetBits(DOUT7_PORT,DOUT7_PIN)

#define BELL_C_PORT         GPIOE
#define BELL_C_PIN          GPIO_Pin_2
#define _BELL_C_ON          GPIO_SetBits(BELL_C_PORT,BELL_C_PIN)
#define _BELL_C_OFF         GPIO_ResetBits(BELL_C_PORT,BELL_C_PIN)


#define RS485_DRE_PORT      GPIOA
#define RS485_DRE_PIN       GPIO_Pin_8
#define _RS485_DE           GPIO_SetBits(RS485_DRE_PORT,RS485_DRE_PIN)
#define _RS485_RE           GPIO_ResetBits(RS485_DRE_PORT,RS485_DRE_PIN)

#define SPI3_CS_PORT        GPIOC
#define SPI3_CS_PIN         GPIO_Pin_14
#define _SPI3_CS_SET        GPIO_SetBits(SPI3_CS_PORT,SPI3_CS_PIN)
#define _SPI3_CS_RESET      GPIO_ResetBits(SPI3_CS_PORT,SPI3_CS_PIN)

#define DM_SD_PORT          GPIOE
#define DM_SD_PIN           GPIO_Pin_7
#define _GET_DM_SD_STATUS   GPIO_ReadInputDataBit(DM_SD_PORT, DM_SD_PIN)

//#define DM_PWM1_PORT        GPIOC
//#define DM_PWM1_PIN         GPIO_Pin_9
//#define _DM_PWM1_OUT_H      GPIO_SetBits(DM_PWM1_PORT, DM_PWM1_PIN)     //DM PWM1 output high
//#define _DM_PWM1_OUT_L      GPIO_ResetBits(DM_PWM1_PORT, DM_PWM1_PIN)     //DM PWM1 output low

//#define DM_PWM2_PORT        GPIOE
//#define DM_PWM2_PIN         GPIO_Pin_14
//#define _DM_PWM2_OUT_H      GPIO_SetBits(DM_PWM2_PORT, DM_PWM2_PIN)     //DM PWM1 output high
//#define _DM_PWM2_OUT_L      GPIO_ResetBits(DM_PWM2_PORT, DM_PWM2_PIN)     //DM PWM1 output low

//#define SSTB_PORT           GPIOA
//#define SSTB_PIN            GPIO_Pin_15
//#define _SSTB_SET           GPIO_SetBits(SSTB_PORT,SSTB_PIN)
//#define _SSTB_RESET         GPIO_ResetBits(SSTB_PORT,SSTB_PIN)

//#define SCLK_PORT           GPIOC
//#define SCLK_PIN            GPIO_Pin_12
//#define _SCLK_SET           GPIO_SetBits(SCLK_PORT,SCLK_PIN)
//#define _SCLK_RESET         GPIO_ResetBits(SCLK_PORT,SCLK_PIN)

//#define SDATA_PORT          GPIOD
//#define SDATA_PIN           GPIO_Pin_2
//#define _SDATA_SET          GPIO_SetBits(SDATA_PORT,SDATA_PIN)
//#define _SDATA_RESET        GPIO_ResetBits(SDATA_PORT,SDATA_PIN)

//#define DOUT1_C_PORT        GPIOC
//#define DOUT1_C_PIN         GPIO_Pin_14
//#define _DOUT1_ON           GPIO_SetBits(OUT1_C_PORT,OUT1_C_PIN)
//#define _DOUT1_OFF          GPIO_ResetBits(OUT1_C_PORT,OUT1_C_PIN)

//#define DOUT2_C_PORT        GPIOC
//#define DOUT2_C_PIN         GPIO_Pin_15
//#define _DOUT2_ON           GPIO_SetBits(OUT2_C_PORT,OUT2_C_PIN)
//#define _DOUT2_OFF          GPIO_ResetBits(OUT2_C_PORT,OUT2_C_PIN)

#define ENC_ABZ_BIT_ERR     1
#define ENC_UVW_BIT_ERR     1

typedef struct
{
    GPIO_TypeDef*       Port;
    INT16U              Pin;
    GPIOMode_TypeDef    Mode;
    GPIOSpeed_TypeDef   Speed;
    GPIOOType_TypeDef   Otype;
    GPIOPuPd_TypeDef    Pud;
    BitAction           InitStatus;
}AppGpioPropertyTypedef;

typedef union
{
    AppGpioPropertyTypedef  AppGpioPro[MAX_APP_GPIO_NUM];
    struct
    {
        AppGpioPropertyTypedef  ErrLed;
        AppGpioPropertyTypedef  RunLed;
        AppGpioPropertyTypedef  DefLed;

        AppGpioPropertyTypedef  Encp1C;
        AppGpioPropertyTypedef  Enc1Z;
        AppGpioPropertyTypedef  Enc1AbzErr;
        //AppGpioPropertyTypedef  Enc1UvwErr;
        //AppGpioPropertyTypedef  Enc1U;
        //AppGpioPropertyTypedef  Enc1V;
        //AppGpioPropertyTypedef  Enc1W;

        AppGpioPropertyTypedef  Encp2C;
        AppGpioPropertyTypedef  Enc2Z;
        AppGpioPropertyTypedef  Enc2AbzErr;
        //AppGpioPropertyTypedef  Enc2UvwErr;
        //AppGpioPropertyTypedef  Enc2U;
        //AppGpioPropertyTypedef  Enc2V;
        //AppGpioPropertyTypedef  Enc2W;

        AppGpioPropertyTypedef  LvErr;
        AppGpioPropertyTypedef  Drive1C;
        AppGpioPropertyTypedef  Drive2C;
        AppGpioPropertyTypedef  BrakeC;
        AppGpioPropertyTypedef  Din1;
        AppGpioPropertyTypedef  Din2;
        AppGpioPropertyTypedef  Din3;
        AppGpioPropertyTypedef  Din4;
        AppGpioPropertyTypedef  Din5;
        AppGpioPropertyTypedef  Din6;
        AppGpioPropertyTypedef  Din7;
        AppGpioPropertyTypedef  Din8;

        AppGpioPropertyTypedef  Dout1;
        AppGpioPropertyTypedef  Dout2;
        AppGpioPropertyTypedef  Dout3;
        AppGpioPropertyTypedef  Dout4;
        AppGpioPropertyTypedef  Dout5;
        AppGpioPropertyTypedef  Dout6;
        AppGpioPropertyTypedef  Dout7;

        AppGpioPropertyTypedef  BellC;

        AppGpioPropertyTypedef  Rs485rde;
        AppGpioPropertyTypedef  SPI3_CS;
        AppGpioPropertyTypedef  DM_SDin;
        //AppGpioPropertyTypedef  DM_PWM1;
        //AppGpioPropertyTypedef  DM_PWM2;
        //AppGpioPropertyTypedef  SSTB;
        //AppGpioPropertyTypedef  SCLK;
        //AppGpioPropertyTypedef  SDATA;

    }Split;
}UNION_APP_GPIO_INIT_TYPEDEF;

extern void GpioInit(void);
extern void InitLogicVoltErrInterrupt(void);
//serial data transfer
extern void SDataTransfer(INT32U Data, INT8U Length);
#endif
