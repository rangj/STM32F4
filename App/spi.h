/*
 * SR2D1_0V3_MCU_S1 -- Service robot 2In1 servo driver
 * (C)2016 Kinco Electric (Shenzhen) Ltd. -- All rights reserved
 */
#ifndef _SPI_H
#define _SPI_H
#define SPI2_PORT_RCC           RCC_AHB1Periph_GPIOB
#define SPI2_PORT               GPIOB
#define SPI2_NSS_PIN            GPIO_Pin_12
#define SPI2_SCK_PIN            GPIO_Pin_13
#define SPI2_MISO_PIN           GPIO_Pin_14
#define SPI2_MOSI_PIN           GPIO_Pin_15
#define SPI2_NSS_PIN_SOURCE     GPIO_PinSource12
#define SPI2_SCK_PIN_SOURCE     GPIO_PinSource13
#define SPI2_MISO_PIN_SOURCE    GPIO_PinSource14
#define SPI2_MOSI_PIN_SOURCE    GPIO_PinSource15
//#define _SPI2_DE_LOW          SPI_NSSInternalSoftwareConfig(SPI2, SPI_NSSInternalSoft_Reset);
//#define _SPI2_DE_HIGH         SPI_NSSInternalSoftwareConfig(SPI2, SPI_NSSInternalSoft_Set);
#define _SPI2_DE_LOW    GPIO_ResetBits(SPI2_PORT, SPI2_NSS_PIN)     //
#define _SPI2_DE_HIGH   GPIO_SetBits(SPI2_PORT, SPI2_NSS_PIN)       //

//SPI3
#define SPI3_PORT_RCC           RCC_AHB1Periph_GPIOC
#define SPI3_PORT               GPIOC
//#define SPI3_NSS_PIN            GPIO_Pin_12
#define SPI3_SCK_PIN            GPIO_Pin_10
#define SPI3_MISO_PIN           GPIO_Pin_11
#define SPI3_MOSI_PIN           GPIO_Pin_12
//#define SPI3_NSS_PIN_SOURCE     GPIO_PinSource12
#define SPI3_SCK_PIN_SOURCE     GPIO_PinSource10
#define SPI3_MISO_PIN_SOURCE    GPIO_PinSource11
#define SPI3_MOSI_PIN_SOURCE    GPIO_PinSource12

extern INT16U  SPI3_DMA_RD_Buff[16];
extern INT16U  SPI3_DMA_TD_Buff[16];
extern INT16U  SPI3_DMA_RD_Buff2[16];
extern INT16U  SPI3_DMA_TD_Buff2[16];
extern void SPI2_Init(void);
extern void SPI3_Init(void);
#endif
