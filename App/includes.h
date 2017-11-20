/*
 * SR2D1_0V3_MCU_S1 -- Service robot 2In1 servo driver
 * (C)2016 Kinco Electric (Shenzhen) Ltd. -- All rights reserved
 */
#ifndef  INCLUDES_PRESENT
#define  INCLUDES_PRESENT
#ifndef NULL
#define NULL    ((void *)0)
#endif
#ifndef FALSE
#define FALSE   (0)
#endif
#ifndef TRUE
#define TRUE    (1)
#endif
/*
*********************************************************************************************************
*                                         STANDARD LIBRARIES
*********************************************************************************************************
*/
#include  <stdarg.h>
#include  <stdio.h>
#include  <stdlib.h>
#include  <math.h>
/*
*********************************************************************************************************
*                                                 ST
*********************************************************************************************************
*/
#include  "stm32f4xx.h"
/*
*********************************************************************************************************
*                                              DATA TYPES
*                                         (Compiler Specific)
*********************************************************************************************************
*/
typedef unsigned char       BOOLEAN;
typedef unsigned char       INT8U;                    /* Unsigned  8 bit quantity           */
typedef signed   char       INT8S;                    /* Signed    8 bit quantity           */
typedef unsigned short      INT16U;                   /* Unsigned 16 bit quantity           */
typedef signed   short      INT16S;                   /* Signed   16 bit quantity           */
typedef unsigned int        INT32U;                   /* Unsigned 32 bit quantity           */
typedef signed   int        INT32S;                   /* Signed   32 bit quantity           */
typedef unsigned long long  INT64U;                   /* Unsigned 64 bit quantity           */
typedef signed   long long  INT64S;                   /* Signed   64 bit quantity           */
typedef float               FP32;                     /* Single precision floating point    */
typedef double              FP64;                     /* Double precision floating point    */
/*
*********************************************************************************************************
*                                              LIBRARIES
*********************************************************************************************************
*/
/*
*********************************************************************************************************
*                                              APP / BSP
*********************************************************************************************************
*/
/*
*********************************************************************************************************
*                                                 OS
*********************************************************************************************************
*/
/*
*********************************************************************************************************
*                                            App
*********************************************************************************************************
*/
#include "queue.h"
#include "od_lib.h"
#include "ctrl_base.h"
#include "ctrl_loop.h"
#include "scope.h"
#include "uart.h"
#include "can.h"
#include "gpio.h"
#include "time.h"
#include "adc.h"
#include "i2c_ee.h"
#include "spi.h"
#include "crc6.h"
#include "od_func.h"
#include "tick_time.h"
/*
*********************************************************************************************************
*                                            INCLUDES END
*********************************************************************************************************
*/
#endif
