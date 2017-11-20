#ifndef _OLED_H
#define _OLED_H

#include "stm32f4xx.h"
#include "gpio.h"
//////////////////////////////////////////////////////////////////////////////////
//TXW不会写，百度无欺。	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途	   
//                                                                   修改人BY TXW								  
//////////////////////////////////////////////////////////////////////////////////

#define LCD_SCL_L   _OLED_D0_RESET  //  D0
#define LCD_SCL_H   _OLED_D0_SET  //K60的管脚配置  D0
#define LCD_SDA_L   _OLED_D1_RESET   //D1
#define LCD_SDA_H	_OLED_D1_SET   //D1
#define LCD_RST_L   _OLED_RST_RESET   
#define LCD_RST_H   _OLED_RST_SET  
#define LCD_DC_L    _OLED_DS_RESET 
#define LCD_DC_H    _OLED_DS_SET


#define   Absolute(n) ( (n)>0?(n):(-(n)) )                            //求绝对值 
#define byte unsigned char  //自己加的
#define word unsigned int   //自己加的

 void LCD_Init(void);
 void LCD_CLS(void);
 void LCD_P6x8Str(byte x,byte y,byte ch[]);
 void LCD_Print(byte x, byte y, byte ch[]);
 void LCD_PutPixel(byte x,byte y);
 void LCD_Rectangle(byte x1,byte y1,byte x2,byte y2,byte gif);
 void LCD_Fill(byte dat);
 void Dis_Num(byte y, byte x, int num,byte N);
 void Dis_Num2(byte y, byte x, float nummm,byte N);

#endif



