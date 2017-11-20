#ifndef _OLED_H
#define _OLED_H

#include "stm32f4xx.h"
#include "gpio.h"
//////////////////////////////////////////////////////////////////////////////////
//TXW����д���ٶ����ۡ�	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;	   
//                                                                   �޸���BY TXW								  
//////////////////////////////////////////////////////////////////////////////////

#define LCD_SCL_L   _OLED_D0_RESET  //  D0
#define LCD_SCL_H   _OLED_D0_SET  //K60�Ĺܽ�����  D0
#define LCD_SDA_L   _OLED_D1_RESET   //D1
#define LCD_SDA_H	_OLED_D1_SET   //D1
#define LCD_RST_L   _OLED_RST_RESET   
#define LCD_RST_H   _OLED_RST_SET  
#define LCD_DC_L    _OLED_DS_RESET 
#define LCD_DC_H    _OLED_DS_SET


#define   Absolute(n) ( (n)>0?(n):(-(n)) )                            //�����ֵ 
#define byte unsigned char  //�Լ��ӵ�
#define word unsigned int   //�Լ��ӵ�

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



