#include "OLED.h"
  
//////////////////////////////////////////////////////////////////////////////////
//TXW不会写，百度无欺。	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途	   
//                                                                   修改人BY TXW								  
//////////////////////////////////////////////////////////////////////////////////
/*
4线SPI使用说明：
VBT 供内部DC-DC电压，3.3~4.3V，如果使用5V电压，为保险起见串一个100~500欧的电阻
*/

#define XLevelL		0x00
#define XLevelH		0x10
#define XLevel		((XLevelH&0x0F)*16+XLevelL)
#define Max_Column	128
#define Max_Row		  64
#define	Brightness	0xCF 

#define X_WIDTH 128
#define Y_WIDTH 64
const byte F6x8[][6] =
{
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },   // sp
    { 0x00, 0x00, 0x00, 0x2f, 0x00, 0x00 },   // !
    { 0x00, 0x00, 0x07, 0x00, 0x07, 0x00 },   // "
    { 0x00, 0x14, 0x7f, 0x14, 0x7f, 0x14 },   // #
    { 0x00, 0x24, 0x2a, 0x7f, 0x2a, 0x12 },   // $
    { 0x00, 0x62, 0x64, 0x08, 0x13, 0x23 },   // %
    { 0x00, 0x36, 0x49, 0x55, 0x22, 0x50 },   // &
    { 0x00, 0x00, 0x05, 0x03, 0x00, 0x00 },   // '
    { 0x00, 0x00, 0x1c, 0x22, 0x41, 0x00 },   // (
    { 0x00, 0x00, 0x41, 0x22, 0x1c, 0x00 },   // )
    { 0x00, 0x14, 0x08, 0x3E, 0x08, 0x14 },   // *
    { 0x00, 0x08, 0x08, 0x3E, 0x08, 0x08 },   // +
    { 0x00, 0x00, 0x00, 0xA0, 0x60, 0x00 },   // ,
    { 0x00, 0x08, 0x08, 0x08, 0x08, 0x08 },   // -
    { 0x00, 0x00, 0x60, 0x60, 0x00, 0x00 },   // .
    { 0x00, 0x20, 0x10, 0x08, 0x04, 0x02 },   // /
    { 0x00, 0x3E, 0x51, 0x49, 0x45, 0x3E },   // 0
    { 0x00, 0x00, 0x42, 0x7F, 0x40, 0x00 },   // 1
    { 0x00, 0x42, 0x61, 0x51, 0x49, 0x46 },   // 2
    { 0x00, 0x21, 0x41, 0x45, 0x4B, 0x31 },   // 3
    { 0x00, 0x18, 0x14, 0x12, 0x7F, 0x10 },   // 4
    { 0x00, 0x27, 0x45, 0x45, 0x45, 0x39 },   // 5
    { 0x00, 0x3C, 0x4A, 0x49, 0x49, 0x30 },   // 6
    { 0x00, 0x01, 0x71, 0x09, 0x05, 0x03 },   // 7
    { 0x00, 0x36, 0x49, 0x49, 0x49, 0x36 },   // 8
    { 0x00, 0x06, 0x49, 0x49, 0x29, 0x1E },   // 9
    { 0x00, 0x00, 0x36, 0x36, 0x00, 0x00 },   // :
    { 0x00, 0x00, 0x56, 0x36, 0x00, 0x00 },   // ;
    { 0x00, 0x08, 0x14, 0x22, 0x41, 0x00 },   // <
    { 0x00, 0x14, 0x14, 0x14, 0x14, 0x14 },   // =
    { 0x00, 0x00, 0x41, 0x22, 0x14, 0x08 },   // >
    { 0x00, 0x02, 0x01, 0x51, 0x09, 0x06 },   // ?
    { 0x00, 0x32, 0x49, 0x59, 0x51, 0x3E },   // @
    { 0x00, 0x7C, 0x12, 0x11, 0x12, 0x7C },   // A
    { 0x00, 0x7F, 0x49, 0x49, 0x49, 0x36 },   // B
    { 0x00, 0x3E, 0x41, 0x41, 0x41, 0x22 },   // C
    { 0x00, 0x7F, 0x41, 0x41, 0x22, 0x1C },   // D
    { 0x00, 0x7F, 0x49, 0x49, 0x49, 0x41 },   // E
    { 0x00, 0x7F, 0x09, 0x09, 0x09, 0x01 },   // F
    { 0x00, 0x3E, 0x41, 0x49, 0x49, 0x7A },   // G
    { 0x00, 0x7F, 0x08, 0x08, 0x08, 0x7F },   // H
    { 0x00, 0x00, 0x41, 0x7F, 0x41, 0x00 },   // I
    { 0x00, 0x20, 0x40, 0x41, 0x3F, 0x01 },   // J
    { 0x00, 0x7F, 0x08, 0x14, 0x22, 0x41 },   // K
    { 0x00, 0x7F, 0x40, 0x40, 0x40, 0x40 },   // L
    { 0x00, 0x7F, 0x02, 0x0C, 0x02, 0x7F },   // M
    { 0x00, 0x7F, 0x04, 0x08, 0x10, 0x7F },   // N
    { 0x00, 0x3E, 0x41, 0x41, 0x41, 0x3E },   // O
    { 0x00, 0x7F, 0x09, 0x09, 0x09, 0x06 },   // P
    { 0x00, 0x3E, 0x41, 0x51, 0x21, 0x5E },   // Q
    { 0x00, 0x7F, 0x09, 0x19, 0x29, 0x46 },   // R
    { 0x00, 0x46, 0x49, 0x49, 0x49, 0x31 },   // S
    { 0x00, 0x01, 0x01, 0x7F, 0x01, 0x01 },   // T
    { 0x00, 0x3F, 0x40, 0x40, 0x40, 0x3F },   // U
    { 0x00, 0x1F, 0x20, 0x40, 0x20, 0x1F },   // V
    { 0x00, 0x3F, 0x40, 0x38, 0x40, 0x3F },   // W
    { 0x00, 0x63, 0x14, 0x08, 0x14, 0x63 },   // X
    { 0x00, 0x07, 0x08, 0x70, 0x08, 0x07 },   // Y
    { 0x00, 0x61, 0x51, 0x49, 0x45, 0x43 },   // Z
    { 0x00, 0x00, 0x7F, 0x41, 0x41, 0x00 },   // [
    { 0x00, 0x55, 0x2A, 0x55, 0x2A, 0x55 },   // 55
    { 0x00, 0x00, 0x41, 0x41, 0x7F, 0x00 },   // ]
    { 0x00, 0x04, 0x02, 0x01, 0x02, 0x04 },   // ^
    { 0x00, 0x40, 0x40, 0x40, 0x40, 0x40 },   // _
    { 0x00, 0x00, 0x01, 0x02, 0x04, 0x00 },   // '
    { 0x00, 0x20, 0x54, 0x54, 0x54, 0x78 },   // a
    { 0x00, 0x7F, 0x48, 0x44, 0x44, 0x38 },   // b
    { 0x00, 0x38, 0x44, 0x44, 0x44, 0x20 },   // c
    { 0x00, 0x38, 0x44, 0x44, 0x48, 0x7F },   // d
    { 0x00, 0x38, 0x54, 0x54, 0x54, 0x18 },   // e
    { 0x00, 0x08, 0x7E, 0x09, 0x01, 0x02 },   // f
    { 0x00, 0x18, 0xA4, 0xA4, 0xA4, 0x7C },   // g
    { 0x00, 0x7F, 0x08, 0x04, 0x04, 0x78 },   // h
    { 0x00, 0x00, 0x44, 0x7D, 0x40, 0x00 },   // i
    { 0x00, 0x40, 0x80, 0x84, 0x7D, 0x00 },   // j
    { 0x00, 0x7F, 0x10, 0x28, 0x44, 0x00 },   // k
    { 0x00, 0x00, 0x41, 0x7F, 0x40, 0x00 },   // l
    { 0x00, 0x7C, 0x04, 0x18, 0x04, 0x78 },   // m
    { 0x00, 0x7C, 0x08, 0x04, 0x04, 0x78 },   // n
    { 0x00, 0x38, 0x44, 0x44, 0x44, 0x38 },   // o
    { 0x00, 0xFC, 0x24, 0x24, 0x24, 0x18 },   // p
    { 0x00, 0x18, 0x24, 0x24, 0x18, 0xFC },   // q
    { 0x00, 0x7C, 0x08, 0x04, 0x04, 0x08 },   // r
    { 0x00, 0x48, 0x54, 0x54, 0x54, 0x20 },   // s
    { 0x00, 0x04, 0x3F, 0x44, 0x40, 0x20 },   // t
    { 0x00, 0x3C, 0x40, 0x40, 0x20, 0x7C },   // u
    { 0x00, 0x1C, 0x20, 0x40, 0x20, 0x1C },   // v
    { 0x00, 0x3C, 0x40, 0x30, 0x40, 0x3C },   // w
    { 0x00, 0x44, 0x28, 0x10, 0x28, 0x44 },   // x
    { 0x00, 0x1C, 0xA0, 0xA0, 0xA0, 0x7C },   // y
    { 0x00, 0x44, 0x64, 0x54, 0x4C, 0x44 },   // z
    { 0x14, 0x14, 0x14, 0x14, 0x14, 0x14 }    // horiz lines
};


void LCD_WrDat(byte data)
{
	byte i=8;
	LCD_DC_H;
    LCD_SCL_L;    
  while(i--)
  {
    if(data&0x80){LCD_SDA_H;}
    else{LCD_SDA_L;}
    LCD_SCL_H; 
           
    LCD_SCL_L;;    
    data<<=1;    
  }

}
void LCD_WrCmd(byte cmd)
{
	byte i=8;
	LCD_DC_L;
    LCD_SCL_L;
   
  while(i--)
  {
    if(cmd&0x80){LCD_SDA_H;}
    else{LCD_SDA_L;}
    LCD_SCL_H;          
    LCD_SCL_L;    
    cmd<<=1;;   
  } 	
}
void LCD_Set_Pos(byte x, byte y)
{ 
  LCD_WrCmd(0xb0+y);
  LCD_WrCmd(((x&0xf0)>>4)|0x10);
  LCD_WrCmd((x&0x0f)|0x01); 
} 
void LCD_Fill(byte bmp_data)
{
	byte y,x;
	
	for(y=0;y<8;y++)
	{
		LCD_WrCmd(0xb0+y);
		LCD_WrCmd(0x01);
		LCD_WrCmd(0x10);
		for(x=0;x<X_WIDTH;x++)
			LCD_WrDat(bmp_data);
	}
}
void LCD_CLS(void)
{
	byte y,x;	
	for(y=0;y<8;y++)
	{
		LCD_WrCmd(0xb0+y);
		LCD_WrCmd(0x01);
		LCD_WrCmd(0x10); 
		for(x=0;x<X_WIDTH;x++)
			LCD_WrDat(0);
	}
}
void LCD_DLY_ms(word ms)
{                         
  word a;
  while(ms)
  {
    a=1335;
    while(a--);
    ms--;
  }
  return;
}
void adjust(byte a)
{
  LCD_WrCmd(a);	//指令数据0x0000~0x003f  
}
void SetStartColumn(unsigned char d)
{
	LCD_WrCmd(0x00+d%16);		// Set Lower Column Start Address for Page Addressing Mode
						//   Default => 0x00
	LCD_WrCmd(0x10+d/16);		// Set Higher Column Start Address for Page Addressing Mode
						//   Default => 0x10
}

void SetAddressingMode(unsigned char d)
{
	LCD_WrCmd(0x20);			// Set Memory Addressing Mode
	LCD_WrCmd(d);			//   Default => 0x02
						//     0x00 => Horizontal Addressing Mode
						//     0x01 => Vertical Addressing Mode
						//     0x02 => Page Addressing Mode
}

void SetColumnAddress(unsigned char a, unsigned char b)
{
	LCD_WrCmd(0x21);			// Set Column Address
	LCD_WrCmd(a);			//   Default => 0x00 (Column Start Address)
	LCD_WrCmd(b);			//   Default => 0x7F (Column End Address)
}

void SetPageAddress(unsigned char a, unsigned char b)
{
	LCD_WrCmd(0x22);			// Set Page Address
	LCD_WrCmd(a);			//   Default => 0x00 (Page Start Address)
	LCD_WrCmd(b);			//   Default => 0x07 (Page End Address)
}

void SetStartLine(unsigned char d)
{
	LCD_WrCmd(0x40|d);			// Set Display Start Line
						//   Default => 0x40 (0x00)
}

void SetContrastControl(unsigned char d)
{
	LCD_WrCmd(0x81);			// Set Contrast Control
	LCD_WrCmd(d);			//   Default => 0x7F
}

void Set_Charge_Pump(unsigned char d)
{
	LCD_WrCmd(0x8D);			// Set Charge Pump
	LCD_WrCmd(0x10|d);			//   Default => 0x10
						//     0x10 (0x00) => Disable Charge Pump
						//     0x14 (0x04) => Enable Charge Pump
}

void Set_Segment_Remap(unsigned char d)
{
	LCD_WrCmd(0xA0|d);			// Set Segment Re-Map
						//   Default => 0xA0
						//     0xA0 (0x00) => Column Address 0 Mapped to SEG0
						//     0xA1 (0x01) => Column Address 0 Mapped to SEG127
}

void Set_Entire_Display(unsigned char d)
{
	LCD_WrCmd(0xA4|d);			// Set Entire Display On / Off
						//   Default => 0xA4
						//     0xA4 (0x00) => Normal Display
						//     0xA5 (0x01) => Entire Display On
}

void Set_Inverse_Display(unsigned char d)
{
	LCD_WrCmd(0xA6|d);			// Set Inverse Display On/Off
						//   Default => 0xA6
						//     0xA6 (0x00) => Normal Display
						//     0xA7 (0x01) => Inverse Display On
}

void Set_Multiplex_Ratio(unsigned char d)
{
	LCD_WrCmd(0xA8);			// Set Multiplex Ratio
	LCD_WrCmd(d);			//   Default => 0x3F (1/64 Duty)
}

void Set_Display_On_Off(unsigned char d)
{
	LCD_WrCmd(0xAE|d);			// Set Display On/Off
						//   Default => 0xAE
						//     0xAE (0x00) => Display Off
						//     0xAF (0x01) => Display On
}

void SetStartPage(unsigned char d)
{
	LCD_WrCmd(0xB0|d);			// Set Page Start Address for Page Addressing Mode
						//   Default => 0xB0 (0x00)
}

void Set_Common_Remap(unsigned char d)
{
	LCD_WrCmd(0xC0|d);			// Set COM Output Scan Direction
						//   Default => 0xC0
						//     0xC0 (0x00) => Scan from COM0 to 63
						//     0xC8 (0x08) => Scan from COM63 to 0
}

void Set_Display_Offset(unsigned char d)
{
	LCD_WrCmd(0xD3);			// Set Display Offset
	LCD_WrCmd(d);			//   Default => 0x00
}

void Set_Display_Clock(unsigned char d)
{
	LCD_WrCmd(0xD5);			// Set Display Clock Divide Ratio / Oscillator Frequency
	LCD_WrCmd(d);			//   Default => 0x80
						//     D[3:0] => Display Clock Divider
						//     D[7:4] => Oscillator Frequency
}

void Set_Precharge_Period(unsigned char d)
{
	LCD_WrCmd(0xD9);			// Set Pre-Charge Period
	LCD_WrCmd(d);			//   Default => 0x22 (2 Display Clocks [Phase 2] / 2 Display Clocks [Phase 1])
						//     D[3:0] => Phase 1 Period in 1~15 Display Clocks
						//     D[7:4] => Phase 2 Period in 1~15 Display Clocks
}

void Set_Common_Config(unsigned char d)
{
	LCD_WrCmd(0xDA);			// Set COM Pins Hardware Configuration
	LCD_WrCmd(0x02|d);			//   Default => 0x12 (0x10)
						//     Alternative COM Pin Configuration
						//     Disable COM Left/Right Re-Map
}

void Set_VCOMH(unsigned char d)
{
	LCD_WrCmd(0xDB);			// Set VCOMH Deselect Level
	LCD_WrCmd(d);			//   Default => 0x20 (0.77*VCC)
}

void Set_NOP(void)
{
	LCD_WrCmd(0xE3);			// Command for No Operation
}

void LCD_Init(void)        
{
 
	LCD_SCL_H;
	//LCD_CS=1;	//预制SLK和SS为高电平   	
	
	LCD_RST_L;
	LCD_DLY_ms(50);
	LCD_RST_H;
	
	//从上电到下面开始初始化要有足够的时间，即等待RC复位完毕
  Set_Display_On_Off(0x00);		  // Display Off (0x00/0x01)
  Set_Display_Clock(0x80);		  // Set Clock as 100 Frames/Sec
  Set_Multiplex_Ratio(0x3F);		// 1/64 Duty (0x0F~0x3F)
  Set_Display_Offset(0x00);		  // Shift Mapping RAM Counter (0x00~0x3F)
  SetStartLine(0x00);			      // Set Mapping RAM Display Start Line (0x00~0x3F)
  Set_Charge_Pump(0x04);		    // Enable Embedded DC/DC Converter (0x00/0x04)
  SetAddressingMode(0x02);		  // Set Page Addressing Mode (0x00/0x01/0x02)
  Set_Segment_Remap(0x01);		  // Set SEG/Column Mapping     0x00左右反置 0x01正常
  Set_Common_Remap(0x08);			  // Set COM/Row Scan Direction 0x00上下反置 0x08正常
  Set_Common_Config(0x10);		  // Set Sequential Configuration (0x00/0x10)
  SetContrastControl(Brightness);	// Set SEG Output Current
  Set_Precharge_Period(0xF1);		// Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
  Set_VCOMH(0x40);			        // Set VCOM Deselect Level
  Set_Entire_Display(0x00);		  // Disable Entire Display On (0x00/0x01)
  Set_Inverse_Display(0x00);		// Disable Inverse Display On (0x00/0x01)  
  Set_Display_On_Off(0x01);		  // Display On (0x00/0x01)
  LCD_Fill(0x00);  //初始清屏
	LCD_Set_Pos(0,0); 
	
} 
//==============================================================
//函数名： void LCD_PutPixel(byte x,byte y)
//功能描述：绘制一个点（x,y）
//参数：真实坐标值(x,y),x的范围0～127，y的范围0～64
//返回：无
//==============================================================
void LCD_PutPixel(byte x,byte y)
{
	byte data1;  //data1当前点的数据 
	 
  LCD_Set_Pos(x,y); 
	data1 = 0x01<<(y%8); 	
	LCD_WrCmd(0xb0+(y>>3));
	LCD_WrCmd(((x&0xf0)>>4)|0x10);
	LCD_WrCmd((x&0x0f)|0x00);
	LCD_WrDat(data1); 	 	
}
//==============================================================
//函数名： void LCD_Rectangle(byte x1,byte y1,
//                   byte x2,byte y2,byte color,byte gif)
//功能描述：绘制一个实心矩形
//参数：左上角坐标（x1,y1）,右下角坐标（x2，y2）
//      其中x1、x2的范围0～127，y1，y2的范围0～63，即真实坐标值
//返回：无
//==============================================================
void LCD_Rectangle(byte x1,byte y1,byte x2,byte y2,byte gif)
{
	byte n; 
		
	LCD_Set_Pos(x1,y1>>3);
	for(n=x1;n<=x2;n++)
	{
		LCD_WrDat(0x01<<(y1%8)); 			
		if(gif == 1) 	LCD_DLY_ms(50);
	}  
	LCD_Set_Pos(x1,y2>>3);
  for(n=x1;n<=x2;n++)
	{
		LCD_WrDat(0x01<<(y2%8)); 			
		if(gif == 1) 	LCD_DLY_ms(5);
	}
	
}  
//==============================================================
//函数名：LCD_P6x8Str(byte x,byte y,byte *p)
//功能描述：写入一组标准ASCII字符串
//参数：显示的位置（x,y），y为页范围0～7，要显示的字符串
//返回：无
//==============================================================  
void LCD_P6x8Str(byte x,byte y,byte ch[])
{
  byte c=0,i=0,j=0;      
  while (ch[j]!='\0')
  {    
    c =ch[j]-32;
    if(x>126){x=0;y++;}
    LCD_Set_Pos(x,y);    
  	for(i=0;i<6;i++)     
  	  LCD_WrDat(F6x8[c][i]);  
  	x+=6;
  	j++;
  }
}

//输出汉字和字符混合字符串
void LCD_Print(byte x, byte y, byte ch[])
{
	byte ch2[3];
	byte ii=0;        
	while(ch[ii] != '\0')
	{
		if(ch[ii] > 127)
		{
			ch2[0] = ch[ii];
	 		ch2[1] = ch[ii + 1];
			ch2[2] = '\0';			//汉字为两个字节
			LCD_P6x8Str(x , y, ch2);	//显示汉字
			x += 14;
			ii += 2;
		}
		else
		{
			ch2[0] = ch[ii];	
			ch2[1] = '\0';			//字母占一个字节
			LCD_P6x8Str(x , y , ch2);	//显示字母
			x += 14;
			ii+= 1;
		}
	}
} 


void Dis_Num(byte y, byte x, int num,byte N) 
{

  byte n[7]={0};
 // x=x*8;
  if(num>=0)
  {
  n[0]=(num/1000)+16+32; 
  n[1]=(num%1000/100)+16+32;
  n[2]=(num%1000%100/10)+16+32;
  n[3]=(num%1000%100%10)+16+32;
  n[4]=46;
  n[5]=48;
  n[6]='\0';
  }
  if(num<0)
  {
    num=Absolute(num);
  n[0]=45;
  n[1]=(num/1000)+16+32; 
  n[2]=(num%1000/100)+16+32;
  n[3]=(num%1000%100/10)+16+32;
  n[4]=(num%1000%100%10)+16+32;
  n[5]=46;
  n[6]='\0';
  }
  LCD_P6x8Str(x,y,&n[7-N]);//从ACSII码表中读取字节，然后写入液晶
}

void Dis_Num2(byte y, byte x,float num1,byte N) 
{
  int num2;
  num2=(int)(10000*num1);
  byte n[7]={0};
//  x=x*8;
  if(num2>=0)
  {
  n[0]=43; 
  n[1]=(num2/10000)+16+32; 
  n[2]=46;
  n[3]=(num2%10000/1000)+16+32;
  n[4]=(num2%10000%1000/100)+16+32;
  n[5]=(num2%10000%1000%100/10)+16+32;
  n[6]='\0';
  }
   if(num2<0)
  {
    num2=Absolute(num2);
  n[0]=45;
  n[1]=(num2/10000)+16+32; 
  n[2]=46;
  n[3]=(num2%10000/1000)+16+32;
  n[4]=(num2%10000%1000/100)+16+32;
  n[5]=(num2%10000%1000%100/10)+16+32;
  n[6]='\0';
  }
  LCD_P6x8Str(x,y,&n[7-N]);//从ACSII码表中读取字节，然后写入液晶
}

