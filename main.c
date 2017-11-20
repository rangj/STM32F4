#include "stm32f4xx.h"
#include "delay.h"
#include "gpio.h"
#include "adc.h"
#include "OLED.h"
#include "timer.h"
#include "filter.h"


void Display(void); 
void dcmi_clock_init();	 
int16_t ran = 0;
int main(void)
{ 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); // 2 bits for pre-emption priority,2 bits for subpriority
   /* GpioInit();
	TIM1_Driver1_Init();
	TIM_Cmd(TIM1,ENABLE);

	AppADC_RegInjInit();
    LCD_Init();
	TIM9_DC_MOTOR_Init();*/
	dcmi_clock_init();
   while(1)
   {
	;
   	/*_RUN_LED_ON;
	DC_MotorOn();
   	Display();	*/
   }
}
void dcmi_clock_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	/* TIM3 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	/* GPIOC clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	/* GPIOC Configuration:  TIM3 CH3 (PC8)  */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* Connect TIM3 pins to AF2 */;
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM3);

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 3;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	/* PWM1 Mode configuration: Channel3 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 2;// TIM_TimeBaseStructure.TIM_Period/2;

	TIM_OC3Init(TIM3, &TIM_OCInitStructure);

	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM3, ENABLE);

	/* TIM3 enable counter */
	TIM_Cmd(TIM3, ENABLE);
}

void Display(void)
{
	LCD_Print(0,0,"DV:");//X Y
    Dis_Num(0,30,ADC1_2_DMA_DATA2.Split.DcbusV_Org,7); //Y X
    
   
    LCD_Print(68,0,"RS:");
    Dis_Num(0,98,ADC1_2_DMA_DATA2.Split.DcbusV_Org2,7); 
    
    /*LCD_Print(0,1,"LH:");
    Dis_Num(1,30,left_h,7); 
    
    LCD_Print(68,1,"RH:");
    Dis_Num(1,98,right_h,7); 
    
    LCD_Print(0,2,"LH2:");
    Dis_Num(2,30,left_h2,7); 
    
      LCD_Print(68,2,"RH2:");
    Dis_Num(2,98,right_h2,7); 
    
    LCD_Print(0,3,"c1:");
    Dis_Num2(3,30,c1,7); 
    
       LCD_Print(68,3,"q:");
    Dis_Num(3,98,distance,7);
    
    LCD_Print(0,4,"c2:");
    Dis_Num2(4,30,c2,7); 
    
    LCD_Print(0,5,"err:");
    Dis_Num2(5,30,err,7);
    LCD_Print(0,6,"tq:");
    Dis_Num(6,30,esp2,7);
      
    LCD_Print(68,6,"c4:");
    Dis_Num2(6,98,c4,7); */

     
}
