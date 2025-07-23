#include "touch.h"
#include "xpt2046.h"
#include "lcd.h"
#include "led.h"
#include "common.h"
#include "uart.h"       // UART 串口驱动 (USART1/UART4)

/*********************************************************************************
*********************启明欣欣 STM32F407应用开发板(高配版)*************************
**********************************************************************************
* 文件名称: touch.c                                                              *
* 文件简述：触摸屏测试程序                                                       *
* 创建日期：2018.08.30                                                           *
* 版    本：V1.0                                                                 *
* 作    者：Clever                                                               *
* 说    明：                                                                     * 
**********************************************************************************
*********************************************************************************/

//触摸屏初始化
void Touch_Init(void)
{
	if(lcd_id==0x9341)
	{
	  XPT2046_Init();
	}

}

//清空屏幕
void Clear_Screen(void)
{
	LCD_Clear(WHITE);//清屏   
 	BRUSH_COLOR=BLUE;//设置字体为蓝色 
	LCD_DisplayString(lcd_width-40,lcd_height-18,16,"Clear");//显示清屏区域
  BRUSH_COLOR=RED;//设置画笔蓝色 
}

/****************************************************************************
* 名    称: u8 Draw_Point(u16 x,u16 y,u16 color)
* 功    能：画一个点(4*4的点)	
* 入口参数：x,y:坐标
            color:颜色
* 返回参数: 无  
* 说    明：        
****************************************************************************/
void Draw_Point(u16 x,u16 y,u16 color)
{	    	
	u8 i=0;

	BRUSH_COLOR=color;
	for(i=0;i<4;i++)
	{
	 LCD_DrawPoint(x,y+i);
	 LCD_DrawPoint(x+1,y+i);
	 LCD_DrawPoint(x+2,y+i);
	 LCD_DrawPoint(x+3,y+i);
	} 	  	
}	

//电阻触摸屏测试函数
void R_Touch_test(void)
{
	u8 i=0;	  
	while(1)
	{
		if(PEN) LED1=0;
		else LED1=1;
		XPT2046_Scan(0); 		 
	
		 	if(Xdown<lcd_width&&Ydown<lcd_height)
			{	
				if(Xdown>(lcd_width-40)&&Ydown>lcd_height-18)Clear_Screen();  //清空屏幕
				else Draw_Point(Xdown,Ydown,RED);		//画图	  			   
			} 
		LCD_DisplayNum(10,10,Xdown,5,16,1);
		LCD_DisplayNum(150,10,Ydown,5,16,1);
			//LCD_DisplayString_color(10,10,16,*Xdown,RED,WHITE);
    	Draw_Point(Xdown,Ydown,RED);
		//i++;
		//if(i%20==0)
		//	LED1=!LED1;
	}
}


/**
 * @brief   LCD上触屏交互
 * @hint 		屏幕为320*250
 */
u8 button_cnt = 4;
u8 button_startX[4] = {25,75,125,175};
u8 button_startY = 125;
u8 button_endX[4] = {65,115,165,215};
u8 button_endY = 145;
u8 button_centerX[4] = {45,95,145,195};
u8 button_centerY = 135;
u8 judge_point;
u8 flagForTouch = 0;

// 用于判断所按按钮，返回中心点横坐标
int judge_button(){
	u8 i;
	for(i=0;i<button_cnt;i++){
		if(Ydown > button_startY && Ydown < button_endY && Xdown>button_startX[i] && Xdown<button_endX[i]){
			return button_centerX[i];
		}
	}
}

void LCD_TOUCH_TEST(){
		//初始化，设定四个按钮
		// Clear_Screen();
		for(u8 i=0;i<button_cnt;i++){
		LCD_Color_DrawRectangle(button_startX[i],button_startY,button_endX[i],button_endY,BLACK);
		}
		//扫描屏幕
		XPT2046_Scan(0);
		//sprintf(String, "%d %d",Xdown,Ydown);
		//LCD_DisplayString(10, 100, 12, (u8*)String);
		
		// 判断落点与输出
		judge_point = judge_button();
		for(u8 i=0;i<button_cnt;i++){
			// 此处DIY
			if(judge_point == button_centerX[i]){
				// 模拟按键，修改值
				flagForTouch = i;
				
				sprintf(String, "HI There");
				LCD_DisplayString(judge_point - 18, button_centerY + 10 , 12, (u8*)String);
				sprintf(String, "flag:%d",flagForTouch);
				LCD_DisplayString(judge_point - 18, button_centerY + 20 , 12, (u8*)String);
			}
		}
}
	

