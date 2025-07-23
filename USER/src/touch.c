#include "touch.h"
#include "xpt2046.h"
#include "lcd.h"
#include "led.h"
#include "common.h"
#include "uart.h"       // UART �������� (USART1/UART4)

/*********************************************************************************
*********************�������� STM32F407Ӧ�ÿ�����(�����)*************************
**********************************************************************************
* �ļ�����: touch.c                                                              *
* �ļ����������������Գ���                                                       *
* �������ڣ�2018.08.30                                                           *
* ��    ����V1.0                                                                 *
* ��    �ߣ�Clever                                                               *
* ˵    ����                                                                     * 
**********************************************************************************
*********************************************************************************/

//��������ʼ��
void Touch_Init(void)
{
	if(lcd_id==0x9341)
	{
	  XPT2046_Init();
	}

}

//�����Ļ
void Clear_Screen(void)
{
	LCD_Clear(WHITE);//����   
 	BRUSH_COLOR=BLUE;//��������Ϊ��ɫ 
	LCD_DisplayString(lcd_width-40,lcd_height-18,16,"Clear");//��ʾ��������
  BRUSH_COLOR=RED;//���û�����ɫ 
}

/****************************************************************************
* ��    ��: u8 Draw_Point(u16 x,u16 y,u16 color)
* ��    �ܣ���һ����(4*4�ĵ�)	
* ��ڲ�����x,y:����
            color:��ɫ
* ���ز���: ��  
* ˵    ����        
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

//���败�������Ժ���
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
				if(Xdown>(lcd_width-40)&&Ydown>lcd_height-18)Clear_Screen();  //�����Ļ
				else Draw_Point(Xdown,Ydown,RED);		//��ͼ	  			   
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
 * @brief   LCD�ϴ�������
 * @hint 		��ĻΪ320*250
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

// �����ж�������ť���������ĵ������
int judge_button(){
	u8 i;
	for(i=0;i<button_cnt;i++){
		if(Ydown > button_startY && Ydown < button_endY && Xdown>button_startX[i] && Xdown<button_endX[i]){
			return button_centerX[i];
		}
	}
}

void LCD_TOUCH_TEST(){
		//��ʼ�����趨�ĸ���ť
		// Clear_Screen();
		for(u8 i=0;i<button_cnt;i++){
		LCD_Color_DrawRectangle(button_startX[i],button_startY,button_endX[i],button_endY,BLACK);
		}
		//ɨ����Ļ
		XPT2046_Scan(0);
		//sprintf(String, "%d %d",Xdown,Ydown);
		//LCD_DisplayString(10, 100, 12, (u8*)String);
		
		// �ж���������
		judge_point = judge_button();
		for(u8 i=0;i<button_cnt;i++){
			// �˴�DIY
			if(judge_point == button_centerX[i]){
				// ģ�ⰴ�����޸�ֵ
				flagForTouch = i;
				
				sprintf(String, "HI There");
				LCD_DisplayString(judge_point - 18, button_centerY + 10 , 12, (u8*)String);
				sprintf(String, "flag:%d",flagForTouch);
				LCD_DisplayString(judge_point - 18, button_centerY + 20 , 12, (u8*)String);
			}
		}
}
	

