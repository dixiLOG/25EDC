/*********************************************************************************
**********************************************************************************
* 文件名称: key.c                                                               	 *
* 文件简述：按键扫描程序                                                        	 *
* 创建日期：2024.07.11                                                          	 *
* 说    明：该按键扫描，包涵各种按键模式的扫描                                   	 * 
**********************************************************************************
*********************************************************************************/

#include "key.h"

/**
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~按键模式剖析~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
一、按键返回值，即按键扫描结果值，分为以下几种情况
		1、按键按下未抬起时的返回值，对应代码中 keydown_data 变量的值
		2、按键按下抬起后才返回的值，对应代码中 keyup_data   变量的值
		3、按任意键有效，只需在程序中判断key_time>某个值，该值取决于执行一次扫描函数的
		   时间长短，扫描函数时间长，相应这该值就小，反之亦然。
		4、需要长按一个键一段时间才执行相应的程序，就需要该键键值key_tem与key_time
		   配合使用，即实现某个按键(key_tem)长按多久(key_time)后执行相应的程序
二、按键使用模式
    1、单按：按键按下（一直按着）只返回一次有效按键值
		2、连按：按键一直按着每执行一次按键扫描函数就返回一次按键值
		3、void key_scan(u8 mode) 中 mode   0：单按   1:连按
三、按键使用注意事项;
    1、当使用抬起返回值keyup_data时，按键的连按模式无效，因为键抬起才返回按键值，你
		   按再久都没用，所以在使用keyup_data时，模式mode必须设置为0
		2、如果需要keydown_data与key_time配合构成长按，模式mode需设置为1，才可用
		3、如果需要keyup_data与key_time配合构成长按，就要对程序作出相应的改动，在函数的
		   末尾，将 key_time=0; 改为 key_tem=0;就可以使用长按一个按键一段时间，等时间到
			 并且按键抬起才执行相应的代码。不过这样改动，按任意键就失效了，因为按键抬起没
			 把key_time清零，key_time一直有值。或者用户还是需要任意键有效就要自己在相应的
			 代码中把key_time清零。这种长按配合使用在实际是很少用到的。
**/

u8  keydown_data=0x00;    //按键按下后就返回的值
u8  keyup_data=0x00;      //按键抬起返回值
u16  key_time=0x00;       //按键按下之后的时间计数，该值乘以扫描一次按键函数的时间就等于按键按下的时间

u8  key_tem=0x00;         //长按的按键值与按键扫描程序过渡变量
u8  key_bak=0x00;         //按键扫描程序过渡变量

//按键IO口初始化函数
void KEY_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD, ENABLE);    //使能GPIOA/C/D时钟
 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_3|GPIO_Pin_2; //KEY1 KEY2 KEY3对应引脚
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;             //普通输入模式
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;       //100M
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;             //上拉
  GPIO_Init(GPIOD, &GPIO_InitStructure);                   //初始化GPIOD6/3/2
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; //KEY4对应引脚
  GPIO_Init(GPIOC, &GPIO_InitStructure);                   //初始化GPIOC12
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15; //KEY5对应引脚
  GPIO_Init(GPIOA, &GPIO_InitStructure);                   //初始化GPIOA15
} 
/****************************************************************************
* 名    称: void key_scan(u8 mode)
* 功    能：按键扫描函数
* 入口参数：mode：0：单按 
                  1: 连按
* 返回参数：无
* 说    明：响应优先级,KEY1>KEY2>KEY3>KEY4/KEY5
****************************************************************************/
void key_scan(u8 mode)
{	   
	keyup_data=0;         //键抬起后按键值一次有效
	if(KEY1==0||KEY2==0||KEY3==0||KEY4==0||KEY5==0)   //有键正按下
	{
		if(KEY1==0)      key_tem=1;
		else if(KEY2==0) key_tem=2;
		else if(KEY3==0) key_tem=3;
		else if(KEY4==0) key_tem=4;
		else if(KEY5==0) key_tem=5;
		if (key_tem == key_bak)	//有键按下后第一次扫描不处理，与else配合第二次扫描有效，这样实现了去抖动
		{
			key_time++;	//有键按下后执行一次扫描函数，该变量加1
			keydown_data=key_tem;	//按键值赋予keydown_data
			if( (mode==0)&&(key_time>1) )	//key_time>1按键值无效，这就是单按，如果mode为1就为连按
				keydown_data=0;
		}
		else	//去抖动
		{
			key_time=0;
			key_bak=key_tem;
		}
	}
	else	//键抬起
	{
		if(key_time>2)	//按键抬起后返回一次按键值
			keyup_data=key_tem;	//键抬起后按键值赋予keydown_data
		key_bak=0;	//要清零，不然下次执行扫描程序时按键的值跟上次按的值一样，就没有去抖动处理了
		key_time=0;
		keydown_data=0;
	}
}
