#include "GPIO.h"
#include "stm32f4xx.h"
#include "common.h"

void GPIO_INIT(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;		//定义输入结构体变量
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOE,ENABLE);	//使能PB端口
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;						//端口模式设置
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;	//端口设置
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;				//刷新速度设置
	GPIO_Init(GPIOB, &GPIO_InitStructure);							//调用初始化函数，初始化指定
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_12|GPIO_Pin_13;	//端口设置
	GPIO_Init(GPIOD, &GPIO_InitStructure);							//调用初始化函数，初始化指定

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_14|GPIO_Pin_15|GPIO_Pin_13;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8|GPIO_Pin_12|GPIO_Pin_11|GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	
	
}


