/*********************************************************************************
**********************************************************************************
* 文件名称: uart.c                                                         	     *
* 文件简述：串口查看变量+发送ADC/DAC/FFT数据								                     *
* 创建日期：2024.05.06                                                           *
**********************************************************************************
*********************************************************************************/

#include "uart.h"

char String[Str_Len];	//格式化字符串

/****************************************************************************
* 名    称: void uart1_init(u32 bound)
* 功    能：USART1初始化
* 入口参数：bound：波特率
* 返回参数：无
* 说    明： 
****************************************************************************/
void uart1_init(u32 bound)
{ 
	/* 使能时钟 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟 168MHz
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1时钟 84MHz

	/* USART1端口配置 */
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;	//GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	//推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	//上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure);	//初始化PA9，PA10
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);	//GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);	//GPIOA10复用为USART1
	
  /* USART1初始化设置 */
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;  //一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART1, &USART_InitStructure);	//初始化串口1	
	USART_ClearFlag(USART1, USART_FLAG_TC);
	
	/* USART1使能 */
  USART_Cmd(USART1, ENABLE);  //使能串口1 
}

void uart4_init(u32 bound)
{ 
	/* 使能时钟 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //使能GPIOC时钟 168MHz
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);//使能UART4时钟 42MHz

	/* USART4端口配置 */
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;	//GPIOC10与GPIOC11
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	//推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	//上拉
	GPIO_Init(GPIOC,&GPIO_InitStructure);	//初始化PC10，PC11
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4);	//GPIOC10复用为UART4
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4);	//GPIOC11复用为UART4
	
  /* UART4初始化设置 */
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;  //一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(UART4, &USART_InitStructure);	//初始化串口4
	USART_ClearFlag(UART4, USART_FLAG_TC);
	
	/* UART4使能 */
  USART_Cmd(UART4, ENABLE);  //使能串口4
}

//串口1发送一个字符
void uart1SendChar(u8 ch)
{      
	while((USART1->SR&0x40)==0){}	//循环发送,直到发送完毕
    USART1->DR = (u8) ch;
}

//串口4发送一个字符
void uart4SendChar(u8 ch)
{      
	while((UART4->SR&0x40)==0){}	//循环发送,直到发送完毕
    UART4->DR = (u8) ch;
}

/****************************************************************************
* 名    称: void uart1SendChars(u8 *str, u8 strlen)
* 功    能：串口1发送一字符串
* 入口参数：*str：发送的字符串
            strlen：字符串长度
* 返回参数：无
* 说    明： 
****************************************************************************/
void uart1SendChars(u8 *str, u8 strlen)
{ 
	  u8 k= 0 ; 
   do { uart1SendChar(*(str + k)); k++; }
    while (k < strlen); 
}

void uart4SendChars(u8 *str, u8 strlen)
{ 
	  u8 k= 0 ; 
   do { uart4SendChar(*(str + k)); k++; }
    while (k < strlen); 
}

/****************************************************************************
* 名    称: int fputc(int ch, FILE *f)
* 功    能：重定向，让printf输出到串口
* 入口参数：
* 返回参数：
* 说    明：因printf()之类的函数，使用了半主机模式。使用标准库会导致程序无法
            运行,以下是解决方法:使用微库,因为使用微库的话,不会使用半主机模式.
            请在工程属性的“Target“-》”Code Generation“中勾选”Use MicroLIB“这
            样以后就可以使用printf，sprintf函数了
****************************************************************************/
int fputc(int ch, FILE *f)   //重定向，让printf输出到串口  
{
    uart4SendChar(ch);
    while (USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET);
    return ch;
}
