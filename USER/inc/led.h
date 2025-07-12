#ifndef __LED_H
#define __LED_H
#include "common.h"

////////////////////////////////////////////////////////////////////////////////////	

//LED端口定义
#define LED1 PBout(9)
#define LED2 PBout(8)
#define LED3 PBout(7)

//函数声明
void LED_Init(void);	//初始化	

#endif
