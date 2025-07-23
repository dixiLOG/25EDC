#include "dac7612.h"

//模拟SPI
void DAC7612_Write_Reg(uint16_t data)
{
	uint8_t i;
	
	DAC7612_LOAD_1();
	delay_us(1);
	DAC7612_CS_0();
	delay_us(1);
	for(i=0;i<14;i++) {
		DAC7612_CLK_0();
		if(data&0x2000)
			DAC7612_SDI_1();
		else
			DAC7612_SDI_0();
		delay_us(1);
		data<<=1;
		DAC7612_CLK_1();
		delay_us(1);
	}
	delay_us(1);
	DAC7612_CS_1();
	delay_us(1);
	DAC7612_LOAD_0();	
}

//设置OUTA/OUTB
void DAC7612_Write_CHAB(uint16_t data)
{
	DAC7612_Write_Reg(0x0000|data);
	delay_us(1);
}

//DAC7612初始化
void DAC7612_Init(void)
{
	// 使能GPIOC时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
	// PC0/1/14/15初始化设置
  GPIO_InitTypeDef  GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = DAC7612_SDI_Pin|DAC7612_CLK_Pin|DAC7612_CS_Pin|DAC7612_LOAD_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;     	// 普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;    	// 推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		// 50MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(DAC7612_Port,&GPIO_InitStructure);
	
	// DAC7612初始化
	DAC7612_CS_1();
	DAC7612_CLK_1();
	DAC7612_LOAD_0();
	
	//初始设置双通道输出为0
	DAC7612_Write_Reg(0x0000|0x000);
}

//设置OUTA
void DAC7612_Write_CHA(uint16_t data)
{
  DAC7612_Write_Reg(0x2000|data);
	delay_us(1);
}

//设置OUTB
void DAC7612_Write_CHB(uint16_t data)
{
	DAC7612_Write_Reg(0x3000|data);
	delay_us(1);
}
