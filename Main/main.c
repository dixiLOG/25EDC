/*********************************************************************************
 **********************************************************************************
 * @file      main.c
 * @brief     主程序文件
 * @date      2024-07-11
 * @modify    2025-07-16
 * @note
 * 系统时钟配置:
 * - HSE: 8MHz (外部高速时钟)
 * - SYSCLK: 168MHz (系统时钟)
 * - HCLK: 168MHz (AHB总线时钟, SYSCLK / 1)
 * - PCLK1: 42MHz (APB1总线时钟, HCLK / 4)
 * - PCLK2: 84MHz (APB2总线时钟, HCLK / 2)
 *
 * // 获取并打印时钟频率的示例代码:
 * //   RCC_ClocksTypeDef rcc;
 * //   RCC_GetClocksFreq(&rcc);
 * //   printf("HCLK 频率: %d\r\n", rcc.HCLK_Frequency);
 *
 **********************************************************************************
 **********************************************************************************/

/*----------------- 头文件包含 -----------------*/
#include "adc.h"        // ADC 驱动 (ADC1双通道, 使用TIM2和DMA2)
#include "dac.h"        // DAC 驱动 (DAC1, 使用TIM6和DMA1)
#include "lcd.h"        // LCD 驱动 (使用FSMC)
#include "uart.h"       // UART 串口驱动 (USART1/UART4)
#include "fft.h"        // FFT (快速傅里叶变换) 处理函数
#include "spi.h"        // SPI 驱动 (SPI2/SPI3)
#include "i2c.h"        // I2C 驱动 (I2C3)
#include "w25qxx.h"     // W25QXX SPI Flash 存储器驱动
#include "key.h"        // 按键驱动
#include "led.h"        // LED 驱动
#include "math.h"       // 标准数学库
#include "AD9833.h"     // AD9833 DDS 信号发生器驱动
#include "pid.h"        // PID 控制器逻辑
#include "touch.h"      // 触摸屏驱动
#include "xpt2046.h"    // XPT2046 电阻触摸屏控制器
#include "pwm.h"        // PWM (脉冲宽度调制) 输出驱动
#include "stm32f4xx.h"  // STM32F4xx 设备头文件
#include "freqmeas.h"   // 频率测量函数
#include <stdbool.h>    // 标准布尔类型
#include "convolution.h"// 卷积计算函数
#include "arm_cmsis_dsp.h" // ARM CMSIS-DSP 库
#include "arm_math.h"   // ARM DSP 数学函数
#include "wavetest.h"   // 波形分析函数头文件

/*----------------- 全局变量 -----------------*/
// 用于格式化字符串并显示在LCD上的缓冲区
char String1[100];
char String2[100];
char String3[100];

// Flash 存储器测试参数
#define ByteCount 2     // Flash读写测试的字节数
#define Address   0x0000  // Flash测试的起始地址

// 用于存储FFT峰值频率分量的变量
float fc_max_data = 0;
float fc_max_data_n = 0;
float fc_secmax_data = 0;
float fc_secmax_data_n = 0;
float fc_thirmax_data = 0;
float fc_thirmax_data_n = 0;
float fc_forthmax_data = 0;
float fc_forthmax_data_n = 0;

// 与信号生成或分析相关的变量
u8 phase1 = 0;
u8 phase2 = 0;
u8 wavetype[2];
float DDSFre = 0;

// 用于UART通信的变量 (具体用途从上下文中不完全明确)
u8 *fc_sin_uart;
u8 *fc_tri_uart;
u8 fc_tri = 0;
u8 fc_sin = 0;
u8 addr = 0;

// 定义用于接收波形分析结果的变量
WaveResult detected_waves[MAX_WAVES];

// 演示用的偏移值，可能用于信号调理
double offset_demo = 0.227;

// 用于PID调优或其他目的的计数器
int kp_cnt = 0;
int kd_cnt = 0;

//-------PID测试参数-------//
float32_t  setV = 1200;//code 
float32_t  PID_tol = 0.5;
float32_t  Kp = 3;
float32_t  Ki = 0.8;
float32_t  Kd = 0.08;
float32_t  UPLIMIT = 5000;
float32_t  DOWNLIMIT = 0;
float32_t realOut = 1200; 	// 被控对象初始值 | 经过仿真后的输出值


float32_t error, adjust;    // 误差与PID后的输出值
DSP_PID_Controller my_pid;	// 结构体

//======================


//---------LMS参数-------//
		
#define TEST_LENGTH_SAMPLES 1024
#define NUM_TAPS 64
float32_t mu = 0.005f;											// 学习率
float32_t testInput[TEST_LENGTH_SAMPLES];   // d[n]：目标 + 干扰
float32_t refInput[TEST_LENGTH_SAMPLES];    // x[n]：干扰信号
float32_t lmsOutput[TEST_LENGTH_SAMPLES];   // y[n]：LMS输出
float32_t errorOutput[TEST_LENGTH_SAMPLES]; // e[n] = d[n] - y[n]：滤波结果
float32_t fir_coeffs[NUM_TAPS];             // 自适应权重系数
float32_t lms_state[TEST_LENGTH_SAMPLES + NUM_TAPS - 1]; // buffer，必须外部定义，生命周期够长

// plot 参数

#define Y_BUF_SIZE 230  // LCD X轴显示 230 个点
float32_t y_in_buf[Y_BUF_SIZE];   // 原始信号段
float32_t y_out_buf[Y_BUF_SIZE];  // 滤波后信号段
//======================

/**
 * @brief  将一个频率值四舍五入到最接近的5的倍数。
 * @param  data 输入的频率值。
 * @retval  四舍五入后的频率值。
 */
float calculate_fc(float data) {
    int rounded_fc = 0;
    rounded_fc = (int)((data + 2.5) / 5) * 5;
    return rounded_fc;
}

/**
 * @brief  计算两个浮点数之间的绝对差。
 * @param  a 第一个数。
 * @param  b 第二个数。
 * @retval 绝对差 |a - b|。
 */
float abs_sub(float a, float b) {
    return (a > b) ? (a - b) : (b - a);
}


/**
 * @brief  将测试数据写入W25QXX Flash存储器。
 * @note   此函数将 `pBuffer` 的内容写入指定扇区，
 * 然后在LCD上显示写入的值。为下一次写入操作，会递增缓冲区的值。
 * `pBuffer` 假定为在其他文件中定义的全局变量。
 */
void write_flash() {
    // 将 pBuffer 的数据写入Flash的指定地址
    // extern u8 pBuffer[]; // 假设 pBuffer 在别处定义
    W25QXX_SectorWrite((u8*)pBuffer, Address, Byte_Count);
    
    // 格式化并在LCD上显示写入的数据
    sprintf(String, "flash_Write:%d %d", pBuffer[0], pBuffer[1]);
    pBuffer[0]++;
    pBuffer[1]++;
    LCD_Fill_onecolor(5, 120, 235, 140, WHITE);
    LCD_DisplayString_color(10, 120, 16, (u8*)String, BLUE, WHITE);
}


/**
 * @brief  从W25QXX Flash存储器中读取测试数据。
 * @note   从指定地址读取数据到 `DataBuffer`，并在LCD上显示读取的值。
 * `DataBuffer` 假定为全局变量。
 */
void read_flash() {
    // 从Flash读取数据到 DataBuffer
    // extern u8 DataBuffer[]; // 假设 DataBuffer 在别处定义
    W25QXX_Read((u8*)DataBuffer, Address, Byte_Count);
    
    // 格式化并在LCD上显示读取的数据
    sprintf(String, "flash_Read:%d %d", DataBuffer[0], DataBuffer[1]);
    LCD_Fill_onecolor(5, 140, 235, 160, WHITE);
    LCD_DisplayString_color(10, 140, 16, (u8*)String, BLUE, WHITE);
}


/**
 * @brief  在LCD上显示ADC转换值。
 * @note   从ADC转换缓冲区 (`ADC1_Data_Rx`, `ADC3_Data_Rx`) 读取值，
 * 将其转换为电压，然后显示出来。
 * 假设ADC为12位(4096级)，参考电压约为3.17V。
 */
void ADC_Show() {
    // ADC1 通道0 的值 (来自DMA缓冲区)
    LCD_DisplayString_color(10, 10, 16, (u8*)"ADC1_WAVEC_Value: ", RED, WHITE);
    sprintf(String, "%.2f V", ADC1_Data_Rx[0] * 3.29 / 4095);
    LCD_DisplayString(150, 10, 16, (u8*)String);

    // ADC3 通道0 & 1 的值 (来自DMA缓冲区)
    LCD_DisplayString_color(10, 30, 16, (u8*)"ADC3_Channel_0&1_Value: ", RED, WHITE);
    sprintf(String, "%.2f V", ADC3_Data_Rx[20] * 3.29 / 4095);
    LCD_DisplayString(30, 48, 16, (u8*)String);
    sprintf(String, "%.2f V", ADC3_Data_Rx[21] * 3.29 / 4095);
    LCD_DisplayString(120, 48, 16, (u8*)String);
}


/**
 * @brief  处理按键按下事件。
 * @note   扫描按键状态并执行相应的操作。
 */
void buttom_function() {
    // XPT2046_Scan(0); // 可选：扫描触摸屏
    key_scan(1); // 扫描硬件按键，1为连按

    if (keydown_data == KEY1_DATA) { // KEY1 被按下
        printf("KEY1 Pressed~\n"); // 通过串口输出测试信息
        LED3 = !LED3; // 翻转LED3状态
				
				//AD9833_WaveOut(wavetype[0],detected_waves[0].base_freq*1000,0,2);
				//DDSFre = detected_waves[0].base_freq*1000;
			
			
				//-------PID测试-------//
				setV += 50;
				//--------------
			
    } else if (keydown_data == KEY2_DATA) { // KEY2 被按下
        LED2 = !LED2; // 翻转LED2状态
        
				/*float f = freqMeasurement(); // 执行一次频率测量
        sprintf(String, "freqMeasurement:%.6f Hz", f);
        LCD_Fill_onecolor(5, 160, 235, 180, WHITE);
        LCD_DisplayString_color(10, 160, 12, (u8*)String, BLUE, WHITE);
				
				DDSFre += 1;
				*/
			//-------PID测试-------//
				setV -= 50;
			//--------------
			
    } else if (keydown_data == KEY3_DATA) { // KEY3 被按下
        LED3 = !LED3; // 翻转LED3状态
				
				//DDSFre -= 1;
			
    } else if (keydown_data == KEY4_DATA) { // KEY4 被按下
				AD9833_WaveOut(wavetype[0],DDSFre,0,2);
        write_flash(); // 写入数据到Flash
    } else if (keydown_data == KEY5_DATA) { // KEY5 被按下
        read_flash(); // 从Flash读取数据
    }
}


/**
 * @brief  处理DAC输出和显示。
 * @note   当前，此函数是一个占位符。被注释掉的代码展示了如何
 * 设置并显示DAC的输出电压。
 */
void DAC_OUTPUT() {
    // 以下代码被禁用，但展示了示例用法：
    DAC_Data_Tx = 1.5/3.3*4095; // 设置DAC输出电压为1.5V
    DAC_SetChannel1Data(DAC_Align_12b_R, DAC_Data_Tx);
    //
    // LCD_DisplayString_color(10,70,16,(u8*)"DAC_Value_Set: ",RED,WHITE);
    // sprintf(String,"%.2f V",DAC_GetDataOutputValue(DAC_Channel_1)*3.3/4095);
    // LCD_DisplayString(30,90,24,(u8*)String);
}


/**
 * @brief  执行FFT计算并在LCD上显示频谱。
 * @note   此函数由DMA完成标志触发。它调用FFT处理函数，
 * 然后在LCD屏幕上绘制生成的单边幅度频谱。
 */
void FFT_SHOW() {
    // 检查ADC采样和DMA传输是否完成
    if (DMA_FLAG) {
        DMA_FLAG = 0; // 清除标志位，等待下一次传输
        DSP_FFT();    // 对新的采样数据执行FFT

        // 显示FFT参数
        sprintf(String1, "fs=%d kHz", Fs);
        LCD_DisplayString_color(150, 175, 12, (u8*)String1, GREEN, WHITE);
        sprintf(String, "N=%d", FFT_N);
        LCD_DisplayString_color(100, 175, 12, (u8*)String, GREEN, WHITE);

        // 绘制频谱图区域
        LCD_Fill_onecolor(5, 190, 235, 315, WHITE);
        LCD_Color_DrawRectangle(5, 188, 235, 315, BLACK); // 绘制边框

        // 绘制X轴频率刻度
        for (u16 x = 0; x <= 230; x++) {
            if (x % 10 == 0) {
                LCD_Color_DrawLine(5 + x, 315, 5 + x, 320, RED); // 主刻度线
            } else if (x % 2 == 0) {
                LCD_Color_DrawLine(5 + x, 315, 5 + x, 317, BLACK); // 次刻度线
            }
        }
        
        // 将幅度数组归一化以便显示 (缩放到100像素高)
        for (u16 i = 0; i < (FFT_N >> 1); i++) {
            Mag_Single[i] = Mag_Single[i] * 100 / Mag_max;
        }

        // 绘制单边频谱图
        for (u16 x = 0; x <= 230; x++) {
            LCD_Color_DrawLine(5 + x, 315 - Mag_Single[x], 5 + x, 315, BLUE);
        }
    }
}


/**
 * @brief  主程序入口
 * @retval int
 */
int main(void) {
    // 系统和外设初始化
    delay_init();
    uart4_init(115200);   // 初始化UART4用于串行通信
    // SPI2_Init();
    KEY_Init();           // 初始化硬件按键
    AD9833_Init();        // 初始化AD9833 DDS信号发生器
    LED_Init();           // 初始化LED
    W25QXX_Init();        // 初始化W25QXX SPI Flash
    // I2C3_Init();
    ADC1_Init();          // 初始化ADC1
    ADC3_Init();          // 初始化ADC3
    TIM2_Init(2, 41);     // 初始化TIM2用于触发ADC采样
    TIM4_Init(10, 21000); // 初始化TIM4用于通用定时
    DAC1_Init();          // 初始化DAC1
    LCD_Init();           // 通过FSMC初始化LCD
    TIM3_PWM_Init(999, 83, 0.4); // 初始化TIM3 PWM。频率=84M/(83+1)/(999+1)=1kHz。占空比=40%
		Touch_Init();					// 触摸屏初始化;320*250
	
		// DDS测试
		// AD9833_WaveOut(SIN_WAVE,25000,0,2);

		// PID初始化
		DSP_PID_Init(&my_pid, Kp, Ki, Kd,PID_tol ,UPLIMIT , DOWNLIMIT);
		
		// LMS 系数初始化
		memset(fir_coeffs, 0, sizeof(fir_coeffs));

		
    while (1) {
        //R_Touch_test(); // 运行触摸屏测试;测试时后面功能失效
        buttom_function(); // 检查按键
				
        if (DMA_FLAG) {
            // ADC/DMA周期结束后会进入此代码块。
            // 此时上面的 FFT_SHOW() 很可能已经执行过了。
//						FFT_SHOW();        // 当新数据准备好时，计算并显示FFT频谱
//						wave_testv2();// 对新的FFT数据执行波形分析
//						printf("%f  %f %d\n",detected_waves[0].amplitude,detected_waves[0].base_freq,wavetype[0]);
//						delay_ms(500); // 一秒两刷
						
				}
				//sprintf(String, "%.3f Hz",DDSFre);
				//LCD_DisplayString(10, 150, 12, (u8*)String);
				
				
				
        ADC_Show();        // 显示最新的ADC采样值
        DAC_OUTPUT();      // 处理DAC输出 (当前为占位符)
				
				
				
				//-------PID测试-------//
				
				// 计算理论控制值
				/*
				float32_t error = setV - realOut;
        float32_t adjust = DSP_PID_Compute(&my_pid, error);
				// PID反馈系统仿真
				DSP_PID_SIMU(adjust,&realOut,2);// 1 or 2
				
				// PID控制打印与绘制 
				
				sprintf(String, "ADJ=%.3f ,y=%.3f,setV=%.1f",adjust,realOut,setV);
				LCD_DisplayString(10, 150, 12, (u8*)String);
				DSP_PID_DRAW(setV,realOut);*/

				//---------------------//
				
				
			
				//LCD_TOUCH_TEST(); //虚拟触屏按键测试
				
        // 循环延时以控制刷新率
          delay_ms(33);      // 大约 30 帧/秒
        // delay_ms(250);  // 大约 4 帧/秒
        // delay_ms(1000); // 大约 1 帧/秒
    }
}