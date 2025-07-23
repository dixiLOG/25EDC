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
#include "dac7612.h"		// 程控比例放大

/*----------------- 全局变量 -----------------*/
// 用于格式化字符串并显示在LCD上的缓冲区
char String1[100];
char String2[100];
char String3[100];

// Flash 存储器测试参数
#define ByteCount 2     // Flash读写测试的字节数
#define Address   0x0000  // Flash测试的起始地址


//---------LMS参数-------//
#define VREF 3.3f
#define ADC_MAX_VAL 4095.0f
#define TEST_LENGTH_SAMPLES 4096
#define NUM_TAPS 128
float32_t mu = 0.000002f;					  // 学习率
//float32_t testInput[TEST_LENGTH_SAMPLES];   // d[n]：目标 + 干扰
//float32_t refInput[TEST_LENGTH_SAMPLES];    // x[n]：干扰信号
//float32_t lmsOutput[TEST_LENGTH_SAMPLES];   // y[n]：LMS输出
//float32_t errorOutput[TEST_LENGTH_SAMPLES]; // e[n] = d[n] - y[n]：滤波结果
//float32_t fir_coeffs[NUM_TAPS];             // 自适应权重系数
//float32_t lms_state[TEST_LENGTH_SAMPLES + NUM_TAPS - 1]; // buffer，必须外部定义，生命周期够长
typedef struct {
    union {
        struct {
            float32_t lms_refInput[TEST_LENGTH_SAMPLES];
            float32_t lms_testInput[TEST_LENGTH_SAMPLES];
            float32_t lms_lmsOutput[TEST_LENGTH_SAMPLES];
        };
        float32_t fft_buffer[TEST_LENGTH_SAMPLES * 2];
    };
    
    float32_t error_buf[TEST_LENGTH_SAMPLES];
    float32_t fir_coeffs[NUM_TAPS];
    float32_t lms_state[NUM_TAPS + TEST_LENGTH_SAMPLES - 1];
} dsp_workspace_t;

dsp_workspace_t dsp_workspace;

arm_lms_instance_f32 S;


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

    // ADC3 通道0 的值 (来自DMA缓冲区)
    LCD_DisplayString_color(10, 30, 16, (u8*)"ADC3_WAVEC_Value: ", RED, WHITE);
    sprintf(String, "%.2f V", ADC3_Data_Rx[0] * 3.29 / 4095);
    LCD_DisplayString(150, 30, 16, (u8*)String);

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
    } else if (keydown_data == KEY2_DATA) { // KEY2 被按下
        LED2 = !LED2; // 翻转LED2状态
    } else if (keydown_data == KEY3_DATA) { // KEY3 被按下
        LED3 = !LED3; // 翻转LED3状态
    } else if (keydown_data == KEY4_DATA) { // KEY4 被按下
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
    //DAC_Data_Tx = 1.5/3.3*4095; // 设置DAC输出电压为1.5V
    //DAC_SetChannel1Data(DAC_Align_12b_R, DAC_Data_Tx);
    //
    // LCD_DisplayString_color(10,70,16,(u8*)"DAC_Value_Set: ",RED,WHITE);
    // sprintf(String,"%.2f V",DAC_GetDataOutputValue(DAC_Channel_1)*3.3/4095);
    // LCD_DisplayString(30,90,24,(u8*)String);
}

/**
 * @brief  LMS信号滤波
 * @note   从ADC转换缓冲区 (`ADC1_Data_Rx`, `ADC3_Data_Rx`) 读取值，
 * 计算 LMS 信号滤波结果，并适当缩放
 */
void ADC_TRANS_CAL(){
		u8 ITER_NUM = 4;
		// 将 ADC1 的 u16 数据转换为 f32 电压值，存入 testInput
		// 将 ADC3 的 u16 数据转换为 f32 电压值，存入 refInput
		for (int i = 0; i < TEST_LENGTH_SAMPLES; i++) {
				dsp_workspace.lms_testInput[i] = (float32_t)ADC1_Data_Rx[i] * VREF / ADC_MAX_VAL;
				dsp_workspace.lms_refInput[i] = (float32_t)ADC3_Data_Rx[i] * VREF / ADC_MAX_VAL;
		}		
		// 去直流分量，已集成
        DSP_LMS_FILTER(dsp_workspace.lms_refInput, dsp_workspace.lms_testInput, 
                                        dsp_workspace.lms_lmsOutput, dsp_workspace.error_buf, 
                                        TEST_LENGTH_SAMPLES,dsp_workspace.fir_coeffs, NUM_TAPS, 
                                        mu,dsp_workspace.lms_state,&S);
        // 缩放
        DSP_SCALE(dsp_workspace.error_buf,3.2,dsp_workspace.error_buf,TEST_LENGTH_SAMPLES);
}



/**
 * @brief  主程序入口
 * @retval int
0. */
int main(void) {
		
    // 系统和外设初始化
    delay_init();
    uart4_init(115200);   // 初始化UART4用于串行通信
    SPI2_Init();
    KEY_Init();           // 初始化硬件按键
    AD9833_Init();        // 初始化AD9833 DDS信号发生器
    LED_Init();           // 初始化LED
    W25QXX_Init();        // 初始化W25QXX SPI Flash
    ADC1_Init();          // 初始化ADC1
    ADC3_Init();          // 初始化ADC3
    TIM2_Init(2, 164);     // 初始化TIM2用于触发ADC采样(256KHz采样率)
    TIM4_Init(10, 21000); // 初始化TIM4用于通用定时
    DAC1_Init();          // 初始化DAC1
    LCD_Init();           // 通过FSMC初始化LCD
    TIM3_PWM_Init(999, 83, 0.4); // 初始化TIM3 PWM。频率=84M/(83+1)/(999+1)=1kHz。占空比=40%
    DAC7612_Init();       // 初始化DAC7612 程控放大器
    
    // LMS 系数初始化
    memset(dsp_workspace.fir_coeffs, 0, sizeof(dsp_workspace.fir_coeffs));
    arm_lms_init_f32(&S, NUM_TAPS, dsp_workspace.fir_coeffs, dsp_workspace.lms_state, mu, TEST_LENGTH_SAMPLES);
    // 输出参数初始化
    float32_t maxid_ave,maxAmp_ave;
    float32_t maxAmp[2],maxid[2];
    u16 cnt = 0; 

    while (1) {
        //R_Touch_test(); // 运行触摸屏测试;测试时后面功能失效
        buttom_function(); // 检查按键
		

        /*================== 信号识别核心处理部分====================*/
        if (DMA_FLAG[0]==1 && DMA_FLAG[1]==1) {
            // 清标志位
            DMA_FLAG[0] = 0;DMA_FLAG[1] = 0;
            // 计算 LMS 信号滤波结果
            ADC_TRANS_CAL();
        }
        //计算频点
        maxid[cnt] = DSP_FFT(dsp_workspace.error_buf,dsp_workspace.fft_buffer);
        //计算幅值
        maxAmp[cnt] = find_max_value_500(dsp_workspace.error_buf);
        /*==================================================*/
        // 计数器累加
        cnt++;

        sprintf(String, "mu = %.10f",mu);
        LCD_DisplayString(10, 100, 12, (u8*)String);

        // 计算平均+补偿幅值
        if(cnt == 2){
            // 求均值
            DSP_MEAN(maxid,2,&maxid_ave);
            DSP_MEAN(maxAmp,2,&maxAmp_ave);
            float amp_comp = estimate_theoretical_amp(maxid_ave, maxAmp_ave/1000.0f);  // 幅值补偿
            // 将补偿后的幅值转换为DAC7612的输入值
            u16 cntForAmp = Amp2Num(amp_comp*1000.0f);

            // 显示结果
            sprintf(String, "maxid_ave  = %.3f KHz",maxid_ave);
            LCD_DisplayString(10, 120, 12, (u8*)String);
            sprintf(String, "maxAmp_ave = %.3f mVp",maxAmp_ave);
            LCD_DisplayString(10, 135, 12, (u8*)String);
            sprintf(String, "amp_comp = %.3f mVp",amp_comp*1000.0f);
            LCD_DisplayString(10, 150, 12, (u8*)String);
            
            // 信号输出
            DAC7612_Write_CHA(cntForAmp);// 本电路下，320→880mVpp(峰峰值)...505→2000mVpp(峰值)，大致线性
            AD9833_WaveOut(SIN_WAVE,maxid_ave*1000.0f,0,1);
        }
        // 计数器循环
        cnt = cnt %	2;			


        // 提取一段波形用于绘图（共230点，为LCD屏幕宽度像素点）
        for (int i = 0; i < Y_BUF_SIZE; i++) {
                y_in_buf[i] = dsp_workspace.lms_testInput[i+300];
                y_out_buf[i] =  dsp_workspace.error_buf[i+300];
        }
			
		// 调用绘图函数
        DSP_LMS_DRAW(y_in_buf, y_out_buf, 0.0f, 4.0f);  // y轴中心0，±2范围
				
        ADC_Show();        // 显示最新的ADC采样值
				
    }
}