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
#include "filter_analyzer.h" 	//滤波器学习
#include "learning_mode.h"


/*----------------- 全局变量 -----------------*/
// 用于格式化字符串并显示在LCD上的缓冲区
char String1[100];
char String2[100];
char String3[100];

// Flash 存储器测试参数
#define ByteCount 2     // Flash读写测试的字节数
#define Address   0x0000  // Flash测试的起始地址


/* ================================================================== */
/* =================== 软件CRC32计算核心代码 ====================== */
/* ================================================================== */

// CRC32查表法
const uint32_t crc32_table[256] = {
    0x00000000, 0x77073096, 0xEE0E612C, 0x990951BA, 0x076DC419, 0x706AF48F, 0xE963A535, 0x9E6495A3,
    0x0EDB8832, 0x79DCB8A4, 0xE0D5E91E, 0x97D2D988, 0x09B64C2B, 0x7EB17CBD, 0xE7B82D07, 0x90BF1D91,
    0x1DB71064, 0x6AB020F2, 0xF3B97148, 0x84BE41DE, 0x1ADAD47D, 0x6DDDE4EB, 0xF4D4B551, 0x83D385C7,
    0x136C9856, 0x646BA8C0, 0xFD62F97A, 0x8A65C9EC, 0x14015C4F, 0x63066CD9, 0xFA0F3D63, 0x8D080DF5,
    0x3B6E20C8, 0x4C69105E, 0xD56041E4, 0xA2677172, 0x3C03E4D1, 0x4B04D447, 0xD20D85FD, 0xA50AB56B,
    0x35B5A8FA, 0x42B2986C, 0xDBBBC9D6, 0xACBCF940, 0x32D86CE3, 0x45DF5C75, 0xDCD60DCF, 0xABD13D59,
    0x26D930AC, 0x51DE003A, 0xC8D75180, 0xBFD06116, 0x21B4F4B5, 0x56B3C423, 0xCFBA9599, 0xB8BDA50F,
    0x2802B89E, 0x5F058808, 0xC60CD9B2, 0xB10BE924, 0x2F6F7C87, 0x58684C11, 0xC1611DAB, 0xB6662D3D,
    0x76DC4190, 0x01DB7106, 0x98D220BC, 0xEFD5102A, 0x71B18589, 0x06B6B51F, 0x9FBFE4A5, 0xE8B8D433,
    0x7807C9A2, 0x0F00F934, 0x9609A88E, 0xE10E9818, 0x7F6A0DBB, 0x086D3D2D, 0x91646C97, 0xE6635C01,
    0x6B6B51F4, 0x1C6C6162, 0x856530D8, 0xF262004E, 0x6C0695ED, 0x1B01A57B, 0x8208F4C1, 0xF50FC457,
    0x65B0D9C6, 0x12B7E950, 0x8BBEB8EA, 0xFCB9887C, 0x62DD1DDF, 0x15DA2D49, 0x8CD37CF3, 0xFBD44C65,
    0x4DB26158, 0x3AB551CE, 0xA3BC0074, 0xD4BB30E2, 0x4ADFA541, 0x3DD895D7, 0xA4D1C46D, 0xD3D6F4FB,
    0x4369E96A, 0x346ED9FC, 0xAD678846, 0xDA60B8D0, 0x44042D73, 0x33031DE5, 0xAA0A4C5F, 0xDD0D7CC9,
    0x5005713C, 0x270241AA, 0xBE0B1010, 0xC90C2086, 0x5768B525, 0x206F85B3, 0xB966D409, 0xCE61E49F,
    0x5EDEF90E, 0x29D9C998, 0xB0D09822, 0xC7D7A8B4, 0x59B33D17, 0x2EB40D81, 0xB7BD5C3B, 0xC0BA6CAD,
    0xEDB88320, 0x9ABFB3B6, 0x03B6E20C, 0x74B1D29A, 0xEAD54739, 0x9DD277AF, 0x04DB2615, 0x73DC1683,
    0xE3630B12, 0x94643B84, 0x0D6D6A3E, 0x7A6A5AA8, 0xE40ECF0B, 0x9309FF9D, 0x0A00AE27, 0x7D079EB1,
    0xF00F9344, 0x8708A3D2, 0x1E01F268, 0x6906C2FE, 0xF762575D, 0x806567CB, 0x196C3671, 0x6E6B06E7,
    0xFED41B76, 0x89D32BE0, 0x10DA7A5A, 0x67DD4ACC, 0xF9B9DF6F, 0x8EBEEFF9, 0x17B7BE43, 0x60B08ED5,
    0xD6D6A3E8, 0xA1D1937E, 0x38D8C2C4, 0x4FDFF252, 0xD1BB67F1, 0xA6BC5767, 0x3FB506DD, 0x48B2364B,
    0xD80D2BDA, 0xAF0A1B4C, 0x36034AF6, 0x41047A60, 0xDF60EFC3, 0xA867DF55, 0x316E8EEF, 0x4669BE79,
    0xCB61B38C, 0xBC66831A, 0x256FD2A0, 0x5268E236, 0xCC0C7795, 0xBB0B4703, 0x220216B9, 0x5505262F,
    0xC5BA3BBE, 0xB2BD0B28, 0x2BB45A92, 0x5CB36A04, 0xC2D7FFA7, 0xB5D0CF31, 0x2CD99E8B, 0x5BDEAE1D,
    0x9B64C2B0, 0xEC63F226, 0x756AA39C, 0x026D930A, 0x9C0906A9, 0xEB0E363F, 0x72076785, 0x05005713,
    0x95BF4A82, 0xE2B87A14, 0x7BB12BAE, 0x0CB61B38, 0x92D28E9B, 0xE5D5BE0D, 0x7CDCEFB7, 0x0BDBDF21,
    0x86D3D2D4, 0xF1D4E242, 0x68DDB3F8, 0x1FDA836E, 0x81BE16CD, 0xF6B9265B, 0x6FB077E1, 0x18B74777,
    0x88085AE6, 0xFF0F6A70, 0x66063BCA, 0x11010B5C, 0x8F659EFF, 0xF862AE69, 0x616BFFD1, 0x166CCF47,
    0xA00AE278, 0xD70DD2EE, 0x4E048354, 0x3903B3C2, 0xA7672661, 0xD06016F7, 0x4969474D, 0x3E6E77DB,
    0xAED16A4A, 0xD9D65ADC, 0x40DF0B66, 0x37D83BF0, 0xA9BCAE53, 0xDEBB9EC5, 0x47B2CF7F, 0x30B5FFE9,
    0xBDBDF21C, 0xCABAC28A, 0x53B39330, 0x24B4A3A6, 0xBAD03605, 0xCDD70693, 0x54DE5729, 0x23D967BF,
    0xB3667A2E, 0xC4614AB8, 0x5D681B02, 0x2A6F2B94, 0xB40BBE37, 0xC30C8EA1, 0x5A05DF1B, 0x2D02EF8D
};

/**
 * @brief  使用查表法计算一段数据的CRC32值
 * @param  data:   指向数据缓冲区的指针
 * @param  length: 数据长度 (字节)
 * @return 32位的CRC值
 */
uint32_t sw_crc32(const uint8_t *data, size_t length) {
    uint32_t crc = 0xFFFFFFFF;
    while (length--) {
        crc = crc32_table[(crc ^ *data++) & 0xFF] ^ (crc >> 8);
    }
    return crc ^ 0xFFFFFFFF;
}

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
// --- 1. 设计参数 (您可以直接在这里修改) ---
//float Fc = 10000.0f;   // 截止频率
//float Q  = 1.0f;       // 品质因数
////================================================================
//                 IIR 滤波器 - 硬编码系数
// 设计参数: Fs = 823.6kHz, 截止频率 Fc = 10kHz, Q = 1.0
//================================================================
//const float b0 = 0.003539f;
//const float b1 = 0.007078f;
//const float b2 = 0.003539f;
//const float _a1_ = -1.870603f;
//const float _a2_ = 0.884759f;

// 滤波器历史状态 (必须为static或全局)
static float x_prev1 = 0.0f; // x[n-1]
static float x_prev2 = 0.0f; // x[n-2]
static float y_prev1 = 0.0f; // y[n-1]
static float y_prev2 = 0.0f; // y[n-2]

/**
 * @brief 重置IIR滤波器的历史状态变量为0
 */
void reset_filter_state(void)
{
    x_prev1 = 0.0f;
    x_prev2 = 0.0f;
    y_prev1 = 0.0f;
    y_prev2 = 0.0f;
}

// plot 参数

#define Y_BUF_SIZE 230  // LCD X轴显示 230 个点
float32_t y_in_buf[Y_BUF_SIZE];   // 原始信号段
float32_t y_out_buf[Y_BUF_SIZE];  // 滤波后信号段
//======================




/* ================================================================== */
/* =================== 参数断电保存核心代码 ======================= */
/* ================================================================== */

// 定义在W25QXX芯片中存储数据的起始地址
#define PERSISTENT_DATA_ADDR   0x00000000

// 定义我们自己的参数结构体 (与之前的回答一致)
typedef struct {
    FilterType     filter_type;
    FilterParams   analyzed_params;
    DigitalCoeffs  iir_coeffs;
    uint32_t       crc32;
} PersistentData_t;


/**
 * @brief  【新】将参数结构体保存到W25QXX Flash。
 * @param  p_data_to_save: 指向包含要保存参数的结构体指针。
 * @return 0: 成功, -1: 失败
 */
int32_t save_persistent_data(const PersistentData_t* p_data_to_save)
{
    // W25QXX_SectorWrite 会自动处理擦除，我们直接写入即可。
    // 注意：函数原型是 (u8* pBuffer, u32 WriteAddr, u16 NumByteToWrite)
    // 请确保 Byte_Count 的类型匹配，这里我们用 sizeof()
    W25QXX_SectorWrite((u8*)p_data_to_save, PERSISTENT_DATA_ADDR, sizeof(PersistentData_t));
    
    // 写入操作比较简单，直接返回成功
    return 0;
}


/**
 * @brief  【新】从W25QXX Flash中读取参数结构体。
 * @param  p_data_buffer: 指向用于存放读取数据的结构体指针。
 * @return 0: 成功, -1: 失败或数据无效
 */
int32_t load_persistent_data(PersistentData_t* p_data_buffer)
{
    W25QXX_Read((u8*)p_data_buffer, PERSISTENT_DATA_ADDR, sizeof(PersistentData_t));

    if (p_data_buffer->crc32 == 0xFFFFFFFF) {
        return -1;
    }

    // 调用软件CRC函数进行计算
    uint32_t calculated_crc = sw_crc32((const uint8_t*)p_data_buffer, sizeof(PersistentData_t) - sizeof(uint32_t));

    if (calculated_crc == p_data_buffer->crc32) {
        return 0;
    } else {
        return -1;
    }
}

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
								// --- 【在需要时保存参数的示例】 ---
        // 比如，当某个条件满足时（如学习完成），我们执行保存操作
      

            // 1. 准备要保存的数据
            PersistentData_t data_to_save;
            data_to_save.filter_type     = g_identified_filter_type;
            data_to_save.analyzed_params = g_analyzed_params;
            data_to_save.iir_coeffs      = g_iir_coeffs;
           

            // 调用软件CRC函数计算校验码
        data_to_save.crc32 = sw_crc32((const uint8_t*)&data_to_save, sizeof(PersistentData_t) - sizeof(uint32_t));

        if (save_persistent_data(&data_to_save) == 0)
        {
            printf("参数已成功保存到W25QXX!\r\n");
        }
        
			
    } else if (keydown_data == KEY2_DATA) { // KEY2 被按下
        LED2 = !LED2; // 翻转LED2状态
    } else if (keydown_data == KEY3_DATA) { // KEY3 被按下
      
			LED3 = !LED3; // 翻转LED3状态
    } else if (keydown_data == KEY4_DATA) { // KEY4 被按下
        //write_flash(); // 写入数据到Flash
    } else if (keydown_data == KEY5_DATA) { // KEY5 被按下
        // read_flash(); // 从Flash读取数据
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


/*===超参数设置===*/
// touch.h调整定义
// PARAM_COUNT 4
// Parameter g_params[PARAM_COUNT];
// 0：设定频率
// 1：设定幅值
// 2：幅值偏置补偿
// touch.c→ui_init_params()
// 1370对应AD603过跟随输出峰峰值2Vpp

/*===硬件键盘设置===*/
// touch.c→ui_handle_hardware_keys()
// KEY5—KEY1配置：
// HOME键
// DDS 输出
// 学习模式
// 模拟启动
// 预留键
// 暂时无操作



/*==扫频说明==*/
/*
.h文件配置

// --- 扫频参数 ---
#define SWEEP_FREQ_START_HZ     100.0f
#define SWEEP_FREQ_END_HZ       2000.0f
#define SWEEP_FREQ_STEP_HZ      100.0f
//#define SWEEP_AMP_CNT 					1370 		// 2Vpp
#define SWEEP_AMP_CNT 					1212 		// 1Vpp


// 计算扫频所需的点数
#define SWEEP_POINTS            (int)((SWEEP_FREQ_END_HZ - SWEEP_FREQ_START_HZ) / SWEEP_FREQ_STEP_HZ) + 1

// --- ADC 稳定性检查 ---
#define ADC_STABILITY_CHECKS    3       // 需要连续多少次读数稳定才算收敛
#define ADC_STABILITY_THRESHOLD 3       // ADC值相差多少以内认为稳定 (满量程4095)
#define ADC_CHECK_DELAY_MS      50      // 两次稳定性检查之间的延迟


void execute_learning_sequence(void) {
    // 1. 绘制学习过程中的静态UI和进度条
    draw_learning_ui_layout();
    
    // 2. 执行耗时的扫频和数据采集
    perform_frequency_sweep();
			取得稳定的数据：get_stable_adc_reading（get_average_adc_value）
			
    // 3. 对采集的数据进行分析
    analyze_response_data();
}

===================================




*/



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
//    TIM2_Init(2, 51);     // 初始化TIM2用于触发ADC采样(823.592KHz采样率)
		TIM2_Init(2, 80);
    TIM4_Init(10, 21000); // 初始化TIM4用于通用定时
    DAC1_Init();          // 初始化DAC1
    LCD_Init();           // 通过FSMC初始化LCD
    TIM3_PWM_Init(999, 83, 0.4); // 初始化TIM3 PWM。频率=84M/(83+1)/(999+1)=1kHz。占空比=40%
    DAC7612_Init();       // 初始化DAC7612 程控放大器
    // 设置显示为竖屏模式
    // 重要提示: 此设置必须与 touch.h 中的宏定义相匹配
    Set_Display_Mode(0); // 0: 竖屏, 1: 横屏
		// UI 初始化 (此函数会初始化触摸屏、参数等)
		Touch_Init();					// 触摸屏初始化;320*250
    ui_init();						// UI初始化
		//AD9833_WaveOut(SIN_WAVE,1000,0,1);
		
		
		 // --- 【上电加载参数】 ---
    PersistentData_t loaded_data; // 创建一个临时的结构体用于接收数据
    if (load_persistent_data(&loaded_data) == 0) 
    {
        // 加载成功，将数据更新到全局变量
        g_identified_filter_type = loaded_data.filter_type;
        g_analyzed_params        = loaded_data.analyzed_params;
        g_iir_coeffs             = loaded_data.iir_coeffs;
        
        printf("从W25QXX加载参数成功!\r\n");
    } 
    else 
    {
        // 加载失败或无有效数据，将使用默认值
        printf("W25QXX中无有效参数，使用默认值。\r\n");
    }
;
    while (1) {
        //R_Touch_test(); // 运行触摸屏测试;测试时后面功能失效
        buttom_function(); // 检查按键
				// 运行UI主循环，它会处理所有事件
        ui_run();

			} 


				
        
				
    
}

void ADC_IRQHandler(void)
{
    // 检查是否是ADC3的转换结束中断
    if(ADC_GetITStatus(ADC3, ADC_IT_EOC) != RESET)
    {
        // 必须先清除中断标志位，否则会重复进入中断
        ADC_ClearITPendingBit(ADC3, ADC_IT_EOC);

        // 1. 读取ADC原始值并归一化到电压
        float x_n = (float)ADC_GetConversionValue(ADC3);
        x_n = x_n * 3.3f / 4095.0f;

        // 2. 应用差分方程，使用全局的volatile的系数       // <<< 这里是核心修改
        float y_n = g_iir_coeffs.b[0] * x_n + g_iir_coeffs.b[1] * x_prev1 + g_iir_coeffs.b[2] * x_prev2
                    - g_iir_coeffs.a[1] * y_prev1 - g_iir_coeffs.a[2] * y_prev2;
//					float y_n =0.02861525f  * x_n +0.05723050f * x_prev1 + 0.02861525f * x_prev2
//                    + 1.29289992f * y_prev1 - 0.40736092f * y_prev2;
        // 3. 更新历史状态变量
        x_prev2 = x_prev1;
        x_prev1 = x_n;
        y_prev2 = y_prev1;
        y_prev1 = y_n;
        
        // 将计算结果从电压转换回DAC值
        y_n = y_n * 4095.0f/3.3f;
        
				
				if(g_identified_filter_type ==  FILTER_TYPE_HIGHPASS|| g_identified_filter_type == FILTER_TYPE_BANDPASS)
				{
					y_n = y_n + 1862;
	
				}
				
        // 4. 输出限幅
        if (y_n < 0.0f) {
            y_n = 0.0f;
        } else if (y_n > 4095.0f) {
            y_n = 4095.0f;
        }
        
        // 5. 将结果写入DAC
        DAC_SetChannel1Data(DAC_Align_12b_R, (uint16_t)y_n);
    }
}

