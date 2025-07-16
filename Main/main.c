/*********************************************************************************
 **********************************************************************************
 * @file      main.c
 * @brief     �������ļ�
 * @date      2024-07-11
 * @modify    2025-07-16
 * @note
 * ϵͳʱ������:
 * - HSE: 8MHz (�ⲿ����ʱ��)
 * - SYSCLK: 168MHz (ϵͳʱ��)
 * - HCLK: 168MHz (AHB����ʱ��, SYSCLK / 1)
 * - PCLK1: 42MHz (APB1����ʱ��, HCLK / 4)
 * - PCLK2: 84MHz (APB2����ʱ��, HCLK / 2)
 *
 * // ��ȡ����ӡʱ��Ƶ�ʵ�ʾ������:
 * //   RCC_ClocksTypeDef rcc;
 * //   RCC_GetClocksFreq(&rcc);
 * //   printf("HCLK Ƶ��: %d\r\n", rcc.HCLK_Frequency);
 *
 **********************************************************************************
 **********************************************************************************/

/*----------------- ͷ�ļ����� -----------------*/
#include "adc.h"        // ADC ���� (ADC1˫ͨ��, ʹ��TIM2��DMA2)
#include "dac.h"        // DAC ���� (DAC1, ʹ��TIM6��DMA1)
#include "lcd.h"        // LCD ���� (ʹ��FSMC)
#include "uart.h"       // UART �������� (USART1/UART4)
#include "fft.h"        // FFT (���ٸ���Ҷ�任) ������
#include "spi.h"        // SPI ���� (SPI2/SPI3)
#include "i2c.h"        // I2C ���� (I2C3)
#include "w25qxx.h"     // W25QXX SPI Flash �洢������
#include "key.h"        // ��������
#include "led.h"        // LED ����
#include "math.h"       // ��׼��ѧ��
#include "AD9833.h"     // AD9833 DDS �źŷ���������
#include "pid.h"        // PID �������߼�
#include "touch.h"      // ����������
#include "xpt2046.h"    // XPT2046 ���败����������
#include "pwm.h"        // PWM (�����ȵ���) �������
#include "stm32f4xx.h"  // STM32F4xx �豸ͷ�ļ�
#include "freqmeas.h"   // Ƶ�ʲ�������
#include <stdbool.h>    // ��׼��������
#include "convolution.h"// ������㺯��
#include "arm_cmsis_dsp.h" // ARM CMSIS-DSP ��
#include "arm_math.h"   // ARM DSP ��ѧ����
#include "wavetest.h"   // ���η�������ͷ�ļ�

/*----------------- ȫ�ֱ��� -----------------*/
// ���ڸ�ʽ���ַ�������ʾ��LCD�ϵĻ�����
char String1[100];
char String2[100];
char String3[100];

// Flash �洢�����Բ���
#define ByteCount 2     // Flash��д���Ե��ֽ���
#define Address   0x0000  // Flash���Ե���ʼ��ַ

// ���ڴ洢FFT��ֵƵ�ʷ����ı���
float fc_max_data = 0;
float fc_max_data_n = 0;
float fc_secmax_data = 0;
float fc_secmax_data_n = 0;
float fc_thirmax_data = 0;
float fc_thirmax_data_n = 0;
float fc_forthmax_data = 0;
float fc_forthmax_data_n = 0;

// ���ź����ɻ������صı���
u8 phase1 = 0;
u8 phase2 = 0;
u8 wavetype[2];
float DDSFre = 0;

// ����UARTͨ�ŵı��� (������;���������в���ȫ��ȷ)
u8 *fc_sin_uart;
u8 *fc_tri_uart;
u8 fc_tri = 0;
u8 fc_sin = 0;
u8 addr = 0;

// �������ڽ��ղ��η�������ı���
WaveResult detected_waves[MAX_WAVES];

// ��ʾ�õ�ƫ��ֵ�����������źŵ���
double offset_demo = 0.227;

// ����PID���Ż�����Ŀ�ĵļ�����
int kp_cnt = 0;
int kd_cnt = 0;

//-------PID���Բ���-------//
float32_t  setV = 1200;//code 
float32_t  PID_tol = 0.5;
float32_t  Kp = 3;
float32_t  Ki = 0.8;
float32_t  Kd = 0.08;
float32_t  UPLIMIT = 5000;
float32_t  DOWNLIMIT = 0;
float32_t realOut = 1200; 	// ���ض����ʼֵ | �������������ֵ


float32_t error, adjust;    // �����PID������ֵ
DSP_PID_Controller my_pid;	// �ṹ��

//======================


//---------LMS����-------//
		
#define TEST_LENGTH_SAMPLES 1024
#define NUM_TAPS 64
float32_t mu = 0.005f;											// ѧϰ��
float32_t testInput[TEST_LENGTH_SAMPLES];   // d[n]��Ŀ�� + ����
float32_t refInput[TEST_LENGTH_SAMPLES];    // x[n]�������ź�
float32_t lmsOutput[TEST_LENGTH_SAMPLES];   // y[n]��LMS���
float32_t errorOutput[TEST_LENGTH_SAMPLES]; // e[n] = d[n] - y[n]���˲����
float32_t fir_coeffs[NUM_TAPS];             // ����ӦȨ��ϵ��
float32_t lms_state[TEST_LENGTH_SAMPLES + NUM_TAPS - 1]; // buffer�������ⲿ���壬�������ڹ���

// plot ����

#define Y_BUF_SIZE 230  // LCD X����ʾ 230 ����
float32_t y_in_buf[Y_BUF_SIZE];   // ԭʼ�źŶ�
float32_t y_out_buf[Y_BUF_SIZE];  // �˲����źŶ�
//======================

/**
 * @brief  ��һ��Ƶ��ֵ�������뵽��ӽ���5�ı�����
 * @param  data �����Ƶ��ֵ��
 * @retval  ����������Ƶ��ֵ��
 */
float calculate_fc(float data) {
    int rounded_fc = 0;
    rounded_fc = (int)((data + 2.5) / 5) * 5;
    return rounded_fc;
}

/**
 * @brief  ��������������֮��ľ��Բ
 * @param  a ��һ������
 * @param  b �ڶ�������
 * @retval ���Բ� |a - b|��
 */
float abs_sub(float a, float b) {
    return (a > b) ? (a - b) : (b - a);
}


/**
 * @brief  ����������д��W25QXX Flash�洢����
 * @note   �˺����� `pBuffer` ������д��ָ��������
 * Ȼ����LCD����ʾд���ֵ��Ϊ��һ��д��������������������ֵ��
 * `pBuffer` �ٶ�Ϊ�������ļ��ж����ȫ�ֱ�����
 */
void write_flash() {
    // �� pBuffer ������д��Flash��ָ����ַ
    // extern u8 pBuffer[]; // ���� pBuffer �ڱ𴦶���
    W25QXX_SectorWrite((u8*)pBuffer, Address, Byte_Count);
    
    // ��ʽ������LCD����ʾд�������
    sprintf(String, "flash_Write:%d %d", pBuffer[0], pBuffer[1]);
    pBuffer[0]++;
    pBuffer[1]++;
    LCD_Fill_onecolor(5, 120, 235, 140, WHITE);
    LCD_DisplayString_color(10, 120, 16, (u8*)String, BLUE, WHITE);
}


/**
 * @brief  ��W25QXX Flash�洢���ж�ȡ�������ݡ�
 * @note   ��ָ����ַ��ȡ���ݵ� `DataBuffer`������LCD����ʾ��ȡ��ֵ��
 * `DataBuffer` �ٶ�Ϊȫ�ֱ�����
 */
void read_flash() {
    // ��Flash��ȡ���ݵ� DataBuffer
    // extern u8 DataBuffer[]; // ���� DataBuffer �ڱ𴦶���
    W25QXX_Read((u8*)DataBuffer, Address, Byte_Count);
    
    // ��ʽ������LCD����ʾ��ȡ������
    sprintf(String, "flash_Read:%d %d", DataBuffer[0], DataBuffer[1]);
    LCD_Fill_onecolor(5, 140, 235, 160, WHITE);
    LCD_DisplayString_color(10, 140, 16, (u8*)String, BLUE, WHITE);
}


/**
 * @brief  ��LCD����ʾADCת��ֵ��
 * @note   ��ADCת�������� (`ADC1_Data_Rx`, `ADC3_Data_Rx`) ��ȡֵ��
 * ����ת��Ϊ��ѹ��Ȼ����ʾ������
 * ����ADCΪ12λ(4096��)���ο���ѹԼΪ3.17V��
 */
void ADC_Show() {
    // ADC1 ͨ��0 ��ֵ (����DMA������)
    LCD_DisplayString_color(10, 10, 16, (u8*)"ADC1_WAVEC_Value: ", RED, WHITE);
    sprintf(String, "%.2f V", ADC1_Data_Rx[0] * 3.29 / 4095);
    LCD_DisplayString(150, 10, 16, (u8*)String);

    // ADC3 ͨ��0 & 1 ��ֵ (����DMA������)
    LCD_DisplayString_color(10, 30, 16, (u8*)"ADC3_Channel_0&1_Value: ", RED, WHITE);
    sprintf(String, "%.2f V", ADC3_Data_Rx[20] * 3.29 / 4095);
    LCD_DisplayString(30, 48, 16, (u8*)String);
    sprintf(String, "%.2f V", ADC3_Data_Rx[21] * 3.29 / 4095);
    LCD_DisplayString(120, 48, 16, (u8*)String);
}


/**
 * @brief  �����������¼���
 * @note   ɨ�谴��״̬��ִ����Ӧ�Ĳ�����
 */
void buttom_function() {
    // XPT2046_Scan(0); // ��ѡ��ɨ�败����
    key_scan(1); // ɨ��Ӳ��������1Ϊ����

    if (keydown_data == KEY1_DATA) { // KEY1 ������
        printf("KEY1 Pressed~\n"); // ͨ���������������Ϣ
        LED3 = !LED3; // ��תLED3״̬
				
				//AD9833_WaveOut(wavetype[0],detected_waves[0].base_freq*1000,0,2);
				//DDSFre = detected_waves[0].base_freq*1000;
			
			
				//-------PID����-------//
				setV += 50;
				//--------------
			
    } else if (keydown_data == KEY2_DATA) { // KEY2 ������
        LED2 = !LED2; // ��תLED2״̬
        
				/*float f = freqMeasurement(); // ִ��һ��Ƶ�ʲ���
        sprintf(String, "freqMeasurement:%.6f Hz", f);
        LCD_Fill_onecolor(5, 160, 235, 180, WHITE);
        LCD_DisplayString_color(10, 160, 12, (u8*)String, BLUE, WHITE);
				
				DDSFre += 1;
				*/
			//-------PID����-------//
				setV -= 50;
			//--------------
			
    } else if (keydown_data == KEY3_DATA) { // KEY3 ������
        LED3 = !LED3; // ��תLED3״̬
				
				//DDSFre -= 1;
			
    } else if (keydown_data == KEY4_DATA) { // KEY4 ������
				AD9833_WaveOut(wavetype[0],DDSFre,0,2);
        write_flash(); // д�����ݵ�Flash
    } else if (keydown_data == KEY5_DATA) { // KEY5 ������
        read_flash(); // ��Flash��ȡ����
    }
}


/**
 * @brief  ����DAC�������ʾ��
 * @note   ��ǰ���˺�����һ��ռλ������ע�͵��Ĵ���չʾ�����
 * ���ò���ʾDAC�������ѹ��
 */
void DAC_OUTPUT() {
    // ���´��뱻���ã���չʾ��ʾ���÷���
    DAC_Data_Tx = 1.5/3.3*4095; // ����DAC�����ѹΪ1.5V
    DAC_SetChannel1Data(DAC_Align_12b_R, DAC_Data_Tx);
    //
    // LCD_DisplayString_color(10,70,16,(u8*)"DAC_Value_Set: ",RED,WHITE);
    // sprintf(String,"%.2f V",DAC_GetDataOutputValue(DAC_Channel_1)*3.3/4095);
    // LCD_DisplayString(30,90,24,(u8*)String);
}


/**
 * @brief  ִ��FFT���㲢��LCD����ʾƵ�ס�
 * @note   �˺�����DMA��ɱ�־������������FFT��������
 * Ȼ����LCD��Ļ�ϻ������ɵĵ��߷���Ƶ�ס�
 */
void FFT_SHOW() {
    // ���ADC������DMA�����Ƿ����
    if (DMA_FLAG) {
        DMA_FLAG = 0; // �����־λ���ȴ���һ�δ���
        DSP_FFT();    // ���µĲ�������ִ��FFT

        // ��ʾFFT����
        sprintf(String1, "fs=%d kHz", Fs);
        LCD_DisplayString_color(150, 175, 12, (u8*)String1, GREEN, WHITE);
        sprintf(String, "N=%d", FFT_N);
        LCD_DisplayString_color(100, 175, 12, (u8*)String, GREEN, WHITE);

        // ����Ƶ��ͼ����
        LCD_Fill_onecolor(5, 190, 235, 315, WHITE);
        LCD_Color_DrawRectangle(5, 188, 235, 315, BLACK); // ���Ʊ߿�

        // ����X��Ƶ�ʿ̶�
        for (u16 x = 0; x <= 230; x++) {
            if (x % 10 == 0) {
                LCD_Color_DrawLine(5 + x, 315, 5 + x, 320, RED); // ���̶���
            } else if (x % 2 == 0) {
                LCD_Color_DrawLine(5 + x, 315, 5 + x, 317, BLACK); // �ο̶���
            }
        }
        
        // �����������һ���Ա���ʾ (���ŵ�100���ظ�)
        for (u16 i = 0; i < (FFT_N >> 1); i++) {
            Mag_Single[i] = Mag_Single[i] * 100 / Mag_max;
        }

        // ���Ƶ���Ƶ��ͼ
        for (u16 x = 0; x <= 230; x++) {
            LCD_Color_DrawLine(5 + x, 315 - Mag_Single[x], 5 + x, 315, BLUE);
        }
    }
}


/**
 * @brief  ���������
 * @retval int
 */
int main(void) {
    // ϵͳ�������ʼ��
    delay_init();
    uart4_init(115200);   // ��ʼ��UART4���ڴ���ͨ��
    // SPI2_Init();
    KEY_Init();           // ��ʼ��Ӳ������
    AD9833_Init();        // ��ʼ��AD9833 DDS�źŷ�����
    LED_Init();           // ��ʼ��LED
    W25QXX_Init();        // ��ʼ��W25QXX SPI Flash
    // I2C3_Init();
    ADC1_Init();          // ��ʼ��ADC1
    ADC3_Init();          // ��ʼ��ADC3
    TIM2_Init(2, 41);     // ��ʼ��TIM2���ڴ���ADC����
    TIM4_Init(10, 21000); // ��ʼ��TIM4����ͨ�ö�ʱ
    DAC1_Init();          // ��ʼ��DAC1
    LCD_Init();           // ͨ��FSMC��ʼ��LCD
    TIM3_PWM_Init(999, 83, 0.4); // ��ʼ��TIM3 PWM��Ƶ��=84M/(83+1)/(999+1)=1kHz��ռ�ձ�=40%
		Touch_Init();					// ��������ʼ��;320*250
	
		// DDS����
		// AD9833_WaveOut(SIN_WAVE,25000,0,2);

		// PID��ʼ��
		DSP_PID_Init(&my_pid, Kp, Ki, Kd,PID_tol ,UPLIMIT , DOWNLIMIT);
		
		// LMS ϵ����ʼ��
		memset(fir_coeffs, 0, sizeof(fir_coeffs));

		
    while (1) {
        //R_Touch_test(); // ���д���������;����ʱ���湦��ʧЧ
        buttom_function(); // ��鰴��
				
        if (DMA_FLAG) {
            // ADC/DMA���ڽ���������˴���顣
            // ��ʱ����� FFT_SHOW() �ܿ����Ѿ�ִ�й��ˡ�
//						FFT_SHOW();        // ��������׼����ʱ�����㲢��ʾFFTƵ��
//						wave_testv2();// ���µ�FFT����ִ�в��η���
//						printf("%f  %f %d\n",detected_waves[0].amplitude,detected_waves[0].base_freq,wavetype[0]);
//						delay_ms(500); // һ����ˢ
						
				}
				//sprintf(String, "%.3f Hz",DDSFre);
				//LCD_DisplayString(10, 150, 12, (u8*)String);
				
				
				
        ADC_Show();        // ��ʾ���µ�ADC����ֵ
        DAC_OUTPUT();      // ����DAC��� (��ǰΪռλ��)
				
				
				
				//-------PID����-------//
				
				// �������ۿ���ֵ
				/*
				float32_t error = setV - realOut;
        float32_t adjust = DSP_PID_Compute(&my_pid, error);
				// PID����ϵͳ����
				DSP_PID_SIMU(adjust,&realOut,2);// 1 or 2
				
				// PID���ƴ�ӡ����� 
				
				sprintf(String, "ADJ=%.3f ,y=%.3f,setV=%.1f",adjust,realOut,setV);
				LCD_DisplayString(10, 150, 12, (u8*)String);
				DSP_PID_DRAW(setV,realOut);*/

				//---------------------//
				
				
			
				//LCD_TOUCH_TEST(); //���ⴥ����������
				
        // ѭ����ʱ�Կ���ˢ����
          delay_ms(33);      // ��Լ 30 ֡/��
        // delay_ms(250);  // ��Լ 4 ֡/��
        // delay_ms(1000); // ��Լ 1 ֡/��
    }
}