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
#include "dac7612.h"		// �̿ر����Ŵ�

/*----------------- ȫ�ֱ��� -----------------*/
// ���ڸ�ʽ���ַ�������ʾ��LCD�ϵĻ�����
char String1[100];
char String2[100];
char String3[100];

// Flash �洢�����Բ���
#define ByteCount 2     // Flash��д���Ե��ֽ���
#define Address   0x0000  // Flash���Ե���ʼ��ַ


//---------LMS����-------//
#define VREF 3.3f
#define ADC_MAX_VAL 4095.0f
#define TEST_LENGTH_SAMPLES 4096
#define NUM_TAPS 128
float32_t mu = 0.000002f;					  // ѧϰ��
//float32_t testInput[TEST_LENGTH_SAMPLES];   // d[n]��Ŀ�� + ����
//float32_t refInput[TEST_LENGTH_SAMPLES];    // x[n]�������ź�
//float32_t lmsOutput[TEST_LENGTH_SAMPLES];   // y[n]��LMS���
//float32_t errorOutput[TEST_LENGTH_SAMPLES]; // e[n] = d[n] - y[n]���˲����
//float32_t fir_coeffs[NUM_TAPS];             // ����ӦȨ��ϵ��
//float32_t lms_state[TEST_LENGTH_SAMPLES + NUM_TAPS - 1]; // buffer�������ⲿ���壬�������ڹ���
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

    // ADC3 ͨ��0 ��ֵ (����DMA������)
    LCD_DisplayString_color(10, 30, 16, (u8*)"ADC3_WAVEC_Value: ", RED, WHITE);
    sprintf(String, "%.2f V", ADC3_Data_Rx[0] * 3.29 / 4095);
    LCD_DisplayString(150, 30, 16, (u8*)String);

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
    } else if (keydown_data == KEY2_DATA) { // KEY2 ������
        LED2 = !LED2; // ��תLED2״̬
    } else if (keydown_data == KEY3_DATA) { // KEY3 ������
        LED3 = !LED3; // ��תLED3״̬
    } else if (keydown_data == KEY4_DATA) { // KEY4 ������
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
    //DAC_Data_Tx = 1.5/3.3*4095; // ����DAC�����ѹΪ1.5V
    //DAC_SetChannel1Data(DAC_Align_12b_R, DAC_Data_Tx);
    //
    // LCD_DisplayString_color(10,70,16,(u8*)"DAC_Value_Set: ",RED,WHITE);
    // sprintf(String,"%.2f V",DAC_GetDataOutputValue(DAC_Channel_1)*3.3/4095);
    // LCD_DisplayString(30,90,24,(u8*)String);
}

/**
 * @brief  LMS�ź��˲�
 * @note   ��ADCת�������� (`ADC1_Data_Rx`, `ADC3_Data_Rx`) ��ȡֵ��
 * ���� LMS �ź��˲���������ʵ�����
 */
void ADC_TRANS_CAL(){
		u8 ITER_NUM = 4;
		// �� ADC1 �� u16 ����ת��Ϊ f32 ��ѹֵ������ testInput
		// �� ADC3 �� u16 ����ת��Ϊ f32 ��ѹֵ������ refInput
		for (int i = 0; i < TEST_LENGTH_SAMPLES; i++) {
				dsp_workspace.lms_testInput[i] = (float32_t)ADC1_Data_Rx[i] * VREF / ADC_MAX_VAL;
				dsp_workspace.lms_refInput[i] = (float32_t)ADC3_Data_Rx[i] * VREF / ADC_MAX_VAL;
		}		
		// ȥֱ���������Ѽ���
        DSP_LMS_FILTER(dsp_workspace.lms_refInput, dsp_workspace.lms_testInput, 
                                        dsp_workspace.lms_lmsOutput, dsp_workspace.error_buf, 
                                        TEST_LENGTH_SAMPLES,dsp_workspace.fir_coeffs, NUM_TAPS, 
                                        mu,dsp_workspace.lms_state,&S);
        // ����
        DSP_SCALE(dsp_workspace.error_buf,3.2,dsp_workspace.error_buf,TEST_LENGTH_SAMPLES);
}



/**
 * @brief  ���������
 * @retval int
0. */
int main(void) {
		
    // ϵͳ�������ʼ��
    delay_init();
    uart4_init(115200);   // ��ʼ��UART4���ڴ���ͨ��
    SPI2_Init();
    KEY_Init();           // ��ʼ��Ӳ������
    AD9833_Init();        // ��ʼ��AD9833 DDS�źŷ�����
    LED_Init();           // ��ʼ��LED
    W25QXX_Init();        // ��ʼ��W25QXX SPI Flash
    ADC1_Init();          // ��ʼ��ADC1
    ADC3_Init();          // ��ʼ��ADC3
    TIM2_Init(2, 164);     // ��ʼ��TIM2���ڴ���ADC����(256KHz������)
    TIM4_Init(10, 21000); // ��ʼ��TIM4����ͨ�ö�ʱ
    DAC1_Init();          // ��ʼ��DAC1
    LCD_Init();           // ͨ��FSMC��ʼ��LCD
    TIM3_PWM_Init(999, 83, 0.4); // ��ʼ��TIM3 PWM��Ƶ��=84M/(83+1)/(999+1)=1kHz��ռ�ձ�=40%
    DAC7612_Init();       // ��ʼ��DAC7612 �̿طŴ���
    
    // LMS ϵ����ʼ��
    memset(dsp_workspace.fir_coeffs, 0, sizeof(dsp_workspace.fir_coeffs));
    arm_lms_init_f32(&S, NUM_TAPS, dsp_workspace.fir_coeffs, dsp_workspace.lms_state, mu, TEST_LENGTH_SAMPLES);
    // ���������ʼ��
    float32_t maxid_ave,maxAmp_ave;
    float32_t maxAmp[2],maxid[2];
    u16 cnt = 0; 

    while (1) {
        //R_Touch_test(); // ���д���������;����ʱ���湦��ʧЧ
        buttom_function(); // ��鰴��
		

        /*================== �ź�ʶ����Ĵ�����====================*/
        if (DMA_FLAG[0]==1 && DMA_FLAG[1]==1) {
            // ���־λ
            DMA_FLAG[0] = 0;DMA_FLAG[1] = 0;
            // ���� LMS �ź��˲����
            ADC_TRANS_CAL();
        }
        //����Ƶ��
        maxid[cnt] = DSP_FFT(dsp_workspace.error_buf,dsp_workspace.fft_buffer);
        //�����ֵ
        maxAmp[cnt] = find_max_value_500(dsp_workspace.error_buf);
        /*==================================================*/
        // �������ۼ�
        cnt++;

        sprintf(String, "mu = %.10f",mu);
        LCD_DisplayString(10, 100, 12, (u8*)String);

        // ����ƽ��+������ֵ
        if(cnt == 2){
            // ���ֵ
            DSP_MEAN(maxid,2,&maxid_ave);
            DSP_MEAN(maxAmp,2,&maxAmp_ave);
            float amp_comp = estimate_theoretical_amp(maxid_ave, maxAmp_ave/1000.0f);  // ��ֵ����
            // ��������ķ�ֵת��ΪDAC7612������ֵ
            u16 cntForAmp = Amp2Num(amp_comp*1000.0f);

            // ��ʾ���
            sprintf(String, "maxid_ave  = %.3f KHz",maxid_ave);
            LCD_DisplayString(10, 120, 12, (u8*)String);
            sprintf(String, "maxAmp_ave = %.3f mVp",maxAmp_ave);
            LCD_DisplayString(10, 135, 12, (u8*)String);
            sprintf(String, "amp_comp = %.3f mVp",amp_comp*1000.0f);
            LCD_DisplayString(10, 150, 12, (u8*)String);
            
            // �ź����
            DAC7612_Write_CHA(cntForAmp);// ����·�£�320��880mVpp(���ֵ)...505��2000mVpp(��ֵ)����������
            AD9833_WaveOut(SIN_WAVE,maxid_ave*1000.0f,0,1);
        }
        // ������ѭ��
        cnt = cnt %	2;			


        // ��ȡһ�β������ڻ�ͼ����230�㣬ΪLCD��Ļ������ص㣩
        for (int i = 0; i < Y_BUF_SIZE; i++) {
                y_in_buf[i] = dsp_workspace.lms_testInput[i+300];
                y_out_buf[i] =  dsp_workspace.error_buf[i+300];
        }
			
		// ���û�ͼ����
        DSP_LMS_DRAW(y_in_buf, y_out_buf, 0.0f, 4.0f);  // y������0����2��Χ
				
        ADC_Show();        // ��ʾ���µ�ADC����ֵ
				
    }
}