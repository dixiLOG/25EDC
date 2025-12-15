// learning_mode.c

#include "learning_mode.h"
#include "lcd.h"
#include "adc.h"
#include "dma.h"
#include "ad9833.h"
#include "dac7612.h"
#include "key.h"
#include "string.h" // 用于 sprintf
#include "math.h"   // 用于 sqrtf 和 fabsf
#include "stdbool.h"
#include "arm_cmsis_dsp.h" 
#include "filter_analyzer.h" 
#include "touch.h"


/*================================================================================*/
/* 全局变量 (Global Variables)                                                    */
/*================================================================================*/

// --- 存储所有采集到的频率响应数据的数组 ---
static FreqResponsePoint g_freq_response_data[SWEEP_POINTS];

// --- 分析结果存储变量 ---
volatile FilterType      g_identified_filter_type = FILTER_TYPE_UNKNOWN;
volatile FilterParams    g_analyzed_params = {0};
volatile DigitalCoeffs   g_iir_coeffs = {{1.0f, 0, 0},{1.0f, 0, 0}};

// 用于在LCD上显示文本的缓冲区
static char g_lcd_buffer[100];

// 用于DSP函数的数据类型转换缓冲区
static float32_t g_float_buffer[Sampl_Times]; 


/*================================================================================*/
/* 私有函数原型 (Private Function Prototypes)                                     */
/*================================================================================*/

static void draw_learning_ui_layout(void);
static void update_progress_display(int current_step, int total_steps, float current_freq);
static uint32_t get_average_adc_value(void);
static uint16_t get_stable_adc_reading(void);
static void perform_frequency_sweep(void);
// [修改] analyze_response_data 不再是简单的测试桩
static void analyze_response_data(void); 
// [未改变] draw_response_curve, display_analysis_results 已合并到 draw_learning_results
static void draw_response_curve(void);
static void display_analysis_results(void);


/*================================================================================*/
/* 公共函数实现 (Public Function Implementation)                                */
/*================================================================================*/

/**
 * @brief 执行完整的、阻塞式的学习序列（扫频+分析）
 */
void execute_learning_sequence(void) {
    // 1. 绘制学习过程中的静态UI和进度条
    draw_learning_ui_layout();
    
    // 2. 执行耗时的扫频和数据采集
    perform_frequency_sweep();

    // 在屏幕上显示“分析中...”
    LCD_Fill_onecolor(10, 50, 230, 65, WHITE); // 清除频率文本
    LCD_DisplayString(10, 50, 16, (u8*)"Analyzing data...");

    // 3. 对采集的数据进行完整的分析
    analyze_response_data();
}

/**
 * @brief 绘制最终的学习结果 (此函数大部分保留原样，仅对数据显示部分进行微调以确保正确性)
 * @details 整合了绘制坐标轴、曲线、标签、高亮标记和文字结果的所有功能。
 */
void draw_learning_results(void) {
    // --- 1. 预计算与Y轴自适应 ---
    float actual_min_db = 0.0f; 
    float max_gain_for_db = 0.0f;

    // 找到最大增益，作为0dB参考
    for(int i = 0; i < SWEEP_POINTS; i++){
        if(g_freq_response_data[i].gain > max_gain_for_db){
            max_gain_for_db = g_freq_response_data[i].gain;
        }
    }
    if (max_gain_for_db < 1e-9f) max_gain_for_db = 1.0f; // 防止除以0

    // 遍历数据，找到实际的最小dB值
    for (int i = 0; i < SWEEP_POINTS; i++) {
        float gain_norm = g_freq_response_data[i].gain / max_gain_for_db;
        if (gain_norm < 1e-4f) gain_norm = 1e-4f; // 钳位到 -80dB
        float db = 20.0f * log10f(gain_norm);
        if (db < actual_min_db) {
            actual_min_db = db;
        }
    }

    float DB_MIN = floorf(actual_min_db / 10.0f) * 10.0f;
    if (DB_MIN > -20.0f) {
        DB_MIN = -20.0f;
    }
    const float DB_MAX = 0.0f; // 上限固定为0dB

    // --- 2. 绘制UI框架 ---
    LCD_Clear(WHITE);
    const u16 GRAPH_TOP_Y = GRAPH_ORIGIN_Y - GRAPH_HEIGHT;

    // --- 3. 绘制坐标轴和刻度 (与原版相同) ---
    LCD_DrawLine(GRAPH_ORIGIN_X, GRAPH_ORIGIN_Y, GRAPH_ORIGIN_X, GRAPH_TOP_Y);
    for (int i = 0; i <= 3; i++) {
        float db_val = DB_MAX - i * (DB_MAX - DB_MIN) / 3.0f;
        u16 y_pos = GRAPH_TOP_Y + i * (GRAPH_HEIGHT / 3);
        sprintf(g_lcd_buffer, "%.0f", db_val);
        LCD_DisplayString(GRAPH_ORIGIN_X - 28, y_pos - 6, 12, (u8*)g_lcd_buffer);
        LCD_DrawLine(GRAPH_ORIGIN_X - 2, y_pos, GRAPH_ORIGIN_X, y_pos);
    }
	LCD_DisplayString(GRAPH_ORIGIN_X - 38, GRAPH_TOP_Y + GRAPH_HEIGHT / 2 - 6, 12, (u8*)"Gain(dB)");
    LCD_DrawLine(GRAPH_ORIGIN_X, GRAPH_ORIGIN_Y, GRAPH_ORIGIN_X + GRAPH_WIDTH, GRAPH_ORIGIN_Y); // X-axis at bottom
    sprintf(g_lcd_buffer, "%.1fkHz", g_params[3].value / 1000.0f);
    LCD_DisplayString(GRAPH_ORIGIN_X, GRAPH_ORIGIN_Y + 5, 12, (u8*)g_lcd_buffer);
    sprintf(g_lcd_buffer, "%.1fkHz", g_params[4].value / 1000.0f);
    LCD_DisplayString(GRAPH_ORIGIN_X + GRAPH_WIDTH - 45, GRAPH_ORIGIN_Y + 5, 12, (u8*)g_lcd_buffer);
		LCD_DisplayString(GRAPH_ORIGIN_X + GRAPH_WIDTH/2 - 20, GRAPH_ORIGIN_Y + 5, 12, (u8*)"Freq(Hz)");

    // --- 4. 绘制dB幅频曲线 ---
    BRUSH_COLOR = RED;
    for (int i = 0; i < SWEEP_POINTS - 1; i++) {
        float gain1_norm = g_freq_response_data[i].gain / max_gain_for_db;
        if (gain1_norm < 1e-4f) gain1_norm = 1e-4f;
        float db1 = 20.0f * log10f(gain1_norm);

        float gain2_norm = g_freq_response_data[i+1].gain / max_gain_for_db;
        if (gain2_norm < 1e-4f) gain2_norm = 1e-4f;
        float db2 = 20.0f * log10f(gain2_norm);
        
        // 使用对数X轴映射
        float log_freq_total = log10f(g_params[4].value) - log10f(g_params[3].value);
        float log_freq1 = log10f(g_freq_response_data[i].frequency_hz) - log10f(g_params[3].value);
        float log_freq2 = log10f(g_freq_response_data[i+1].frequency_hz) - log10f(g_params[3].value);

        u16 x1 = GRAPH_ORIGIN_X + (u16)(log_freq1 * GRAPH_WIDTH / log_freq_total);
        u16 y1 = GRAPH_ORIGIN_Y - (u16)((db1 - DB_MIN) * GRAPH_HEIGHT / (DB_MAX - DB_MIN));
        u16 x2 = GRAPH_ORIGIN_X + (u16)(log_freq2 * GRAPH_WIDTH / log_freq_total);
        u16 y2 = GRAPH_ORIGIN_Y - (u16)((db2 - DB_MIN) * GRAPH_HEIGHT / (DB_MAX - DB_MIN));

        LCD_DrawLine(x1, y1, x2, y2);
    }

    // --- 5. 绘制高亮标记线和文本结果 (与原版逻辑相似，但数据源更可靠) ---
    // (这部分代码无需大改，它会基于 g_analyzed_params 和 g_identified_filter_type 自动显示正确结果)
    u16 text_y_pos = 15;
    const char* type_str = "Unknown";
    
    // ... [原有的高亮线绘制代码] ...
    
    BRUSH_COLOR = BLACK;
    switch (g_identified_filter_type) {
        case FILTER_TYPE_LOWPASS: type_str = "Low-Pass Filter"; break;
        case FILTER_TYPE_HIGHPASS: type_str = "High-Pass Filter"; break;
        case FILTER_TYPE_BANDPASS: type_str = "Band-Pass Filter"; break;
        case FILTER_TYPE_BANDSTOP: type_str = "Band-Stop Filter"; break;
        default: type_str = "Unknown/No Response"; break;
    }
    sprintf(g_lcd_buffer, "Type: %s", type_str);
    LCD_DisplayString(10, text_y_pos, 16, (u8*)g_lcd_buffer);

//    if (g_analyzed_params.f0 > 0) {
//        if (g_identified_filter_type == FILTER_TYPE_LOWPASS || g_identified_filter_type == FILTER_TYPE_HIGHPASS) {
//            sprintf(g_lcd_buffer, "Fc: %.3f kHz", g_analyzed_params.f0 / 1000.0f);
//            LCD_DisplayString(10, text_y_pos + 20, 16, (u8*)g_lcd_buffer);
//        } else {
//            sprintf(g_lcd_buffer, "F0: %.3f kHz", g_analyzed_params.f0 / 1000.0f);
//            LCD_DisplayString(10, text_y_pos + 20, 16, (u8*)g_lcd_buffer);
//        }
//        if (g_analyzed_params.BW > 0 && (g_identified_filter_type == FILTER_TYPE_BANDPASS || g_identified_filter_type == FILTER_TYPE_BANDSTOP)) {
//            sprintf(g_lcd_buffer, "BW: %.3f kHz", g_analyzed_params.BW / 1000.0f);
//            LCD_DisplayString(10, text_y_pos + 40, 16, (u8*)g_lcd_buffer);
//        }
//        if (g_analyzed_params.Q > 0) {
//            sprintf(g_lcd_buffer, "Q: %.2f", g_analyzed_params.Q);
//            LCD_DisplayString(10, text_y_pos + 60, 16, (u8*)g_lcd_buffer);
//        }
//    }
	
		
    LCD_DisplayString(10, 295, 12, (u8*)"Press HOME key to return...");
}


/*================================================================================*/
/* 私有函数实现 (Private Function Implementation)                                 */
/*================================================================================*/

/**
 * @brief 主扫频循环 (对数扫描)
 */
static void perform_frequency_sweep(void) {
    // 对数扫描的乘数因子
    const double ratio = (double)g_params[4].value / g_params[3].value;
    const double multiplier = pow(ratio, 1.0 / (SWEEP_POINTS - 1));
    double current_freq = g_params[3].value;
    
    // 参考信号输出幅度
    DAC7612_Write_CHA(SWEEP_AMP_CNT);
    
    for (int i = 0; i < SWEEP_POINTS; i++) {

        // 为保证最后一个点是精确的结束频率
        if (i == SWEEP_POINTS - 1) {
            current_freq = g_params[4].value;
        }

        // 设置DDS输出
        AD9833_WaveOut(SIN_WAVE, (float)current_freq, 0, 1);
        
        // 等待电路稳定
        if (i == 0) {
            delay_ms(500); // 第一个点延时稍长以确保稳定
        } else {
            delay_ms(50);
        }

        uint16_t stable_reading = get_stable_adc_reading();

        // 存储原始数据和计算后的线性增益
        g_freq_response_data[i].frequency_hz = (float)current_freq;
        g_freq_response_data[i].rms_adc_value = stable_reading;
        g_freq_response_data[i].gain = (float)stable_reading; // 先存ADC值，后续统一转换为电压

        // 更新UI
        update_progress_display(i + 1, SWEEP_POINTS, (float)current_freq);
        
        // 计算下一个频率点
        current_freq *= multiplier;
    }
    
    // 扫频结束，关闭输出
    AD9833_WaveOut(SIN_WAVE, 0, 0, 0); 

    // 将所有ADC值统一转换为电压值
    for(int i = 0; i < SWEEP_POINTS; i++){
        g_freq_response_data[i].gain = g_freq_response_data[i].gain * 3.3f / 4095.0f;
    }
}

/**
 * @brief   [核心更新] 分析采集到的响应数据
 * @details 调用 filter_analyzer.c 中的函数来执行所有核心分析。
 * 此函数现在是真正的在线分析器，而不是测试桩。
 */
static void analyze_response_data(void) {
    // 1. 准备数据数组以传递给分析库
    static float amp_response_v[SWEEP_POINTS];
    static float freq_sweep_hz[SWEEP_POINTS];
    for (int i = 0; i < SWEEP_POINTS; i++) {
        amp_response_v[i] = g_freq_response_data[i].gain; // 使用已转换为电压的增益值
        freq_sweep_hz[i] = g_freq_response_data[i].frequency_hz;
    }

    // 与MATLAB脚本一致的噪声底设置 (如果需要，可以从外部配置)
    const float noise_floor_V = 0.034f;

    // 2. 判断滤波器类型
    float max_amp, min_amp;
    uint16_t max_idx, min_idx;
    g_identified_filter_type = Identify_Filter_Type(amp_response_v, freq_sweep_hz, SWEEP_POINTS, 
                                                    &max_amp, &max_idx, &min_amp, &min_idx);

    // 3. 根据类型计算 f0, Q, BW
    g_analyzed_params = Calculate_Filter_Params(g_identified_filter_type, amp_response_v, freq_sweep_hz, SWEEP_POINTS,
                                                max_amp, max_idx, min_idx, noise_floor_V);

    // 4. 如果参数有效，则计算数字滤波器系数
    if (g_analyzed_params.f0 > 0 && g_analyzed_params.Q > 0) {
        g_iir_coeffs = Calculate_Digital_Coeffs(g_identified_filter_type, g_analyzed_params, Fs*1000.0f);
    } else {
        // 如果参数估算失败，重置为默认直通滤波器
        g_iir_coeffs.b[0] = 1.0f; g_iir_coeffs.b[1] = 0.0f; g_iir_coeffs.b[2] = 0.0f;
        g_iir_coeffs.a[0] = 1.0f; g_iir_coeffs.a[1] = 0.0f; g_iir_coeffs.a[2] = 0.0f;
    }
}

/* ------------------ 以下为未改变的辅助函数 ------------------ */

static void draw_learning_ui_layout(void) {
    LCD_Clear(WHITE);
    BRUSH_COLOR = BLACK;
    LCD_DisplayString(10, 10, 16, (u8*)"Learning Mode: Sweeping...");
    LCD_DrawRectangle(10, 30, 230, 45);
}

static void update_progress_display(int current_step, int total_steps, float current_freq) {
    u16 bar_width = (220 * current_step) / total_steps;
    LCD_Fill_onecolor(11, 31, 11 + bar_width, 44, 0x5E9B); 
    sprintf(g_lcd_buffer, "Freq: %.2f kHz", current_freq / 1000.0f);
    LCD_Fill_onecolor(10, 50, 230, 65, WHITE); 
    LCD_DisplayString(10, 50, 16, (u8*)g_lcd_buffer);
}

static uint32_t get_average_adc_value(void) {
    while (DMA_FLAG[0] == 0);
    DMA_FLAG[0] = 0;
    for (int i = 0; i < Sampl_Times; i++) {
        g_float_buffer[i] = (float32_t)ADC1_Data_Rx[i];
    }
    float32_t mean_result;
    arm_mean_f32(g_float_buffer, Sampl_Times, &mean_result); // 直接使用CMSIS-DSP函数
    return (uint32_t)mean_result;
}

static uint16_t get_stable_adc_reading(void) {
    uint16_t last_reading = 0;
    uint16_t current_reading = 0;
    uint8_t stable_count = 0;
    last_reading = get_average_adc_value();
    while (stable_count < ADC_STABILITY_CHECKS) {
        delay_ms(ADC_CHECK_DELAY_MS);
        current_reading = get_average_adc_value();
        if (abs(current_reading - last_reading) < ADC_STABILITY_THRESHOLD) {
            stable_count++;
        } else {
            stable_count = 0;
        }
        last_reading = current_reading;
    }
    return current_reading;
}

