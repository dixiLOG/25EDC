// =================================================================
// 文件名: wavetest.c
// 简述: 波形分析模块的实现 (修改后)
// =================================================================

#include "wavetest.h"   // 必须首先包含自己的头文件
#include "lcd.h"        // 需要LCD显示功能
#include "stdio.h"      // 需要sprintf
#include <math.h>       // 需要fabsf (绝对值函数)

// 声明外部定义的FFT参数

//=============================== 内部辅助函数 ===============================//

static float calculate_fc(float data) {
    int rounded_fc = (int)((data + 2.5) / 5) * 5;
    return (float)rounded_fc;
}

static FreqPeak* find_peak_in_all_lists(float target_freq,
                                        float freq_tolerance,
                                        FreqPeak* fundamentals, int num_fundamentals,
                                        FreqPeak* harmonic_pool, int num_harmonics)
{
    for (int i = 0; i < num_fundamentals; i++) {
        if (fabsf(fundamentals[i].freq - target_freq) <= freq_tolerance) {
            return &fundamentals[i];
        }
    }
    for (int i = 0; i < num_harmonics; i++) {
        if (fabsf(harmonic_pool[i].freq - target_freq) <= freq_tolerance) {
            return &harmonic_pool[i];
        }
    }
    return NULL;
}

/**
 * @brief V3版核心分析函数实现 (已修改)
 * @param results       用于存储结果的数组
 * @param max_results   数组最大容量
 * @return int          检测到的基波数量
 */
void analyze_waves_by_threshold()
{
    FreqPeak fundamentals[4];
    int num_fundamentals = 0;
    FreqPeak harmonic_pool[20];
    int num_harmonics = 0;
    WaveFormType fundamental_types[4];
    char result_str[50];

    const float FREQ_TOLERANCE_HZ = (1.0f * Fs / FFT_N * 2.5f);

    // 1. 筛选基波
    for (int i = 1; i < (FFT_N >> 1); i++) {
        float current_amp = Mag_Single[i];
        float current_freq = (float)i * Fs / FFT_N;
        if (current_amp > FUNDAMENTAL_AMP_THRESHOLD &&
            current_freq >= FUNDAMENTAL_FREQ_MIN_HZ &&
            current_freq <= FUNDAMENTAL_FREQ_MAX_HZ)
        {
            if (num_fundamentals < 4) {
                fundamentals[num_fundamentals].freq = current_freq;
                fundamentals[num_fundamentals].amp = current_amp;
                num_fundamentals++;
            }
        }
    }

    // 2. 筛选谐波
    for (int i = 1; i < (FFT_N >> 1); i++) {
        float current_amp = Mag_Single[i];
        float current_freq = (float)i * Fs / FFT_N;
        if (current_amp > HARMONIC_AMP_THRESHOLD && current_amp < FUNDAMENTAL_AMP_THRESHOLD &&
            current_freq >= HARMONIC_FREQ_MIN_HZ && current_freq <= HARMONIC_FREQ_MAX_HZ)
        {
            bool is_near_fundamental = false;
            for(int j = 0; j < num_fundamentals; j++) {
                if (fabsf(current_freq - fundamentals[j].freq) < 5.0f) {
                    is_near_fundamental = true;
                    break;
                }
            }
            if (!is_near_fundamental && num_harmonics < 20) {
                harmonic_pool[num_harmonics].freq = current_freq;
                harmonic_pool[num_harmonics].amp = current_amp;
                num_harmonics++;
            }
        }
    }

    // 3. 对每个基波进行分类
    for (int i = 0; i < num_fundamentals; i++) {
        FreqPeak* base_peak = &fundamentals[i];
        FreqPeak* other_base_peak = (num_fundamentals == 2) ? ((i == 0) ? &fundamentals[1] : &fundamentals[0]) : NULL;
        fundamental_types[i] = WAVE_SINE; // 默认为正弦波

        float target_3f = base_peak->freq * 3;
        float target_5f = base_peak->freq * 5;

        bool is_interfered = (other_base_peak != NULL && fabsf(target_3f - other_base_peak->freq) <= FREQ_TOLERANCE_HZ);
        FreqPeak* harmonic_peak;

        if (is_interfered) {
            harmonic_peak = find_peak_in_all_lists(target_5f, FREQ_TOLERANCE_HZ, fundamentals, num_fundamentals, harmonic_pool, num_harmonics);
        } else {
            harmonic_peak = find_peak_in_all_lists(target_3f, FREQ_TOLERANCE_HZ, fundamentals, num_fundamentals, harmonic_pool, num_harmonics);
        }

        if (harmonic_peak != NULL) {
            fundamental_types[i] = WAVE_TRIANGLE;
        }
    }
		// 30K三角波&50KHzXX波 特例 | 人工核验
		if(fundamentals[1].freq==50&&fundamentals[0].freq==30){
				FreqPeak* base_peak = &fundamentals[1];
				float target_5f = base_peak->freq * 5;
				FreqPeak* harmonic_peak;
				harmonic_peak = find_peak_in_all_lists(target_5f, FREQ_TOLERANCE_HZ, fundamentals, num_fundamentals, harmonic_pool, num_harmonics);

				if (harmonic_peak != NULL) {
					fundamental_types[1] = WAVE_TRIANGLE;
				}
				else{
						fundamental_types[1] = WAVE_SINE;
				}
			
		}
    // 4. ============ 核心修改：填充返回结果的数组 ============
    int results_to_return = (num_fundamentals < MAX_WAVES) ? num_fundamentals : MAX_WAVES;
    for (int i = 0; i < results_to_return; i++) {
        detected_waves[i].base_freq = fundamentals[i].freq;
        detected_waves[i].amplitude = fundamentals[i].amp;
        detected_waves[i].wave_type = fundamental_types[i];
    }
    
    // 5. 在LCD上显示分析结果 (此部分逻辑可以保留用于调试)
    LCD_Fill_onecolor(10, 85, 235, 120, WHITE);
    if (num_fundamentals == 0) {
        LCD_DisplayString_color(10, 85, 12, (u8*)"No fundamental wave found.", RED, WHITE);
    } else {
        for (int i = 0; i < 2; i++) { // 只保留前两个基波输出
            const char* type_str = (fundamental_types[i] == WAVE_TRIANGLE) ? "Triangle" : "Sine";
						wavetype[i] = (fundamental_types[i] == WAVE_TRIANGLE) ? 2 : 3;
            sprintf(result_str, "Base(%.2fkHz): %s", fundamentals[i].freq / 1.0f, type_str);
            LCD_DisplayString_color(10, 85 + i * 15, 12, (u8*)result_str, BLUE, WHITE);
					
        }
    }
    
}

//=============================== 公共接口函数 (已修改) ===============================//

void wave_testv2() {
    char String1[100];
    // 从全局变量计算主要频率点信息
    float fc_max_data_n = calculate_fc(1.0f * Mag_max_index * Fs / FFT_N);
    float fc_max_data = Mag_Single[Mag_max_index];

    LCD_Fill_onecolor(10, 70, 235, 85, WHITE);
    
    // 显示频谱中幅度最大的峰值信息 (用于调试)
    sprintf(String1, "Peak1:%.1fHz, Amp:%.2f", 1.0f * Mag_max_index * Fs / FFT_N, fc_max_data);
    LCD_DisplayString_color(10, 70, 12, (u8*)String1, RED, WHITE);

    // 调用核心分析函数，并返回其结果
    analyze_waves_by_threshold();
}