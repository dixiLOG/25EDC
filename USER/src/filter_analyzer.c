/**
 ******************************************************************************
 * @file    filter_analyzer.c
 * @brief   模拟滤波器分析和重建核心算法 (归一化+正则化 LSF 最终版)
 * @date    2025-08-02
 * @note    此代码为从 MATLAB V8.3 混合策略仿真脚本的完整移植。
 * - V3: 采用归一化+正则化的双重保障策略，实现单精度下高稳定性和高精度。
 ******************************************************************************
 */
#include "filter_analyzer.h"
#include <math.h>   // 用于 sqrtf, log10f, fabsf, sinf, cosf, expf, logf
#include <string.h> // 用于 memcpy
#include <stdbool.h>
#include "lcd.h"
#include "uart.h"       // UART 串口驱动 (USART1/UART4)


// 定义 PI (如果您的工程中没有定义)
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 正则化系数 (一个很小的正数，用于增强矩阵求逆的稳定性)
#define LSF_LAMBDA (1e-9f)


/* Exported functions ---------------------------------------------------------*/

/**
  * @brief 根据幅频响应数据判断滤波器类型
  * @note  此函数无需修改
  */
FilterType Identify_Filter_Type(const float* amp_response, const float* freq_sweep, uint16_t num_points,
                                float* max_amp_out, uint16_t* max_idx_out, float* min_amp_out, uint16_t* min_idx_out)
{
    // ... 代码与原版相同 ...
    if (amp_response == NULL || freq_sweep == NULL || num_points < 10) {
        return FILTER_TYPE_UNKNOWN;
    }
    float max_amp, min_amp;
    uint16_t max_idx, min_idx;
    find_array_max(amp_response, num_points, &max_amp, &max_idx);
    find_array_min(amp_response, num_points, &min_amp, &min_idx);
    if(max_amp_out) *max_amp_out = max_amp;
    if(max_idx_out) *max_idx_out = max_idx;
    if(min_amp_out) *min_amp_out = min_amp;
    if(min_idx_out) *min_idx_out = min_idx;
    float threshold = max_amp / sqrtf(2.0f);
    uint16_t start_count = 0;
    float f_start_limit = freq_sweep[0] * 2.0f;
    for(uint16_t i = 0; i < num_points; ++i) { if(freq_sweep[i] <= f_start_limit) { start_count++; } else { break; } }
    if(start_count == 0) start_count = 1;
    uint16_t end_start_idx = num_points - 1;
    uint16_t end_count = 0;
    float f_end_limit = freq_sweep[num_points-1] / 2.0f;
    for(int16_t i = num_points - 1; i >= 0; --i) { if(freq_sweep[i] >= f_end_limit) { end_start_idx = i; end_count++; } else { break; } }
    if(end_count == 0) end_count = 1;
    float start_gain = array_mean(amp_response, 0, start_count);
    float end_gain = array_mean(amp_response, end_start_idx, end_count);
    bool is_start_high = (start_gain > threshold);
    bool is_end_high = (end_gain > threshold);
    if (!is_start_high && !is_end_high) { return FILTER_TYPE_BANDPASS; } 
    else if (is_start_high && is_end_high) { return FILTER_TYPE_BANDSTOP; } 
    else if (is_start_high && !is_end_high) { return FILTER_TYPE_LOWPASS; } 
    else if (!is_start_high && is_end_high) { return FILTER_TYPE_HIGHPASS; }
    return FILTER_TYPE_UNKNOWN;
}


/**
  * @brief [阶段4.1] 根据滤波器类型和幅频响应数据计算其关键参数f0, Q和BW
  */
FilterParams Calculate_Filter_Params(FilterType type, const float* amp_response, const float* freq_sweep, uint16_t num_points,
                                     float max_amp, uint16_t max_idx, uint16_t min_idx, float noise_floor_V)
{
    FilterParams params = {-1.0f, -1.0f, -1.0f};

    switch(type)
    {
        case FILTER_TYPE_LOWPASS:
        case FILTER_TYPE_HIGHPASS:
        {
            float passband_gain = 1.0f;
            
            // 1. 计算通带增益
            if(type == FILTER_TYPE_LOWPASS) {
                uint16_t start_count = 0;
                float f_start_limit = freq_sweep[0] * 2.0f;
                for(uint16_t i = 0; i < num_points; ++i) { if(freq_sweep[i] <= f_start_limit) start_count++; else break; }
                if(start_count == 0) start_count = 1;
                passband_gain = array_mean(amp_response, 0, start_count);
            } else { // HIGHPASS
                uint16_t end_start_idx = num_points - 1;
                uint16_t end_count = 0;
                float f_end_limit = freq_sweep[num_points-1] / 2.0f;
                for(int16_t i = num_points - 1; i >= 0; --i) { if(freq_sweep[i] >= f_end_limit) { end_start_idx = i; end_count++; } else break; }
                if(end_count == 0) end_count = 1;
                passband_gain = array_mean(amp_response, end_start_idx, end_count);
            }

            // 2. 智能筛选用于拟合的数据点
            uint16_t fit_indices[num_points];
            uint16_t fit_count = 0;
            // ... (筛选逻辑与原版相同) ...
            float noise_floor_db = -80.0f; 
            if(passband_gain > 1e-9f && noise_floor_V > 1e-9f) { noise_floor_db = 20.0f * log10f(noise_floor_V / passband_gain); }
            float rejection_db_limit = noise_floor_db + 6.0f;
            for(uint16_t i = 0; i < num_points; ++i) {
                if(passband_gain > 1e-9f) {
                    float gain_norm = amp_response[i] / passband_gain;
                    if(gain_norm < 1e-4f) gain_norm = 1e-4f;
                    float db_response = 20.0f * log10f(gain_norm);
                    if(fabsf(db_response) > 0.5f && db_response > rejection_db_limit) {
                        fit_indices[fit_count++] = i;
                    }
                }
            }
            if(fit_count < 3) return params;

            // --- [新增] 步骤 3: 数据归一化 ---
            // 计算被选中数据点的中心频率(几何平均值)，作为归一化的基准
            double log_freq_sum = 0.0;
            for(uint16_t i = 0; i < fit_count; ++i) {
                log_freq_sum += logf(freq_sweep[fit_indices[i]]);
            }
            float f_center = expf((float)(log_freq_sum / fit_count));
            double wc = 2.0 * M_PI * f_center; // 中心角频率

            // --- 步骤 4: 使用归一化数据构建并求解最小二乘问题 ---
            double sum_A11 = 0, sum_A12 = 0, sum_A22 = 0; 
            double sum_A_T_Y1 = 0, sum_A_T_Y2 = 0;     

            for(uint16_t i = 0; i < fit_count; ++i) {
                uint16_t idx = fit_indices[i];
                
                // 使用原始数据计算 Y 值 (这部分不受归一化影响)
                double amp_norm_sq = (double)(amp_response[idx] / passband_gain) * (amp_response[idx] / passband_gain);
                double y_val = (1.0 / amp_norm_sq) - 1.0;
                
                // [核心修改] 使用归一化的频率来构造X矩阵
                double w_norm = (2.0 * M_PI * freq_sweep[idx]) / wc;
                double x1_norm, x2_norm;

                if(type == FILTER_TYPE_LOWPASS) {
                    double w_norm_sq = w_norm * w_norm;
                    x1_norm = w_norm_sq;
                    x2_norm = w_norm_sq * w_norm_sq;
                } else { // HIGHPASS
                    double inv_w_norm_sq = 1.0 / (w_norm * w_norm);
                    x1_norm = inv_w_norm_sq;
                    x2_norm = inv_w_norm_sq * inv_w_norm_sq;
                }
                
                sum_A11 += x1_norm * x1_norm;
                sum_A12 += x1_norm * x2_norm;
                sum_A22 += x2_norm * x2_norm;
                sum_A_T_Y1 += y_val * x1_norm;
                sum_A_T_Y2 += y_val * x2_norm;
            }

            // --- [修改] 步骤 5: 应用正则化并求解 ---
            double det = (sum_A11 + LSF_LAMBDA) * (sum_A22 + LSF_LAMBDA) - sum_A12 * sum_A12;

            if(fabs(det) > 1e-30) {
                double inv_det = 1.0 / det;
                double C1_norm = inv_det * ((sum_A22 + LSF_LAMBDA) * sum_A_T_Y1 - sum_A12 * sum_A_T_Y2);
                double C2_norm = inv_det * (-sum_A12 * sum_A_T_Y1 + (sum_A11 + LSF_LAMBDA) * sum_A_T_Y2);

                // --- [新增] 步骤 6: 反归一化，得到真实的物理系数 ---
                double C1, C2;
                double wc_sq = wc * wc;
                double wc_p4 = wc_sq * wc_sq;
                if(type == FILTER_TYPE_LOWPASS) {
                    C1 = C1_norm / wc_sq;
                    C2 = C2_norm / wc_p4;
                } else { // HIGHPASS
                    C1 = C1_norm * wc_sq;
                    C2 = C2_norm * wc_p4;
                }

                if (C2 > 0) {
                    double w0_sq_est;
                    double q_sq_inv;
                    
                    if (type == FILTER_TYPE_LOWPASS) {
                        w0_sq_est = 1.0 / sqrt(C2);
                        q_sq_inv = C1 * w0_sq_est + 2.0;
                    } else { // HIGHPASS
                        w0_sq_est = sqrt(C2);
                        q_sq_inv = C1 / w0_sq_est + 2.0;
                    }
                    
                    params.f0 = (float)(sqrt(w0_sq_est) / (2.0 * M_PI));
                    if (q_sq_inv > 0) {
                        params.Q = (float)(1.0 / sqrt(q_sq_inv));
                    } else {
                        params.Q = 1e6f;
                    }
                }
            }
            if(params.Q > 0 && params.f0 > 0) params.BW = params.f0 / params.Q;
            break;
        }
				
				
        case FILTER_TYPE_BANDPASS:
        {
            // --- Bandpass: 使用3dB特征点提取法 ---
            params.f0 = freq_sweep[max_idx];
            float amp_3dB = max_amp / sqrtf(2.0f);
            float f_l = -1.0f, f_h = -1.0f;

            // 找下边带-3dB点 (从峰值向低频搜索)
            for(int16_t i = max_idx; i >= 1; --i) {
                if(amp_response[i-1] < amp_3dB && amp_response[i] >= amp_3dB) {
                    // 线性插值
                    f_l = freq_sweep[i-1] + (freq_sweep[i] - freq_sweep[i-1]) * (amp_3dB - amp_response[i-1]) / (amp_response[i] - amp_response[i-1]);
                    break;
                }
            }
            // 找上边带-3dB点 (从峰值向高频搜索)
            for(uint16_t i = max_idx; i < num_points - 1; ++i) {
                if(amp_response[i] >= amp_3dB && amp_response[i+1] < amp_3dB) {
                    // 线性插值
                    f_h = freq_sweep[i] + (freq_sweep[i+1] - freq_sweep[i]) * (amp_3dB - amp_response[i]) / (amp_response[i+1] - amp_response[i]);
                    break;
                }
            }
            
            if (f_l > 0 && f_h > 0) {
                params.BW = f_h - f_l;
                if (params.BW > 1e-9f) params.Q = params.f0 / params.BW;
            } else {
                params.f0 = -1.0f; // 表示失败
            }
            break;
        }

        case FILTER_TYPE_BANDSTOP:
        {
            // --- Bandstop: 使用鲁棒的3dB特征点提取法 ---
            uint16_t margin_len = (uint16_t)(num_points * 0.15f);
            if (margin_len == 0) margin_len = 1;
            
            float avg_gain_start = array_mean(amp_response, 0, margin_len);
            float avg_gain_end = array_mean(amp_response, num_points - margin_len, margin_len);
            float passband_gain_est = (avg_gain_start + avg_gain_end) / 2.0f;
            float amp_3dB = passband_gain_est / sqrtf(2.0f);
            float f_l = -1.0f, f_h = -1.0f;

            // 找下边带-3dB点 (从谷值向低频搜索)
            for(int16_t i = min_idx; i >= 1; --i) {
                if(amp_response[i-1] > amp_3dB && amp_response[i] <= amp_3dB) {
                    f_l = freq_sweep[i-1] + (freq_sweep[i] - freq_sweep[i-1]) * (amp_3dB - amp_response[i-1]) / (amp_response[i] - amp_response[i-1]);
                    break;
                }
            }
            // 找上边带-3dB点 (从谷值向高频搜索)
            for(uint16_t i = min_idx; i < num_points-1; ++i) {
                if(amp_response[i] <= amp_3dB && amp_response[i+1] > amp_3dB) {
                     f_h = freq_sweep[i] + (freq_sweep[i+1] - freq_sweep[i]) * (amp_3dB - amp_response[i]) / (amp_response[i+1] - amp_response[i]);
                    break;
                }
            }

            if(f_l > 0 && f_h > 0 && f_h > f_l) {
                params.BW = f_h - f_l;
                params.f0 = sqrtf(f_l * f_h); // 中心频率用几何平均值
                if (params.BW > 1e-9f) params.Q = params.f0 / params.BW;
            } else {
                 params.f0 = -1.0f; // 表示失败
            }
            break;
        }
        
        default:
            // 返回无效参数
            break;
    }
    return params;
}

/**
  * @brief [阶段4.2] 根据估算出的f0, Q和滤波器类型，计算数字IIR滤波器系数
  */
DigitalCoeffs Calculate_Digital_Coeffs(FilterType type, FilterParams params, float fs)
{
    DigitalCoeffs coeffs = {{1.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}}; // 默认直通
    if (params.f0 <= 0 || params.Q <= 0 || fs <= 0) {
        return coeffs; // 返回无效系数
    }

    float w0 = 2.0f * M_PI * params.f0 / fs;
    float alpha = sinf(w0) / (2.0f * params.Q);
    float cos_w0 = cosf(w0);

    float b_temp[3] = {0};
    float a_temp[3] = {0};
    
    // 使用标准的Audio EQ Cookbook公式
    switch(type)
    {
        case FILTER_TYPE_LOWPASS:
            b_temp[0] = (1.0f - cos_w0) / 2.0f;
            b_temp[1] = 1.0f - cos_w0;
            b_temp[2] = (1.0f - cos_w0) / 2.0f;
            a_temp[0] = 1.0f + alpha;
            a_temp[1] = -2.0f * cos_w0;
            a_temp[2] = 1.0f - alpha;
            break;
        case FILTER_TYPE_HIGHPASS:
            b_temp[0] = (1.0f + cos_w0) / 2.0f;
            b_temp[1] = -(1.0f + cos_w0);
            b_temp[2] = (1.0f + cos_w0) / 2.0f;
            a_temp[0] = 1.0f + alpha;
            a_temp[1] = -2.0f * cos_w0;
            a_temp[2] = 1.0f - alpha;
            break;
        case FILTER_TYPE_BANDPASS:
            b_temp[0] = alpha; // MATLAB版本是 alpha * [1, 0, -1]，Q值定义可能不同，但此为标准形式
            b_temp[1] = 0;
            b_temp[2] = -alpha;
            a_temp[0] = 1.0f + alpha;
            a_temp[1] = -2.0f * cos_w0;
            a_temp[2] = 1.0f - alpha;
            break;
        case FILTER_TYPE_BANDSTOP:
            b_temp[0] = 1.0f;
            b_temp[1] = -2.0f * cos_w0;
            b_temp[2] = 1.0f;
            a_temp[0] = 1.0f + alpha;
            a_temp[1] = -2.0f * cos_w0;
            a_temp[2] = 1.0f - alpha;
            break;
        default:
            return coeffs;
    }

    // 归一化：所有系数除以a0
    float a0_norm = a_temp[0];
    if (fabsf(a0_norm) > 1e-9f) {
        coeffs.a[0] = 1.0f; // a0总是1
        coeffs.a[1] = a_temp[1] / a0_norm;
        coeffs.a[2] = a_temp[2] / a0_norm;
        coeffs.b[0] = b_temp[0] / a0_norm;
        coeffs.b[1] = b_temp[1] / a0_norm;
        coeffs.b[2] = b_temp[2] / a0_norm;
    }

    return coeffs;
}


/*==============================================================================
 * Private functions (now exported via .h file)
 *============================================================================*/

/**
  * @brief 计算浮点数组指定范围的平均值
  */
float array_mean(const float* arr, uint16_t start_idx, uint16_t len)
{
    double sum = 0.0; // 使用double进行累加保证精度
    if (len == 0 || arr == NULL) return 0.0f;
    for(uint16_t i = 0; i < len; ++i) {
        sum += arr[start_idx + i];
    }
    return (float)(sum / len);
}

/**
  * @brief 寻找浮点数组的最大值及其索引
  */
void find_array_max(const float* arr, uint16_t len, float* max_val, uint16_t* max_idx)
{
    if (len == 0 || arr == NULL) {
        if(max_val) *max_val = 0.0f;
        if(max_idx) *max_idx = 0;
        return;
    }
    *max_val = arr[0];
    *max_idx = 0;
    for(uint16_t i = 1; i < len; ++i) {
        if(arr[i] > *max_val) {
            *max_val = arr[i];
            *max_idx = i;
        }
    }
}

/**
  * @brief 寻找浮点数组的最小值及其索引
  */
void find_array_min(const float* arr, uint16_t len, float* min_val, uint16_t* min_idx)
{
    if (len == 0 || arr == NULL) {
        if(min_val) *min_val = 0.0f;
        if(min_idx) *min_idx = 0;
        return;
    }
    *min_val = arr[0];
    *min_idx = 0;
    for(uint16_t i = 1; i < len; ++i) {
        if(arr[i] < *min_val) {
            *min_val = arr[i];
            *min_idx = i;
        }
    }
}

