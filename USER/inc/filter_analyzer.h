#ifndef __FILTER_ANALYZER_H
#define __FILTER_ANALYZER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h> // 用于标准整数类型

/* Exported types ------------------------------------------------------------*/

/**
  * @brief 滤波器类型的枚举定义
  */
typedef enum {
    FILTER_TYPE_UNKNOWN,
    FILTER_TYPE_LOWPASS,
    FILTER_TYPE_HIGHPASS,
    FILTER_TYPE_BANDPASS,
    FILTER_TYPE_BANDSTOP
} FilterType;

/**
  * @brief 模拟滤波器关键参数的结构体
  */
typedef struct {
    float f0; // 谐振/中心频率 (Hz)
    float Q;  // 品质因数
    float BW; // -3dB带宽 (Hz)
} FilterParams;

/**
  * @brief 数字IIR滤波器系数的结构体
  */
typedef struct {
    float b[3]; // b0, b1, b2
    float a[3]; // a0, a1, a2 (a0通常为1)
} DigitalCoeffs;

/* Exported functions prototypes ---------------------------------------------*/

/**
  * @brief [阶段3] 根据幅频响应数据判断滤波器类型 (移植自MATLAB V8.3)
  * @param amp_response: (输入) 测量到的幅值响应数组 (线性电压值)
  * @param freq_sweep:   (输入) 扫频频率点数组 (Hz)
  * @param num_points:   (输入) 数据点数量
  * @param max_amp_out:  (输出) 寻找到的最大幅值 (可选, 传入NULL则不输出)
  * @param max_idx_out:  (输出) 最大幅值对应的索引 (可选, 传入NULL则不输出)
  * @param min_amp_out:  (输出) 寻找到的最小幅值 (可选, 传入NULL则不输出)
  * @param min_idx_out:  (输出) 最小幅值对应的索引 (可选, 传入NULL则不输出)
  * @retval FilterType 枚举值
  */
FilterType Identify_Filter_Type(const float* amp_response, const float* freq_sweep, uint16_t num_points,
                                float* max_amp_out, uint16_t* max_idx_out, float* min_amp_out, uint16_t* min_idx_out);

/**
  * @brief [阶段4.1] 根据滤波器类型和幅频响应数据计算其关键参数f0, Q和BW (移植自MATLAB V8.3)
  * @param type:         (输入) 已辨识出的滤波器类型
  * @param amp_response: (输入) 测量到的幅值响应数组 (线性电压值)
  * @param freq_sweep:   (输入) 扫频频率点数组 (Hz)
  * @param num_points:   (输入) 数据点数量
  * @param max_amp:      (输入) 从类型判断中得到的最大幅值
  * @param max_idx:      (输入) 最大幅值对应的索引
  * @param min_idx:      (输入) 最小幅值对应的索引
  * @param noise_floor_V:(输入) 模拟的噪声底电压值，用于LSF数据筛选
  * @retval FilterParams 结构体，包含估算出的f0, Q和BW
  */
FilterParams Calculate_Filter_Params(FilterType type, const float* amp_response, const float* freq_sweep, uint16_t num_points,
                                     float max_amp, uint16_t max_idx, uint16_t min_idx, float noise_floor_V);

/**
  * @brief [阶段4.2] 根据估算出的f0, Q和滤波器类型，计算数字IIR滤波器系数 (移植自MATLAB V8.3)
  * @param type:   (输入) 已辨识出的滤波器类型
  * @param params: (输入) 包含f0和Q的参数结构体
  * @param fs:     (输入) 系统的采样率 (Hz)
  * @retval DigitalCoeffs 结构体，包含b和a系数
  */
DigitalCoeffs Calculate_Digital_Coeffs(FilterType type, FilterParams params, float fs);

/* ------------------ 辅助工具函数 ------------------ */
/**
  * @brief 寻找浮点数组的最小值及其索引
  */
extern void find_array_min(const float* arr, uint16_t len, float* min_val, uint16_t* min_idx);
/**
  * @brief 寻找浮点数组的最大值及其索引
  */
extern void find_array_max(const float* arr, uint16_t len, float* max_val, uint16_t* max_idx);
/**
  * @brief 计算浮点数组指定范围的平均值
  */
extern float array_mean(const float* arr, uint16_t start_idx, uint16_t len);


#ifdef __cplusplus
}
#endif

#endif /* __FILTER_ANALYZER_H */

