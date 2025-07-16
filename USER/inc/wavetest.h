// =================================================================
// 文件名: wavetest.h
// 简述: 波形分析模块的头文件
// =================================================================

#ifndef __WAVETEST_H
#define __WAVETEST_H

#include "stm32f4xx.h"  // 包含基础类型定义，如 u8, u16 等
#include "arm_math.h"   // 为了 float32_t 和可能的其他DSP函数
#include "stdbool.h"
#include "fft.h"
//=============================== 1. 宏定义和类型定义 ===============================//

// 将main.c中与波形分析相关的宏定义和类型定义移到此处，使其公用
#define FUNDAMENTAL_AMP_THRESHOLD   60.0f
#define HARMONIC_AMP_THRESHOLD      2.5f
#define FUNDAMENTAL_FREQ_MIN_HZ     15.0f
#define FUNDAMENTAL_FREQ_MAX_HZ     100.0f
#define HARMONIC_FREQ_MIN_HZ        15.0f
#define HARMONIC_FREQ_MAX_HZ        500.0f
#define MAX_WAVES 4


// 波形类型的枚举
typedef enum {
    WAVE_SINE,
    WAVE_TRIANGLE,
    WAVE_UNKNOWN
} WaveFormType;

// 用于存储峰值信息的内部结构体
typedef struct {
    float freq;
    float amp;
} FreqPeak;
// ======================= 新增：用于返回分析结果的结构体 =======================
typedef struct {
    float           base_freq;      // 基波频率 (Hz)
    float           amplitude;      // 基波幅值
    WaveFormType    wave_type;      // 波形类型 (WAVE_SINE 或 WAVE_TRIANGLE)
} WaveResult;

//=============================== 2. 外部变量声明 (核心步骤) ===============================//

// 使用 extern 关键字声明这些变量是在其他文件（如 fft.c 或 main.c）中定义的
extern float32_t Mag_Single[]; // FFT 幅度谱数据
extern u32 Mag_max_index;      // 最大峰值索引
extern u32 Mag_secmax_index;
extern u32 Mag_thirmax_index;
extern int wave_count;
extern WaveResult detected_waves[MAX_WAVES];
extern u8 wavetype[2];

//=============================== 3. 函数原型声明 ===============================//

/**
 * @brief 基于阈值的V3版波形分析函数
 * @note  分析全局变量 Mag_Single[] 中的频谱数据，并通过LCD显示结果。
 */
void wave_testv2();
void wave_test_showinuart();
#endif // __WAVETEST_H