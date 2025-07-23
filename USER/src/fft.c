/*********************************************************************************
**********************************************************************************
* 文件名称: fft.c                                                         	     	 *
* 文件简述：DSP								                     		 													 *
* 创建日期：2024.07.11                                                          	 *
* 说    明：FFT函数，幅度谱																						   				 *
**********************************************************************************
*********************************************************************************/

#include "fft.h"
#include <math.h>


// 魔改FFT，从ADC输入→数据处理
float32_t DSP_FFT(float32_t *FFT_Data,float32_t *fft_buffer)
{
	  float32_t max_idx = 1.0f;
	  float32_t max_energy = 0.0f;

    // 构造复数输入：窗口范围内保留实部，其余置0
    for (uint16_t i = 0; i < FFT_N; i++) {
        fft_buffer[2 * i]     = (i < 1000 || i > 3500) ? 0.0f : FFT_Data[i];		
        fft_buffer[2 * i + 1] = 0.0f; // 虚部
    }

    // 初始化和执行FFT
    arm_cfft_radix4_instance_f32 S;
    arm_cfft_radix4_init_f32(&S, FFT_N, 0, 1);  // 正向FFT, bit reversal
    arm_cfft_radix4_f32(&S, fft_buffer);

    // 查找最大能量频率分量（跳过直流）
    for (uint16_t i = 1; i < (FFT_N >> 1); i++) {
        float32_t energy = fft_buffer[2 * i] * fft_buffer[2 * i] + 
														fft_buffer[2 * i + 1] * fft_buffer[2 * i + 1];
        if (energy > max_energy) {
            max_energy = energy;
            max_idx = (float32_t)i;
        }
    }

    // 返回频率值（单位kHz）
    return max_idx * (Fs / (float32_t)FFT_N);
}

float32_t find_max_value_500(float32_t* data) {
    float peak_sum = 0.0f;
    uint16_t peak_count = 0;

    // 限定在区间 [500, 1500] 内找极大值
    for (uint16_t i = 150; i < 2000; i++) // 避免越界，i=501，才能访问 i-1 与 i+1
    {
        if (data[i] > data[i - 1] && data[i] > data[i + 1] && data[i] >= 0.15)
        {
            peak_sum += data[i];
            peak_count++;
        }
    }

    if (peak_count == 0)
        return 0.0f; // 没有找到峰值，返回0（也可返回 NAN）

    return 1403.0f * peak_sum / (peak_count*1.0f);
}


//================幅值检测补偿================//

#define FREQ_TABLE_SIZE (7)
#define AMP_TABLE_SIZE  (7)

const float freq_table_kHz[FREQ_TABLE_SIZE] = { 10.0f, 20.0f, 30.0f, 40.0f, 50.0f, 75.0f, 100.0f };
const float amp_table_V[AMP_TABLE_SIZE]     = { 0.5f, 0.6f, 0.7f, 0.75f, 0.8f, 0.9f, 1.0f };

// 按 freq 行, amp 列顺序
const float meas_table[FREQ_TABLE_SIZE * AMP_TABLE_SIZE] = {
    0.515f, 0.614f, 0.718f, 0.768f, 0.818f, 0.920f, 1.020f,
    0.520f, 0.618f, 0.720f, 0.770f, 0.823f, 0.928f, 1.029f,
    0.510f, 0.612f, 0.713f, 0.765f, 0.815f, 0.918f, 1.020f,
    0.500f, 0.601f, 0.702f, 0.752f, 0.803f, 0.904f, 1.006f,
    0.490f, 0.590f, 0.688f, 0.736f, 0.786f, 0.885f, 0.983f,
    0.450f, 0.546f, 0.637f, 0.682f, 0.729f, 0.820f, 0.911f,
    0.415f, 0.488f, 0.569f, 0.609f, 0.650f, 0.732f, 0.813f
};

// 线性查找辅助
static int find_index(const float* array, int len, float val) {
    if (val <= array[0]) return 0;
    if (val >= array[len - 2]) return len - 2;
    for (int i = 0; i < len - 1; i++) {
        if (val >= array[i] && val <= array[i + 1])
            return i;
    }
    return len - 2;
}

// 双线性反插值核心函数
float estimate_theoretical_amp(float f_query, float V_meas) {
    int i = find_index(freq_table_kHz, FREQ_TABLE_SIZE, f_query);
    float f1 = freq_table_kHz[i];
    float f2 = freq_table_kHz[i + 1];
    float df = (f_query - f1) / (f2 - f1);

    float min_err = 1e10f;
    float best_amp = 0.0f;

    for (float a = amp_table_V[0]; a <= amp_table_V[AMP_TABLE_SIZE - 1]; a += 0.001f) {
        int j = find_index(amp_table_V, AMP_TABLE_SIZE, a);
        float a1 = amp_table_V[j];
        float a2 = amp_table_V[j + 1];
        float da = (a - a1) / (a2 - a1);

        // 四个角点
        float v11 = meas_table[i * AMP_TABLE_SIZE + j];
        float v12 = meas_table[i * AMP_TABLE_SIZE + j + 1];
        float v21 = meas_table[(i + 1) * AMP_TABLE_SIZE + j];
        float v22 = meas_table[(i + 1) * AMP_TABLE_SIZE + j + 1];

        // 双线性插值，估测 V_meas 对应的理论测量值
        float V_interp = v11 * (1 - df) * (1 - da) +
                         v21 * df * (1 - da) +
                         v12 * (1 - df) * da +
                         v22 * df * da;

        float err = fabsf(V_interp - V_meas);
        if (err < min_err) {
            min_err = err;
            best_amp = a;
        }
    }

    return best_amp;
}

//================幅值输出映射================//


#define AMP2NUM_TABLE_SIZE 20

const int amp2num_table_num[AMP2NUM_TABLE_SIZE] = {
    320, 330, 340, 350, 360, 370, 380, 390, 400, 410, 
    420, 430, 440, 450, 460, 470, 480, 490, 505, 520
};

const float amp2num_table_amp[AMP2NUM_TABLE_SIZE] = {
    440, 460, 480, 500, 520, 540, 560, 580, 600, 625, 
    660, 680, 720, 760, 800, 850, 900, 950, 1000, 1050
};


// 辅助查找函数：返回 val 所在的区间索引 i，使得 array[i] <= val <= array[i+1]
static int find_amp_index(const float* array, int len, float val) {
    if (val <= array[0]) return 0;
    if (val >= array[len - 2]) return len - 2;
    for (int i = 0; i < len - 1; i++) {
        if (val >= array[i] && val <= array[i + 1])
            return i;
    }
    return len - 2;  // fallback（不应该执行到）
}

// 插值函数：给定目标幅值，返回估计对应的数字量 num（int）
int Amp2Num(float amp_query) {
    const int len = AMP2NUM_TABLE_SIZE;
    const float* amp = amp2num_table_amp;
    const int* num = amp2num_table_num;

    // 边界处理
    if (amp_query <= amp[0]) return num[0];
    if (amp_query >= amp[len - 1]) return num[len - 1];

    // 查找区间
    int i = find_amp_index(amp, len, amp_query);
    float a1 = amp[i];
    float a2 = amp[i + 1];
    int n1 = num[i];
    int n2 = num[i + 1];

    // 线性插值
    float ratio = (amp_query - a1) / (a2 - a1);
    float n_interp = n1 + ratio * (n2 - n1);
    return (int)(n_interp + 0.5f);  // 四舍五入为整数
}

