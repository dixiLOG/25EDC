// learning_mode.h

#ifndef __LEARNING_MODE_H__
#define __LEARNING_MODE_H__

#include "common.h"
#include "stm32f4xx.h"
#include "filter_analyzer.h" // 新增：包含分析器头文件
#include "math_helper.h"
/*================================================================================*/
/* 配置常量 (Configuration Constants)                                           */
/*================================================================================*/

// --- 扫频参数 ---
#define SWEEP_FREQ_START_HZ     500.0f
#define SWEEP_FREQ_END_HZ       250000.0f
//#define SWEEP_FREQ_STEP_HZ      500.0f
#define SWEEP_AMP_CNT 					1370 			// 2Vpp
//#define SWEEP_AMP_CNT 					1212 		// 1Vpp

// [NEW] 经过校准的参考电平对应的ADC值 (无滤波器直连时的读数)
#define ADC_REF_AMP          (3456.0f)

// [NEW] 用于计算通带增益时，采样的点数，以增加稳定性
#define PASSBAND_AVG_POINTS  (5)


// 计算扫频所需的点数
#define SWEEP_POINTS            250

// --- ADC 稳定性检查 ---
#define ADC_STABILITY_CHECKS    3       // 需要连续多少次读数稳定才算收敛
#define ADC_STABILITY_THRESHOLD 3       // ADC值相差多少以内认为稳定 (满量程4095)
#define ADC_CHECK_DELAY_MS      50      // 两次稳定性检查之间的延迟

// --- 学习界面UI布局 ---
#define GRAPH_ORIGIN_X          40
#define GRAPH_ORIGIN_Y          260
#define GRAPH_WIDTH             190
#define GRAPH_HEIGHT            180
#define AXIS_COLOR              BLACK
#define GRID_COLOR              GRAY
#define CURVE_COLOR             RED


/*================================================================================*/
/* 类型定义 (Type Definitions)                                                    */
/*================================================================================*/

/**
 * @brief 用于存储单个频点测量结果的结构体
 */
typedef struct {
    float32_t frequency_hz;
    uint16_t rms_adc_value; // 采集到的有效值对应的ADC值
    float gain;             // 计算出的相对增益
} FreqResponsePoint;

/*================================================================================*/
/* 2. 全局变量声明 (Global Variable Declarations)                                 */
/*================================================================================*/

// --- 分析结果的全局变量 ---
// 这些变量在 learning_mode.c 中定义，在其他文件中使用
extern volatile FilterType g_identified_filter_type;
extern volatile FilterParams g_analyzed_params;
extern volatile DigitalCoeffs g_iir_coeffs;

// --- 用于Flash读写的数据缓冲区 ---
// 这个缓冲区在 main.c 中定义


/*================================================================================*/
/* 公共函数原型 (Public Function Prototypes)                                  */
/*================================================================================*/
extern float golden_amp_response[SWEEP_POINTS];
extern float golden_freq_sweep[SWEEP_POINTS];
extern void Run_Offline_Algorithm_Test();
/**
 * @brief   运行整个学习和分析过程的主函数
 * @details 当系统进入 STATE_LEARNING_MODE 状态时调用此函数。
 * 它负责处理UI绘制、扫频、数据分析和结果显示。
 */
void run_learning_mode(void);
void execute_learning_sequence(void);
void draw_learning_results(void);
void analyze_response_data(void);

#endif // __LEARNING_MODE_H__