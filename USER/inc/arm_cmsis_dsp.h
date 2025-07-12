/*********************************************************************************
**********************************************************************************
* 文件名称: arm_cmsis_dsp.h
* 文件简述：DSP	
* 创建日期：2024.07.24
* 作者：hzl
* 说    明： DSP 函数运算调用 与 综合函数
参考网址：
https://arm-software.github.io/CMSIS-DSP/v1.10.1/group__groupExamples.html

info:
1. basic math functions
    1.1 取绝对值		DSP_ABS
		1.2 加法			DSP_ADD
		1.3 区间剪裁		DSP_CLIP
		1.4 点积			DSP_DOT
		1.5 乘法			DSP_MULT
		1.6 取反  		DSP_NEG
		1.7 偏移 			DSP_OFFSET
		1.8 比例缩放 	DSP_SCALE
		1.9 位移>>1  	DSP_SHIFT
		1.10 减法			DSP_SUB
2. fast math functions
		2.1 cos				DSP_COS
		2.2 sin				DSP_SIN
		2.3 开根			DSP_SQRT
3. complex math fuctions
		3.1 共轭								DSP_CMPLX_CONJ
		3.2 点积								DSP_CMPLX_DOT
		3.3 取模								DSP_CMPLX_MAG
		3.4 模的平方							DSP_CMPLX_MAG_SQ
		3.5 乘法								DSP_CM_MUL_CM
		3.6 复数与实数相乘				DSP_CM_MUL_RE
4. filtering functions
		4.1 采用直接 I 型结构的双二阶级联 IIR 滤波器<TODO>
		4.2 采用直接型 II 转置结构的双二阶级联 IIR 滤波器<TODO>
		4.3 有限脉冲响应 (FIR) 滤波器		DSP_FIR_FIL
		4.4 卷积												DSP_CONV
		4.5 区域卷积											DSP_CONV_P
		4.6 相关性												DSP_CORR
5. Matrix functions
		5.1 矩阵初始化			DSP_MAT_INIT
		5.2 加法					DSP_MAT_ADD
		5.3 减法					DSP_MAT_SUB
		5.4 乘法					DSP_MAT_MUL
		5.5 比例变换				DSP_MAT_SCA	
		5.6 求逆					DSP_MAT_INV
		5.7 转置					DSP_MAT_TRA
6. Transforms
		6.1 fft
7. Control functions
		7.1 PID						DSP_PID
8. Statistical functions
		8.1 最大值					DSP_MAX
		8.2 最小值					DSP_MIN
		8.3 平均值					DSP_MEAN
		8.4 平方和					DSP_POW
		8.5 均方根					DSP_RMS
		8.6 标准差					DSP_STD
		8.7 方差					DSP_VAR
9. Support functions
		1. 复制						DSP_COPY
		2. 填充						DSP_FILL
10. Interpolation functions
		1. 线性差值				DSP_LIN
//////////////////////////////	
11. 自定义函数
	1. 简单FIFO滤波			DSP_FILT
**********************************************************************************
*********************************************************************************/


#ifndef _ARM_CMSIS_DSP_H_
#define _ARM_CMSIS_DSP_H_

#include "common.h"
#include "arm_math.h"
#include "math_helper.h"		//有些函数在这个文件下
#include "math.h"
#include "stdio.h"


#define Q12QUARTER	//Q15开根宏
#define Q28QUARTER	//Q31开根宏

//======ATTATION==================
/*
	数组变量输入函数调用如下：
		DSP_ADD(pSrcA,pSrcB,pDst,blockSize);
	单个数字变量调用方式如下：
		DSP_ADD(&pSrcA,&pSrcB,&pDst,blockSize);
*/

/* basic math 的单输入运算允许输入输出为同一个变量 */
/* fast math 输入为单个变量，一进一出 */
/* complex math 数据交错存储，偶数存real,奇数存img*/
/* 对数字滤波器而言，高阶的IIR滤波器很难稳定，高阶的FIR滤波器会使计算量大增 ; FIR 为线性相位差 */
/* 矩阵运算需要初始化，在指针下运算 ;  矩阵要转化为一维数组 */

//===============================


//===================================================单体函数========================================
//1. basic math functions
//1.1 取绝对值
void DSP_ABS(float32_t *pSrc ,float32_t *pDst ,uint32_t blockSize);
//	pSrc：输入 数字/数组
//	pDst：结果
//	blockSize： 数组大小，1 则为单个数字
//  调用：DSP_ABS(pSrc,pDst,blockSize);
//======

//1.2 加法
void DSP_ADD(float32_t *pSrcA ,float32_t *pSrcB ,float32_t *pDst ,uint32_t blockSize);
//	pSrcA：加数 A
//  pSrcB：加数 B
//	pDst：结果
//	blockSize： 数组大小，1 则为单个数字
//  调用：DSP_ADD(pSrcA,pSrcB,pDst,blockSize);
//======

//1.3 区间剪裁
void DSP_CLIP(float *pIn, uint32_t blockSize, float32_t low, float32_t high);
//  pIn:         输入/输出
//  numSamples : 数组大小
//  low:         下限
//  high:        上限
//  调用：DSP_CLIP(pSrcA,blockSize,-6,5);
//======

//1.4 点积
void DSP_DOT(float32_t *pSrcA ,float32_t *pSrcB ,uint32_t blockSize,float32_t * result);
//  pSrcA： A
//  pSrcB： B
//  blockSize： 数组大小
//  result：结果
//  调用：DSP_DOT(pSrcA,pSrcB, blockSize ,&tmp);
//======

//1.5 乘法
void DSP_MULT(float32_t *pSrcA ,float32_t *pSrcB ,float32_t *pDst ,uint32_t blockSize);
//  pSrcA：乘数 A
//  pSrcB：乘数 B
//  pDst：结果
//  blockSize： 数组大小
//  调用：DSP_MULT(pSrcA,pSrcB,pDst,blockSize);
//======

//1.6 取反
void DSP_NEG(float32_t *pSrc ,float32_t *pDst ,uint32_t blockSize);
//  pSrc：输入
//  pDst：结果
//  blockSize： 数组大小
//  调用：DSP_NEG(pSrc,pDst,blockSize);
//======

//1.7 偏移
void DSP_OFFSET(float32_t *pSrc ,float32_t offset ,float32_t *pDst ,uint32_t blockSize);
//  pSrc：输入
//  offset:偏置
//  pDst： 结果
//  blockSize： 数组大小
//  调用：DSP_OFFSET(pSrc, offset ,pDst,blockSize);
//======

//1.8 比例缩放
void DSP_SCALE(float32_t *pSrc ,float32_t scale,float32_t *pDst ,uint32_t blockSize);
//  pSrc：输入
//  scale:比例
//  pDst： 结果
//  blockSize： 数组大小
//  调用：DSP_SCALE(pSrc, scale,pDst,blockSize);
//======

//1.9 移位
void DSP_SHIFT(q31_t *pSrc ,int8_t shiftBits,q31_t *pDst ,uint32_t blockSize);
//  pSrc：输入
//  shiftBits:移动位数
//  pDst： 结果
//  blockSize： 数组大小
//  调用：DSP_SHIFT(pSrc, shiftBits , pDst ,blockSize);
//======

//1.10 减法
void DSP_SUB(float32_t *pSrcA ,float32_t *pSrcB ,float32_t *pDst ,uint32_t blockSize);
//        pSrcA：被减数 A
//        pSrcB：减数 B
//        pDst：结果
//        blockSize： 数组大小
//  调用： DSP_SUB(pSrcA,pSrcB,pDst,blockSize);
//======

////////////////////////////////////////////////////////

//2. fast math
//2.1 cos
float32_t  DSP_COS(float32_t x);
//  x： 输入,rad
//  调用：DSP_COS(x);
//======

//2.2 sin
float32_t  DSP_SIN(float32_t x);
//  x： 输入,rad
//  产生正弦信号
//	  for (uint32_t i = 0; i < TEST_LENGTH_SAMPLES; i++) {
//        testInput[i] = 幅值A * DSP_SIN(2 * PI * 频率freq * i / 采样率[HZ] ) ;
//    }
//  调用：DSP_SIN(x);
//======

//2.3 sqrt 
void  DSP_SQRT(float32_t x,float32_t * result);
//  x： 输入
//  result:结果
//  调用：  DSP_SQRT(x,&result);
//======

////////////////////////////////////////////////////////

//3.complex math

//3.1 共轭
void DSP_CMPLX_CONJ(float32_t *pSrc ,float32_t *pDst ,uint32_t blockSize);
//  pSrc：输入
//  pDst： 结果
//  blockSize： 复数个数
//  调用：  DSP_CMPLX_CONJ(pSrc,pDst,blockSize);
//======

//3.2 点积
void DSP_CMPLX_DOT(float32_t *pSrcA ,float32_t *pSrcB , uint32_t blockSize , float32_t * realresult,  float32_t * imagresult);
//  pSrcA：A输入
//  pSrcB：B输入
//  blockSize： 复数个数
//  realresult: 实部结果
//  imageresult:虚部结果
//  调用：  DSP_CMPLX_DOT(pSrcA,pSrcB,blockSize,&realresult,&imagresult);
//======

//3.3 取模
void DSP_CMPLX_MAG(float32_t *pSrc ,float32_t *pDst ,uint32_t blockSize);
//  pSrc：输入
//  pDst： 结果
//  blockSize： 复数个数
//  调用：  DSP_CMPLX_MAG(pSrc,pDst,blockSize);
//======

//3.4 模的平方
void DSP_CMPLX_MAG_SQ(float32_t *pSrc ,float32_t *pDst ,uint32_t blockSize);
//  pSrc：输入
//  pDst： 结果
//  blockSize： 复数个数
//  调用：  DSP_CMPLX_MAG_SQ(pSrc,pDst,blockSize);
//======

//3.5 乘法
void DSP_CM_MUL_CM(float32_t *pSrcA ,float32_t *pSrcB , float32_t *pDst  , uint32_t blockSize);
//  pSrcA：A输入
//  pSrcB：B输入
//  pDst： 结果
//  blockSize： 复数个数
//  调用：  DSP_CM_MUL_CM(pSrcA,pSrcB,pDst,blockSize);
//======

//3.6 复数与实数相乘
void DSP_CM_MUL_RE(float32_t *pSrcA ,float32_t *pSrcB , float32_t *pDst  , uint32_t blockSize);
//  pSrcA：复数A输入
//  pSrcB：实数B输入
//  pDst： 结果
//  blockSize： 复数个数
//  调用：  DSP_CM_MUL_RE(pSrcA,pSrcB,pDst,blockSize);
//======

////////////////////////////////////////////////////////
//4. filtering functions
//参考：<FIR 介绍> 	https://www.cnblogs.com/Jamesjiang/p/8986632.html
//		 <FIR 设计> 	https://shequ.stmicroelectronics.cn/thread-601106-1-1.html
//		 <IIR 设计> 	https://shequ.stmicroelectronics.cn/thread-601141-1-1.html
//		 <IIR 设计>   https://www.cnblogs.com/embInn/p/12904847.html
//		 <NMLS 设计><需要设定波形，放弃>  https://www.cnblogs.com/armfly/p/15320857.html
//
//4.1 直接 I 型结构双二阶级联 IIR 滤波器
// <TODO>
//======

//4.2 直接 II 型结构双二阶级联 IIR 滤波器
// <TODO>
//======

//4.3 有限脉冲响应 (FIR) 滤波器
void DSP_FIR_FIL( float32_t * 	pSrc , float32_t * 	pDst , uint16_t sampleP , uint32_t blockSize , float32_t * pCoeffs , uint16_t numTaps );
// pSrc:输入信号
// pDst:输出信号
// sampleP:采样点数(输入信号长度)
// blockSize:每次处理点数
// pCoeffs:滤波器系数数组(MATLAB -> filterDesigner)
// numTaps:滤波器系数个数
//======

//4.4 卷积
void DSP_CONV( float32_t * 	pSrcA, uint32_t 	srcALen, float32_t * 	pSrcB, uint32_t 	srcBLen, float32_t * 	pDst );
//  pSrcA: A 信号
//  srcALen: A 信号时域宽度
//  pSrcB: B 信号
//  srcBLen: B 信号时域宽度
//	pDst: 卷积结果，数组宽度 = srcALen + srcBLen - 1
//  调用：  DSP_CONV(pSrcA,srcALen,pSrcB,srcBLen,pDst);
//======

//4.5 区域卷积
void DSP_CONV_P( float32_t * 	pSrcA, uint32_t 	srcALen, float32_t * 	pSrcB, uint32_t 	srcBLen, float32_t * 	pDst , uint32_t first_index , uint32_t numP );
//  pSrcA: A 信号
//  srcALen: A 信号时域宽度
//  pSrcB: B 信号
//  srcBLen: B 信号时域宽度
//	pDst: 卷积结果，数组宽度 = srcALen + srcBLen - 1
//	first_index:起始位置
//  numP：抓取点数；[first_index, ..., first_index+numP-1]
//	//输出索引的允许范围是 [0 , srcALen+srcBLen-2]//
//  调用：  DSP_CONV_P(pSrcA,srcALen,pSrcB,srcBLen,pDst,first_index,numP);
//======

//4.6 相关性
void DSP_CORR( float32_t * 	pSrcA, uint32_t 	srcALen, float32_t * 	pSrcB, uint32_t 	srcBLen, float32_t * 	pDst );
//  pSrcA: A 信号
//  srcALen: A 信号时域宽度
//  pSrcB: B 信号
//  srcBLen: B 信号时域宽度
//	pDst: 结果，数组宽度 = 2 * MAX(srcALen, srcBLen) - 1
//	//初始化需要把pDst清零
//  调用：  DSP_CONV(pSrcA,srcALen,pSrcB,srcBLen,pDst);
//======
////////////////////////////////////////////////////////

//5. Matrix functions
//5.1 矩阵初始化
void DSP_MAT_INIT(arm_matrix_instance_f32 * S, uint16_t 	nRows, uint16_t nColumns, float32_t * pData );	
// S:使指向pData的矩阵指针，需提前声明 arm_matrix_instance_f32 <name>； 
// nRows:行数
// nColumns:列数
// pData:输入矩阵
// 调用：  DSP_MAT_INIT(S,nRows,nColumns,pData);
//======

//5.2 加法
void DSP_MAT_ADD(float32_t *pSrcA,float32_t *pSrcB ,float32_t *pDst,uint16_t 	nRows, uint16_t nColumns );	
// pSrcA:A 矩阵
// pSrcB:B 矩阵
// pDst:结果
// nRows:A/B行数，默认一致
// nColumns:A/B列数
// 其中的指针运算闭包
// 调用：  DSP_MAT_ADD(pSrcA,pSrcB,pDst,nRows,nColumns);
//======


//5.3 减法
void DSP_MAT_SUB(float32_t *pSrcA,float32_t *pSrcB ,float32_t *pDst,uint16_t 	nRows, uint16_t nColumns );	
// pSrcA:A 矩阵
// pSrcB:B 矩阵
// pDst:结果
// nRows:A/B行数，默认一致
// nColumns:A/B列数
// 其中的指针运算闭包
// 调用：  DSP_MAT_SUB(pSrcA,pSrcB,pDst,nRows,nColumns);
//======

//5.4 乘法
void DSP_MAT_MUL(float32_t *pSrcA,float32_t *pSrcB ,float32_t *pDst,uint16_t 	nRows, uint16_t nColumns );	
// pSrcA:A 矩阵
// pSrcB:B 矩阵
// pDst:结果
// nRows:A/B行数，默认一致
// nColumns:A/B列数
// 其中的指针运算闭包
// 调用：  DSP_MAT_MUL(pSrcA,pSrcB,pDst,nRows,nColumns);
//======

//5.5 比例变换
void DSP_MAT_SCA(float32_t *pSrc, float32_t scale ,float32_t *pDst,uint16_t 	nRows, uint16_t nColumns);	
// pSrc:输入 矩阵
// scale: 缩放比例
// pDst:结果
// nRows:A/B行数，默认一致
// nColumns:A/B列数
// 其中的指针运算闭包
// 调用：  DSP_MAT_SCA(pSrc,scale,pDst,nRows,nColumns);
//======

//5.6 求逆
void DSP_MAT_INV(float32_t *pSrc ,float32_t *pDst,uint16_t 	nRows, uint16_t nColumns);	
// pSrc:输入 矩阵
// pDst:结果
// nRows:A/B行数，默认一致;必须是方阵
// nColumns:A/B列数
// 其中的指针运算闭包
// 调用：  DSP_MAT_INV(pSrc,pDst,nRows,nColumns);
//======

//5.7 转置
void DSP_MAT_TRA(float32_t *pSrc ,float32_t *pDst,uint16_t 	nRows, uint16_t nColumns);	
// pSrc:输入 矩阵
// pDst:结果
// nRows:A/B行数，默认一致;必须是方阵
// nColumns:A/B列数
// 其中的指针运算闭包
// 调用：  DSP_MAT_TRA(pSrc,pDst,nRows,nColumns);
//======

////////////////////////////////////////////////////////

//6. Transforms
//6.1 fft
//见 fft.c
////////////////////////////////////////////////////////

//7. Control functions
//7.1 PID
//参考：https://www.cnblogs.com/fang-d/articles/CMSIS-DSP_PID_Control.html

float32_t DSP_PID(float32_t Kp, float32_t Ki, float32_t Kd,float32_t error , float32_t tol , float32_t UPLIMIT , float32_t DOWNLIMIT);
// Kp Ki Kd:看着给
// error: 误差
// tol:绝对精度要求
// UPLIMIT: 调整上限
// DOWNLIMIT：调整下限
// 具体运算已闭包，直接调用即可

////////////////////////////////////////////////////////

//8. Statistical functions
//8.1 最大值
void DSP_MAX(	float32_t * pSrc,uint32_t blockSize , float32_t * pResult,uint32_t * pIndex );
//  pSrc： 输入
//  blockSize：个数
//	pResult:返回结果
//	pIndex：所在数组下标
//  调用：  DSP_MAX(pSrc,blockSize,&MAX,&index);
//======

//8.2 最小值
void DSP_MIN(	float32_t * pSrc,uint32_t blockSize , float32_t * pResult,uint32_t * pIndex );
//  pSrc： 输入
//  blockSize：个数
//	pResult:返回结果
//	pIndex：所在数组下标
//  调用：  DSP_MIN(pSrc,blockSize,&MIN,&index);
//======

//8.3 平均值
void DSP_MEAN(	float32_t * pSrc,uint32_t blockSize , float32_t * pResult );
//  pSrc： 输入
//  blockSize：个数
//	pResult:返回结果
//  调用：  DSP_MEAN(pSrc,blockSize,&result);
//======

//8.4 p平方和
void DSP_POW(	float32_t * pSrc,uint32_t blockSize , float32_t * pResult );
//  pSrc： 输入
//  blockSize：个数
//	pResult:返回结果
//  调用：  DSP_POW(pSrc,blockSize,&result);
//======

//8.5 均方根
void DSP_RMS(	float32_t * pSrc,uint32_t blockSize , float32_t * pResult );
//  pSrc： 输入
//  blockSize：个数
//	pResult:返回结果
//  调用：  DSP_RMS(pSrc,blockSize,&result);
//======

//8.6 标准差
void DSP_STD(	float32_t * pSrc,uint32_t blockSize , float32_t * pResult );
//  pSrc： 输入
//  blockSize：个数
//	pResult:返回结果
//  调用：  DSP_STD(pSrc,blockSize,&result);
//======

//8.7 方差
void DSP_VAR(	float32_t * pSrc,uint32_t blockSize , float32_t * pResult );
//  pSrc： 输入
//  blockSize：个数
//	pResult:返回结果
//  调用：  DSP_VAR(pSrc,blockSize,&result);
//======

////////////////////////////////////////////////////////

//9. Support functions
//1. 复制
void DSP_COPY(float32_t *pSrc ,float32_t *pDst ,uint32_t blockSize);
//	pSrc：输入 数字/数组
//	pDst：结果
//	blockSize： 数组大小，1 则为单个数字
//  调用：DSP_COPY(pSrc,pDst,blockSize);
//======

//2. 填充
////////////////////////////////////////////////////////
void DSP_FILL(float32_t value ,float32_t *pDst ,uint32_t blockSize);
//	value:填充值
//	pDst：结果
//	blockSize： 数组大小，1 则为单个数字
//  调用：DSP_FILL(value,pDst,blockSize);
//======

//10. Interpolation functions

//1. 线性差值
void DSP_LIN(uint32_t nValue, float32_t x1 , float32_t xSpacing , float32_t * 	pYData , float32_t * x , uint32_t blockSize);
//	nValue: ydata个数
//	x1：ydata横坐标起始值
//	xSpacing： ydata坐标步长
//	pYData：ydata
//  x：差值横坐标数组（不得超过ydata横坐标范围）；同时作为输出
//  blockSize: x数组长度
//  调用：DSP_LIN(nValue, x1 , xSpacing , 	pYData , x , blockSize);
//======





//===================================================综合函数========================================

float32_t DSP_FILT(float32_t pSrc);
//	pSrc: 输入值
//	返回滤波值
//  调用：DSP_FILT(pSrc);
//说明：
//	FIFO平滑滤波；可用于数值处理；运行后不可重置
//======





#endif /* _ARM_CMSIS_DSP_H_ */


/*
//===============================================测试脚本.c==================================================


#include "adc.h"        //TIM6->DAC1+DMA1
#include "dac.h"        //TIM2->ADC1(ch0/ch1)+DMA2
#include "lcd.h"        //FSMC->LCD
#include "uart.h"        //USART1/UART4
#include "fft.h"        //FFT
#include "spi.h"        //spi2/spi3
#include "i2c.h"        //i2c3
#include "w25qxx.h"        //spi3->flash
#include "key.h"        //key
#include "led.h"        //led
#include "math.h"
#include "AD9833.h"			//DDS
#include "pid.h"
#include "touch.h"		//lcd触摸
#include "xpt2046.h"	//电阻屏坐标
#include "pwm.h"
#include "stm32f4xx.h"
#include "freqmeas.h"

#include "convolution.h" //卷积计算
#include "arm_cmsis_dsp.h" //DSP计算库	
//=================================================定义area=========================================

#define blockSize 5 
float32_t  pSrc[blockSize] = {1,3,5,7,9};
float32_t  pSrcA[blockSize] = {1.5,2.5,-3,4.5,6};
float32_t  pSrcB[blockSize] = {5,4,3,2,1};
float32_t  pDst[blockSize];
float32_t  tA = 1;
float32_t  tB = -5;
float32_t  tmp;
float32_t  OFFSET = 3.71;

float32_t  MAX;
uint32_t   MAXindex;


q31_t INPUT[blockSize] = {10,20,30,40,50};
int8_t shiftBits = 2;
q31_t OUTPUT[blockSize];

float32_t  pSrc_CM[2*blockSize] = {1,0,1,1,2,1,3,0,0,4};
float32_t  pSrcA_CM[2*blockSize-1] = {1,3,0.1,1,2,1,3,5,-2.2};
float32_t  pSrcB_CM[2*blockSize-1] = {-1,7,12,1,2,1,-3,0.2,9};
float32_t  pDst_CM[2*blockSize-1];
float32_t  rS;
float32_t  iS;


float32_t  setV = 2642;//code 
float32_t  PID_error;
float32_t  PID_adjust;
float32_t  PID_tol = 5 ;
float32_t  Kp = 0.1;
float32_t  Ki = 0.01;
float32_t  Kd = 0.01;
float32_t  UPLIMIT = 800;
float32_t  DOWNLIMIT = -800;

#define  CONVSIZE  5
float32_t CONVA[CONVSIZE] = {1,2,3,4,5};
float32_t CONVB[CONVSIZE] = {1,1,2,2,3};
float32_t CONV[CONVSIZE*2-1];


float32_t FIFO_TEST[40] = { 2.2449 , 2.3790 , 2.2703 , 2.2156 , 2.6337,2.7997,2.2085,2.8963,2.7457,2.5364,2.9700,2.5649,2.2177,2.8578,2.8620, 2.3133,2.3270,2.8400, 2.4925,
2.2449 , 2.3790 , 2.2703 , 2.2156 , 2.6337,2.7997,2.2085,2.8963,2.7457,2.5364,2.9700,2.5649,2.2177,2.8578,2.8620, 2.3133,2.3270,2.8400, 2.4925,2.9700,2.5649};

float32_t FIFO_TEST_OUT[40];
	
	
#define TEST_LENGTH_SAMPLES 1024
float32_t testInput[TEST_LENGTH_SAMPLES];
float32_t testOUTput[TEST_LENGTH_SAMPLES];
uint16_t sP = 1024;
uint32_t BS = 64;
uint16_t BL = 43;
float32_t B[43] = {
  0.0009822167548855, 0.001238465937261, 0.001611887399054,  0.00161034126344,
   0.000962662252028,-0.0005412716112432, -0.00295459674958,-0.006086113974358,
  -0.009444986394931, -0.01224409290652,  -0.0134751485071, -0.01205596041209,
  -0.007029008883373,  0.00222162324817,  0.01577853316867,  0.03307201999975,
    0.05286813978916,  0.07337948701852,  0.09248663361053,   0.1080454587276,
     0.1182136675606,   0.1217493641686,   0.1182136675606,   0.1080454587276,
    0.09248663361053,  0.07337948701852,  0.05286813978916,  0.03307201999975,
    0.01577853316867,  0.00222162324817,-0.007029008883373, -0.01205596041209,
    -0.0134751485071, -0.01224409290652,-0.009444986394931,-0.006086113974358,
   -0.00295459674958,-0.0005412716112432, 0.000962662252028,  0.00161034126344,
   0.001611887399054, 0.001238465937261,0.0009822167548855
};

uint16_t HL = 35;
float32_t H[35] = {
   0.005185762230549,  0.00502513575715, 0.006607441526758,  0.00768434679692,
   0.007801873626999, 0.006487506171134, 0.003326995925685,-0.001985149050759,
  -0.009562191489692, -0.01928817760534,  -0.0308076154283, -0.04350967880607,
   -0.05659338623778, -0.06912826750352,  -0.0801535521815, -0.08876907788733,
   -0.09425711852974,   0.9038595166657, -0.09425711852974, -0.08876907788733,
    -0.0801535521815, -0.06912826750352, -0.05659338623778, -0.04350967880607,
    -0.0308076154283, -0.01928817760534,-0.009562191489692,-0.001985149050759,
   0.003326995925685, 0.006487506171134, 0.007801873626999,  0.00768434679692,
   0.006607441526758,  0.00502513575715, 0.005185762230549
};
	
//===============注意，转移时需要复制 arm_cmsis_dsp.c/.h  ;   math_helper.c / .h 以及mian.c 的#include "arm_cmsis_dsp.h" 

//======================================================================================================
int main(void)
{       
			delay_init();
			uart4_init(115200);        //UART4初始化，波特率115200
			//SPI2_Init();
			KEY_Init();
			AD9833_Init();
			AD9833_AmpSet(100);
			LED_Init();
			W25QXX_Init();        //W25QXX初始化
	//  I2C3_Init();
			ADC1_Init();        //ADC1初始化
			ADC3_Init();
			TIM2_Init(2,41);										//FFT1024点变换，因此采样率1024倍数效果更好
			TIM4_Init(10,21000);
			DAC1_Init();        //DAC1初始化
			LCD_Init();        //初始化LCD FSMC接口
			TIM3_PWM_Init(999,83,0.4);//需要频率-1，时钟分频-1(84MHz)，占空比
			
	
	
			
			printf("begin\n");
			
			
			/*
			for(int i=0;i<blockSize;i++){
				printf("%.2f %.2f\n",pSrcA[i],pSrcB[i]);   
			}
			DSP_ADD(pSrcA,pSrcB,pDst,blockSize);
			printf("abs_result:\n");
			for(int i=0;i<blockSize;i++){
				printf("%.2f\n",pDst[i]);   
			}
			printf("1-5=:\n");
			DSP_ADD(&tA,&tB,&tmp,1);
			printf("-4=%.2f\n",tmp);
			DSP_ABS(&tmp,&tmp,1);
			printf("%.2f\n",tmp);
			*/
			/*
			DSP_CLIP(pSrcA,blockSize,-6,5);
			for(int i=0;i<blockSize;i++){
				printf("%.2f\n",pSrcA[i]);   
			}
			*/
			/*
			DSP_DOT(pSrcA,pSrcB, blockSize ,&tmp);
			printf("%.2f\n",tmp);  
			*/
			
			//DSP_MULT(pSrcA,pSrcB,pDst,blockSize);
			
			/*
			DSP_NEG( pSrc, pDst ,blockSize );	
			DSP_OFFSET(pSrc, OFFSET ,pDst,blockSize);
			DSP_SCALE(pSrc,OFFSET,pDst,blockSize);
			DSP_SHIFT(INPUT,shiftBits,OUTPUT,blockSize);
			DSP_SUB(pSrcA,pSrcB,pDst,blockSize);*/
			//for(int i=0;i<blockSize;i++){
				//printf("%.2f\n",pDst[i]);   
			//}			
			//tmp = DSP_SIN(2);
		
			//DSP_SQRT(554.64,&tmp);
			//printf("%.2f",tmp);
			/*DSP_CMPLX_CONJ(pSrc_CM,pDst_CM,blockSize);
			for(int i=0;i<blockSize*2;i+=2){
				printf("%.2f , %.2f\n",pDst_CM[i],pDst_CM[i+1]);   
			}	*/
			/*DSP_CMPLX_DOT(pSrcA_CM,pSrcB_CM,blockSize,&rS,&iS);
			printf("%.2f , %.2f\n",rS,iS); */  
			//DSP_CMPLX_MAG_SQ(pSrc_CM,pDst_CM,blockSize);
			//DSP_CM_MUL_RE(pSrcA_CM,pSrc,pDst_CM,blockSize);
			//printf("%.2f , %.2f\n",rS,iS); 
			
			
			
			
			
			//DSP_MAT_ADD(pSrcA_CM,pSrcB_CM,pDst_CM,2,5);
			//DSP_MAT_SUB(pSrcA_CM,pSrcB_CM,pDst_CM,2,5);
			//DSP_MAT_MUL(pSrcA_CM,pSrcB_CM,pDst_CM,3,3);
			/*for(int i=0;i<blockSize*2-1;i=i+3){
				printf("%.2f , %.2f ,  %.2f  \n",pDst_CM[i],pDst_CM[i+1],pDst_CM[i+2]);  
			}	*/
			//DSP_MAT_SCA(pSrcB_CM,5.4,pDst_CM,2,5);
			
			/*for(int i=0;i<blockSize*2-1;i=i+3){
				printf("%.2f , %.2f ,  %.2f \n",pSrcA_CM[i],pSrcA_CM[i+1],pSrcA_CM[i+2]);  
			}
			//DSP_MAT_INV(pSrcA_CM,pDst_CM,3,3);
			DSP_MAT_TRA(pSrcA_CM,pDst_CM,3,3);
			for(int i=0;i<blockSize*2-1;i=i+3){
				printf("%.4f , %.4f ,  %.4f \n",pDst_CM[i],pDst_CM[i+1],pDst_CM[i+2]);  
			}*/
			//DSP_MAX(pSrc,blockSize,&MAX,&MAXindex);
			//DSP_MIN(pSrc,blockSize,&MAX,&MAXindex);
			//DSP_MEAN(pSrc,blockSize,&MAX);
			//DSP_POW(pSrc,blockSize,&MAX);
			//DSP_RMS(pSrc,blockSize,&MAX);
			//DSP_STD(pSrc,blockSize,&MAX);
			//DSP_VAR(pSrc,blockSize,&MAX);
			//printf("%.4f \n",MAX); 
			
			
			//DSP_COPY(pSrc,pDst,blockSize);
			//DSP_FILL(5.32,pDst,blockSize);
			//for(int i=0;i<blockSize;i++){
				//printf("%.2f\n",pDst[i]);   
			//}
			/*DSP_LIN(blockSize,1,1,pSrc,pSrcA,blockSize);
			for(int i=0;i<blockSize;i++){
				printf("%.2f\n",pSrcA[i]);   
			}
			printf("done\n");*/
			
					
			
			
			//DSP_CONV(CONVA,CONVSIZE,CONVB,CONVSIZE,CONV);
			//DSP_CONV_P(CONVA,CONVSIZE,CONVB,CONVSIZE,CONV,2,2);//只有对应 索引 有值
			//DSP_CORR(CONVA,CONVSIZE,CONVB,CONVSIZE,CONV);
			
			//for(int i=0;i<CONVSIZE*2-1;i++){
				//printf("%.2f\n",CONV[i]);   
			//}
			
			//DAC_Data_Tx = 2642;				//初值
			
			/*for(int i = 0 ;i < 40 ; i++){		//FIFO滤波测试
				FIFO_TEST_OUT[i] = DSP_FILT(FIFO_TEST[i]);
			}
			for(int i = 0 ;i < 40 ; i++){
				printf("%.4f,",FIFO_TEST_OUT[i]);
			}*/
			
			
			/*for (uint32_t i = 0; i < TEST_LENGTH_SAMPLES; i++) {
        testInput[i] =DSP_SIN(2 * PI * 500 * i / 50000 ) + DSP_SIN(2 * PI * 5000 * i / 50000 ) ;
			}	
			for(int i = 0 ;i < TEST_LENGTH_SAMPLES ; i++){
				printf("%.4f,",testInput[i]);
			}
			printf(" \n");
			printf(" \n");
			printf(" \n");
			printf(" \n");
			DSP_FIR_FIL( testInput, testOUTput ,sP , BS , B , BL);
			//DSP_FIR_FIL( testInput, testOUTput ,sP , BS , H , HL);
			for(int i = 0 ;i < TEST_LENGTH_SAMPLES ; i++){
				printf("%.4f,",testOUTput[i]);
			}*/
			
	//while(1)
       // {					
						//PID_error = setV - ADC1_Data_Rx[0]; 
						//PID_adjust = DSP_PID(Kp, Ki, Kd, PID_error ,PID_tol , UPLIMIT , DOWNLIMIT);
						//DAC_Data_Tx = setV + PID_adjust;
						//printf("DAC_Data_Tx=%d\n",DAC_Data_Tx);
						//delay_ms(10);
				//		delay_ms(33);        //约30帧
						//delay_ms(250);        //约4帧
						//delay_ms(333);        //约3帧
            //delay_ms(1000);        //约1帧
       // }
//}

