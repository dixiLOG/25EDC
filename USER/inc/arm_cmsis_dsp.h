/*********************************************************************************
**********************************************************************************
* �ļ�����: arm_cmsis_dsp.h
* �ļ�������DSP	
* �������ڣ�2024.07.24
* ���ߣ�hzl
* ˵    ���� DSP ����������� �� �ۺϺ���
�ο���ַ��
https://arm-software.github.io/CMSIS_6/latest/DSP/index.html
https://github.com/ARM-software/CMSIS-DSP
https://arm-software.github.io/CMSIS-DSP/v1.10.1/group__groupExamples.html

//===============ע�⣬ת��ʱ��Ҫ���� arm_cmsis_dsp.c/.h  ;   math_helper.c / .h �Լ�mian.c ��#include "arm_cmsis_dsp.h" 
info:
1. basic math functions
    1.1 ȡ����ֵ		DSP_ABS
		1.2 �ӷ�			DSP_ADD
		1.3 �������		DSP_CLIP
		1.4 ���			DSP_DOT
		1.5 �˷�			DSP_MULT
		1.6 ȡ��  		DSP_NEG
		1.7 ƫ�� 			DSP_OFFSET
		1.8 �������� 	DSP_SCALE
		1.9 λ�� << Offset  	DSP_SHIFT
		1.10 ����			DSP_SUB
2. fast math functions
		2.1 cos				DSP_COS
		2.2 sin				DSP_SIN
		2.3 ����			DSP_SQRT
3. complex math fuctions
		3.1 ����								DSP_CMPLX_CONJ
		3.2 ���								DSP_CMPLX_DOT
		3.3 ȡģ								DSP_CMPLX_MAG
		3.4 ģ��ƽ��							DSP_CMPLX_MAG_SQ
		3.5 �˷�								DSP_CM_MUL_CM
		3.6 ������ʵ�����				DSP_CM_MUL_RE
4. filtering functions
		4.1 ����ֱ�� I �ͽṹ��˫���׼��� IIR �˲���<TODO>
		4.2 ����ֱ���� II ת�ýṹ��˫���׼��� IIR �˲���<TODO>
		4.3 ����������Ӧ (FIR) �˲���		DSP_FIR_FIL
		4.4 ���												DSP_CONV
		4.5 ������											DSP_CONV_P
		4.6 �����												DSP_CORR
		4.7 �²�����δ��֤��							DSP_FIR_DECIMATE
		4.8	�ϲ�����δ��֤��							DSP_FIR_INTERPOLATE
		4.9	LMS����Ӧ�˲�								DSP_LMS_FILTER	
		4.10 NLMS												DSP_LMS_NORM_FILTER
5. Matrix functions
		5.1 �����ʼ��			DSP_MAT_INIT
		5.2 �ӷ�					DSP_MAT_ADD
		5.3 ����					DSP_MAT_SUB
		5.4 �˷�					DSP_MAT_MUL
		5.5 �����任				DSP_MAT_SCA	
		5.6 ����					DSP_MAT_INV
		5.7 ת��					DSP_MAT_TRA
6. Transforms
		6.1 cfft
		6.2 DCT ��TODO��	ͼ��ѹ��JPEG
		6.3 MFCC ��TODO��	������������
		6.4 rfft
7. Control functions
		7.1 PID						DSP_PID
				������PID���ƺ���������demo����ӻ����ơ�
		7.2 ������������FOC��TODO��
				Park | Clark ���棩�任
		7.3 ͬʱ���sin&cos��TODO��
8. Statistical functions
		8.1 ���ֵ					DSP_MAX
		8.2 ��Сֵ					DSP_MIN
		8.3 ƽ��ֵ					DSP_MEAN
		8.4 ƽ����					DSP_POW
		8.5 ������					DSP_RMS
		8.6 ��׼��					DSP_STD
		8.7 ����					DSP_VAR
9. Support functions
		1. ����						DSP_COPY
		2. ���						DSP_FILL
10. Interpolation functions
		1. ���Բ�ֵ				DSP_LIN
		2. ��ά���Բ�ֵ��TODO��
//////////////////////////////	
11. �Զ��庯��
	1. ��FIFO�˲�			DSP_FILT
**********************************************************************************
*********************************************************************************/

//========================================ATTATION==========================================

/* �����ÿһ�㶼�ҳ���Ҫ���� */

/*
	����������뺯���������£�
		DSP_ADD(pSrcA,pSrcB,pDst,blockSize);
	���������ֱ��������÷�ʽ���£�����Ҫ���÷���
		DSP_ADD(&pSrcA,&pSrcB,&pDst,blockSize);
*/

/* ��basic math�� �ĵ��������������������Ϊͬһ����������������ַͬ���á� */

/* ��fast math�� ����Ϊ��������������һ��һ�� */

/* ��complex math�� ���ݽ���洢��ż����real,������img*/

/* �������˲������ԣ��߽׵�IIR�˲��������ȶ����߽׵�FIR�˲�����ʹ���������� ; FIR Ϊ������λ�� */

/* ���������㡿��Ҫ��ʼ�����ڡ�ָ�롿������ ;  ����Ϊ��һά���顿 */

//==========================================================================================



#ifndef _ARM_CMSIS_DSP_H_
#define _ARM_CMSIS_DSP_H_

#include "common.h"
#include "arm_math.h"
#include "math_helper.h"		//��Щ����������ļ���
#include "math.h"
#include "lcd.h"
#include "stdio.h"


#define Q12QUARTER	//Q15������
#define Q28QUARTER	//Q31������



//===================================================���庯��========================================

//* -------------------------------------------------------------------------------------------------
//1. basic math functions
//* -------------------------------------------------------------------------------------------------
//1.1 ȡ����ֵ
void DSP_ABS(float32_t *pSrc ,float32_t *pDst ,uint32_t blockSize);
//	pSrc������ ����/����
//	pDst�����
//	blockSize�� �����С��1 ��Ϊ��������
//  ���ã�DSP_ABS(pSrc,pDst,blockSize);
//======

//1.2 �ӷ�
void DSP_ADD(float32_t *pSrcA ,float32_t *pSrcB ,float32_t *pDst ,uint32_t blockSize);
//	pSrcA������ A
//  pSrcB������ B
//	pDst�����
//	blockSize�� �����С��1 ��Ϊ��������
//  ���ã�DSP_ADD(pSrcA,pSrcB,pDst,blockSize);
//======

//1.3 �������
void DSP_CLIP(float *pIn, uint32_t blockSize, float32_t low, float32_t high);
//  pIn:         ����/���
//  numSamples : �����С
//  low:         ����
//  high:        ����
//  ���ã�DSP_CLIP(pSrcA,blockSize,-6,5);
//======

//1.4 ���
void DSP_DOT(float32_t *pSrcA ,float32_t *pSrcB ,uint32_t blockSize,float32_t * result);
//  pSrcA�� A
//  pSrcB�� B
//  blockSize�� �����С
//  result�����
//  ���ã�DSP_DOT(pSrcA,pSrcB, blockSize ,&tmp);
//======

//1.5 �˷�
void DSP_MULT(float32_t *pSrcA ,float32_t *pSrcB ,float32_t *pDst ,uint32_t blockSize);
//  pSrcA������ A
//  pSrcB������ B
//  pDst�����
//  blockSize�� �����С
//  ���ã�DSP_MULT(pSrcA,pSrcB,pDst,blockSize);
//======

//1.6 ȡ��
void DSP_NEG(float32_t *pSrc ,float32_t *pDst ,uint32_t blockSize);
//  pSrc������
//  pDst�����
//  blockSize�� �����С
//  ���ã�DSP_NEG(pSrc,pDst,blockSize);
//======

//1.7 ƫ��
void DSP_OFFSET(float32_t *pSrc ,float32_t offset ,float32_t *pDst ,uint32_t blockSize);
//  pSrc������
//  offset:ƫ��
//  pDst�� ���
//  blockSize�� �����С
//  ���ã�DSP_OFFSET(pSrc, offset ,pDst,blockSize);
//======

//1.8 ��������
void DSP_SCALE(float32_t *pSrc ,float32_t scale,float32_t *pDst ,uint32_t blockSize);
//  pSrc������
//  scale:����
//  pDst�� ���
//  blockSize�� �����С
//  ���ã�DSP_SCALE(pSrc,scale,pDst,blockSize);
//======

//1.9 ��λ
void DSP_SHIFT(q31_t *pSrc ,int8_t shiftBits,q31_t *pDst ,uint32_t blockSize);
//  pSrc������
//  shiftBits:�ƶ�λ����������
//  pDst�� ���
//  blockSize�� �����С
//  ���ã�DSP_SHIFT(pSrc, shiftBits , pDst ,blockSize);
//======

//1.10 ����
void DSP_SUB(float32_t *pSrcA ,float32_t *pSrcB ,float32_t *pDst ,uint32_t blockSize);
//        pSrcA�������� A
//        pSrcB������ B
//        pDst�����
//        blockSize�� �����С
//  ���ã� DSP_SUB(pSrcA,pSrcB,pDst,blockSize);
//======

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//* -------------------------------------------------------------------------------------------------
//2. fast math
//* -------------------------------------------------------------------------------------------------
//2.1 cos
float32_t  DSP_COS(float32_t x);
//  x�� ����,rad
//  ���ã�DSP_COS(x);
//======

//2.2 sin
float32_t  DSP_SIN(float32_t x);
//  x�� ����,rad
//  ���������ź�
//	  for (uint32_t i = 0; i < TEST_LENGTH_SAMPLES; i++) {
//        testInput[i] = ��ֵA * DSP_SIN(2 * PI * Ƶ��freq * i / ������[HZ] ) ;
//    }
//  ���ã�DSP_SIN(x);
//======

//2.3 sqrt 
void  DSP_SQRT(float32_t x,float32_t * result);
//  x�� ����
//  result:���
//  ���ã�  DSP_SQRT(x,&result);
//======

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//* -------------------------------------------------------------------------------------------------
//3.complex math
//* -------------------------------------------------------------------------------------------------

//3.1 ����
void DSP_CMPLX_CONJ(float32_t *pSrc ,float32_t *pDst ,uint32_t blockSize);
//  pSrc������
//  pDst�� ���
//  blockSize�� ��������
//  ���ã�  DSP_CMPLX_CONJ(pSrc,pDst,blockSize);
//======

//3.2 ���
void DSP_CMPLX_DOT(float32_t *pSrcA ,float32_t *pSrcB , uint32_t blockSize , float32_t * realresult,  float32_t * imagresult);
//  pSrcA��A����
//  pSrcB��B����
//  blockSize�� ��������
//  realresult: ʵ�����
//  imageresult:�鲿���
//  ���ã�  DSP_CMPLX_DOT(pSrcA,pSrcB,blockSize,&realresult,&imagresult);
//======

//3.3 ȡģ
void DSP_CMPLX_MAG(float32_t *pSrc ,float32_t *pDst ,uint32_t blockSize);
//  pSrc������
//  pDst�� ���
//  blockSize�� ��������
//  ���ã�  DSP_CMPLX_MAG(pSrc,pDst,blockSize);
//======

//3.4 ģ��ƽ��
void DSP_CMPLX_MAG_SQ(float32_t *pSrc ,float32_t *pDst ,uint32_t blockSize);
//  pSrc������
//  pDst�� ���
//  blockSize�� ��������
//  ���ã�  DSP_CMPLX_MAG_SQ(pSrc,pDst,blockSize);
//======

//3.5 �˷�
void DSP_CM_MUL_CM(float32_t *pSrcA ,float32_t *pSrcB , float32_t *pDst  , uint32_t blockSize);
//  pSrcA��A����
//  pSrcB��B����
//  pDst�� ���
//  blockSize�� ��������
//  ���ã�  DSP_CM_MUL_CM(pSrcA,pSrcB,pDst,blockSize);
//======

//3.6 ������ʵ�����
void DSP_CM_MUL_RE(float32_t *pSrcA ,float32_t *pSrcB , float32_t *pDst  , uint32_t blockSize);
//  pSrcA������A����
//  pSrcB��ʵ��B����
//  pDst�� ���(����)
//  blockSize�� ��������
//  ���ã�  DSP_CM_MUL_RE(pSrcA,pSrcB,pDst,blockSize);
//======

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//* -------------------------------------------------------------------------------------------------
//4. filtering functions
//* -------------------------------------------------------------------------------------------------

//�ο���<FIR ����> 	https://www.cnblogs.com/Jamesjiang/p/8986632.html
//		 <FIR ���> 	https://shequ.stmicroelectronics.cn/thread-601106-1-1.html
//		 <IIR ���> 	https://shequ.stmicroelectronics.cn/thread-601141-1-1.html
//		 <IIR ���>   https://www.cnblogs.com/embInn/p/12904847.html
//		 <NMLS ���><��Ҫ�趨���Σ�����>  https://www.cnblogs.com/armfly/p/15320857.html
//
//4.1 ֱ�� I �ͽṹ˫���׼��� IIR �˲���
// <TODO>
//======

//4.2 ֱ�� II �ͽṹ˫���׼��� IIR �˲���
// <TODO>
//======

//4.3 ����������Ӧ (FIR) �˲���
void DSP_FIR_FIL( float32_t * 	pSrc , float32_t * 	pDst , uint16_t sampleP , uint32_t blockSize , float32_t * pCoeffs , uint16_t numTaps );
// pSrc:�����ź�
// pDst:����ź�
// sampleP:��������(�����źų���)
// blockSize:ÿ�δ������
// pCoeffs:�˲���ϵ������(MATLAB -> filterDesigner)
// numTaps:�˲���ϵ������
//======

//4.4 ���
void DSP_CONV( float32_t * 	pSrcA, uint32_t 	srcALen, float32_t * 	pSrcB, uint32_t 	srcBLen, float32_t * 	pDst );
//  pSrcA: A �ź�
//  srcALen: A �ź�ʱ����
//  pSrcB: B �ź�
//  srcBLen: B �ź�ʱ����
//	pDst: �������������� = srcALen + srcBLen - 1
//  ���ã�  DSP_CONV(pSrcA,srcALen,pSrcB,srcBLen,pDst);
//======

//4.5 ������
void DSP_CONV_P( float32_t * 	pSrcA, uint32_t 	srcALen, float32_t * 	pSrcB, uint32_t 	srcBLen, float32_t * 	pDst , uint32_t first_index , uint32_t numP );
//  pSrcA: A �ź�
//  srcALen: A �ź�ʱ����
//  pSrcB: B �ź�
//  srcBLen: B �ź�ʱ����
//	pDst: �������������� = srcALen + srcBLen - 1
//	first_index:��ʼλ��
//  numP��ץȡ������[first_index, ..., first_index+numP-1]
//	//�������������Χ�� [0 , srcALen+srcBLen-2]//
//  ���ã�  DSP_CONV_P(pSrcA,srcALen,pSrcB,srcBLen,pDst,first_index,numP);
//======

//4.6 �����
void DSP_CORR( float32_t * 	pSrcA, uint32_t 	srcALen, float32_t * 	pSrcB, uint32_t 	srcBLen, float32_t * 	pDst );
//  pSrcA: A �ź�
//  srcALen: A �ź�ʱ����
//  pSrcB: B �ź�
//  srcBLen: B �ź�ʱ����
//	pDst: ����������� = 2 * MAX(srcALen, srcBLen) - 1
//	��ʼ����Ҫ��pDst����
//  ���ã�  DSP_CONV(pSrcA,srcALen,pSrcB,srcBLen,pDst);
//======

// 4.7 �²��� FIR �˲��� (FIR Decimation)��δ��֤��
void DSP_FIR_DECIMATE( float32_t *pSrc , float32_t *pDst , uint16_t inputLen , uint32_t blockSize , float32_t *pCoeffs , uint16_t numTaps , uint8_t M );
// pSrc: �����ź�
// pDst: ����ź�
// inputLen: �����ź��ܳ���
// blockSize: ÿ�δ������
// pCoeffs: FIR�˲���ϵ�����飨MATLAB ��ƣ�
// numTaps: �˲���ϵ������
// M: �²������ӣ�����ÿ M �������е� 1 ����

// 4.8 �ϲ��� FIR ��ֵ�� (FIR Interpolation)��δ��֤��
void DSP_FIR_INTERPOLATE( float32_t *pSrc , float32_t *pDst , uint16_t inputLen , uint32_t blockSize , float32_t *pCoeffs , uint16_t numTaps , uint8_t L );
// pSrc: �����ź�
// pDst: ����ź�
// inputLen: �����ź��ܳ���
// blockSize: ÿ�δ������
// pCoeffs: FIR�˲���ϵ�����飨MATLAB ��ƣ�
// numTaps: �˲���ϵ��������ӦΪ L ����������
// L: ��ֵ���ӣ�ÿ�������������� L-1 ������˲���
//======

// 4.9 LMS ����Ӧ�˲��� (LMS Adaptive Filter)
void DSP_LMS_FILTER( float32_t *pSrc , float32_t *pRef , float32_t *pDst , float32_t *err , uint16_t inputLen , float32_t *pCoeffs , uint16_t numTaps , float32_t mu, float32_t *state);
// pSrc: �����źţ������ź�/�ο��źţ�
// pRef: �����źţ�Ŀ���źţ�
// pDst: �˲�������ź�
// err: ����ź������pRef - pDst��
// inputLen: �����źų���
// pCoeffs: �˲�����ʼȨ��ϵ���������лᶯ̬������
// numTaps: �˲���������ϵ��������
// mu: �������ӣ����������ٶ����ȶ���
// state:����buffer
// ���ã�DSP_LMS_FILTER(refInput, testInput, lmsOutput, errorOutput, TEST_LENGTH_SAMPLES,fir_coeffs, NUM_TAPS, mu,lms_state);
//======

// 4.10 ��һ�� LMS ����Ӧ�˲��� (NLMS)
void DSP_LMS_NORM_FILTER( float32_t *pSrc , float32_t *pRef , float32_t *pDst , float32_t *err , uint16_t inputLen , float32_t *pCoeffs , uint16_t numTaps , float32_t mu,float32_t *state);
// pSrc: �����źţ������ź�/�ο��źţ�
// pRef: �����źţ�Ŀ���źţ�
// pDst: �˲�������ź�
// err: ����ź������pRef - pDst��
// inputLen: �����źų���
// pCoeffs: �˲�����ʼȨ��ϵ���������лᶯ̬������
// numTaps: �˲���������ϵ��������
// mu: �������ӣ����Ƹ������ʣ�
// state:����buffer
// DSP_LMS_NORM_FILTER(refInput, testInput, lmsOutput, errorOutput, TEST_LENGTH_SAMPLES,fir_coeffs, NUM_TAPS, mu,lms_state);
//======

// 4.11 �˲�ǰ��ģ�ͻ���
void DSP_LMS_DRAW(float32_t *in_buf, float32_t *out_buf, float32_t y_mid, float32_t y_range);
// in_buf:�˲�ǰ�ź�
// out_buf:�˲����ź�
// y_mid:����λ��
// y_range:��ֵ��Χ
// ����:DSP_LMS_DRAW(y_in_buf, y_out_buf, 0.0f, 4.0f);  // y������0����2��Χ
/* һ��demo
	// LMS ϵ����ʼ��
	memset(fir_coeffs, 0, sizeof(fir_coeffs));

	for (uint32_t i = 0; i < TEST_LENGTH_SAMPLES; i++) {
			testInput[i] = DSP_SIN(2 * PI * 500 * i / 50000 ) + DSP_SIN(2 * PI * 5000 * i / 50000 ) ;
			refInput[i] = DSP_SIN(2 * PI * 5000 * i / 50000 );
		}	

		DSP_LMS_FILTER(refInput, testInput, lmsOutput, errorOutput, TEST_LENGTH_SAMPLES,fir_coeffs, NUM_TAPS, mu,lms_state);
		//DSP_LMS_NORM_FILTER(refInput, testInput, lmsOutput, errorOutput, TEST_LENGTH_SAMPLES,fir_coeffs, NUM_TAPS, mu,lms_state);
		
	// ��ȡ�м�һ�β������ڻ�ͼ����100~330�㣬��230�㣩
	for (int i = 0; i < Y_BUF_SIZE; i++) {
			y_in_buf[i] = testInput[i];
			y_out_buf[i] = errorOutput[i];
	}

	// ���û�ͼ������ִֻ��һ��
	DSP_LMS_DRAW(y_in_buf, y_out_buf, 0.0f, 4.0f);  // y������0����2��Χ
*/
//======

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//* -------------------------------------------------------------------------------------------------
//5. Matrix functions
//* -------------------------------------------------------------------------------------------------

//5.1 �����ʼ��
void DSP_MAT_INIT(arm_matrix_instance_f32 * S, uint16_t 	nRows, uint16_t nColumns, float32_t * pData );	
// S:ʹָ��pData�ľ���ָ�룬����ǰ���� arm_matrix_instance_f32 <name>�� 
// nRows:����
// nColumns:����
// pData:�������
// ���ã�  DSP_MAT_INIT(S,nRows,nColumns,pData);
//======

//5.2 �ӷ�
void DSP_MAT_ADD(float32_t *pSrcA,float32_t *pSrcB ,float32_t *pDst,uint16_t 	nRows, uint16_t nColumns );	
// pSrcA:A ����
// pSrcB:B ����
// pDst:���
// nRows:A/B������Ĭ��һ��
// nColumns:A/B����
// ���е�ָ������հ�
// ���ã�  DSP_MAT_ADD(pSrcA,pSrcB,pDst,nRows,nColumns);
//======


//5.3 ����
void DSP_MAT_SUB(float32_t *pSrcA,float32_t *pSrcB ,float32_t *pDst,uint16_t 	nRows, uint16_t nColumns );	
// pSrcA:A ����
// pSrcB:B ����
// pDst:���
// nRows:A/B������Ĭ��һ��
// nColumns:A/B����
// ���е�ָ������հ�
// ���ã�  DSP_MAT_SUB(pSrcA,pSrcB,pDst,nRows,nColumns);
//======

//5.4 �˷�
void DSP_MAT_MUL(float32_t *pSrcA,float32_t *pSrcB ,float32_t *pDst,uint16_t 	nRows, uint16_t nColumns );	
// pSrcA:A ����
// pSrcB:B ����
// pDst:���
// nRows:A/B������Ĭ��һ��
// nColumns:A/B����
// ���е�ָ������հ�
// ���ã�  DSP_MAT_MUL(pSrcA,pSrcB,pDst,nRows,nColumns);
//======

//5.5 �����任
void DSP_MAT_SCA(float32_t *pSrc, float32_t scale ,float32_t *pDst,uint16_t 	nRows, uint16_t nColumns);	
// pSrc:���� ����
// scale: ���ű���
// pDst:���
// nRows:A/B������Ĭ��һ��
// nColumns:A/B����
// ���е�ָ������հ�
// ���ã�  DSP_MAT_SCA(pSrc,scale,pDst,nRows,nColumns);
//======

//5.6 ����
void DSP_MAT_INV(float32_t *pSrc ,float32_t *pDst,uint16_t 	nRows, uint16_t nColumns);	
// pSrc:���� ����
// pDst:���
// nRows:A/B������Ĭ��һ��;�����Ƿ���
// nColumns:A/B����
// ���е�ָ������հ�
// ���ã�  DSP_MAT_INV(pSrc,pDst,nRows,nColumns);
//======

//5.7 ת��
void DSP_MAT_TRA(float32_t *pSrc ,float32_t *pDst,uint16_t 	nRows, uint16_t nColumns);	
// pSrc:���� ����
// pDst:���
// nRows:A/B������Ĭ��һ��;�����Ƿ���
// nColumns:A/B����
// ���е�ָ������հ�
// ���ã�  DSP_MAT_TRA(pSrc,pDst,nRows,nColumns);
//======

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//* -------------------------------------------------------------------------------------------------
//6. Transforms
//* -------------------------------------------------------------------------------------------------

//�ο���https://arm-software.github.io/CMSIS-DSP/v1.10.1/group__groupTransforms.html
//ע�⣺������֧�� [32, 64, 128, ..., 4096] ���������ȣ�һ��ȡ4096
// ��һ������Ҫ��fftƵ��ʶ����������� fft.c
// DCT��MFCC�ݲ���װ
// ��DCTΪ�������ò���Ϊ��
// 1. �ṹ���ʼ��
// arm_dct4_instance_f32 dct4;
// arm_rfft_instance_f32 rfft;
// arm_cfft_radix4_instance_f32 cfft;
// 2. ��ʼ����arm_dct4_init_f32(&dct4, &rfft, &cfft, N, N/2, 1.0f);
// 3. ���ã�arm_dct4_f32(&dct4, state_buffer, input_buffer);

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//* -------------------------------------------------------------------------------------------------
//7. Control functions
//* -------------------------------------------------------------------------------------------------

//7.1 PID
//�ο���https://www.cnblogs.com/fang-d/articles/CMSIS-DSP_PID_Control.html

#ifndef __PID_CONTROLLER_H
#define __PID_CONTROLLER_H


// PID�������ṹ��
typedef struct {
    arm_pid_instance_f32 ctrl;  // CMSIS PID ������ʵ��
    float32_t tol;              // �����ݲ�
    float32_t upper_limit;      // �����������
    float32_t lower_limit;      // �����������
    float32_t last_output;      // ���������ڱ�����һ�ε����ֵ
} DSP_PID_Controller;
// �ṹ�崴����  DSP_PID_Controller my_pid;
//======


// ��ʼ��PID������
void DSP_PID_Init(DSP_PID_Controller *pid, float32_t Kp, float32_t Ki, float32_t Kd,
                  float32_t tol, float32_t upper_limit, float32_t lower_limit);
// pid:�ṹ��ʵ��
// Kp Ki Kd:�Լ����ŵ�
// tol:������̶�
// upper_limit:�ϱ߽�
// lower_limit:�±߽�
// ���ã�  DSP_PID_Init(&my_pid, Kp, Ki, Kd,PID_tol ,UPLIMIT , DOWNLIMIT);
//======


// ����PID��������ؿ�������
float32_t DSP_PID_Compute(DSP_PID_Controller *pid, float32_t error);
// pid:�ṹ��ʵ��
// error:���
// ע�⣺�����Ϊ�������ֵ����������
// ���ã�  float32_t adjust = DSP_PID_Compute(&my_pid, error);
//======



// ����PID�ڲ�״̬
void DSP_PID_Reset(DSP_PID_Controller *pid);
// ��ʱ����
//======


void DSP_PID_SIMU(float32_t adjust,float32_t *realOut,u8 PID_flag);
// adjust:PID���ֵ
// *realOut:�������ֵ��ַͬ
// PID_flag:��1��Ϊһ��ϵͳ������Ϊ����ϵͳ+�Ŷ�
// ע�⣺����Ϊ���������֤��ʵ����Ŀ�в���Ҫ�˺���
// ���ã�  DSP_PID_SIMU(adjust,&realOut,2);
//======

void DSP_PID_DRAW(float32_t  setV,float32_t realOut);
// setV:�趨ֵ
// realOut:��ADC����������ֵ
// ���ã�  DSP_PID_DRAW(setV,realOut);
//======



#endif
// tol:���Ծ���Ҫ��
// UPLIMIT: ��������
// DOWNLIMIT����������
// ���������ѱհ���ֱ�ӵ��ü���

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//* -------------------------------------------------------------------------------------------------
//8. Statistical functions
//* -------------------------------------------------------------------------------------------------

//8.1 ���ֵ
void DSP_MAX(	float32_t * pSrc,uint32_t blockSize , float32_t * pResult,uint32_t * pIndex );
//  pSrc�� ����
//  blockSize������
//	pResult:���ؽ��
//	pIndex�����������±�
//  ���ã�  DSP_MAX(pSrc,blockSize,&MAX,&index);
//======

//8.2 ��Сֵ
void DSP_MIN(	float32_t * pSrc,uint32_t blockSize , float32_t * pResult,uint32_t * pIndex );
//  pSrc�� ����
//  blockSize������
//	pResult:���ؽ��
//	pIndex�����������±�
//  ���ã�  DSP_MIN(pSrc,blockSize,&MIN,&index);
//======

//8.3 ƽ��ֵ
void DSP_MEAN(	float32_t * pSrc,uint32_t blockSize , float32_t * pResult );
//  pSrc�� ����
//  blockSize������
//	pResult:���ؽ��
//  ���ã�  DSP_MEAN(pSrc,blockSize,&result);
//======

//8.4 ƽ����
void DSP_POW(	float32_t * pSrc,uint32_t blockSize , float32_t * pResult );
//  pSrc�� ����
//  blockSize������
//	pResult:���ؽ��
//  ���ã�  DSP_POW(pSrc,blockSize,&result);
//======

//8.5 ������
void DSP_RMS(	float32_t * pSrc,uint32_t blockSize , float32_t * pResult );
//  pSrc�� ����
//  blockSize������
//	pResult:���ؽ��
//  ���ã�  DSP_RMS(pSrc,blockSize,&result);
//======

//8.6 ��׼��
void DSP_STD(	float32_t * pSrc,uint32_t blockSize , float32_t * pResult );
//  pSrc�� ����
//  blockSize������
//	pResult:���ؽ��
//  ���ã�  DSP_STD(pSrc,blockSize,&result);
//======

//8.7 ����
void DSP_VAR(	float32_t * pSrc,uint32_t blockSize , float32_t * pResult );
//  pSrc�� ����
//  blockSize������
//	pResult:���ؽ��
//  ���ã�  DSP_VAR(pSrc,blockSize,&result);
//======

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//* -------------------------------------------------------------------------------------------------
//9. Support functions
//* -------------------------------------------------------------------------------------------------

//1. ����
void DSP_COPY(float32_t *pSrc ,float32_t *pDst ,uint32_t blockSize);
//	pSrc������ ����/����
//	pDst�����
//	blockSize�� �����С��1 ��Ϊ��������
//  ���ã�DSP_COPY(pSrc,pDst,blockSize);
//======

//2. ���
////////////////////////////////////////////////////////
void DSP_FILL(float32_t value ,float32_t *pDst ,uint32_t blockSize);
//	value:���ֵ
//	pDst�����
//	blockSize�� �����С��1 ��Ϊ��������
//  ���ã�DSP_FILL(value,pDst,blockSize);
//======

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//* -------------------------------------------------------------------------------------------------
//10. Interpolation functions
//* -------------------------------------------------------------------------------------------------

//1. ���Բ�ֵ
void DSP_LIN(uint32_t nValue, float32_t x1 , float32_t xSpacing , float32_t * 	pYData , float32_t * x , uint32_t blockSize);
//	nValue: ydata����
//	x1��ydata��������ʼֵ
//	xSpacing�� ydata���경��
//	pYData��ydata
//  x����ֵ���������飨���ó���ydata�����귶Χ����ͬʱ��Ϊ���
//  blockSize: x���鳤��
//  ���ã�DSP_LIN(nValue, x1 , xSpacing , 	pYData , x , blockSize);
//======


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//===================================================�ۺϺ���========================================

float32_t DSP_FILT(float32_t pSrc);
//	pSrc: ����ֵ
//	�����˲�ֵ
//  ���ã�DSP_FILT(pSrc);
//˵����
//	FIFOƽ���˲�����������ֵ�������к󲻿�����
//======





#endif /* _ARM_CMSIS_DSP_H_ */


/*
//===============================================���Խű�.c==================================================

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
#include "touch.h"		//lcd����
#include "xpt2046.h"	//����������
#include "pwm.h"
#include "stm32f4xx.h"
#include "freqmeas.h"

#include "convolution.h" //�������
#include "arm_cmsis_dsp.h" //DSP�����	
//=================================================����area=========================================

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
	
//-------PID���Բ���-------//
float32_t  setV = 1200;//code 
float32_t  PID_tol = 0.5;
float32_t  Kp = 3;
float32_t  Ki = 0.8;
float32_t  Kd = 0.08;
float32_t  UPLIMIT = 5000;
float32_t  DOWNLIMIT = 0;
float32_t realOut = 1200; 	// ���ض����ʼֵ | �������������ֵ	

//===============ע�⣬ת��ʱ��Ҫ���� arm_cmsis_dsp.c/.h  ;   math_helper.c / .h �Լ�mian.c ��#include "arm_cmsis_dsp.h" 

//======================================================================================================
int main(void)
{       
			delay_init();
			uart4_init(115200);        //UART4��ʼ����������115200
			//SPI2_Init();
			KEY_Init();
			AD9833_Init();
			AD9833_AmpSet(100);
			LED_Init();
			W25QXX_Init();        //W25QXX��ʼ��
	//  I2C3_Init();
			ADC1_Init();        //ADC1��ʼ��
			ADC3_Init();
			TIM2_Init(2,41);										//FFT1024��任����˲�����1024����Ч������
			TIM4_Init(10,21000);
			DAC1_Init();        //DAC1��ʼ��
			LCD_Init();        //��ʼ��LCD FSMC�ӿ�
			TIM3_PWM_Init(999,83,0.4);//��ҪƵ��-1��ʱ�ӷ�Ƶ-1(84MHz)��ռ�ձ�
			
			
			
		// PID��ʼ��
		DSP_PID_Init(&my_pid, Kp, Ki, Kd,PID_tol ,UPLIMIT , DOWNLIMIT);
	
			
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
			//DSP_CONV_P(CONVA,CONVSIZE,CONVB,CONVSIZE,CONV,2,2);//ֻ�ж�Ӧ ���� ��ֵ
			//DSP_CORR(CONVA,CONVSIZE,CONVB,CONVSIZE,CONV);
			
			//for(int i=0;i<CONVSIZE*2-1;i++){
				//printf("%.2f\n",CONV[i]);   
			//}
			
			//DAC_Data_Tx = 2642;				//��ֵ
			
			/*
			// LMS ϵ����ʼ��
			memset(fir_coeffs, 0, sizeof(fir_coeffs));
		
			for (uint32_t i = 0; i < TEST_LENGTH_SAMPLES; i++) {
					testInput[i] = DSP_SIN(2 * PI * 500 * i / 50000 ) + DSP_SIN(2 * PI * 5000 * i / 50000 ) ;
					refInput[i] = DSP_SIN(2 * PI * 5000 * i / 50000 );
				}	

				DSP_LMS_FILTER(refInput, testInput, lmsOutput, errorOutput, TEST_LENGTH_SAMPLES,fir_coeffs, NUM_TAPS, mu,lms_state);
				//DSP_LMS_NORM_FILTER(refInput, testInput, lmsOutput, errorOutput, TEST_LENGTH_SAMPLES,fir_coeffs, NUM_TAPS, mu,lms_state);
				
			// ��ȡ�м�һ�β������ڻ�ͼ����100~330�㣬��230�㣩
			for (int i = 0; i < Y_BUF_SIZE; i++) {
					y_in_buf[i] = testInput[i];
					y_out_buf[i] = errorOutput[i];
			}

			// ���û�ͼ������ִֻ��һ��
			DSP_WAVE_DRAW2(y_in_buf, y_out_buf, 0.0f, 4.0f);  // y������0����2��Χ
			*/
			
			/*for(int i = 0 ;i < 40 ; i++){		//FIFO�˲�����
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
						
						//-------PID����-------//
						/*
						// �������ۿ���ֵ
						float32_t error = setV - realOut;
						float32_t adjust = DSP_PID_Compute(&my_pid, error);
						// PID����ϵͳ����
						DSP_PID_SIMU(adjust,&realOut,2);// 1 or 2
						
						// PID���ƴ�ӡ����� 
						
						sprintf(String, "ADJ=%.3f ,y=%.3f,setV=%.1f",adjust,realOut,setV);
						LCD_DisplayString(10, 150, 12, (u8*)String);
						DSP_PID_DRAW(setV,realOut);*/

						//---------------------//
						
						
						//delay_ms(10);
				//		delay_ms(33);        //Լ30֡
						//delay_ms(250);        //Լ4֡
						//delay_ms(333);        //Լ3֡
            //delay_ms(1000);        //Լ1֡
       // }
//}

