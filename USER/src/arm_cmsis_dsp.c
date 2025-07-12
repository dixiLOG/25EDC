/*********************************************************************************
**********************************************************************************
* 文件名称: arm_cmsis_dsp.h                                                         	     	 *
* 文件简述：DSP								                     		 													 *
* 创建日期：2024.07.24                                                          	 *
* 说    明： DSP 函数运算调用集
* 具体见 .h文件
**********************************************************************************
*********************************************************************************/


#include "arm_cmsis_dsp.h"



void DSP_ABS(float32_t *pSrc ,float32_t *pDst ,uint32_t blockSize){
    arm_abs_f32(pSrc, pDst, blockSize);
}
//================================================//


void DSP_ADD(float32_t *pSrcA ,float32_t *pSrcB ,float32_t *pDst ,uint32_t blockSize){
    arm_add_f32(pSrcA, pSrcB , pDst, blockSize);
}
//================================================//


void DSP_CLIP(float *pIn, uint32_t blockSize, float32_t low, float32_t high){
		arm_clip_f32( pIn , blockSize , low , high );
}
//================================================//


void DSP_DOT(float32_t *pSrcA ,float32_t *pSrcB ,uint32_t blockSize,float32_t * result){
		arm_dot_prod_f32(pSrcA, pSrcB , blockSize, result );
}
//================================================//


void DSP_MULT(float32_t *pSrcA ,float32_t *pSrcB ,float32_t *pDst ,uint32_t blockSize){
		arm_mult_f32 (pSrcA,pSrcB,pDst,blockSize);
}
//================================================//


void DSP_NEG(float32_t *pSrc ,float32_t *pDst ,uint32_t blockSize){
		arm_negate_f32(pSrc,pDst,blockSize);
}
//================================================//

void DSP_OFFSET(float32_t *pSrc ,float32_t offset ,float32_t *pDst ,uint32_t blockSize){
		arm_offset_f32(pSrc, offset ,pDst,blockSize);
}
//================================================//

void DSP_SCALE(float32_t *pSrc ,float32_t scale,float32_t *pDst ,uint32_t blockSize){
		arm_scale_f32(pSrc, scale ,pDst,blockSize);
}
//================================================//

void DSP_SHIFT(q31_t *pSrc ,int8_t shiftBits,q31_t *pDst ,uint32_t blockSize){
	arm_shift_q31 (pSrc, shiftBits , pDst ,blockSize);
}
//================================================//

void DSP_SUB(float32_t *pSrcA ,float32_t *pSrcB ,float32_t *pDst ,uint32_t blockSize){
	arm_sub_f32(pSrcA,pSrcB,pDst,blockSize);
}
//================================================//

float32_t  DSP_COS(float32_t x){
	return arm_cos_f32(x);
}
//================================================//

float32_t  DSP_SIN(float32_t x){
	return arm_sin_f32(x);
}
//================================================//

void  DSP_SQRT(float32_t x,float32_t * result){
		arm_sqrt_f32(x,result);
}
//================================================//

void DSP_CMPLX_CONJ(float32_t *pSrc ,float32_t *pDst ,uint32_t blockSize){
		arm_cmplx_conj_f32(pSrc,pDst,blockSize);
}
//================================================//

void DSP_CMPLX_DOT(float32_t *pSrcA ,float32_t *pSrcB , uint32_t blockSize , float32_t * realresult,  float32_t * imagresult){
		arm_cmplx_dot_prod_f32(pSrcA,pSrcB,blockSize,realresult,imagresult);
}
//================================================//

void DSP_CMPLX_MAG(float32_t *pSrc ,float32_t *pDst ,uint32_t blockSize){
		arm_cmplx_mag_f32(pSrc,pDst,blockSize);
}
//================================================//

void DSP_CMPLX_MAG_SQ(float32_t *pSrc ,float32_t *pDst ,uint32_t blockSize){
		arm_cmplx_mag_squared_f32(pSrc,pDst,blockSize);
}
//================================================//

void DSP_CM_MUL_CM(float32_t *pSrcA ,float32_t *pSrcB , float32_t *pDst  , uint32_t blockSize){
		arm_cmplx_mult_cmplx_f32(pSrcA,pSrcB,pDst,blockSize);
}
//================================================//

void DSP_CM_MUL_RE(float32_t *pSrcA ,float32_t *pSrcB , float32_t *pDst  , uint32_t blockSize){
		arm_cmplx_mult_real_f32(pSrcA,pSrcB,pDst,blockSize);
}
//================================================//






void DSP_MAT_INIT(arm_matrix_instance_f32 * S, uint16_t 	nRows, uint16_t nColumns, float32_t * pData ){
		arm_mat_init_f32(S,nRows,nColumns,pData);
}
//================================================//

void DSP_MAT_ADD(float32_t *pSrcA,float32_t *pSrcB ,float32_t *pDst ,uint16_t 	nRows, uint16_t nColumns){
		arm_matrix_instance_f32 dataA;	//A 矩阵指针
		arm_matrix_instance_f32 dataB;
		arm_matrix_instance_f32 dataD;
		//初始化
		DSP_MAT_INIT(&dataA,nRows,nColumns,pSrcA);
		DSP_MAT_INIT(&dataB,nRows,nColumns,pSrcB);
		DSP_MAT_INIT(&dataD,nRows,nColumns,pDst);
		
		arm_mat_add_f32(&dataA,&dataB,&dataD);
	
}

//================================================//

void DSP_MAT_SUB(float32_t *pSrcA,float32_t *pSrcB ,float32_t *pDst,uint16_t 	nRows, uint16_t nColumns ){
		arm_matrix_instance_f32 dataA;	//A 矩阵指针
		arm_matrix_instance_f32 dataB;
		arm_matrix_instance_f32 dataD;
		//初始化
		DSP_MAT_INIT(&dataA,nRows,nColumns,pSrcA);
		DSP_MAT_INIT(&dataB,nRows,nColumns,pSrcB);
		DSP_MAT_INIT(&dataD,nRows,nColumns,pDst);
		
		arm_mat_sub_f32(&dataA,&dataB,&dataD);		
}

//================================================//
void DSP_MAT_MUL(float32_t *pSrcA,float32_t *pSrcB ,float32_t *pDst,uint16_t 	nRows, uint16_t nColumns ){
		arm_matrix_instance_f32 dataA;	//A 矩阵指针
		arm_matrix_instance_f32 dataB;
		arm_matrix_instance_f32 dataD;
		//初始化
		DSP_MAT_INIT(&dataA,nRows,nColumns,pSrcA);
		DSP_MAT_INIT(&dataB,nRows,nColumns,pSrcB);
		DSP_MAT_INIT(&dataD,nRows,nColumns,pDst);
		
		arm_mat_mult_f32(&dataA,&dataB,&dataD);		
}
//================================================//

void DSP_MAT_SCA(float32_t *pSrc, float32_t scale ,float32_t *pDst,uint16_t 	nRows, uint16_t nColumns ){
		arm_matrix_instance_f32 dataA;	//A 矩阵指针
		arm_matrix_instance_f32 dataD;
		//初始化
		DSP_MAT_INIT(&dataA,nRows,nColumns,pSrc);
		DSP_MAT_INIT(&dataD,nRows,nColumns,pDst);
		
		arm_mat_scale_f32(&dataA,scale,&dataD);		
}
//================================================//

void DSP_MAT_INV(float32_t *pSrc ,float32_t *pDst,uint16_t 	nRows, uint16_t nColumns){
		arm_matrix_instance_f32 dataA;	//A 矩阵指针
		arm_matrix_instance_f32 dataD;
		//初始化
		DSP_MAT_INIT(&dataA,nRows,nColumns,pSrc);
		DSP_MAT_INIT(&dataD,nRows,nColumns,pDst);
		
		arm_mat_inverse_f32	(&dataA,&dataD);		
}
//================================================//

void DSP_MAT_TRA(float32_t *pSrc ,float32_t *pDst,uint16_t 	nRows, uint16_t nColumns){
		arm_matrix_instance_f32 dataA;	//A 矩阵指针
		arm_matrix_instance_f32 dataD;
		//初始化
		DSP_MAT_INIT(&dataA,nRows,nColumns,pSrc);
		DSP_MAT_INIT(&dataD,nRows,nColumns,pDst);
		
		arm_mat_trans_f32	(&dataA,&dataD);
}
//================================================//







void DSP_MAX(	float32_t * pSrc,uint32_t blockSize , float32_t * pResult,uint32_t * pIndex ){
		arm_max_f32(pSrc , blockSize , pResult , pIndex);
}
//================================================//

void DSP_MIN(	float32_t * pSrc,uint32_t blockSize , float32_t * pResult,uint32_t * pIndex ){
		arm_min_f32(pSrc , blockSize , pResult , pIndex);
}
//================================================//

void DSP_MEAN(	float32_t * pSrc,uint32_t blockSize , float32_t * pResult ){
		arm_mean_f32(pSrc , blockSize , pResult );
}
//================================================//

void DSP_POW(	float32_t * pSrc,uint32_t blockSize , float32_t * pResult ){
		arm_power_f32(pSrc , blockSize , pResult );
}
//================================================//

void DSP_RMS(	float32_t * pSrc,uint32_t blockSize , float32_t * pResult ){
		arm_rms_f32(pSrc , blockSize , pResult );
}
//================================================//

void DSP_STD(	float32_t * pSrc,uint32_t blockSize , float32_t * pResult ){
		arm_std_f32(pSrc , blockSize , pResult );
}
//================================================//


void DSP_VAR(	float32_t * pSrc,uint32_t blockSize , float32_t * pResult ){
		arm_var_f32(pSrc , blockSize , pResult );
}
//================================================//


void DSP_COPY(float32_t *pSrc ,float32_t *pDst ,uint32_t blockSize){
    arm_copy_f32(pSrc, pDst, blockSize);
}
//================================================//

void DSP_FILL(float32_t value ,float32_t *pDst ,uint32_t blockSize){
		arm_fill_f32(value , pDst , blockSize);
}
//================================================//

void DSP_LIN(uint32_t nValue, float32_t x1 , float32_t xSpacing , float32_t * 	pYData , float32_t* x, uint32_t blockSize){
		arm_linear_interp_instance_f32 S = {nValue,  x1 , xSpacing, pYData};	//定义结构体S
		for(int i=0;i < blockSize ;i++){
				x[i] = arm_linear_interp_f32(&S, x[i]);
		}
}
//================================================//


float32_t DSP_PID(float32_t Kp, float32_t Ki, float32_t Kd,float32_t error , float32_t tol , float32_t UPLIMIT , float32_t DOWNLIMIT) {
    // 在CMSIS-DSP中的PID实现中，认为Δt = 1，
    // 故需要调整Kp和Ki的值，保证系统在采样率变化时的鲁棒性
	arm_pid_instance_f32 controller = {
        .Kp = Kp,
        .Ki = Ki,    
        .Kd = Kd 
    };
    arm_pid_init_f32(&controller, 1);       // 初始化结构体，要记得清空state数组
		float32_t tmp_error;
		DSP_ABS(&error,&tmp_error,1);
		printf("PID error:%.6f",error);	//打印误差值
	if(	tmp_error > tol ){								//当误差大于设定值
				arm_pid_f32(&controller, error);    // 计算PID的操纵量
        // 进行输出限幅
        if (controller.state[2] > UPLIMIT) {
            controller.state[2] = UPLIMIT;
        } else if (controller.state[2] < DOWNLIMIT) {
            controller.state[2] = DOWNLIMIT;
        }
        // 完成输出限幅，用限幅后的操纵量控制系统
        return controller.state[2];   // 使用最新的操纵量，调整执行元件，实现控制
	}else{
			  return controller.state[2];   // 保持
	}
}
 //================================================//


void DSP_CONV( float32_t * 	pSrcA, uint32_t 	srcALen, float32_t * 	pSrcB, uint32_t 	srcBLen, float32_t * 	pDst ){
		arm_conv_f32(pSrcA,srcALen,pSrcB,srcBLen,pDst);
}
 //================================================//


void DSP_CONV_P( float32_t * 	pSrcA, uint32_t 	srcALen, float32_t * 	pSrcB, uint32_t 	srcBLen, float32_t * 	pDst , uint32_t first_index , uint32_t numP ){
		arm_conv_partial_f32(pSrcA,srcALen,pSrcB,srcBLen,pDst,first_index,numP);
}
 //================================================//

void DSP_CORR( float32_t * 	pSrcA, uint32_t 	srcALen, float32_t * 	pSrcB, uint32_t 	srcBLen, float32_t * 	pDst ){
		arm_correlate_f32(pSrcA,srcALen,pSrcB,srcBLen,pDst);
}
 //================================================//

void DSP_FIR_FIL( float32_t * 	pSrc , float32_t * 	pDst , uint16_t sampleP , uint32_t blockSize , float32_t * pCoeffs , uint16_t numTaps ){
		uint32_t numBlocks = sampleP/blockSize;            // 需要调用arm_fir_f32的次数,整数
		float32_t firStateF32[blockSize + numTaps - 1];    // 状态缓存，大小 numTaps + blockSize - 1
		arm_fir_instance_f32 S;			//声明 S
		/* 初始化结构体S */
		arm_fir_init_f32(&S, numTaps,pCoeffs , firStateF32 , blockSize);
		/* 实现FIR滤波 */
		for (int i=0; i < numBlocks; i++)
		{
			arm_fir_f32(&S, pSrc + (i * blockSize), pDst + (i * blockSize), blockSize);
		}
}
 //================================================//


//-----进ADC滤波-----
//FIFO
#define FILSIZE 8		//定义滤波数组宽度
u8 flag_FIL = 0;			//初始化标志位
u8 pFIL = 0;			//指针
float32_t FIL_data[FILSIZE];	//滤波栅格

float32_t DSP_FILT(float32_t  pSrc){
		float32_t result;
		if ( pFIL < FILSIZE){				//初始化
			FIL_data[pFIL] = pSrc;
			pFIL++;
		}
		else if(pFIL == FILSIZE){
			pFIL = 0;									//指针循环
			flag_FIL = 1;							//初始化结束，标志位置 1
			FIL_data[pFIL] = pSrc;
			pFIL++;;
		}
		
		if(flag_FIL == 1){		//输出 mean result 
			DSP_MEAN(FIL_data,FILSIZE,&result);		
			return result;
		}
		else{							//输出 raw result
			return pSrc;
		}
}
	//----------------	
