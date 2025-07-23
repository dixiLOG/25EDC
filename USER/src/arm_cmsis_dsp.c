/*********************************************************************************
**********************************************************************************
* 文件名称: arm_cmsis_dsp.h                                                         	     	 *
* 文件简述：DSP								                     		 													 *
* 创建日期：2024.07.24                                                          	 *
* 更新日期：2025.07.22
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
		arm_mult_f32(pSrcA,pSrcB,pDst,blockSize);
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

void DSP_SUM(float32_t *pSrcA ,float32_t *sum ,uint32_t blockSize){
	float32_t fill[blockSize];
	DSP_FILL(1,fill,blockSize);
	DSP_DOT(pSrcA,fill,blockSize,sum);
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
		
		arm_mat_trans_f32(&dataA,&dataD);
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

void DSP_MEAN(float32_t * pSrc,uint32_t blockSize , float32_t * pResult ){
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


void DSP_PID_Init(DSP_PID_Controller *pid, float32_t Kp, float32_t Ki, float32_t Kd,
                  float32_t tol, float32_t upper_limit, float32_t lower_limit) {
    pid->ctrl.Kp = Kp;
    pid->ctrl.Ki = Ki;
    pid->ctrl.Kd = Kd;
    pid->tol = tol;
    pid->upper_limit = upper_limit;
    pid->lower_limit = lower_limit;
		pid->last_output = 0.0f; // 初始化 last_output
    // 初始化并清空内部状态
    arm_pid_init_f32(&pid->ctrl, 1);
}

float32_t DSP_PID_Compute(DSP_PID_Controller *pid, float32_t error) {
    float32_t abs_error = fabsf(error);

    if (abs_error < pid->tol) {
				return pid->last_output;	// 保持
    }

    float32_t output = arm_pid_f32(&pid->ctrl, error);

    // 限幅
    if (output >= pid->upper_limit) {
        output = pid->upper_limit - 1;
    } else if (output <= pid->lower_limit) {
        output = pid->lower_limit + 1;
    }
		pid->last_output = output;
    return output;
}

void DSP_PID_Reset(DSP_PID_Controller *pid) {
    arm_pid_reset_f32(&pid->ctrl);
}
 //================================================//


// 模拟二阶系统 + 电源纹波扰动
static float32_t re_prev1 = 0;
static float32_t re_prev2 = 0;
static float32_t t_sim = 0.0f;
// 系统参数
float32_t alpha = 0.05; 		// 惰性系数，越小系统越“慢”
float32_t a1 = 1.6f;
float32_t a2 = -0.64f;
float32_t b = 0.05f;

void DSP_PID_SIMU(float32_t adjust,float32_t *realOut,u8 PID_flag){
		// 扰动仿真（可选）
		if (PID_flag ==1){
		// I 模拟被控对象（简单一阶系统）
			*realOut += alpha * (adjust - *realOut);
		}
		else {
		// I 二阶系统 + 小扰动
			float32_t ripple = 2.0f * sinf(2 * 3.14159f * 5 * t_sim); // 小扰动
			t_sim += 0.001f;
			
			float32_t re_new = a1 * re_prev1 + a2 * re_prev2 + b * adjust + ripple;
			re_prev2 = re_prev1;
			re_prev1 = re_new;
			*realOut = re_new;
		}
}

//================================================//

#define Y_BUF_SIZE 231
float32_t y_buff[Y_BUF_SIZE] = {0}; // 显示历史realOut

void DSP_PID_DRAW(float32_t  setV,float32_t realOut){
//--- 更新缓冲区：将新的 realOut 加入波形数组 ---//	
		for (int i = 0; i < Y_BUF_SIZE - 1; i++) {
				y_buff[i] = y_buff[i + 1]; // 向左平移
		}
		y_buff[Y_BUF_SIZE - 1] = realOut; // 加入最新值

		// 设置显示区间中心为 setV，范围 ±200（可根据情况调整）
		float32_t y_mid = setV;
		float32_t y_range = 400.0f;  // 固定上下200的可视范围

		//--- 清屏 & 画边框 ---//
		LCD_Fill_onecolor(5, 190, 235, 315, WHITE);
		LCD_Color_DrawRectangle(5, 188, 235, 315, BLACK); // 绘制边框

		//--- 画 X 轴刻度线 ---//
		for (u16 x = 0; x <= 230; x++) {
				if (x % 10 == 0)
						LCD_Color_DrawLine(5 + x, 315, 5 + x, 320, RED);
				else if (x % 2 == 0)
						LCD_Color_DrawLine(5 + x, 315, 5 + x, 317, BLACK);
		}

		//--- 绘制 y(t) 曲线 ---//
		for (int i = 1; i < Y_BUF_SIZE; i++) {
				int x0 = 5 + i - 1;
				int x1 = 5 + i;
				float32_t y0_val = y_buff[i - 1];
				float32_t y1_val = y_buff[i];

				int y0 = 315 - (int)(100 * (y0_val - (y_mid - y_range / 2)) / y_range);
				int y1 = 315 - (int)(100 * (y1_val - (y_mid - y_range / 2)) / y_range);

				// 裁剪边界，避免越界绘图
				if (y0 < 190) y0 = 190; if (y0 > 315) y0 = 315;
				if (y1 < 190) y1 = 190; if (y1 > 315) y1 = 315;

				LCD_Color_DrawLine(x0, y0, x1, y1, BLUE);
		}

		//--- 绘制目标 setV 参考线 ---//
		int y_set = 315 - (int)(100 * (setV - (y_mid - y_range / 2)) / y_range);
		if (y_set < 190) y_set = 190;
		if (y_set > 315) y_set = 315;
		LCD_Color_DrawLine(5, y_set, 235, y_set, RED); // 画水平线

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
		arm_fir_init_f32(&S, numTaps, pCoeffs, firStateF32, blockSize);
		/* 实现FIR滤波 */
		for (int i=0; i < numBlocks; i++)
		{
			arm_fir_f32(&S, pSrc + (i * blockSize), pDst + (i * blockSize), blockSize);
		}
}
 //================================================//


// 下采样 FIR Decimation
void DSP_FIR_DECIMATE( float32_t *pSrc, float32_t *pDst, uint16_t inputLen,
                       uint32_t blockSize, float32_t *pCoeffs, uint16_t numTaps, uint8_t M )
{
    arm_fir_decimate_instance_f32 S;
    float32_t state[blockSize + numTaps - 1];

    arm_fir_decimate_init_f32(&S, numTaps, M, pCoeffs, state, blockSize);

    uint32_t numBlocks = inputLen / blockSize;
    for (int i = 0; i < numBlocks; i++)
    {
        arm_fir_decimate_f32(&S, pSrc + i * blockSize, pDst + i * (blockSize / M), blockSize);
    }
}

 //================================================//

// 上采样 FIR Interpolation
void DSP_FIR_INTERPOLATE( float32_t *pSrc, float32_t *pDst, uint16_t inputLen,
                          uint32_t blockSize, float32_t *pCoeffs, uint16_t numTaps, uint8_t L )
{
    arm_fir_interpolate_instance_f32 S;
    float32_t state[blockSize + numTaps - 1];

    arm_fir_interpolate_init_f32(&S, L, numTaps, pCoeffs, state, blockSize);

    uint32_t numBlocks = inputLen / blockSize;
    for (int i = 0; i < numBlocks; i++)
    {
        arm_fir_interpolate_f32(&S, pSrc + i * blockSize, pDst + i * blockSize * L, blockSize);
    }
}

 //================================================//

// LMS 自适应滤波器
void DSP_LMS_FILTER( float32_t *pSrc, float32_t *pRef, float32_t *pDst, float32_t *err,
                     uint16_t inputLen, float32_t *pCoeffs, uint16_t numTaps, float32_t mu,float32_t *state)
{
		//计算直流量
		float32_t sum = 0;
		for (u16 i=0;i<inputLen;i++){
			sum += pSrc[i];
			
		}
		sum /= inputLen;
		
		//去除直流
		for (u16 i=0;i<inputLen;i++)
		pSrc[i] -= sum;
		
		sum=0;
		//计算直流量
		for (u16 i=0;i<inputLen;i++){
			sum += pRef[i];
			
		}
		sum /= inputLen;
		
		//去除直流
		for (u16 i=0;i<inputLen;i++)
		pRef[i] -= sum;					
	
	
		arm_lms_instance_f32 S;
	
    arm_lms_init_f32(&S, numTaps, pCoeffs, state, mu, inputLen);

    arm_lms_f32(&S, pSrc, pRef, pDst, err, inputLen);
}

 //================================================//

// NLMS 自适应滤波器
void DSP_LMS_NORM_FILTER( float32_t *pSrc, float32_t *pRef, float32_t *pDst, float32_t *err,
                          uint16_t inputLen, float32_t *pCoeffs, uint16_t numTaps, float32_t mu,float32_t *state)
{
		//计算直流量
		float32_t sum = 0;
		for (u16 i=0;i<inputLen;i++){
			sum += pSrc[i];
			
		}
		sum /= inputLen;
		
		//去除直流
		for (u16 i=0;i<inputLen;i++)
		pSrc[i] -= sum;
		
		sum=0;
		//计算直流量
		for (u16 i=0;i<inputLen;i++){
			sum += pRef[i];
			
		}
		sum /= inputLen;
		
		//去除直流
		for (u16 i=0;i<inputLen;i++)
		pRef[i] -= sum;					
	
	
    arm_lms_norm_instance_f32 S;

    arm_lms_norm_init_f32(&S, numTaps, pCoeffs, state, mu, inputLen);

    arm_lms_norm_f32(&S, pSrc, pRef, pDst, err, inputLen);
}

 //================================================//

void DSP_LMS_DRAW(float32_t *in_buf, float32_t *out_buf, float32_t y_mid, float32_t y_range){
	LCD_Fill_onecolor(5, 190, 235, 315, WHITE); // 清区域
	LCD_Color_DrawRectangle(5, 188, 235, 315, BLACK); // 边框

	// X轴刻度
	for (u16 x = 0; x <= 230; x++) {
		if (x % 10 == 0)
			LCD_Color_DrawLine(5 + x, 315, 5 + x, 320, RED);
		else if (x % 2 == 0)
			LCD_Color_DrawLine(5 + x, 315, 5 + x, 317, BLACK);
	}

	// 蓝色：原始波形
	for (int i = 1; i < Y_BUF_SIZE; i++) {
		int x0 = 5 + i - 1;
		int x1 = 5 + i;
		int y0 = 315 - (int)(100 * (in_buf[i - 1] - (y_mid - y_range / 2)) / y_range);
		int y1 = 315 - (int)(100 * (in_buf[i] - (y_mid - y_range / 2)) / y_range);
		if (y0 < 190) y0 = 190; if (y0 > 315) y0 = 315;
		if (y1 < 190) y1 = 190; if (y1 > 315) y1 = 315;
		LCD_Color_DrawLine(x0, y0, x1, y1, BLUE);
	}

	// 绿色：滤波后波形
	for (int i = 1; i < Y_BUF_SIZE; i++) {
		int x0 = 5 + i - 1;
		int x1 = 5 + i;
		int y0 = 315 - (int)(100 * (out_buf[i - 1] - (y_mid - y_range / 2)) / y_range);
		int y1 = 315 - (int)(100 * (out_buf[i] - (y_mid - y_range / 2)) / y_range);
		if (y0 < 190) y0 = 190; if (y0 > 315) y0 = 315;
		if (y1 < 190) y1 = 190; if (y1 > 315) y1 = 315;
		LCD_Color_DrawLine(x0, y0, x1, y1, GREEN);
	}
	// 【修正部分】添加x轴（0V）
	// 1. 计算0V电压对应的屏幕Y轴像素坐标
	int y_zero_axis = 315 - (int)(100 * (0.0f - (y_mid - y_range / 2)) / y_range);

	// 2. 检查坐标是否在绘图区域内
	if (y_zero_axis >= 190 && y_zero_axis <= 315) {
		// 3. 使用正确的像素坐标画线
		LCD_Color_DrawLine(5, y_zero_axis, 235, y_zero_axis, RED);
	}
}

 //================================================//

//-----进ADC滤波-----
//FIFO
#define FILSIZE 4		//定义滤波数组宽度
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
