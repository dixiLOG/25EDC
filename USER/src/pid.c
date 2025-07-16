#include "pid.h"
#include "AD9833.h"








//======================================2024电赛废弃代码======================================

//u16 min1=4000,max1=0;
//u16 min2=4000,max2=0;
//u16 mid1=0,mid2=0;
//u16 adc_count=1;
//u16 peek1,peek2;
//int phase_control=0;
//int data;
//int err_acc;
//int n=0;
//u16 last_err;
//int peek_err=0;
//int peek_err_pre=0;
//int peek_control=0;

////85K 100K 三角波

//void find_max(float kp,float kd,float ki)
//{	

//	
//	min1=4000; max1=0;
//	min2=4000; max2=0;
//	
//	for (adc_count=0;adc_count<512;adc_count++)
//	//for (adc_count=0;adc_count<150;adc_count++)
//	{
//	if (ADC3_Data_Rx[adc_count*2]>max1) max1=ADC3_Data_Rx[adc_count*2-1];
//	if (ADC3_Data_Rx[adc_count*2]<min1) min1=ADC3_Data_Rx[adc_count*2-1];
//	if (ADC3_Data_Rx[adc_count*2+1]>max2) max2=ADC3_Data_Rx[adc_count*2];
//	if (ADC3_Data_Rx[adc_count*2+1]<min2) min2=ADC3_Data_Rx[adc_count*2];
//	}	
//	peek1=max1-min1;
//	peek2=max2-min2;
//	peek_err=(peek1-peek2)*0.05;
//	peek_control=peek_err-peek_err_pre;
//	data=pid_control(	peek_control,1,kp,kd,ki);
//	phase_control+=data;
//	peek_err_pre=peek_err;
//	
//	
//	if (phase_control>=4095)	phase_control-=4095;
//	if (phase_control<0) phase_control+=4095;
//	//SetAD9833PhaseRegister(phase_control,2);
//	
//	
//	//============SetAD9833PhaseRegister函数==========
//	
//	//int last_phase_switch[2]={0};
//	
//	/* 设置相位寄存器的值 */
//	/*
//	void SetAD9833PhaseRegister(unsigned int Phase,uint8_t drive_id)
//	{
//		unsigned int Phs_data=0;
//	if(last_phase_switch[drive_id-1]==1){
//		 Phs_data=Phase|0xC000;	//相位值
//		last_phase_switch[drive_id-1]=0;
//	}else{

//		 Phs_data=Phase|0xE000;	//相位值
//		last_phase_switch[drive_id-1]=1;
//	}
//		//AD9833_Write(0x2100);
//	//int Phs_data=Phase|0xC000;	//相位值
//	// Phase&=0x0FFF;
//	// Phase|=0xD000;
//	AD9833_Write(Phs_data,drive_id);	//设置相位
//	}
//	*/
//	//=======================================
//	//err=max2-max1
//}


//int pid_control(int err,u8 ispid,float kp,float kd,float ki)
//{
//	int output=0;
//	if (ispid&&(n<=5))
//	{
//		n++;
//		err_acc+=err;
//	}
//	else
//	{
//		n=0;
//		err_acc=0;
//	}
//	output=kp*err+ki*(err_acc/n)+kd*(err-last_err);
//	last_err=err;
//	return(output);
//}


//============================================================================

