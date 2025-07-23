#ifndef __KEY_H
#define __KEY_H
#include "common.h"

//////////////////////////////////////////////////////////////////////////////////	 

//����IO�˿ڶ���
#define KEY1 		PDin(6)
#define KEY2 		PDin(3)
#define KEY3 		PDin(2)
#define KEY4 	  PCin(12)
#define KEY5 	  PAin(15)

//����ֵ����
#define KEY1_DATA	  1
#define KEY2_DATA	  2
#define KEY3_DATA   3
#define KEY4_DATA	  4
#define KEY5_DATA	  5

//��������
extern u8   keydown_data;    //�������º�ͷ��ص�ֵ
extern u8   keyup_data;      //����̧�𷵻�ֵ
extern u16  key_time;
extern u8   key_tem; 

//��������
void KEY_Init(void);	      //IO��ʼ��
void key_scan(u8 mode);  		//����ɨ�躯��	

#endif
