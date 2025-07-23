/*********************************************************************************
**********************************************************************************
* �ļ�����: w25qxx.c                                                            	 *
* �ļ�������W25Q128ʹ��                                                         	 *
* �������ڣ�2017.08.30                                                          	 *
* ˵    ����25Q128����Ϊ16M,����128��Block,4096��Sector                         	 *
            255byteΪһҳ                                                       	 *
						4KbytesΪһ������                                                   	 *
            16������Ϊһ����                                                    	 *
**********************************************************************************
*********************************************************************************/

#include "w25qxx.h"
#include "spi.h"

u16 W25QXX_ID;
u8 pBuffer[Byte_Count];	//��ʼֵ�趨
u8 DataBuffer[Byte_Count];	//��ȡ���ݱ���

//��ʼ��SPI FLASH��IO��
void W25QXX_Init(void)
{ 
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); //ʹ��GPIOBʱ��

  GPIO_InitTypeDef  GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;             //PB6
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;         //���
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;        //�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;    //100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;          //����
  GPIO_Init(GPIOB, &GPIO_InitStructure);                //��ʼ��

	W25QXX_CS=1;			                                    //SPI FLASH��ѡ��
	SPI3_Init();		   			                              //��ʼ��SPI
	W25QXX_ID=W25QXX_ReadID();	                          //��ȡFLASH ID.
}

/****************************************************************************
* ��    ��: u8 W25QXX_ReadSR(void) 
* ��    �ܣ���ȡW25QXX��״̬�Ĵ���
* ��ڲ�������
* ���ز�����״̬�Ĵ�����ֵ
* ˵    ���� 		     
****************************************************************************/
u8 W25QXX_ReadSR(void)   
{  
	u8 byte=0;   
	W25QXX_CS=0;                                  //ʹ������   
	SPI3_ReadWriteByte(W25X_CMD_ReadStatusReg);   //���Ͷ�ȡ״̬�Ĵ�������    
	byte=SPI3_ReadWriteByte(0Xff);                //��ȡһ���ֽ�  
	W25QXX_CS=1;                                  //ȡ��Ƭѡ     
	return byte;   
} 

/****************************************************************************
* ��    ��: void W25QXX_Write_SR(u8 sr)
* ��    �ܣ�дW25QXX״̬�Ĵ���
* ��ڲ�����д���ֵ
* ���ز�������
* ˵    ���� 		     
****************************************************************************/
void W25QXX_Write_SR(u8 sr)   
{   
	W25QXX_CS=0;                                 //ʹ������   
	SPI3_ReadWriteByte(W25X_CMD_WriteStatusReg); //����дȡ״̬�Ĵ�������    
	SPI3_ReadWriteByte(sr);                      //д��һ���ֽ�  
	W25QXX_CS=1;                                 //ȡ��Ƭѡ     	      
} 

//W25QXXдʹ��	 
void W25QXX_Write_Enable(void)   
{
	W25QXX_CS=0;                               //ʹ������   
  SPI3_ReadWriteByte(W25X_CMD_WriteEnable);  //����дʹ��  
	W25QXX_CS=1;                               //ȡ��Ƭѡ     	      
} 

//�ȴ�����
void W25QXX_Wait_Busy(void)   
{   
	while((W25QXX_ReadSR()&0x01)==0x01);      //�ȴ�BUSYλ���
} 

//W25QXXд��ֹ	 
void W25QXX_Write_Disable(void)   
{  
	W25QXX_CS=0;                                //ʹ������   
  SPI3_ReadWriteByte(W25X_CMD_WriteDisable);  //����д��ָֹ��    
	W25QXX_CS=1;                                //ȡ��Ƭѡ     	      
} 

/****************************************************************************
* ��    ��: u16 W25QXX_ReadID(void)
* ��    �ܣ���ȡоƬID
* ��ڲ�������
* ���ز�����оƬID
* ˵    ����0XEF13,��ʾоƬ�ͺ�ΪW25Q80  
            0XEF14,��ʾоƬ�ͺ�ΪW25Q16    
            0XEF15,��ʾоƬ�ͺ�ΪW25Q32  
            0XEF16,��ʾоƬ�ͺ�ΪW25Q64 
            0XEF17,��ʾоƬ�ͺ�ΪW25Q128      
****************************************************************************/  
u16 W25QXX_ReadID(void)
{
	u16 IDnum = 0;	  
	W25QXX_CS=0;				    
	SPI3_ReadWriteByte(0x90); //���Ͷ�ȡID����	    
	SPI3_ReadWriteByte(0x00); 	    
	SPI3_ReadWriteByte(0x00); 	    
	SPI3_ReadWriteByte(0x00); 	 			   
	IDnum|=SPI3_ReadWriteByte(0xFF)<<8;  
	IDnum|=SPI3_ReadWriteByte(0xFF);	 
	W25QXX_CS=1;				    
	return IDnum;
} 

/****************************************************************************
* ��    ��: void W25QXX_Erase_Chip(void) 
* ��    �ܣ���������оƬ		  
* ��ڲ�������
* ���ز�������
* ˵    ���� 		     
****************************************************************************/
void W25QXX_Erase_Chip(void)   
{                                   
    W25QXX_Write_Enable();                   
    W25QXX_Wait_Busy();   
  	W25QXX_CS=0;                             //ʹ������   
    SPI3_ReadWriteByte(W25X_CMD_ChipErase);  //����Ƭ��������  
	  W25QXX_CS=1;                             //ȡ��Ƭѡ     	      
	  W25QXX_Wait_Busy();   				           //�ȴ�оƬ��������
}  

/****************************************************************************
* ��    ��: void W25QXX_Erase_Sector(u32 First_Addr)
* ��    �ܣ�����ĳ���������׵�ַ	  
* ��ڲ�����First_Addr:������ַ
* ���ز�������
* ˵    ���� 		     
****************************************************************************/
void W25QXX_Erase_Sector(u32 First_Addr)   
{    
 	  First_Addr*=4096;
    W25QXX_Write_Enable();                  
    W25QXX_Wait_Busy();   
  	W25QXX_CS=0;                                  //ʹ������   
    SPI3_ReadWriteByte(W25X_CMD_SectorErase);     //������������ָ�� 
    SPI3_ReadWriteByte((u8)((First_Addr)>>16));   //���͵�ַ    
    SPI3_ReadWriteByte((u8)((First_Addr)>>8));   
    SPI3_ReadWriteByte((u8)First_Addr);  
	  W25QXX_CS=1;                                  //ȡ��Ƭѡ     	      
    W25QXX_Wait_Busy();   				                //�ȴ��������
} 

//�������ģʽ
void W25QXX_PowerDown(void)   
{ 
  	W25QXX_CS=0;                             //ʹ������   
    SPI3_ReadWriteByte(W25X_CMD_PowerDown);  //���͵�������  
	  W25QXX_CS=1;                             //ȡ��Ƭѡ     	      
    delay_us(3);                             
}  

//����
void W25QXX_WAKEUP(void)   
{  
  	W25QXX_CS=0;                                      //ʹ������   
    SPI3_ReadWriteByte(W25X_CMD_ReleasePowerDown);    //���ͻ�������
	  W25QXX_CS=1;                                      //ȡ��Ƭѡ     	      
    delay_us(3);                                     
}   

/****************************************************************************
* ��    ��: void W25QXX_Read(u8* pBuffer,u32 ReadAddr,u16 NumByteToRead)
* ��    �ܣ�ָ����ַ��ʼ��ȡָ�����ȵ�����
* ��ڲ�����DataBuffer:���ݴ洢��
            StartAddress:��ʼ��ȡ�ĵ�ַ(24bit)
            ByteCount:Ҫ��ȡ���ֽ���(���65535)
* ���ز�������
* ˵    ���� 		     
****************************************************************************/
void W25QXX_Read(u8* DataBuffer,u32 StartAddress,u16 ByteCount)   
{  										    
	W25QXX_CS=0;                                 //ʹ������   
    SPI3_ReadWriteByte(W25X_CMD_ReadData);     //���Ͷ�ȡ����   
    SPI3_ReadWriteByte((u8)((StartAddress)>>16));  //����24bit��ַ    
    SPI3_ReadWriteByte((u8)((StartAddress)>>8));   
    SPI3_ReadWriteByte((u8)StartAddress);   
	
			while (ByteCount--) 
			{
					*DataBuffer = SPI3_ReadWriteByte(0XFF);
					DataBuffer++;
			}		
	W25QXX_CS=1;  				    	      
}  

/****************************************************************************
* ��    ��: void W25QXX_Write_Page(u8* DataBuffer,u32 StartAddress,u16 ByteCount)
* ��    �ܣ���һҳ��д������256���ֽڵ�����  
* ��ڲ�����DataBuffer:���ݴ洢��
            StartAddress:��ʼд��ĵ�ַ(24bit)
            ByteCount:Ҫд����ֽ���(���256)
* ���ز�������
* ˵    ���� 		     
****************************************************************************/
void W25QXX_Write_Page(u8* DataBuffer,u32 StartAddress,u16 ByteCount)
{  
	W25QXX_Write_Enable();                   
	  W25QXX_CS=0;                                   //ʹ������   
    SPI3_ReadWriteByte(W25X_CMD_PageProgram);      //����дҳ����   
    SPI3_ReadWriteByte((u8)((StartAddress)>>16));  //���͵�ַ    
    SPI3_ReadWriteByte((u8)((StartAddress)>>8));   
    SPI3_ReadWriteByte((u8)StartAddress);   
		while (ByteCount--)
			{
			  SPI3_ReadWriteByte(*DataBuffer);
			  DataBuffer++;
			}	
	W25QXX_CS=1;                   //ȡ��Ƭѡ 
	W25QXX_Wait_Busy();					   //�ȴ�д�����
}

/****************************************************************************
* ��    ��: void W25QXX_PageWrite(u8* DataBuffer,u32 StartAddress,u16 ByteCount) 
* ��    �ܣ���ҳд������
* ��ڲ�����DataBuffer:���ݴ洢��
            StartAddress:��ʼд��ĵ�ַ(24bit)
            ByteCount:Ҫд����ֽ���(���256)
* ���ز�������
* ˵    �������Զ���ҳ���ܣ���Ϊ�ú���û����д֮ǰ�ж�Ҫд��ĵ�ַ�ϵ������Ƿ�Ϊ
            �գ�������д��֮ǰ����ȷ���õ�ַ�ϵ�����Ϊ��0xFF������д��ʧ�� 		     
****************************************************************************/
void W25QXX_PageWrite(u8* DataBuffer,u32 StartAddress,u16 ByteCount)   
{
  u8 NumOfPage = 0, NumOfSingle = 0, Surplus_Addr = 0, Surplus_count = 0, midtemp = 0;

  Surplus_Addr = StartAddress % 256;
  Surplus_count = 256 - Surplus_Addr;
  NumOfPage =  ByteCount / 256;
  NumOfSingle = ByteCount % 256;

  if (Surplus_Addr == 0) //��ʼд�ĵ�ַ�պ���ҳ��ʼ�ĵ�ַ
  {
    if (NumOfPage == 0)  // ByteCount < 256��һҳ�ܵ��ֽ�����  
    {
      W25QXX_Write_Page(DataBuffer, StartAddress, ByteCount);
    }
    else                 // ByteCount > 256��һҳ�ܵ��ֽ�����  
    {
      while (NumOfPage--)
      {
        W25QXX_Write_Page(DataBuffer, StartAddress, 256);
        StartAddress +=  256;
        DataBuffer += 256;
      }
      W25QXX_Write_Page(DataBuffer, StartAddress, NumOfSingle);
    }
  }
  else ///��ʼд�ĵ�ַ����ҳ���׵�ַ��
  {
    if (NumOfPage == 0) // ByteCount < 256��һҳ�ܵ��ֽ����� 
    {
      if (NumOfSingle > Surplus_count)  
      {
        midtemp = NumOfSingle - Surplus_count;
        W25QXX_Write_Page(DataBuffer, StartAddress, Surplus_count);
        StartAddress +=  Surplus_count;
        DataBuffer += Surplus_count;
        W25QXX_Write_Page(DataBuffer, StartAddress, midtemp);
      }
      else
      {
        W25QXX_Write_Page(DataBuffer, StartAddress, ByteCount);
      }
    }
    else //ByteCount > 256��һҳ�ܵ��ֽ�����  
    {
      ByteCount -= Surplus_count;
      NumOfPage =  ByteCount / 256;
      NumOfSingle = ByteCount % 256;

      W25QXX_Write_Page(DataBuffer, StartAddress, Surplus_count);
      StartAddress +=  Surplus_count;
      DataBuffer += Surplus_count;
      while (NumOfPage--)
      {
        W25QXX_Write_Page(DataBuffer, StartAddress, 256);
        StartAddress +=  256;
        DataBuffer += 256;
      }
      if (NumOfSingle != 0)
      {
        W25QXX_Write_Page(DataBuffer, StartAddress, NumOfSingle);
      }
    }
  }
} 

/****************************************************************************
* ��    ��: void W25QXX_SectorWrite(u8* DataBuffer,u32 StartAddress,u16 ByteCount)
* ��    �ܣ�������д������
* ��ڲ�����DataBuffer:���ݴ洢��
            StartAddress:��ʼд��ĵ�ַ(24bit)
            ByteCount:Ҫд����ֽ���(���65536)
* ���ز�������
* ˵    ����д��֮ǰ�жϸĵ�ַ�ϵ������Ƿ�Ϊ�գ������Ȳ�����д�룬�ú����ο����ϴ���http://www.openedv.com      
****************************************************************************/
u8 TS_BUFFER[4096];
void W25QXX_SectorWrite(u8* DataBuffer,u32 StartAddress,u16 ByteCount)   
{ 
	u32 secaddr;
	u16 secused;
	u16 Surplus_count;	   
 	u16 i=0; 
	
	u8 * Cache_BUF;	   
  Cache_BUF=TS_BUFFER;	  
	
 	secaddr=StartAddress/4096;//������ַ  
	secused=StartAddress%4096;//д��ĵ�ַ�������ڵ�ƫ�ƣ��Ӹ��������׵�ַ��ʼ��ʹ�õ��ֽ�����
	Surplus_count=4096-secused;//������ʣ��ռ��С   

 	if(ByteCount<=Surplus_count)Surplus_count=ByteCount;//������4096���ֽ�
	while(1) 
	{	
		W25QXX_Read(Cache_BUF,secaddr*4096,4096);//������������������
		      i=0;
		while(Cache_BUF[secused+i]==0XFF)
				{
					i++;
					if(i==Surplus_count)break;
				}
		if(i<Surplus_count)//��Ҫ����
				{
					W25QXX_Erase_Sector(secaddr);//�����������
					for(i=0;i<Surplus_count;i++)	   //����
					{
						Cache_BUF[i+secused]=DataBuffer[i];	  
					}
					W25QXX_PageWrite(Cache_BUF,secaddr*4096,4096);//д����������  

				}
		else
			  W25QXX_PageWrite(DataBuffer,StartAddress,Surplus_count);//д�Ѿ������˵�,ֱ��д������ʣ������. 				   
		if(ByteCount==Surplus_count)break;//д�������
		else//д��δ����
				{
					secaddr++;//������ַ��1
					secused=0;//ƫ��λ��Ϊ0 	 

					DataBuffer+=Surplus_count;  //ָ��ƫ��
					StartAddress+=Surplus_count;//д��ַƫ��	   
					ByteCount-=Surplus_count;				//�ֽ����ݼ�
					if(ByteCount>4096)Surplus_count=4096;	//��һ����������д����
					else Surplus_count=ByteCount;			//��һ����������д����
				}	 
	};
}
