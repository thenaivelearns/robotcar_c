#include "stepmotor.h"
#define uint unsigned int
#define uchar unsigned char
	
//步进电机正反转数组1
uchar phasecw[8] ={0x09,0x01,0x03,0x02,0x06,0x04,0x0c,0x08};// DA-D-CD-C-BC-B-AB-A   
uchar phaseccw[8]={0x08,0x0c,0x04,0x06,0x02,0x03,0x01,0x09};// A-AB-B-BC-C-CD-D-DA.
int elevate_angle =0;

void Delay_xms(uint x)
{
 uint i,j;
 for(i=0;i<x;i++)
  for(j=0;j<112;j++);
}

void Stepmotor_Init(void)
{
	//电机1
 GPIO_InitTypeDef GPIO_InitStructure;
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,ENABLE);//由于正反转数组已经给定，故控制stepmotor的io口只能是0~3
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1| GPIO_Pin_2 | GPIO_Pin_3 ;
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 GPIO_Init(GPIOD,&GPIO_InitStructure);

 elevate(80,Pillar_speed);
 MotorStop();
}

void Motorcw(int speed)  
{  
    uint8_t i;  
  
    for(i=0;i<8;i++)  
    {  
        GPIO_Write(GPIOD,phaseccw[i]);  
        Delay_xms(speed);  
    }  
}

void Motorccw(int speed)  
{  
    uint8_t i;  
  
    for(i=0;i<8;i++)  
    {  
        GPIO_Write(GPIOD,phasecw[i]);  
        Delay_xms(speed);  
    }  
}

void MotorStop(void)  
{  
  GPIO_Write(GPIOD,0x0000);   
}


//由于   *一个脉冲*   *输出轴*  转0.08789度（电机实转0.08789*64=5.625度），即为5.625度-步进角。
//则转完8拍为  *8个脉冲*  ，即0.08789*8=0.70312度。若称为一个周期，则j为需要的转完angle角度所需的周期数

void elevate(int angle,int speed)
{
	int i,j;
	j=(int)(angle*64/(5.625*8));
	if (angle>0)
	{
	for(i=0;i<j;i++)
	  {
		  Motorccw(speed);
	  }
  }
	else if(angle<0)
	{
		for(i=0;i<-j;i++)
	  {
		  Motorcw(speed);
	  }
	}	
}
