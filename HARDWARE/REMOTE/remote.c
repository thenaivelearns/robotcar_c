#include "remote.h"
#include "delay.h"
#include "usart.h"
#include "pid.h"
#include "linefollow.h"
#include "arm.h"
extern int startswitch;

//����ң�س�ʼ��
//����IO�Լ���ʱ��4�����벶��
void Remote_Init(void)    			  
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef  TIM_ICInitStructure;  
 
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE); //ʹ��GPIOBʱ�� 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);	//TIM4 ʱ��ʹ�� 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;				 //PB9 ���� 
// 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 		//��������
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 		//�������룬���������������Ҳ���ն�OK
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOB, &GPIO_InitStructure);
 	GPIO_SetBits(GPIOB,GPIO_Pin_1);	//��ʼ��GPIOB.9
	
						  
 	TIM_TimeBaseStructure.TIM_Period = 10000; //�趨�������Զ���װֵ ���10ms���  
	TIM_TimeBaseStructure.TIM_Prescaler =(72-1); 	//Ԥ��Ƶ��,1M�ļ���Ƶ��,1us.	   
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ

	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //����ָ���Ĳ�����ʼ��TIMx

  TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;  // ѡ������� IC4ӳ�䵽TI4��
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
  TIM_ICInitStructure.TIM_ICFilter = 0x03;//IC4F=0011 ���������˲��� 8����ʱ��ʱ�������˲�
  TIM_ICInit(TIM3, &TIM_ICInitStructure);//��ʼ����ʱ�����벶��ͨ��

  TIM_Cmd(TIM3,ENABLE); 	//ʹ�ܶ�ʱ��4
 
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM4�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //�����ȼ�3��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���	

	TIM_ITConfig(TIM3,TIM_IT_CC4,ENABLE);//��������ж� ,����CC4IE�����ж�								 
}

//ң��������״̬
//[7]:�յ����������־
//[6]:�õ���һ��������������Ϣ
//[5]:����	
//[4]:����������Ƿ��Ѿ�������								   
//[3:0]:�����ʱ��
u8 	RmtSta=0;	  	  
u16 Dval;		//�½���ʱ��������ֵ
u32 RmtRec=0;	//(��������������Ҫ�õ���)������յ������ݣ���ַ��8λ����ַ����8λ��������8λ�����Ʒ���8λ	   		    
//��ʱ��4�жϷ������	 
/*
void TIM3_IRQHandler(void)
{ 		    	 
	u8 key=0;		
	if(TIM_GetITStatus(TIM3,TIM_IT_CC4)!=RESET)
	{	  
		if(RDATA)//�����ز���RDATA��PB9�ĺ�
		{
  		TIM_OC4PolarityConfig(TIM3,TIM_ICPolarity_Falling);						//CC4P=1	����Ϊ�½��ز���
			TIM_SetCounter(TIM3,0);							//��ն�ʱ��ֵ
			RmtSta|=0X10;							//����������Ѿ�������
		}else //�½��ز���
		{
			Dval=TIM_GetCapture4(TIM3);					//��ȡCCR4Ҳ������CC4IF��־λ.�鵽��������˵���������ʱ��CNT��ֵ�ᱻ����CCR��������GetCapture������GetCounterҲ�ǿ��Ե�
  		TIM_OC4PolarityConfig(TIM3,TIM_ICPolarity_Rising);				//CC4P=0	����Ϊ�����ز���
			if(RmtSta&0X10)							//���һ�θߵ�ƽ���� 
			{
 				if(RmtSta&0X80)//���յ���������
				{
					
					if(Dval>300&&Dval<800)			//560Ϊ��׼ֵ,560us
					{
						RmtRec<<=1;					//����һλ.
						RmtRec|=0;					//���յ�0	   
					}else if(Dval>1400&&Dval<1800)	//1680Ϊ��׼ֵ,1680us
					{
						RmtRec<<=1;					//����һλ.
						RmtRec|=1;					//���յ�1
					}else if(Dval>2200&&Dval<2600)	//�õ�������ֵ���ӵ���Ϣ 2500Ϊ��׼ֵ2.5ms (�����������룩
					{
						key=RmtRec>>8;	
					}
 				}else if(Dval>4200&&Dval<4700)		//4500Ϊ��׼ֵ4.5ms
				{
					RmtSta|=1<<7;					//��ǳɹ����յ���������
				}						 
			}
			RmtSta&=~(1<<4);//���֮ǰ�������ر�־
		}
	
	if(key!=0)
	{	PBout(8)=1;
		delay_ms(500);
		PBout(8)=0;
		
		TIM_ITConfig(TIM3,TIM_IT_CC4,DISABLE);
		delay_ms(500);//�㿪���������ಿ��
		switch(key)
			{	
				case 0:break;//str="ERROR";keyflag = key;break;			   
				case 162:startswitch=1;break;//str="POWER";keyflag = key;break;	    
				case 98:Ki_line+=0.001;break;//str="UP";keyflag = key;break;	    
				case 2:break;//str="PLAY";keyflag = key;break;		 
				case 226:setnum_left+=10;break;//str="ALIENTEK";keyflag = key;break;		  
				case 194:Kp_line+=1;break;//str="RIGHT";keyflag = key;break;	   
				case 34:Kp_line-=1;break;//str="LEFT";keyflag = key;break;		  
				case 224:Kd_line-=1;break;//str="VOL-";keyflag = key;break;		  
				case 168:Ki_line-=0.001;break;//str="DOWN";keyflag = key;break;		   
				case 144:Kd_line+=1;break;//str="VOL+";keyflag = key;break;		    
				case 104:initial_motor_speed+=10;break;//str="1";keyflag = key;break;		  
				case 152:initial_motor_speed-=10;break;//str="2";keyflag = key;break;	   
				case 176:shoulder+=2;TIM_SetCompare1(TIM5,shoulder);break;//str="3";keyflag = key;break;	    
				case 48:shoulder-=2;TIM_SetCompare1(TIM5,shoulder);break;//str="4";keyflag = key;break;		    
				case 24:pwmlevel_left=500;pwmlevel_right=500;Setpwm();break;//str="5";keyflag = key;break;		    
				case 122:clockwise();break;//str="6";keyflag = key;break;		  
				case 16:counterclockwise();break;//str="7";keyflag = key;break;			   					
				case 56:pwmlevel_left+=50;pwmlevel_right+=50;Setpwm();break;//str="8";keyflag = key;break;	 
				case 90:pwmlevel_left-=50;pwmlevel_right-=50;Setpwm();break;//str="9";keyflag = key;break;
				case 66:IsCloseloop=0;break;//���뿪��״̬;break;//str="0";keyflag = key;break;
				case 82:IsCloseloop=1;break;//str="DELETE";keyflag = key;break;
			}

		RmtSta=0;
		RmtRec=0;
		TIM_ITConfig(TIM3,TIM_IT_CC4,ENABLE);
	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update|TIM_IT_CC4);
	}
}*/
