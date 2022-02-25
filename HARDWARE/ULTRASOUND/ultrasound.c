#include "sys.h"
#include "delay.h"
#include "usart.h"
void Ultrasound_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	TIM_ICInitTypeDef  TIM3_ICInitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);	//ʹ��TIM3ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOD, ENABLE);	 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		
	GPIO_Init(GPIOD, &GPIO_InitStructure);					
	GPIO_ResetBits(GPIOD,GPIO_Pin_6);
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_1; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;  
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB,GPIO_Pin_0);
	
	TIM_TimeBaseStructure.TIM_Period = 0XFFFF; //�趨�������Զ���װֵ 
	TIM_TimeBaseStructure.TIM_Prescaler =72-1; 	//Ԥ��Ƶ��   
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
  
	//��ʼ��TIM3���벶�����
	TIM3_ICInitStructure.TIM_Channel = TIM_Channel_4; //CC4S=01 	ѡ������� IC1ӳ�䵽TI1��
  TIM3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
  TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
  TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
  TIM3_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 ���������˲��� ���˲�
  TIM_ICInit(TIM3, &TIM3_ICInitStructure);
	
	//�жϷ����ʼ��
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //��ռ���ȼ�2��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //�����ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ��� 
	
	TIM_ITConfig(TIM3,TIM_IT_Update|TIM_IT_CC4,ENABLE);//��������ж� ,����CC4IE�����ж�	
	
   	TIM_Cmd(TIM3,ENABLE ); 	//ʹ�ܶ�ʱ��5
   
}

u8  TIM3CH4_CAPTURE_STA=0;	//���벶��״̬		    				
u16	TIM3CH4_CAPTURE_VAL;	//���벶��ֵ
 
//��ʱ��3�жϷ������	 
void TIM3_IRQHandler(void)
{ 

 	if((TIM3CH4_CAPTURE_STA&0X80)==0)//��δ�ɹ�����	
	{	  
		if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
		 
		{	    
			if(TIM3CH4_CAPTURE_STA&0X40)//�Ѿ����񵽸ߵ�ƽ��
			{
				if((TIM3CH4_CAPTURE_STA&0X3F)==0X3F)//�ߵ�ƽ̫����
				{
					TIM3CH4_CAPTURE_STA|=0X80;//��ǳɹ�������һ��
					TIM3CH4_CAPTURE_VAL=0XFFFF;
				}else TIM3CH4_CAPTURE_STA++;
			}	 
		}
	if (TIM_GetITStatus(TIM3, TIM_IT_CC4) != RESET)//����1���������¼�
		{	
			if(TIM3CH4_CAPTURE_STA&0X40)		//����һ���½��� 		
			{	  			
				TIM3CH4_CAPTURE_STA|=0X80;		//��ǳɹ�����һ�θߵ�ƽ����
				TIM3CH4_CAPTURE_VAL=TIM_GetCapture4(TIM3);
		   		TIM_OC4PolarityConfig(TIM3,TIM_ICPolarity_Rising); //CC4P=0 ����Ϊ�����ز���
			}else  								//��δ��ʼ,��һ�β���������
			{
				TIM3CH4_CAPTURE_STA=0;			//���
				TIM3CH4_CAPTURE_VAL=0;
	 			TIM_SetCounter(TIM3,0);
				TIM3CH4_CAPTURE_STA|=0X40;		//��ǲ�����������
		   		TIM_OC4PolarityConfig(TIM3,TIM_ICPolarity_Falling);		//CC4P=1 ����Ϊ�½��ز���
			}		    
		}			     	    					   
 	}
 
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC4|TIM_IT_Update); //����жϱ�־λ
 
}

int Measure_DST(void)/*�����ʵ������5~50cm����ʱ������������������ܻᷢ����С�Ĵ��󣡣���*/
{	
	int i=0;
	float dst=0; float temp;
	for(;i<10;i++)
	{
		PDout(6)=!PDout(6);
		delay_us(20);
		PDout(6)=!PDout(6);
					
		while(1)
		{
			if(TIM3CH4_CAPTURE_STA&0X80)//�ɹ�������һ��������
			break;
			delay_us(500);
		}
		
		temp=((TIM3CH4_CAPTURE_STA&0X3F)*65536+TIM3CH4_CAPTURE_VAL)*17.0/1000;
		if(temp>5&&temp<50)
			dst+=temp;
		else
			i--;//��βɼ����ϣ����²�				
		TIM3CH4_CAPTURE_STA=0;//������һ�β���			
	}
	return (dst/10);
}
