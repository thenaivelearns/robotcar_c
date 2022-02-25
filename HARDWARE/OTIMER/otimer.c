#include "timer_encoder.h"
#include "pid.h"
#include "dma.h"
#include "usart.h"
#include "delay.h"
#include "linefollow.h"
#include "strategy.h"
extern int speedbuf[2];
extern int dirbuf[2];
extern int positionbuf[2];

void OrdinaryTimerInit6(u16 arr,u16 psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE); //ʱ��ʹ��
	
	//��ʱ��TIM6��ʼ��
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure); //����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
 
	TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM6�ж�,��������ж�

	//�ж����ȼ�NVIC����
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;  //TIM6�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  //�����ȼ�3��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //��ʼ��NVIC�Ĵ���
	
	TIM_Cmd(TIM6, ENABLE);  //ʹ��TIMx					 
}
void OrdinaryTimerInit7(u16 arr,u16 psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE); //ʱ��ʹ��
	
	
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure); //����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE );

	//�ж����ȼ�NVIC����
	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //�����ȼ�3��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //��ʼ��NVIC�Ĵ���
	
	TIM_Cmd(TIM7, ENABLE);  //ʹ��TIMx					 
}

void TIM6_IRQHandler(void)   //TIM6�ж�
{
	if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)  //���TIM6�����жϷ������
		{		
		int current_position[2];
		current_position[0]=Read_Encoder(2);current_position[1]=Read_Encoder(4);
		Get_Direction(dirbuf);
		if(leftoverflow==0&&rightoverflow==0)
		{
			speedbuf[0]=(current_position[0]-positionbuf[0])/640.0*600;//��һ��ʱ�������ζ�ȡencoder����x/��t�����Ƽ����ٶ�
			speedbuf[1]=(current_position[1]-positionbuf[1])/640.0*600;//
		}
		else
		{
			if(leftoverflow!=0)
			{
				if(dirbuf[0])//��Ϊ���������
					speedbuf[0] = (ENCODER_TIM_PERIOD-positionbuf[0]+current_position[0])/640.0*600;
				else//��Ϊ���������
					speedbuf[0] = -(ENCODER_TIM_PERIOD+positionbuf[0]-current_position[0])/640.0*600;
				leftoverflow=0;
			}
			if(rightoverflow!=0)
			{
				if(dirbuf[1])//��Ϊ���������
					speedbuf[1] = (ENCODER_TIM_PERIOD-positionbuf[1]+current_position[1])/640.0*600;
				else//��Ϊ���������
					speedbuf[1] = -(ENCODER_TIM_PERIOD+positionbuf[1]-current_position[1])/640.0*600;
				rightoverflow=0;
			}
		}
		positionbuf[0]=current_position[0];
		positionbuf[1]=current_position[1];
		
		if(IsCloseloop)
		{
			PID_setpoint(&PID_left,setnum_left);
			PID_setpoint(&PID_right,setnum_right);	
			pwmlevel_left+=incPIDcalc(&PID_left,speedbuf[0]);
			pwmlevel_right+=incPIDcalc(&PID_right,speedbuf[1]);
			Setpwm();
		}
			TIM_ClearITPendingBit(TIM6, TIM_IT_Update);  //���TIMx�����жϱ�־ 
	}
}
void TIM7_IRQHandler(void)
{
	 if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)  //���TIM6�����жϷ������
	 {
			linefollowloop();
			TIM_ClearITPendingBit(TIM7, TIM_IT_Update);  //���TIMx�����жϱ�־ 
	 }
}
