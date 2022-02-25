#include "timer_encoder.h"
#include "usart.h"
#include "delay.h"

int leftoverflow=0;int rightoverflow=0;
	
void Encoder_Init_TIM24(void)	//CH1CH2�ֱ��ӦPA0PA1
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
  TIM_ICInitTypeDef TIM_ICInitStructure; 
  GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2|RCC_APB1Periph_TIM4, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	
	
	GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2,ENABLE);
	GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_6|GPIO_Pin_7;  
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	TIM_DeInit(TIM2);TIM_DeInit(TIM3);
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD;//���������Ҫ������̵�ʵ��������Լ����壬����������ÿתһȦ����һ�ζ�ʱ������жϣ��Ǿ���8-1=7
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);	

  TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);	
	
	TIM_ICStructInit(&TIM_ICInitStructure);//��Ҫ���ʾ�������ˡ���������Ĭ��ֻ��ͨ��1�������˲����Ҳ���⣬��Ϊ1��2��Ӧ�ñ��˲�����Ҫ�Ļ�������һ��
  TIM_ICInitStructure.TIM_ICFilter = 0x0A;
  TIM_ICInit(TIM2, &TIM_ICInitStructure);
	TIM_ICInit(TIM4, &TIM_ICInitStructure);
	

	TIM_ICInitStructure.TIM_Channel=TIM_Channel_2;
	TIM_ICInit(TIM2, &TIM_ICInitStructure);
	TIM_ICInit(TIM4, &TIM_ICInitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //�����ȼ�3��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //��ʼ��NVIC�Ĵ���
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_Init(&NVIC_InitStructure);  //��ʼ��NVIC�Ĵ���
	
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);TIM_ClearFlag(TIM4, TIM_FLAG_Update);
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
  //Reset counter
  TIM_SetCounter(TIM2,0);//����ֵ���㣬����������ǿ��ܻᾭ���õ���������һ��λ�ÿ�ʼ���ʱ�����ڸô��ü���ֵ����
  TIM_SetCounter(TIM4,0);
	
	//TIM_ARRPreloadConfig(TIM2, ENABLE);//��䲻֪���Ƿ���Ҫ��������
	TIM_Cmd(TIM2, ENABLE);
	TIM_Cmd(TIM4, ENABLE);
	
}

int Read_Encoder(u8 TIMX)//��ȡ��ʱ�̱������ļ���ֵ
												 //�������ϣ�����������Ϊw��ÿȦ��ת��ΪVȦ/min����ȡʱ����tӦ����t<=60/wV ��
{
   int Encoder_TIM;    
   switch(TIMX)	//�õ�����������/�������ӿڵ�ʱ��ֻ���ټ��ϼ���case������
   {
      case 2:  Encoder_TIM= (int)TIM2 -> CNT;break;
			case 4:  Encoder_TIM= (int)TIM4 -> CNT;break;
      default:  Encoder_TIM=0;
   }
   return Encoder_TIM;
}

void Get_Direction(int* dirptr)
{
		(*dirptr) = (uint16_t)(TIM2->CR1&(uint16_t)(1<<(uint16_t)4))==0;//����ֵ0�������ϼ�������֮��Ϊ���¼���
		*(dirptr+1) = (uint16_t)(TIM4->CR1&(uint16_t)(1<<(uint16_t)4))==0;//����ֵ0�������ϼ�������֮��Ϊ���¼���
}

void TIM2_IRQHandler(void)//����������/�������ʱ���ᴥ������ͨ����ȡTIMx->CR1��DIRλ���ж��������������������������������Get_Direction����
{                                   
    if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {           
        leftoverflow=1;
    }
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);//����жϱ�־λ
}

void TIM4_IRQHandler(void)//����������/�������ʱ���ᴥ������ͨ����ȡTIMx->CR1��DIRλ���ж��������������������������������Get_Direction����
{                                   
    if(TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
    {           
        rightoverflow=1;
    }
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);//����жϱ�־λ
}


