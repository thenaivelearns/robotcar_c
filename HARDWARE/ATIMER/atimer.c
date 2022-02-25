//�Ĵ����汾�Ĵ�����Դ��http://www.openedv.com/posts/list/62861.htm
//�⺯���汾�Ĵ�����Դ��https://blog.csdn.net/geek_monkey/article/details/82079435
                //�Լ���https://blog.csdn.net/dazhou158/article/details/8806174
								//�Լ���https://blog.csdn.net/u014170207/article/details/50885511
#include "usart.h"
#include "delay.h"

void PWM_Configuration(u16 arr, u16 psc, u8 deadtime)
{
    GPIO_InitTypeDef    GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_BaseInitStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
//    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOE|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOB,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1|RCC_APB2Periph_TIM8, ENABLE);
		
	  GPIO_PinRemapConfig(GPIO_FullRemap_TIM1,ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11;//TIM1_CH1,TIM1_CH2
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;//TIM8 CH1 a7c6
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 		GPIO_Init(GPIOA, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;//TIM8 CH2 b0c7
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
		GPIO_Init(GPIOC, &GPIO_InitStructure);
		
	
	//ɲ��ͨ��������ʹ������	
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;//����ģʽ��˵��硣�����ΪҪ����ɲ����Ч��ƽ��
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;//TIM1 BRAKE	
		GPIO_Init(GPIOE, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;	//TIM8 BRAKE
		GPIO_Init(GPIOA, &GPIO_InitStructure);
	
    GPIO_SetBits(GPIOE, GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11);
		GPIO_SetBits(GPIOA, GPIO_Pin_7);
		GPIO_SetBits(GPIOB, GPIO_Pin_0);
		GPIO_SetBits(GPIOC, GPIO_Pin_6|GPIO_Pin_7);
	
//ɲ���ź�Դ ��������	
		
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;	
		GPIO_Init(GPIOC, &GPIO_InitStructure); //PC8PC9
		GPIO_ResetBits(GPIOC,GPIO_Pin_8|GPIO_Pin_9);
		

    TIM_BaseInitStructure.TIM_Period = arr;    //����arr��psc����Ķ���Ĭ������
    TIM_BaseInitStructure.TIM_Prescaler = psc;
    TIM_BaseInitStructure.TIM_ClockDivision = 0;
    TIM_BaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_BaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_BaseInitStructure);
		TIM_TimeBaseInit(TIM8, &TIM_BaseInitStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//��ʱ��ͨ�����ã��󲿷ֶ���Ĭ��״̬
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;//ָ��ͨ��ɲ���Ժ�����״̬��Ϊ�͵�ƽ
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;//ָ������ͨ��ɲ���Ժ�����״̬��Ϊ�͵�ƽ

    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
		TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
		TIM_OC2Init(TIM1, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
		TIM_OC1Init(TIM8, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
		TIM_OC2Init(TIM8, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);

    //����ʱ������
    TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
    TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
		TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1;//���üĴ�����������
		//TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_2;
    TIM_BDTRInitStructure.TIM_DeadTime = deadtime;
		
		//ɲ����������
	
		TIM_BDTRInitStructure.TIM_Break = TIM_Break_Enable;//ɲ������ʹ��
    TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;//ɲ�����ŵ���Ч��ƽ��������PWMֹͣ,Ĭ�ϴ�PB12����
		TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;//�Զ����ʹ�ܣ���������AOEλ���Ҿ�����һ��ѡdisable����Ҳ����
		TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);
		TIM_BDTRConfig(TIM8, &TIM_BDTRInitStructure);
		
		TIM_CCPreloadControl(TIM1, ENABLE);TIM_CCPreloadControl(TIM8, ENABLE); //��������Ű�
		TIM_ARRPreloadConfig(TIM1, ENABLE);TIM_ARRPreloadConfig(TIM8, ENABLE);//��һ���ǳ���Ҫ��ʹ��ARPE�Ĵ���		
    //�����bitʹ�ܣ�MOEλʹ�ܣ�
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
		TIM_CtrlPWMOutputs(TIM8, ENABLE);
    //TIM1����
    TIM_Cmd(TIM1, ENABLE);
		TIM_Cmd(TIM8, ENABLE);
}
