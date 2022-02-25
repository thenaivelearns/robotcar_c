//寄存器版本的代码来源：http://www.openedv.com/posts/list/62861.htm
//库函数版本的代码来源：https://blog.csdn.net/geek_monkey/article/details/82079435
                //以及：https://blog.csdn.net/dazhou158/article/details/8806174
								//以及：https://blog.csdn.net/u014170207/article/details/50885511
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
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;//TIM8 CH1 a7c6
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 		GPIO_Init(GPIOA, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;//TIM8 CH2 b0c7
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
		GPIO_Init(GPIOC, &GPIO_InitStructure);
		
	
	//刹车通道的引脚使能输入	
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;//输入模式众说纷纭，我认为要考虑刹车有效电平。
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;//TIM1 BRAKE	
		GPIO_Init(GPIOE, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;	//TIM8 BRAKE
		GPIO_Init(GPIOA, &GPIO_InitStructure);
	
    GPIO_SetBits(GPIOE, GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11);
		GPIO_SetBits(GPIOA, GPIO_Pin_7);
		GPIO_SetBits(GPIOB, GPIO_Pin_0);
		GPIO_SetBits(GPIOC, GPIO_Pin_6|GPIO_Pin_7);
	
//刹车信号源 引脚配置	
		
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;	
		GPIO_Init(GPIOC, &GPIO_InitStructure); //PC8PC9
		GPIO_ResetBits(GPIOC,GPIO_Pin_8|GPIO_Pin_9);
		

    TIM_BaseInitStructure.TIM_Period = arr;    //除了arr和psc，别的都是默认配置
    TIM_BaseInitStructure.TIM_Prescaler = psc;
    TIM_BaseInitStructure.TIM_ClockDivision = 0;
    TIM_BaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_BaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_BaseInitStructure);
		TIM_TimeBaseInit(TIM8, &TIM_BaseInitStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//定时器通道配置，大部分都是默认状态
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;//指定通道刹车以后的输出状态，为低电平
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;//指定互补通道刹车以后的输出状态，为低电平

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

    //死区时间设置
    TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
    TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
		TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1;//设置寄存器锁定功能
		//TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_2;
    TIM_BDTRInitStructure.TIM_DeadTime = deadtime;
		
		//刹车功能设置
	
		TIM_BDTRInitStructure.TIM_Break = TIM_Break_Enable;//刹车功能使能
    TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;//刹车引脚的有效电平，可以令PWM停止,默认从PB12输入
		TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;//自动输出使能，操作的是AOE位，我觉得这一步选disable问题也不大
		TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);
		TIM_BDTRConfig(TIM8, &TIM_BDTRInitStructure);
		
		TIM_CCPreloadControl(TIM1, ENABLE);TIM_CCPreloadControl(TIM8, ENABLE); //这句先留着吧
		TIM_ARRPreloadConfig(TIM1, ENABLE);TIM_ARRPreloadConfig(TIM8, ENABLE);//这一步非常重要，使能ARPE寄存器		
    //主输出bit使能（MOE位使能）
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
		TIM_CtrlPWMOutputs(TIM8, ENABLE);
    //TIM1启动
    TIM_Cmd(TIM1, ENABLE);
		TIM_Cmd(TIM8, ENABLE);
}
