#include "timer_encoder.h"
#include "usart.h"
#include "delay.h"

int leftoverflow=0;int rightoverflow=0;
	
void Encoder_Init_TIM24(void)	//CH1CH2分别对应PA0PA1
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
  TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD;//这个东西需要结合码盘的实际情况来自己定义，这里是轮子每转一圈触发一次定时器溢出中断，那就是8-1=7
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);	

  TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);	
	
	TIM_ICStructInit(&TIM_ICInitStructure);//主要疑问就在这句了。这句代码中默认只对通道1进行了滤波，我不理解，认为1和2都应该被滤波，需要的话加上下一句
  TIM_ICInitStructure.TIM_ICFilter = 0x0A;
  TIM_ICInit(TIM2, &TIM_ICInitStructure);
	TIM_ICInit(TIM4, &TIM_ICInitStructure);
	

	TIM_ICInitStructure.TIM_Channel=TIM_Channel_2;
	TIM_ICInit(TIM2, &TIM_ICInitStructure);
	TIM_ICInit(TIM4, &TIM_ICInitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器
	
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);TIM_ClearFlag(TIM4, TIM_FLAG_Update);
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
  //Reset counter
  TIM_SetCounter(TIM2,0);//计数值清零，这个函数我们可能会经常用到，比如在一个位置开始测距时，先在该处让计数值清零
  TIM_SetCounter(TIM4,0);
	
	//TIM_ARRPreloadConfig(TIM2, ENABLE);//这句不知道是否需要，先留着
	TIM_Cmd(TIM2, ENABLE);
	TIM_Cmd(TIM4, ENABLE);
	
}

int Read_Encoder(u8 TIMX)//获取该时刻编码器的计数值
												 //网上资料：编码器线数为w线每圈，转速为V圈/min，读取时间间隔t应满足t<=60/wV 秒
{
   int Encoder_TIM;    
   switch(TIMX)	//用到多个电机码盘/编码器接口的时候，只需再加上几个case就行了
   {
      case 2:  Encoder_TIM= (int)TIM2 -> CNT;break;
			case 4:  Encoder_TIM= (int)TIM4 -> CNT;break;
      default:  Encoder_TIM=0;
   }
   return Encoder_TIM;
}

void Get_Direction(int* dirptr)
{
		(*dirptr) = (uint16_t)(TIM2->CR1&(uint16_t)(1<<(uint16_t)4))==0;//返回值0代表向上计数，反之则为向下计数
		*(dirptr+1) = (uint16_t)(TIM4->CR1&(uint16_t)(1<<(uint16_t)4))==0;//返回值0代表向上计数，反之则为向下计数
}

void TIM2_IRQHandler(void)//计数器向上/向下溢出时均会触发。可通过读取TIMx->CR1的DIR位来判断是向上溢出还是向下溢出，这点类似于Get_Direction函数
{                                   
    if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {           
        leftoverflow=1;
    }
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);//清除中断标志位
}

void TIM4_IRQHandler(void)//计数器向上/向下溢出时均会触发。可通过读取TIMx->CR1的DIR位来判断是向上溢出还是向下溢出，这点类似于Get_Direction函数
{                                   
    if(TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
    {           
        rightoverflow=1;
    }
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);//清除中断标志位
}


