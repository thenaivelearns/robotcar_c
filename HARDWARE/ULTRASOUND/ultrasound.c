#include "sys.h"
#include "delay.h"
#include "usart.h"
void Ultrasound_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	TIM_ICInitTypeDef  TIM3_ICInitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);	//使能TIM3时钟
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
	
	TIM_TimeBaseStructure.TIM_Period = 0XFFFF; //设定计数器自动重装值 
	TIM_TimeBaseStructure.TIM_Prescaler =72-1; 	//预分频器   
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
  
	//初始化TIM3输入捕获参数
	TIM3_ICInitStructure.TIM_Channel = TIM_Channel_4; //CC4S=01 	选择输入端 IC1映射到TI1上
  TIM3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
  TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
  TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
  TIM3_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 配置输入滤波器 不滤波
  TIM_ICInit(TIM3, &TIM3_ICInitStructure);
	
	//中断分组初始化
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //先占优先级2级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //从优先级0级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器 
	
	TIM_ITConfig(TIM3,TIM_IT_Update|TIM_IT_CC4,ENABLE);//允许更新中断 ,允许CC4IE捕获中断	
	
   	TIM_Cmd(TIM3,ENABLE ); 	//使能定时器5
   
}

u8  TIM3CH4_CAPTURE_STA=0;	//输入捕获状态		    				
u16	TIM3CH4_CAPTURE_VAL;	//输入捕获值
 
//定时器3中断服务程序	 
void TIM3_IRQHandler(void)
{ 

 	if((TIM3CH4_CAPTURE_STA&0X80)==0)//还未成功捕获	
	{	  
		if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
		 
		{	    
			if(TIM3CH4_CAPTURE_STA&0X40)//已经捕获到高电平了
			{
				if((TIM3CH4_CAPTURE_STA&0X3F)==0X3F)//高电平太长了
				{
					TIM3CH4_CAPTURE_STA|=0X80;//标记成功捕获了一次
					TIM3CH4_CAPTURE_VAL=0XFFFF;
				}else TIM3CH4_CAPTURE_STA++;
			}	 
		}
	if (TIM_GetITStatus(TIM3, TIM_IT_CC4) != RESET)//捕获1发生捕获事件
		{	
			if(TIM3CH4_CAPTURE_STA&0X40)		//捕获到一个下降沿 		
			{	  			
				TIM3CH4_CAPTURE_STA|=0X80;		//标记成功捕获到一次高电平脉宽
				TIM3CH4_CAPTURE_VAL=TIM_GetCapture4(TIM3);
		   		TIM_OC4PolarityConfig(TIM3,TIM_ICPolarity_Rising); //CC4P=0 设置为上升沿捕获
			}else  								//还未开始,第一次捕获上升沿
			{
				TIM3CH4_CAPTURE_STA=0;			//清空
				TIM3CH4_CAPTURE_VAL=0;
	 			TIM_SetCounter(TIM3,0);
				TIM3CH4_CAPTURE_STA|=0X40;		//标记捕获到了上升沿
		   		TIM_OC4PolarityConfig(TIM3,TIM_ICPolarity_Falling);		//CC4P=1 设置为下降沿捕获
			}		    
		}			     	    					   
 	}
 
    TIM_ClearITPendingBit(TIM3, TIM_IT_CC4|TIM_IT_Update); //清除中断标志位
 
}

int Measure_DST(void)/*如果真实距离在5~50cm以外时调用了这个函数，可能会发生不小的错误！！！*/
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
			if(TIM3CH4_CAPTURE_STA&0X80)//成功捕获到了一次上升沿
			break;
			delay_us(500);
		}
		
		temp=((TIM3CH4_CAPTURE_STA&0X3F)*65536+TIM3CH4_CAPTURE_VAL)*17.0/1000;
		if(temp>5&&temp<50)
			dst+=temp;
		else
			i--;//这次采集作废，重新采				
		TIM3CH4_CAPTURE_STA=0;//开启下一次捕获			
	}
	return (dst/10);
}
