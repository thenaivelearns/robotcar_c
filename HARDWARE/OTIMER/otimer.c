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

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE); //时钟使能
	
	//定时器TIM6初始化
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位
 
	TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE ); //使能指定的TIM6中断,允许更新中断

	//中断优先级NVIC设置
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;  //TIM6中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器
	
	TIM_Cmd(TIM6, ENABLE);  //使能TIMx					 
}
void OrdinaryTimerInit7(u16 arr,u16 psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE); //时钟使能
	
	
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位
	TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE );

	//中断优先级NVIC设置
	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器
	
	TIM_Cmd(TIM7, ENABLE);  //使能TIMx					 
}

void TIM6_IRQHandler(void)   //TIM6中断
{
	if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)  //检查TIM6更新中断发生与否
		{		
		int current_position[2];
		current_position[0]=Read_Encoder(2);current_position[1]=Read_Encoder(4);
		Get_Direction(dirbuf);
		if(leftoverflow==0&&rightoverflow==0)
		{
			speedbuf[0]=(current_position[0]-positionbuf[0])/640.0*600;//在一段时间内两次读取encoder，△x/△t来近似计算速度
			speedbuf[1]=(current_position[1]-positionbuf[1])/640.0*600;//
		}
		else
		{
			if(leftoverflow!=0)
			{
				if(dirbuf[0])//认为是向上溢出
					speedbuf[0] = (ENCODER_TIM_PERIOD-positionbuf[0]+current_position[0])/640.0*600;
				else//认为是向下溢出
					speedbuf[0] = -(ENCODER_TIM_PERIOD+positionbuf[0]-current_position[0])/640.0*600;
				leftoverflow=0;
			}
			if(rightoverflow!=0)
			{
				if(dirbuf[1])//认为是向上溢出
					speedbuf[1] = (ENCODER_TIM_PERIOD-positionbuf[1]+current_position[1])/640.0*600;
				else//认为是向下溢出
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
			TIM_ClearITPendingBit(TIM6, TIM_IT_Update);  //清除TIMx更新中断标志 
	}
}
void TIM7_IRQHandler(void)
{
	 if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)  //检查TIM6更新中断发生与否
	 {
			linefollowloop();
			TIM_ClearITPendingBit(TIM7, TIM_IT_Update);  //清除TIMx更新中断标志 
	 }
}
