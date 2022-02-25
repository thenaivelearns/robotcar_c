#include "remote.h"
#include "delay.h"
#include "usart.h"
#include "pid.h"
#include "linefollow.h"
#include "arm.h"
extern int startswitch;

//红外遥控初始化
//设置IO以及定时器4的输入捕获
void Remote_Init(void)    			  
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef  TIM_ICInitStructure;  
 
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE); //使能GPIOB时钟 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);	//TIM4 时钟使能 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;				 //PB9 输入 
// 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 		//下拉输入
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 		//上拉输入，外接了上拉，这里也许浮空都OK
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOB, &GPIO_InitStructure);
 	GPIO_SetBits(GPIOB,GPIO_Pin_1);	//初始化GPIOB.9
	
						  
 	TIM_TimeBaseStructure.TIM_Period = 10000; //设定计数器自动重装值 最大10ms溢出  
	TIM_TimeBaseStructure.TIM_Prescaler =(72-1); 	//预分频器,1M的计数频率,1us.	   
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式

	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx

  TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;  // 选择输入端 IC4映射到TI4上
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
  TIM_ICInitStructure.TIM_ICFilter = 0x03;//IC4F=0011 配置输入滤波器 8个定时器时钟周期滤波
  TIM_ICInit(TIM3, &TIM_ICInitStructure);//初始化定时器输入捕获通道

  TIM_Cmd(TIM3,ENABLE); 	//使能定时器4
 
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM4中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器	

	TIM_ITConfig(TIM3,TIM_IT_CC4,ENABLE);//允许更新中断 ,允许CC4IE捕获中断								 
}

//遥控器接收状态
//[7]:收到了引导码标志
//[6]:得到了一个按键的所有信息
//[5]:保留	
//[4]:标记上升沿是否已经被捕获								   
//[3:0]:溢出计时器
u8 	RmtSta=0;	  	  
u16 Dval;		//下降沿时计数器的值
u32 RmtRec=0;	//(它才是我们最终要得到的)红外接收到的数据，地址码8位，地址反码8位，控制码8位，控制反码8位	   		    
//定时器4中断服务程序	 
/*
void TIM3_IRQHandler(void)
{ 		    	 
	u8 key=0;		
	if(TIM_GetITStatus(TIM3,TIM_IT_CC4)!=RESET)
	{	  
		if(RDATA)//上升沿捕获；RDATA是PB9的宏
		{
  		TIM_OC4PolarityConfig(TIM3,TIM_ICPolarity_Falling);						//CC4P=1	设置为下降沿捕获
			TIM_SetCounter(TIM3,0);							//清空定时器值
			RmtSta|=0X10;							//标记上升沿已经被捕获
		}else //下降沿捕获
		{
			Dval=TIM_GetCapture4(TIM3);					//读取CCR4也可以清CC4IF标志位.查到的资料上说，捕获完成时，CNT的值会被存入CCR，所以用GetCapture而不是GetCounter也是可以的
  		TIM_OC4PolarityConfig(TIM3,TIM_ICPolarity_Rising);				//CC4P=0	设置为上升沿捕获
			if(RmtSta&0X10)							//完成一次高电平捕获 
			{
 				if(RmtSta&0X80)//接收到了引导码
				{
					
					if(Dval>300&&Dval<800)			//560为标准值,560us
					{
						RmtRec<<=1;					//左移一位.
						RmtRec|=0;					//接收到0	   
					}else if(Dval>1400&&Dval<1800)	//1680为标准值,1680us
					{
						RmtRec<<=1;					//左移一位.
						RmtRec|=1;					//接收到1
					}else if(Dval>2200&&Dval<2600)	//得到按键键值增加的信息 2500为标准值2.5ms (本来是连发码）
					{
						key=RmtRec>>8;	
					}
 				}else if(Dval>4200&&Dval<4700)		//4500为标准值4.5ms
				{
					RmtSta|=1<<7;					//标记成功接收到了引导码
				}						 
			}
			RmtSta&=~(1<<4);//清除之前的上升沿标志
		}
	
	if(key!=0)
	{	PBout(8)=1;
		delay_ms(500);
		PBout(8)=0;
		
		TIM_ITConfig(TIM3,TIM_IT_CC4,DISABLE);
		delay_ms(500);//躲开连发的其余部分
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
				case 66:IsCloseloop=0;break;//进入开环状态;break;//str="0";keyflag = key;break;
				case 82:IsCloseloop=1;break;//str="DELETE";keyflag = key;break;
			}

		RmtSta=0;
		RmtRec=0;
		TIM_ITConfig(TIM3,TIM_IT_CC4,ENABLE);
	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update|TIM_IT_CC4);
	}
}*/
