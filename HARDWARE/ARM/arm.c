#include "arm.h"
#include "delay.h"
#include "stepmotor.h"
#include "pid.h"
#include "linefollow.h"

/*constrain*/
//自下到上分别是channel1234，所有都是增大输入时顺时针转/*以舵机朝人为观察面*/
//空载时的限位值
//肩膀996R，下限45，上限250
//肘部996R，上限250，下限45（到40后会突然松掉，但是可以回来）
//手腕SG90,270为上限，和手臂垂直，270以上时会有些危险，下线050，再减小会碰到手臂
//手指SG90，170恰好夹住，200时并拢，125张开到最大

//should，增大顺时针旋转
//eblow，增大逆时针旋转（在函数入口通过300-input进行修正）
//wrist，增大顺时针旋转
//finger，增大是闭合

int final_gain=0;
int shoulder=187,elbow=45,wrist=100,finger=100;
 int i;
/*柱子相关*/
void Pillar_up(void)/*柱子上升*/
{
		shoulder=187;elbow=45;wrist=110;finger=100;
		Armmove();
	  elevate(140,Pillar_speed);
}

void Pillar_start(void)/*开机复位柱子*/
{
		elevate(-80,Pillar_speed);
}

void Pillar_reset(void)/*得分后复位柱子*/
{
		elevate(-140,Pillar_speed);
}



void Arminit(void)
{
		GPIO_InitTypeDef    GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_BaseInitStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
//    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    TIM_BaseInitStructure.TIM_Period = 2000-1;
    TIM_BaseInitStructure.TIM_Prescaler = 720-1;
    TIM_BaseInitStructure.TIM_ClockDivision = 0;
    TIM_BaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM5, &TIM_BaseInitStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//定时器通道配置，大部分都是默认状态
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		
		TIM_OC1Init(TIM5, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);
		TIM_OC2Init(TIM5, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);
		TIM_OC3Init(TIM5, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);
		TIM_OC4Init(TIM5, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable);
		
    TIM_Cmd(TIM5, ENABLE);
		Armmove();
}

void Armmove(void)
{
	delay_ms(20);
	TIM_SetCompare2(TIM5,300-elbow);
	delay_ms(20);
	TIM_SetCompare1(TIM5,shoulder);
	delay_ms(20);
	TIM_SetCompare3(TIM5,wrist);
	delay_ms(20);
	TIM_SetCompare4(TIM5,finger);
}

void ResetArm(void)
{
	shoulder=187;elbow=45;wrist=100;finger=100;
	Armmove();
}

void Fetch_forward(void)/*之后肯定要优化，如何确保抓得准*/
{
	shoulder=200;elbow=230;wrist=80;finger=100;Armmove();//贴地缓冲
	delay_ms(300);
	shoulder=218;wrist=70;Armmove();//贴地并打开爪子
	delay_ms(500);
  resetPIDline();incPIDinit(&PID_left);incPIDinit(&PID_right);
	IsCloseloop=1;
	releasebrake();

	while((GPIOE->IDR&0XF)!=0)
	{	
	}
	IsCloseloop=0;
	brake();
	finger=150;Armmove();//抓住
	delay_ms(1000);
	shoulder=189;wrist=90;Armmove();//起手缓冲	
	delay_ms(200);//这个延时很短  
	shoulder=189;elbow=45;wrist=100;Armmove();
	delay_ms(1000);
	finger=100;Armmove();//释放
	delay_ms(1000);
	//根据原本的时序，抓东西的时候无法转弯，只能闭环巡直线
	//因此走远了可能会丢线，所以在想要不要把转弯检测并执行的代码合并到红外巡线的中断里面去
	ResetArm();
}

void Fetch_from_pillar(void){
//注意，这个函数启用了以后，小车是处于刹车状态的
	  int i=1;IsCloseloop=0;brake();
	shoulder=180;elbow=228;wrist=115;finger=100;Armmove();//贴近柱子
	delay_ms(1000);
	resetPIDline();incPIDinit(&PID_left);incPIDinit(&PID_right);
	releasebrake();IsCloseloop=1;
  delay_ms(700);
	IsCloseloop=0;brake();
	delay_ms(500);//防止机械手没有到位
  finger=150;Armmove();//抓紧
	delay_ms(1000);
	for(;i<3;i++)
	{    //离开柱子
	  shoulder-=11;delay_ms(300);Armmove();
		elbow-=8;delay_ms(300);Armmove();
		wrist+=8;delay_ms(300);Armmove();
		delay_ms(500);
	}
	delay_ms(500);
	elbow-=10;Armmove();
	shoulder=187;elbow=45;wrist=100;Armmove();
	delay_ms(1000);
	finger=100;Armmove();//释放
	delay_ms(500);
	ResetArm();
}
/*三倍区放1个环*/
void Score_1ring_3(void){
  shoulder=182;wrist=100;finger=100;Armmove();//紧靠能量环
	delay_ms(500);
	finger=150;Armmove();
	delay_ms(500);
	shoulder=187;wrist=120;Armmove();
	delay_ms(500);
	shoulder=175;elbow=195;wrist=150;Armmove();
  delay_ms(1000);
	finger=100;Armmove();//释放
	delay_ms(500);
	ResetArm();
}


/*在三倍区放两个环*/
void Score_2ring_3(void){//175 195 150 100
  shoulder=182;wrist=100;finger=100;Armmove();//紧靠能量环
	delay_ms(500);
	finger=150;Armmove();//抓紧
	delay_ms(500);
	shoulder=187;wrist=120;Armmove();//起手缓冲
	delay_ms(500);
	shoulder=175;elbow=190;wrist=150;Armmove();
	/*for(i=1;i<=5;i++){
	elbow+=30;wrist+=6;Armmove();
	delay_ms(300);
	}*/
  delay_ms(1000);
	finger=100;Armmove();//释放
	delay_ms(500);
	ResetArm();
	
  Pillar_up();  
	
	shoulder=182;wrist=100;finger=100;Armmove();//紧靠能量环
	delay_ms(500);
	finger=150;Armmove();
	delay_ms(500);
	shoulder=187;wrist=120;Armmove();
	delay_ms(500);
	shoulder=175;elbow=190;wrist=150;Armmove();
  delay_ms(1000);
	finger=100;Armmove();//释放
	delay_ms(500);
	ResetArm();	
	Pillar_reset();
}
/*在二倍区放一个环*/
void Score_2ring_2(void){
  shoulder=182;wrist=100;finger=100;Armmove();//紧靠能量环
	delay_ms(500);
	finger=150;Armmove();
	delay_ms(500);
	shoulder=187;wrist=120;Armmove();
	delay_ms(500);
	shoulder=190;elbow=220;wrist=110;Armmove();
  delay_ms(1000);
	finger=100;Armmove();//释放
	delay_ms(500);
	ResetArm();
	
	Pillar_up();
	if(final_gain==1){ 
		releasebrake();
		IsCloseloop=0;
		pwmlevel_left=700;pwmlevel_right=300;Setpwm();
		delay_ms(800);
		pwmlevel_left=700;pwmlevel_right=700;Setpwm();
		delay_ms(1000);
		brake();
	}
	shoulder=182;wrist=100;finger=100;Armmove();//紧靠能量环
	delay_ms(500);
	finger=150;Armmove();
	delay_ms(500);
	shoulder=187;wrist=120;Armmove();
	delay_ms(500);
	shoulder=190;elbow=220;wrist=110;Armmove();
  delay_ms(1000);
	finger=100;Armmove();//释放
	delay_ms(500);
	ResetArm();
	Pillar_reset();
	final_gain++;
}
