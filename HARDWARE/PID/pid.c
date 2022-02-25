#include "pid.h"
#include "atimer.h"
#include "delay.h"
#include "timer_encoder.h"
#include "linefollow.h"
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
int setnum_left=0;//300Hz对应113rpm，输出频率与转速之比为8:3，转速单位rpm
int setnum_right=0;
int IsCloseloop = 1;
PIDtypedef PID_left,PID_right;
extern int speedbuf[2];
extern int lefdelay,rigdelay;
u16 pwmlevel_left=500;
u16 pwmlevel_right=500;

int secutst_incpid=0;

int incPIDcalc(PIDtypedef *PIDx,int nextpoint)  //PID计算
{
	int iError,iincpid;
	iError=PIDx->setpoint-nextpoint;  //当前误差

	iincpid=                                               //增量计算
		(int)(PIDx->proportion*iError                //e[k]项
		-PIDx->integral*PIDx->last_error          //e[k-1]
		+PIDx->derivative*PIDx->prev_error);//e[k-2]
	PIDx->prev_error=PIDx->last_error; //存储误差，便于下次计算
	PIDx->last_error=iError;
	secutst_incpid = (secutst_incpid)<(iincpid)?(iincpid):(secutst_incpid);
	return iincpid;
		//return constrain(iincpid,-125,125);
		
}				

void PID_setpoint(PIDtypedef*PIDx,int setvalue)  //设定 PID预期值
{
	PIDx->setpoint =setvalue;
}

void PIDperiodinit(u16 arr,u16 psc)        //PID 采样定时器设定
{
	
}

void incPIDinit(PIDtypedef*PIDx)//初始化，参数清零
{
	PIDx->last_error=0;
	PIDx->prev_error=0;
	PIDx->sum_error=0;
}

void PID_set(PIDtypedef*PIDx,float pp,float ii,float dd)//设定PID  kp ki kd三个参数
{
	PIDx->proportion=pp;
	PIDx->integral=ii;
	PIDx->derivative=dd;
}

void PID_init(void)//pid初始化
{	
	 PID_setpoint(&PID_left,setnum_left);
	 PID_setpoint(&PID_right,setnum_right);	 
	 PID_set(&PID_left,A,B,C);
	 PID_set(&PID_right,(float)A,(float)B,(float)C);
	 incPIDinit(&PID_left);
	 incPIDinit(&PID_right);
}

void clockwise(void)
{  //顺时针旋转
	IsCloseloop=0;
	TIM_SetCompare1(TIM1,800);//可以修改通道1的占空比，通道2也类似
	TIM_SetCompare2(TIM8,800);
	TIM_SetCompare1(TIM8,200);
	TIM_SetCompare2(TIM1,200);		
	delay_ms(1200);
	while(1)
	{ 
		if(PEin(0)==0)
		{ 
			while(PEin(2)!=0);
			break;
		}
		/*while(PEin(2)!=0);
		delay_ms(50);			
	*/
		}
	//delay_ms(200);//待修改，纯粹只是为了凑一下
	brake();
	delay_ms(200);//防止突然启动
	setnum_left=initial_motor_speed;
	setnum_right=initial_motor_speed;
	resetPIDline();incPIDinit(&PID_left);incPIDinit(&PID_right);
	IsCloseloop=1;
	releasebrake();
}
void counterclockwise(void)
{
	IsCloseloop=0;
	TIM_SetCompare1(TIM1,200);//可以修改通道1的占空比，通道2也类似
	TIM_SetCompare2(TIM8,200);
	TIM_SetCompare1(TIM8,800);
	TIM_SetCompare2(TIM1,800);	
		
	delay_ms(1200);
	while(1)
	{ 
		/*if(PEin(3)==0)
		{ 
			while(PEin(1)!=0);
			break;
		}*/
		 
		while(PEin(1)!=0);
		break;
		
		delay_ms(50);			
	}
	//delay_ms(lefdelay);//待修改，纯粹只是为了凑一下
	brake();
	delay_ms(200);//防止突然启动
	setnum_left=initial_motor_speed;
	setnum_right=initial_motor_speed;
	resetPIDline();incPIDinit(&PID_left);incPIDinit(&PID_right);
	IsCloseloop=1;
	releasebrake();
}


/*两倍区放完后用这个转弯*/
void reverse(void)
{
	//大车的掉头会碰到垂直的线

	IsCloseloop=0;
	releasebrake();
	TIM_SetCompare1(TIM1,200);
	TIM_SetCompare2(TIM8,200);
	TIM_SetCompare1(TIM8,800);
	TIM_SetCompare2(TIM1,800);	
	delay_ms(1200);
	
	while(1)
	{ 
		if(PEin(0)==0)
		{ 
			while(PEin(2)!=0);
			break;
		}
		delay_ms(50);			
	}
	//while(PEin(1)!=0);
	brake();
	delay_ms(800);//防止颤动
	resetPIDline();incPIDinit(&PID_left);incPIDinit(&PID_right);
	IsCloseloop=1;
	releasebrake();
}

void Setpwm(void){
	TIM_SetCompare1(TIM1,pwmlevel_left);//可以修改通道1的占空比，通道2也类似
	TIM_SetCompare2(TIM8,pwmlevel_left);
	TIM_SetCompare1(TIM8,pwmlevel_right);
	TIM_SetCompare2(TIM1,pwmlevel_right);
}
void brake(void)
{
	PCout(8)=1;
	PCout(9)=1;
	TIM_SetCompare1(TIM1,500);
	TIM_SetCompare2(TIM8,500);
	TIM_SetCompare1(TIM8,500);
	TIM_SetCompare2(TIM1,500);
}
void releasebrake(void)
{
	PCout(8)=0;
	PCout(9)=0;
}
