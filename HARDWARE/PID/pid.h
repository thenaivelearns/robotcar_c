#ifndef __PID_H
#define __PID_H

#include "sys.h"
#include "stm32f10x_tim.h"

#define  Kp       3.3//比例常数(0.32)
#define  Ti      0.2  //积分时间常数
#define  Td      0.0025//0.0028   微分时间常数
#define T       0.1 //采样周期0.1ms，想和速度读取的时间同步
#define Ki     Kp*(T/Ti)        // Kp Ki Kd 三个主要参数
#define Kd     Kp*(Td/T)

#define A				Kp+Ki+Kd
#define B				Kp+2*Kd
#define C				Kd

extern int setnum_left,setnum_right;
extern u16 pwmlevel_left,pwmlevel_right;
extern int IsCloseloop;

typedef struct 
{
 int setpoint;//设定目标
 int sum_error;//误差累计
 float proportion ;//比例常数
 float integral ;//积分
 float derivative;//微分
 int last_error;//e[-1]
 int prev_error;//e[-2]
}PIDtypedef;
extern PIDtypedef PID_left,PID_right;

void PIDperiodinit(u16 arr,u16 psc);        //PID 采样定时器设定
void incPIDinit(PIDtypedef*PIDx);                //初始化，参数清零
int incPIDcalc(PIDtypedef*PIDx,int nextpoint);           //PID计算
void PID_setpoint(PIDtypedef*PIDx,int setvalue);  //设定 PID预期值
void PID_set(PIDtypedef*PIDx,float pp,float ii,float dd);//设定PID  kp ki kd三个参数
void PID_init(void);   //初始化函数
void clockwise(void);
void counterclockwise(void);
void Setpwm(void);
void brake(void);
void releasebrake(void);
void reverse(void);
#endif

