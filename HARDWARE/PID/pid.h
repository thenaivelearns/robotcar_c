#ifndef __PID_H
#define __PID_H

#include "sys.h"
#include "stm32f10x_tim.h"

#define  Kp       3.3//��������(0.32)
#define  Ti      0.2  //����ʱ�䳣��
#define  Td      0.0025//0.0028   ΢��ʱ�䳣��
#define T       0.1 //��������0.1ms������ٶȶ�ȡ��ʱ��ͬ��
#define Ki     Kp*(T/Ti)        // Kp Ki Kd ������Ҫ����
#define Kd     Kp*(Td/T)

#define A				Kp+Ki+Kd
#define B				Kp+2*Kd
#define C				Kd

extern int setnum_left,setnum_right;
extern u16 pwmlevel_left,pwmlevel_right;
extern int IsCloseloop;

typedef struct 
{
 int setpoint;//�趨Ŀ��
 int sum_error;//����ۼ�
 float proportion ;//��������
 float integral ;//����
 float derivative;//΢��
 int last_error;//e[-1]
 int prev_error;//e[-2]
}PIDtypedef;
extern PIDtypedef PID_left,PID_right;

void PIDperiodinit(u16 arr,u16 psc);        //PID ������ʱ���趨
void incPIDinit(PIDtypedef*PIDx);                //��ʼ������������
int incPIDcalc(PIDtypedef*PIDx,int nextpoint);           //PID����
void PID_setpoint(PIDtypedef*PIDx,int setvalue);  //�趨 PIDԤ��ֵ
void PID_set(PIDtypedef*PIDx,float pp,float ii,float dd);//�趨PID  kp ki kd��������
void PID_init(void);   //��ʼ������
void clockwise(void);
void counterclockwise(void);
void Setpwm(void);
void brake(void);
void releasebrake(void);
void reverse(void);
#endif

