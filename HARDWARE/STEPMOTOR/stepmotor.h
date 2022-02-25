#ifndef __stepmotor_H
#define __stepmotor_H	 
#include "sys.h"
#define Pillar_speed 100

#define uint unsigned int
extern int elevate_angle;
void Delay_xms(uint x);//延时函数
void Stepmotor_Init(void);  //步进电机初始化
void Motorcw(int speed);    //步进电机正转函数
void Motorcw1(int speed);   //步进电机1正转函数
void Motorccw(int speed);   //步进电机反转函数
void Motorccw1(int speed);  //步进电机1反转函数
void elevate(int angle,int speed);  //步进电机正转角度函数
void Motorcw_angle(int angle,int speed);  //步进电机正转角度函数
void Motorcw_angle1(int angle,int speed); //步进电机1正转角度函数
void Motorccw_angle(int angle,int speed); //步进电机反转角度函数
void Motorccw_angle1(int angle,int speed);//步进电机1反转角度函数
void MotorStop(void);  //步进电机停止函数
void MotorStop1(void); //步进电机1停止函数

#endif
