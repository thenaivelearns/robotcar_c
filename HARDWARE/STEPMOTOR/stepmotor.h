#ifndef __stepmotor_H
#define __stepmotor_H	 
#include "sys.h"
#define Pillar_speed 100

#define uint unsigned int
extern int elevate_angle;
void Delay_xms(uint x);//��ʱ����
void Stepmotor_Init(void);  //���������ʼ��
void Motorcw(int speed);    //���������ת����
void Motorcw1(int speed);   //�������1��ת����
void Motorccw(int speed);   //���������ת����
void Motorccw1(int speed);  //�������1��ת����
void elevate(int angle,int speed);  //���������ת�ǶȺ���
void Motorcw_angle(int angle,int speed);  //���������ת�ǶȺ���
void Motorcw_angle1(int angle,int speed); //�������1��ת�ǶȺ���
void Motorccw_angle(int angle,int speed); //���������ת�ǶȺ���
void Motorccw_angle1(int angle,int speed);//�������1��ת�ǶȺ���
void MotorStop(void);  //�������ֹͣ����
void MotorStop1(void); //�������1ֹͣ����

#endif
