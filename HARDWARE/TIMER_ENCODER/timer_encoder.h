#ifndef __TIMER_ENCODER_H
#define __TIMER_ENCODER_H

#include "sys.h"
#include "stm32f10x_tim.h"

#define ENCODER_TIM_PERIOD 0xFFFF //�������������Ӧ���Ǽ���ǰ��rmp
#define CIR_OF_WHEEL 21.5	//������Ϊ��λ���޸���ʱ�Ĵֲ�����
void Encoder_Init_TIM24(void);
int Read_Encoder(u8 TIMX);
void Get_Direction(int* dirptr);
extern int leftoverflow;
extern int rightoverflow;
#endif
