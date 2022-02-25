#ifndef __ARM_H
#define __ARM_H
#include "sys.h"
#include "stm32f10x_tim.h"

void Arminit(void);
void Armmove(void);
void ResetArm(void);
void Fetch_forward(void);
void Pillar_start(void);
void Pillar_reset(void);
void Pillar_up(void);
void Fetch_from_pillar(void);
void Score_1ring_3(void);
void Score_2ring_3(void);
void Score_2ring_2(void);
extern int shoulder,elbow,wrist,finger;
#endif
