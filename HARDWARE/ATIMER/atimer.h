#ifndef __HTIMER_H
#define __HTIMER_H

#include "sys.h"
#include "stm32f10x_tim.h"

void PWM_Configuration(u16 arr, u16 psc, u8 deadtime);

#endif
