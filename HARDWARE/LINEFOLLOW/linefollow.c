#include "pid.h"
#include "adc.h"
#include "otimer.h"
#include "atimer.h"
#include "timer_encoder.h"
#include "linefollow.h"
#include "delay.h"
#include "usart.h"
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
       
float Kp_line = 3, Ki_line = 0.004*2, Kd_line = 6;
int error_line = 0, P = 0, I = 0, D = 0, PID_line = 0;
int prerror_line = 0, previous_I = 0;

int initial_motor_speed = SPEED_OF_ALL;//initial motor speed 充当了一个左右的setnum的平均值的这么一个作用


void linefollowloop()
{
  read_ir_values();
  calculate_pid();
  motor_control();
}

void read_ir_values()
{
  int irs=(int)(GPIOE->IDR&0xF);
//	printf("irs=%d,",irs);
  switch (irs) {
    
		/*白线之上则亮起，端口输出低电平*/
		case 15:
      if (error_line < 0) {
        error_line = -7;
      } else {
        error_line = 7;
      }
      break;  
    case 14: error_line = 5; break;//1110
    case 12: error_line = 3; break;//1100
    case 8: error_line = 1; break;//1000
    case 9: error_line = 0; break; //1001
    case 1: error_line = -1; break;//0001
    case 3: error_line = -3; break;//0011
    case 7: error_line = -5; break; //0111
    case 11: error_line = -5; break; //1011
		case 13: error_line = 5; break;  //1101
  }
}

void calculate_pid()
{
  P = error_line;
  I = I + error_line;
  D = error_line - prerror_line;

  PID_line = (Kp_line * P) + (Ki_line * I) + (Kd_line * D);
  PID_line = constrain(PID_line, -80, 80);
  prerror_line = error_line;
}

void motor_control()
{
  setnum_left = initial_motor_speed + PID_line;
  setnum_right = initial_motor_speed - PID_line;
  setnum_left=constrain(setnum_left, -80, 80);
	setnum_right=constrain(setnum_right, -80, 80);
}
void resetPIDline(void)
{
	error_line = 0;
	P = 0;
	I = 0;
	D = 0;
	PID_line = 0;
	prerror_line = 0;
}


