
#ifndef __LINEFOLLOW_H
#define __LINEFOLLOW_H
#include "sys.h"

#define SPEED_OF_ALL 80
extern float Kp_line , Ki_line , Kd_line ;
extern int error_line , P , I , D , PID_line ;
extern int prerror_line, previous_I;
extern  int initial_motor_speed; 
extern unsigned int irs;

void read_ir_values(void);
void calculate_pid(void);
void motor_control(void);
void motorsWrite(int setnum_left, int setnum_right);
void linefollowloop(void);
void resetPIDline(void);

#endif



