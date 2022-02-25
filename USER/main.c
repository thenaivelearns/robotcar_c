#include "stm32f10x.h"
#include "delay.h"
#include "timer_encoder.h"
#include "usart.h"
#include "atimer.h"
#include "pid.h"
#include "remote.h"
#include "beep.h"
#include "otimer.h"
#include "infrared.h"
#include "stepmotor.h"
#include "arm.h"
#include "strategy.h"
#include "ultrasound.h"
#include "linefollow.h"
int speedbuf[2]={0,0};int dirbuf[2]={0,0}; int positionbuf[2]={0,0};
int startswitch=0;
int dst=0;
 int main(void)
{		 
	 NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	 delay_init();
	 uart_init(115200);	 //���ڳ�ʼ��Ϊ115200
	 infrared_init();
	 Encoder_Init_TIM24();
	 Arminit();
	 Stepmotor_Init();
	 PWM_Configuration(999,0,0x14);//һ��CLKʱ������0.1ms��һ����������50ms������ʱ��2ms
	 Setpwm();
	 Ultrasound_Init();
	 delay_ms(500);//��������ϵ緢��
	 Pillar_start();//�ϵ�ʱ�Զ���������Ϊ����
//	 

//	 delay_ms(1000);
//	 delay_ms(1000);
	 	 /*
	 while(startswitch==0)
	 {
		 printf("not in loop\n");
		 delay_ms(500);
	 }*/
	 PID_init();
	 OrdinaryTimerInit6(999,7199);//10Khz�ļ���Ƶ�ʣ�������1000Ϊ100ms,�ٶȶ�ȡ���ٶȱջ����� 	 
	 OrdinaryTimerInit7(999,7199);//10Khz�ļ���Ƶ�ʣ�������1000Ϊ100ms,Ѳ������
	 resetPIDline();
	 delay_ms(1500);
	 
//while(1){ }//printf("%d %d \n",TIM2->CNT,TIM4->CNT);delay_ms(1000);}
	 while(*crossmark!= ENDOFARRAY)
	 {
			while(1)
			{ 
				if(PEin(6)==0)
				{ 
					//while(PEin(6)!=0);
					switch (*(crossmark++))
					{
						case KEEPMOVE:delay_ms(500);break;//��ֹ��һ���ط������е�ת��������ù�
						case TURNLEFT:counterclockwise();delay_ms(1000);initial_motor_speed=SPEED_OF_ALL;break;
						case TURNRIGHT:clockwise();delay_ms(1000);break;
						//������
						case FORWARD_RING:
						  IsCloseloop=0;
							brake();
							initial_motor_speed = 20;
							Fetch_forward();
							resetPIDline();incPIDinit(&PID_left);incPIDinit(&PID_right);
						  IsCloseloop=1;
							releasebrake();
							initial_motor_speed=40;//setnum_left=SPEED_OF_ALL;setnum_right=SPEED_OF_ALL;					
							/*���ﲻ��ʱ,ƽ��������ֱ�Ӷ���һ����*/			
							break;
						case TURNLEFT_RING:
							counterclockwise();
							IsCloseloop=0;
							brake();
							initial_motor_speed = 20;
							Fetch_forward();
							resetPIDline();incPIDinit(&PID_left);incPIDinit(&PID_right);
						  IsCloseloop=1;
							releasebrake();
							initial_motor_speed=SPEED_OF_ALL;
							/*���ﲻ��ʱ,ƽ��������ֱ�Ӷ���һ����*/	
							break;
						case TURNRIGHT_RING:							
							clockwise();
							IsCloseloop=0;
							brake();
							initial_motor_speed = 20;
							Fetch_forward();
							resetPIDline();incPIDinit(&PID_left);incPIDinit(&PID_right);
						  IsCloseloop=1;
							releasebrake();
							initial_motor_speed=SPEED_OF_ALL;
							/*���ﲻ��ʱ,ƽ��������ֱ�Ӷ���һ����*/	
							break;												
						//����
						case FORWARD_GAIN:
							counterclockwise();
						  initial_motor_speed = 20;
							while(1)
							{
								dst=Measure_DST();
								if(2*dst<32)
								{								
									Fetch_from_pillar();
									break;
								}
							} 
							releasebrake();IsCloseloop=0;
							pwmlevel_left=400;pwmlevel_right=400;Setpwm();
							while(PEin(5)!=0);
							clockwise();
							initial_motor_speed=SPEED_OF_ALL;delay_ms(1000);
							break;
						case TURNLEFT_GAIN:
							counterclockwise();
							initial_motor_speed = 10;
						while(1)
							{
								dst=Measure_DST();
								if(2*dst<32)
								{	brake();							
									Fetch_from_pillar();
									break;
								}
							}
							releasebrake();IsCloseloop=0;
							pwmlevel_left=400;pwmlevel_right=400;Setpwm();
							while(PEin(5)!=0);
							clockwise();initial_motor_speed=SPEED_OF_ALL;delay_ms(1000);
							
							break;
						case TURNRIGHT_GAIN:
							clockwise();
							initial_motor_speed = 20;
							Fetch_from_pillar();
							initial_motor_speed=SPEED_OF_ALL;
							break;
					//������
						case TURNLEFT_PILLAR_3:
							initial_motor_speed = 5;
							counterclockwise();
							while(1)
							{
								dst=Measure_DST();
								if(2*dst<25)
								{								
									brake();								
									Score_2ring_3();
									break;
								}
							}
							releasebrake();IsCloseloop=0;
							pwmlevel_left=400;pwmlevel_right=400;Setpwm();delay_ms(300);
							while(PEin(5)!=0);
							counterclockwise();initial_motor_speed=SPEED_OF_ALL;delay_ms(1000);
							break;
						case TURNRIGHT_PILLAR_3:
							initial_motor_speed = 5;
						
  IsCloseloop=0;
	TIM_SetCompare1(TIM1,800);//�����޸�ͨ��1��ռ�ձȣ�ͨ��2Ҳ����
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
	delay_ms(200);//���޸ģ�����ֻ��Ϊ�˴�һ��
	brake();
	delay_ms(200);//��ֹͻȻ����
	setnum_left=initial_motor_speed;
	setnum_right=initial_motor_speed;
	resetPIDline();incPIDinit(&PID_left);incPIDinit(&PID_right);
	IsCloseloop=1;
	releasebrake();

		
						  IsCloseloop=0;pwmlevel_left=400;pwmlevel_right=320;Setpwm();delay_ms(300);
						  resetPIDline();incPIDinit(&PID_left);incPIDinit(&PID_right); IsCloseloop=1;
							while(1)
							{
								dst=Measure_DST();
								if(2*dst<25)
								{								
									brake();								
									Score_2ring_3();//��ץ����������һ��û�о���Ȼץ��
									break;
								}
							}
							releasebrake();IsCloseloop=0;
							pwmlevel_left=400;pwmlevel_right=400;Setpwm();
							while((PEin(5)!=0));
							counterclockwise();initial_motor_speed=SPEED_OF_ALL;delay_ms(1000);
							break;
						case FORWARD_PILLAR_2:
							initial_motor_speed = 5;
							while(1)
							{
								dst=Measure_DST();
								if(2*dst<25)
								{								
									brake();								
									Score_2ring_2();//��ץ����������һ��û�о���Ȼץ��
									break;
								}
							}
							releasebrake();IsCloseloop=0;
							pwmlevel_left=400;pwmlevel_right=400;Setpwm();
							while(PEin(5)!=0);
							counterclockwise();initial_motor_speed=SPEED_OF_ALL;delay_ms(1000);
							break;
						case TURNLEFT_PILLAR_2:
							initial_motor_speed = 10;
							counterclockwise();	
							while(1)
							{
								dst=Measure_DST();
								if(2*dst<25)
								{								
									brake();								
									Score_2ring_2();//��ץ����������һ��û�о���Ȼץ��
									break;
								}
							}
							releasebrake();IsCloseloop=0;
							pwmlevel_left=450;pwmlevel_right=450;Setpwm();
							while(PEin(7)!=0);
							IsCloseloop=1;
							initial_motor_speed=SPEED_OF_ALL;
							break;
						case TURNRIGHT_PILLAR_2:
							initial_motor_speed = 10;
							clockwise();	
							while(1)
							{
								dst=Measure_DST();
								if(2*dst<25)
								{								
									brake();								
									Score_2ring_2();//��ץ����������һ��û�о���Ȼץ��
									break;
								}
							}
							releasebrake();IsCloseloop=0;
							pwmlevel_left=450;pwmlevel_right=450;Setpwm();
							while(PEin(7)!=0);
							IsCloseloop=1;
							initial_motor_speed=SPEED_OF_ALL;
							break;
							case REVERSE:
								reverse();
								delay_ms(500);
								break;
					}
					break;//���break�����þ�������·���ͣ���������岻�Ǻܴ�
				}
				delay_ms(10);//����ϵͳ����		
			}
	
	 }
	 brake();
	 while(1){}
	 
}
