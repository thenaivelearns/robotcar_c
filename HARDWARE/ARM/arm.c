#include "arm.h"
#include "delay.h"
#include "stepmotor.h"
#include "pid.h"
#include "linefollow.h"

/*constrain*/
//���µ��Ϸֱ���channel1234�����ж�����������ʱ˳ʱ��ת/*�Զ������Ϊ�۲���*/
//����ʱ����λֵ
//���996R������45������250
//�ⲿ996R������250������45����40���ͻȻ�ɵ������ǿ��Ի�����
//����SG90,270Ϊ���ޣ����ֱ۴�ֱ��270����ʱ����ЩΣ�գ�����050���ټ�С�������ֱ�
//��ָSG90��170ǡ�ü�ס��200ʱ��£��125�ſ������

//should������˳ʱ����ת
//eblow��������ʱ����ת���ں������ͨ��300-input����������
//wrist������˳ʱ����ת
//finger�������Ǳպ�

int final_gain=0;
int shoulder=187,elbow=45,wrist=100,finger=100;
 int i;
/*�������*/
void Pillar_up(void)/*��������*/
{
		shoulder=187;elbow=45;wrist=110;finger=100;
		Armmove();
	  elevate(140,Pillar_speed);
}

void Pillar_start(void)/*������λ����*/
{
		elevate(-80,Pillar_speed);
}

void Pillar_reset(void)/*�÷ֺ�λ����*/
{
		elevate(-140,Pillar_speed);
}



void Arminit(void)
{
		GPIO_InitTypeDef    GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_BaseInitStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
//    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    TIM_BaseInitStructure.TIM_Period = 2000-1;
    TIM_BaseInitStructure.TIM_Prescaler = 720-1;
    TIM_BaseInitStructure.TIM_ClockDivision = 0;
    TIM_BaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM5, &TIM_BaseInitStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;//��ʱ��ͨ�����ã��󲿷ֶ���Ĭ��״̬
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
		
		TIM_OC1Init(TIM5, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);
		TIM_OC2Init(TIM5, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);
		TIM_OC3Init(TIM5, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);
		TIM_OC4Init(TIM5, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable);
		
    TIM_Cmd(TIM5, ENABLE);
		Armmove();
}

void Armmove(void)
{
	delay_ms(20);
	TIM_SetCompare2(TIM5,300-elbow);
	delay_ms(20);
	TIM_SetCompare1(TIM5,shoulder);
	delay_ms(20);
	TIM_SetCompare3(TIM5,wrist);
	delay_ms(20);
	TIM_SetCompare4(TIM5,finger);
}

void ResetArm(void)
{
	shoulder=187;elbow=45;wrist=100;finger=100;
	Armmove();
}

void Fetch_forward(void)/*֮��϶�Ҫ�Ż������ȷ��ץ��׼*/
{
	shoulder=200;elbow=230;wrist=80;finger=100;Armmove();//���ػ���
	delay_ms(300);
	shoulder=218;wrist=70;Armmove();//���ز���צ��
	delay_ms(500);
  resetPIDline();incPIDinit(&PID_left);incPIDinit(&PID_right);
	IsCloseloop=1;
	releasebrake();

	while((GPIOE->IDR&0XF)!=0)
	{	
	}
	IsCloseloop=0;
	brake();
	finger=150;Armmove();//ץס
	delay_ms(1000);
	shoulder=189;wrist=90;Armmove();//���ֻ���	
	delay_ms(200);//�����ʱ�ܶ�  
	shoulder=189;elbow=45;wrist=100;Armmove();
	delay_ms(1000);
	finger=100;Armmove();//�ͷ�
	delay_ms(1000);
	//����ԭ����ʱ��ץ������ʱ���޷�ת�䣬ֻ�ܱջ�Ѳֱ��
	//�����Զ�˿��ܻᶪ�ߣ���������Ҫ��Ҫ��ת���Ⲣִ�еĴ���ϲ�������Ѳ�ߵ��ж�����ȥ
	ResetArm();
}

void Fetch_from_pillar(void){
//ע�⣬��������������Ժ�С���Ǵ���ɲ��״̬��
	  int i=1;IsCloseloop=0;brake();
	shoulder=180;elbow=228;wrist=115;finger=100;Armmove();//��������
	delay_ms(1000);
	resetPIDline();incPIDinit(&PID_left);incPIDinit(&PID_right);
	releasebrake();IsCloseloop=1;
  delay_ms(700);
	IsCloseloop=0;brake();
	delay_ms(500);//��ֹ��е��û�е�λ
  finger=150;Armmove();//ץ��
	delay_ms(1000);
	for(;i<3;i++)
	{    //�뿪����
	  shoulder-=11;delay_ms(300);Armmove();
		elbow-=8;delay_ms(300);Armmove();
		wrist+=8;delay_ms(300);Armmove();
		delay_ms(500);
	}
	delay_ms(500);
	elbow-=10;Armmove();
	shoulder=187;elbow=45;wrist=100;Armmove();
	delay_ms(1000);
	finger=100;Armmove();//�ͷ�
	delay_ms(500);
	ResetArm();
}
/*��������1����*/
void Score_1ring_3(void){
  shoulder=182;wrist=100;finger=100;Armmove();//����������
	delay_ms(500);
	finger=150;Armmove();
	delay_ms(500);
	shoulder=187;wrist=120;Armmove();
	delay_ms(500);
	shoulder=175;elbow=195;wrist=150;Armmove();
  delay_ms(1000);
	finger=100;Armmove();//�ͷ�
	delay_ms(500);
	ResetArm();
}


/*����������������*/
void Score_2ring_3(void){//175 195 150 100
  shoulder=182;wrist=100;finger=100;Armmove();//����������
	delay_ms(500);
	finger=150;Armmove();//ץ��
	delay_ms(500);
	shoulder=187;wrist=120;Armmove();//���ֻ���
	delay_ms(500);
	shoulder=175;elbow=190;wrist=150;Armmove();
	/*for(i=1;i<=5;i++){
	elbow+=30;wrist+=6;Armmove();
	delay_ms(300);
	}*/
  delay_ms(1000);
	finger=100;Armmove();//�ͷ�
	delay_ms(500);
	ResetArm();
	
  Pillar_up();  
	
	shoulder=182;wrist=100;finger=100;Armmove();//����������
	delay_ms(500);
	finger=150;Armmove();
	delay_ms(500);
	shoulder=187;wrist=120;Armmove();
	delay_ms(500);
	shoulder=175;elbow=190;wrist=150;Armmove();
  delay_ms(1000);
	finger=100;Armmove();//�ͷ�
	delay_ms(500);
	ResetArm();	
	Pillar_reset();
}
/*�ڶ�������һ����*/
void Score_2ring_2(void){
  shoulder=182;wrist=100;finger=100;Armmove();//����������
	delay_ms(500);
	finger=150;Armmove();
	delay_ms(500);
	shoulder=187;wrist=120;Armmove();
	delay_ms(500);
	shoulder=190;elbow=220;wrist=110;Armmove();
  delay_ms(1000);
	finger=100;Armmove();//�ͷ�
	delay_ms(500);
	ResetArm();
	
	Pillar_up();
	if(final_gain==1){ 
		releasebrake();
		IsCloseloop=0;
		pwmlevel_left=700;pwmlevel_right=300;Setpwm();
		delay_ms(800);
		pwmlevel_left=700;pwmlevel_right=700;Setpwm();
		delay_ms(1000);
		brake();
	}
	shoulder=182;wrist=100;finger=100;Armmove();//����������
	delay_ms(500);
	finger=150;Armmove();
	delay_ms(500);
	shoulder=187;wrist=120;Armmove();
	delay_ms(500);
	shoulder=190;elbow=220;wrist=110;Armmove();
  delay_ms(1000);
	finger=100;Armmove();//�ͷ�
	delay_ms(500);
	ResetArm();
	Pillar_reset();
	final_gain++;
}
