#include <Arduino.h>
#include "PID.h"
#include "Encoder.h"
#include "grey.h"
#include "Motor.h"
//左轮和右轮的PID实例
 PID_t Left_Inner;
 PID_t Right_Inner;
 //灰度传感器的八位值
extern uint8_t x1,x2,x3,x4,x5,x6,x7,x8;
extern int err;//灰度传感器的偏差值
float Speed_Kp=0.8;
float Speed_Ki=0.13;
float Speed_Kd=0;
int BaseSpeed=10;
int Speed_Maxsize=100;
int Speed_Minsize=0;

void PID_Update(PID_t *p)
{
	p->Error1 = p->Error0;
	p->Error0 = p->Target - p->Actual;

	p->ErrorInt += p->Error0;

	if (p->ErrorInt > 200)
	{
		p->ErrorInt = 200;
	}
	if (p->ErrorInt < -200)
	{
		p->ErrorInt = -200;
	}

	p->Out = p->Kp * p->Error0 +
			 p->Ki * p->ErrorInt +
			 p->Kd * (p->Error0 - p->Error1);

	if (p->Out > p->Maxsize)
		p->Out = p->Maxsize;
	if (p->Out < p->Minsize)
		p->Out = p->Minsize;
}

void Speed_PID_Init(PID_t *p)
{
	p->Kp = Speed_Kp;
	p->Kd = Speed_Kd;
	p->Ki = Speed_Ki;
	p->Target = BaseSpeed;
	p->Maxsize = Speed_Maxsize;
	p->Minsize = Speed_Minsize;
	p->Error0 = 0;
	p->Error1 = 0;
	p->ErrorInt = 0;
	p->Actual = 0;
}



void Record_grey_PID(void)
{   LineWalking();
	Serial.printf("err:%d\n", err);
	Left_Inner.Actual = Speed_Left_Encoder() ;
	Right_Inner.Actual = Speed_Right_Encoder();
	Left_Inner.Target = BaseSpeed + err;  // 左轮目标速度
	Right_Inner.Target = BaseSpeed - err; // 右轮目标速度
	PID_Update(&Left_Inner);
    PID_Update(&Right_Inner);
	Motor_Control(Left_Inner.Out, Right_Inner.Out);
	Serial.printf("Left_Inner.Target:%f,Right_Inner.Target:%f\n", Left_Inner.Target, Right_Inner.Target);
}