
#include "serv0.h"
void Servo_SetAngleX(float Angle)
{
    Angle = Angle < SERVO_MIN ? SERVO_MIN : 
        (Angle > SERVO_MAX ? SERVO_MAX : Angle);	
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,Angle / 180 * 2000 + 500);
	//PWM_SetCompare3(Angle / 270 * 2000 + 500);
}
void Servo_SetAngleY(float Angle)
{
   Angle = Angle < SERVO_MIN ? SERVO_MIN : 
      (Angle > SERVO_MAX ? SERVO_MAX : Angle);		
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,Angle / 180 * 2000 + 500);
	//PWM_SetCompare4(Angle / 270 * 2000 + 500);
}

float abs_f(float num)
{
	if(num>=0)
	{
		return num;
	}
		return -num;
}