#ifndef __serv0_H
#define __serv0_H
#include "main.h"
#include "tim.h"
#define SERVO_MIN   0.0f
#define SERVO_MAX   180.0f
void Servo_SetAngleX(float Angle);//PA6
void Servo_SetAngleY(float Angle);//PA7
float abs_f(float num);
#endif
