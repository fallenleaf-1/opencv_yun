#ifndef __control_H
#define __control_H
#include "math.h"
#include <stdbool.h>
#include "serv0.h"
#include <stdlib.h> // 确保 abs() 函数有声明

#define IMG_CX          320      // 图像中心X坐标
#define IMG_CY          240      // 图像中心Y坐标
#define INIT_ANGLE_X    90.0f    // 舵机X轴初始角度（180度舵机的正中位置）
#define INIT_ANGLE_Y    90.0f    // 舵机Y轴初始角度（原45度偏低，180度舵机建议设为90度或按需调整）
#define DEADZONE        5        // 死区阈值（像素）
#define MAX_LOST_FRAMES 150       // 丢失判定阈值（约3秒 @20ms周期）
#define RETURN_KP       0.10f    // 回归控制比例系数

/* 180度舵机物理边界定义 */
#define SERVO_MIN_LIMIT 0.0f     
#define SERVO_MAX_LIMIT 180.0f
#endif