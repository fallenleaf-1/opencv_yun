#include "control.h"
float angle_x = INIT_ANGLE_X;   
float angle_y = INIT_ANGLE_Y;

//Y轴参数
float Y_Kp = 0.013, Y_Ki = 0.0001, Y_Kd = 0.035;
//X轴参数
float X_Kp = 0.013, X_Ki = 0.0001, X_Kd = 0.035;    

/* 控制状态 */
float X_integral = 0, Y_integral = 0;
uint16_t lost_counter = 0;
float Servo_PID_Y(int current, int target)
{
    static float err_last = 0; 
    float err = current - target;
    
    /* 目标丢失时特殊处理 */
    if(lost_counter > MAX_LOST_FRAMES) {
        Y_integral *= 0.8f;   // 快速衰减积分项
        err_last = 0;
        return 0;             // 修改点：丢失时由中断进行回归，这里不再输出增量
    }
    
    /* 正常PID计算 */
    if (abs(err) < DEADZONE) {
        Y_integral *= 0.9f;   // 渐消积分
        err_last = 0;
        return 0;
    }
    
    Y_integral += err * 0.3f;
    Y_integral = Y_integral > 1000 ? 1000 : (Y_integral < -1000 ? -1000 : Y_integral);
    
    float derivative = err - err_last;
    float output = Y_Kp*err + Y_Ki*Y_integral + Y_Kd*derivative;
    
    err_last = err;
    return output;
}

float Servo_PID_X(int current, int target)
{
    static float err_last = 0; 
    float err = current - target;
    
    if(lost_counter > MAX_LOST_FRAMES) {
        X_integral *= 0.8f;
        err_last = 0;
        return 0;             // 修改点：同上
    }
    
    if (abs_f(err) < (DEADZONE+3)) {
        X_integral *= 0.9f;
        err_last = 0;
        return 0;
    }
    
    X_integral += err * 0.3f;
    X_integral = X_integral > 1000 ? 1000 : (X_integral < -1000 ? -1000 : X_integral);
    
    float derivative = err - err_last;
    float output = X_Kp*err + X_Ki*X_integral + X_Kd*derivative;
    
    err_last = err;
    return output;
}