#include "control.h"
float angle_x = INIT_ANGLE_X;   
float angle_y = INIT_ANGLE_Y;
uint16_t opencv_x = 0;
uint16_t opencv_y = 0;
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
    if (abs_f(err) < DEADZONE) {
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

void control_servo(void)
{
	_Bool target_valid = (opencv_x != 0xFFFF) && 
                            (opencv_y != 0xFFFF) &&
                            (opencv_x <= 640) && 
                            (opencv_y <= 480);
        
        if(target_valid) {
            lost_counter = 0;
            
            /* 正常追踪模式 */
            float x_output = Servo_PID_X(opencv_x - IMG_CX, 0); 
            float y_output = Servo_PID_Y(opencv_y - IMG_CY, 0);
            
            /* Y 轴越过 90 度时反转 X 和 Y 的移动方向（上一问的镜像逻辑） */
            if (angle_y > 90.0f) 
            {
                x_output = -x_output;
                y_output = -y_output;
            }
            
            angle_x += x_output;
            angle_y += y_output;
        } 
        else {
            if(++lost_counter > MAX_LOST_FRAMES) {
                /* 回归初始位置模式 */
                float x_err = INIT_ANGLE_X - angle_x;
                float y_err = INIT_ANGLE_Y - angle_y;
                
                angle_x += x_err * RETURN_KP;
                angle_y += y_err * RETURN_KP;
                
                X_integral = 0; Y_integral = 0;
            }
        }

        /* 舵机角度限幅保护（已适配 180 度舵机：0~180） */
        angle_x = angle_x > 180.0f ? 180.0f : (angle_x < 0.0f ? 0.0f : angle_x);
        angle_y = angle_y > 180.0f ? 180.0f : (angle_y < 0.0f ? 0.0f : angle_y);
        
        /* 更新舵机角度 */
        Servo_SetAngleX(angle_x);
        Servo_SetAngleY(angle_y);
    }
