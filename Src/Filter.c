#include "Filter.h"

/// @brief 一阶低通滤波的初始化
/// @param Low_Pass_Filter 结构体
/// @param frame_period 采样时间
/// @param Trust 加权比例
void Low_Pass_Filter_Init(Low_Pass_Filter_t *Low_Pass_Filter, fp32 frame_period, const fp32 Trust)
{
    Low_Pass_Filter->frame_period = frame_period;
    Low_Pass_Filter->Trust = Trust;
    Low_Pass_Filter->input = 0.0f;
    Low_Pass_Filter->out = 0.0f;
}

/// @brief 一节低通滤波输出，通过加权得到新输出
/// @param Low_Pass_Filter 结构体
/// @param input 输出
void Low_Pass_Filter_OUT(Low_Pass_Filter_t *Low_Pass_Filter, fp32 input)
{
    Low_Pass_Filter->input = input;
    Low_Pass_Filter->out = Low_Pass_Filter-> Trust/ (Low_Pass_Filter->Trust + Low_Pass_Filter->frame_period) * Low_Pass_Filter->out 
                         + Low_Pass_Filter->frame_period / (Low_Pass_Filter->Trust + Low_Pass_Filter->frame_period) * Low_Pass_Filter->input;
}






#define WHEEL_RADIUS 0.08f
//#define CHASSIS_WZ_SET_SCALE 0.5 
//#define MOTOR_DISTANCE_TO_CENTER 0.2
#define BIAS_X 0.001         
#define BIAS_Y 0.001     

double Kp = 1; 
double Ki = 0.02; 
double Kd = 0.2; 

double target_x = 0; 
double feedback_x = 0; 
double error_x = 0; 
double last_error_x = 0; 
double sum_error_x = 0; 

double target_y = 0; 
double feedback_y = 0; 
double error_y = 0; 
double last_error_y = 0; 
double sum_error_y = 0; 


double calc_FB_offset()
{
	double control_x, fb_offset;
	control_x = (Kp * error_x + Ki * sum_error_x + Kd * (error_x - last_error_x)); 
	fb_offset = (control_x * MOTOR_DISTANCE_TO_CENTER) / WHEEL_RADIUS;
	return fb_offset;
}


double calc_LR_offset()
{
	double control_y, lr_offset;
	control_y = (Kp * error_y + Ki * sum_error_y + Kd * (error_y - last_error_y)); 
	lr_offset = (control_y * MOTOR_DISTANCE_TO_CENTER) / WHEEL_RADIUS;
	return lr_offset;
}


void fix_GravityCenter_rotation(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set)
{
	double fb_offset = 0.0; 
	double lr_offset = 0.0; 


	if (*wz_set != 0.0f)
	{
		fb_offset = -CHASSIS_WZ_SET_SCALE * MOTOR_DISTANCE_TO_CENTER * (*wz_set) / WHEEL_RADIUS; 
		lr_offset = CHASSIS_WZ_SET_SCALE * MOTOR_DISTANCE_TO_CENTER * (*wz_set) / WHEEL_RADIUS;	 
	}


	double tmp_vx = -(*vx_set) + (*vy_set) + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * (*wz_set);
	tmp_vx += fb_offset;
	(*vx_set) = -tmp_vx + (*vy_set) + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * (*wz_set);


	double tmp_vy = -(*vx_set) - (*vy_set) + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * (*wz_set);
	tmp_vy += lr_offset;
	(*vy_set) = -(*vx_set) - tmp_vy + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * (*wz_set);
}




