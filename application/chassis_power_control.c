#include "chassis_power_control.h"
#include "referee.h"
#include "referee_lib.h"
#include "arm_math.h"
#include "pid.h"

float ChassisSpeedCTRL_MAX = 0.0f;
	float ChassisSpeedCTRL[4] = {0.0f};
	float MeanSpeed = 0.0f;
	float ChassisGain[4] = {0.0f};
	pid_type_def PDGain[4];

float ChassisPower = 0.0f;
static float ChassisPowerBuff = 0.0f;
static float Chassis_PowerLimit = 0.0f;
float Plimit = 0.0f, Klimit = 0.0f;
float WheelSet[4];
float Scaling1 = 0.0f,Scaling2 = 0.0f,Scaling3 = 0.0f,Scaling4 = 0.0f;

float Chassis_pidout_max = 0.0f;
float Chassis_pidout = 0.0f;
ChassisPowerControl_t ChassisPowerLMT;

double effort_coeff = 3.9;
double vel_coeff = 0.00855;
double power_offset = -9.8;

fp32 ROTATE_INERTIA_RATE[4] = {1.0f, 1.0f, 1.0f, 1.0f};
float PIDGainPar[3] = {1500.0, 0.0, 5.0};

float Accable(void);

void ChassisGainPID_Init(void)
{ 
    PID_init(&PDGain[0], PID_POSITION, PIDGainPar, 16000, 0.);
    PID_init(&PDGain[1], PID_POSITION, PIDGainPar, 16000, 0.);
    PID_init(&PDGain[2], PID_POSITION, PIDGainPar, 16000, 0.);
    PID_init(&PDGain[3], PID_POSITION, PIDGainPar, 16000, 0.);
}

void ChassisPowerLimit(void)
{
    uint8_t i;

    get_chassis_power_and_buffer(&ChassisPower, &ChassisPowerBuff, &Chassis_PowerLimit);
		ChassisPowerLMT.RCData = get_remote_control_point();	
    ChassisPowerLMT.ChassisData = GetChassisData();
	
	
    chassis_vector_to_mecanum_wheel_speed(ChassisPowerLMT.ChassisData->vx_set, ChassisPowerLMT.ChassisData->vy_set, ChassisPowerLMT.ChassisData->wz_set, WheelSet);
    for (i = 0; i < 4; i++)
    {
        ChassisPowerLMT.ChassisData->motor_chassis[i].speed_set = WheelSet[i];
    }
    Chassis_pidout_max = 61536;

    for (i = 0; i < 4; i++)
    {
        PID_calc(&ChassisPowerLMT.ChassisData->motor_speed_pid[i], ChassisPowerLMT.ChassisData->motor_chassis[i].speed, ChassisPowerLMT.ChassisData->motor_chassis[i].speed_set - ChassisPowerLMT.ChassisData->classis_ramp.out);
//				ChassisPowerLMT.ChassisData->motor_speed_pid[i].out = ChassisPowerLMT.ChassisData->motor_speed_pid[i].out * ROTATE_INERTIA_RATE[i];
		}

    if (ChassisPower > 960)
        Chassis_VAL_LIMIT(4096); // 5*4*24;
    else
    {
        Chassis_pidout = (fabs(ChassisPowerLMT.ChassisData->motor_speed_pid[0].set - ChassisPowerLMT.ChassisData->motor_speed_pid[0].fdb) +
                            fabs(ChassisPowerLMT.ChassisData->motor_speed_pid[1].set - ChassisPowerLMT.ChassisData->motor_speed_pid[1].fdb) +
                            fabs(ChassisPowerLMT.ChassisData->motor_speed_pid[2].set - ChassisPowerLMT.ChassisData->motor_speed_pid[2].fdb) +
                            fabs(ChassisPowerLMT.ChassisData->motor_speed_pid[3].set - ChassisPowerLMT.ChassisData->motor_speed_pid[3].fdb));

        Scaling1 = (ChassisPowerLMT.ChassisData->motor_speed_pid[0].set - ChassisPowerLMT.ChassisData->motor_speed_pid[0].fdb) / Chassis_pidout;
        Scaling2 = (ChassisPowerLMT.ChassisData->motor_speed_pid[1].set - ChassisPowerLMT.ChassisData->motor_speed_pid[1].fdb) / Chassis_pidout;
        Scaling3 = (ChassisPowerLMT.ChassisData->motor_speed_pid[2].set - ChassisPowerLMT.ChassisData->motor_speed_pid[2].fdb) / Chassis_pidout;
        Scaling4 = (ChassisPowerLMT.ChassisData->motor_speed_pid[3].set - ChassisPowerLMT.ChassisData->motor_speed_pid[3].fdb) / Chassis_pidout;
        Klimit = Chassis_pidout / 8;
        VAL_LIMIT(&Klimit, -1, 1);

        if (ChassisPowerBuff < 50 && ChassisPowerBuff >= 40)
            Plimit = 0.9;
        else if (ChassisPowerBuff < 40 && ChassisPowerBuff >= 35)
            Plimit = 0.8;
        else if (ChassisPowerBuff < 35 && ChassisPowerBuff >= 30)
            Plimit = 0.65;
        else if (ChassisPowerBuff < 30 && ChassisPowerBuff >= 20)
            Plimit = 0.45;//0.3
        else if (ChassisPowerBuff < 20 && ChassisPowerBuff >= 10)
            Plimit = 0.15;//0.15
        else if (ChassisPowerBuff < 10 && ChassisPowerBuff >= 0)
            Plimit = 0.05;//0.05
        else if (ChassisPowerBuff == 60)
            Plimit = 1;
    }

		
		

    if(fabs(ChassisPowerLMT.ChassisData->motor_chassis[0].speed) < 0.05)
    {
        for(i=0; i<4; i++)
        {
            ChassisGain[i] = 0.0;
        }
    }
    else
    {
        ChassisSpeedCTRL_MAX =  fabs(ChassisPowerLMT.ChassisData->motor_chassis[0].speed) + 
                                fabs(ChassisPowerLMT.ChassisData->motor_chassis[1].speed) +
                                fabs(ChassisPowerLMT.ChassisData->motor_chassis[2].speed) +
                                fabs(ChassisPowerLMT.ChassisData->motor_chassis[3].speed);
        MeanSpeed = ChassisSpeedCTRL_MAX / 4;
        for(i=0; i<4; i++)
        {
            if(ChassisPowerLMT.ChassisData->motor_chassis[i].speed < 0)
            {
                ChassisSpeedCTRL[i] = ChassisPowerLMT.ChassisData->motor_chassis[i].speed - (-1 * MeanSpeed);
            }
            else{
                ChassisSpeedCTRL[i] = ChassisPowerLMT.ChassisData->motor_chassis[i].speed - MeanSpeed;
            }
            PID_calc(&PDGain[i], ChassisSpeedCTRL[i], 0.);
            ChassisGain[i] = PDGain[i].out;
        }
    }

    ChassisPowerLMT.ChassisData->motor_speed_pid[0].out = Scaling1*Chassis_pidout_max*Klimit*Plimit + ChassisGain[0]; 
    ChassisPowerLMT.ChassisData->motor_speed_pid[1].out = Scaling2*Chassis_pidout_max*Klimit*Plimit + ChassisGain[1];	
    ChassisPowerLMT.ChassisData->motor_speed_pid[2].out = Scaling3*Chassis_pidout_max*Klimit*Plimit*Accable() + ChassisGain[2];
    ChassisPowerLMT.ChassisData->motor_speed_pid[3].out = Scaling4*Chassis_pidout_max*Klimit*Plimit*Accable() + ChassisGain[3];

    for (i = 0; i < 4; i++)
    {
        ChassisPowerLMT.ChassisData->motor_chassis[i].give_current = (int16_t)(ChassisPowerLMT.ChassisData->motor_speed_pid[i].out);
    }
}

void Chassis_VAL_LIMIT(int X)
{
    VAL_LIMIT(&ChassisPowerLMT.ChassisData->motor_speed_pid[0].out,-X,X);		
    VAL_LIMIT(&ChassisPowerLMT.ChassisData->motor_speed_pid[1].out,-X,X);		
    VAL_LIMIT(&ChassisPowerLMT.ChassisData->motor_speed_pid[2].out,-X,X);
    VAL_LIMIT(&ChassisPowerLMT.ChassisData->motor_speed_pid[3].out,-X,X);
}



float Accable(void)
{
    float res = 0;
	if(ChassisPowerLMT.ChassisData->chassis_RC->key.v & AccAble)
    {
        res = 2.5f;
    }
    else
    {
        res = 1;
    }
    return res;
}


double zoom_coeff = 1;
double w[20] = {0};
void PowerLimit(void)
{
    ChassisPowerLMT.ChassisData = GetChassisData();
    get_chassis_power_and_buffer(&ChassisPower, &ChassisPowerBuff, &Chassis_PowerLimit);
    double a = 0., b = 0., c = 0.;
    chassis_vector_to_mecanum_wheel_speed(ChassisPowerLMT.ChassisData->vx_set, ChassisPowerLMT.ChassisData->vy_set, ChassisPowerLMT.ChassisData->wz_set, WheelSet);
    for (uint8_t i = 0; i < 4; i++)
    {
        ChassisPowerLMT.ChassisData->motor_chassis[i].speed_set = WheelSet[i];
    }
    for (uint8_t i = 0; i < 4; i++)
    {
        PID_calc(&ChassisPowerLMT.ChassisData->motor_speed_pid[i], ChassisPowerLMT.ChassisData->motor_chassis[i].speed, ChassisPowerLMT.ChassisData->motor_chassis[i].speed_set - ChassisPowerLMT.ChassisData->classis_ramp.out);
    }
    for (uint8_t i = 0; i < 4; i++)
    {
        double cmd_effort = (double)ChassisPowerLMT.ChassisData->motor_speed_pid[i].out;
        double real_val = (double)ChassisPowerLMT.ChassisData->motor_chassis[i].chassis_motor_measure->speed_rpm / 19.0;
        a += pow(cmd_effort, 2);
        b += fabs(cmd_effort * real_val);
        c += pow(real_val, 2);
    }
    a *= effort_coeff;
    w[1] = a;
    w[2] = b;
    c = c * vel_coeff - power_offset - Chassis_PowerLimit;
    w[3] = c;
    double zoom_coeff = (b * b - 4 * a * c) > 0 ? (-b + sqrt(b * b - 4 * a * c)) / (2 * a) : 0.;
    w[0] = zoom_coeff;
    w[4] = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
    for (uint8_t i = 0; i < 4; i++)
    {
        if(zoom_coeff < 1 && zoom_coeff > 0)
            ChassisPowerLMT.ChassisData->motor_speed_pid[i].out = ChassisPowerLMT.ChassisData->motor_speed_pid[i].out * zoom_coeff;
        ChassisPowerLMT.ChassisData->motor_chassis[i].give_current = (int16_t)(ChassisPowerLMT.ChassisData->motor_speed_pid[i].out);
    }
    
}



// void power_limit(float x, float y, float z, float w, float power_limit_value, float *curr_x, float *curr_y, float *curr_z, float *curr_w) {
//     float speed_avg, current_avg, scale;

//     // 计算四个轮子的速度平均值
//     speed_avg = (fabs(x) + fabs(y) + fabs(z) + fabs(w))/4;
    
//     // 计算每个轮子的转矩电流
//     if(speed_avg == 0) {
//         *curr_x = 0;
//         *curr_y = 0;
//         *curr_z = 0;
//         *curr_w = 0;
//     } else {
//         *curr_x = fabs(x) * power_limit_value / speed_avg;
//         *curr_y = fabs(y) * power_limit_value / speed_avg;
//         *curr_z = fabs(z) * power_limit_value / speed_avg;
//         *curr_w = fabs(w) * power_limit_value / speed_avg;
//     }
    
//     // 检查总功率是否超过限制值
//     current_avg = (*curr_x + *curr_y + *curr_z + *curr_w)/4;
    
//     if(current_avg > power_limit_value) {
//         // 如果总功率超过限制值，则缩放每个轮子的转矩电流
//         scale = power_limit_value / current_avg;
//         *curr_x *= scale;
//         *curr_y *= scale;
//         *curr_z *= scale;
//         *curr_w *= scale;
//     }
// }



