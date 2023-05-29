#include "bsp_pid.h"

SystemPIDCalc_t SystemPIDCalc;

//pid_t trigger_motor_pid;
//pid_type_def Fire_left_motor_pid;
//pid_type_def Fire_right_motor_pid; 
//pid_type_def FireLeftMoment;
//pid_type_def FireRightMoment;


//float32_t Fire_left_speed_pid[3] = {13, 0.1, 7};
//float32_t Fire_right_speed_pid[3] = {13, 0.1, 7};
//float32_t FireLeftMomentPID[3] = {10, 0, 7};
//float32_t FireRightMomentPID[3] = {10, 0, 7};

//Low_Pass_Filter_t ShootMotor_Left, ShootMotor_Right;
//int16_t Fire_Left;
//int16_t Fire_Right;
//fp32 Fire_Left_Low, Fire_Right_Low;
float FilterTest[2] = {2, 1.2};
int Vision_Timing = 0;
fp32 Err_angle = 0;

extern int Shoot_AllNum;
int Off_Vision = 0;
int shoot_findflag = 0;

void SystemPIDInit(void)
{
//    /*Shoot*/
//    {
//        SystemPIDCalc.ShootPIDControl = GetShootData();
//        /*Shoot*/
//        PID_struct_init(&pid_poke, POSITION_PID, BULLET_ANG_PID_MAX_OUT, BULLET_ANG_PID_MAX_IOUT, TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD);         // 1.0.10
//        PID_struct_init(&pid_poke_omg, POSITION_PID, BULLET_SPEED_PID_MAX_OUT, BUFFET_SPEED_PID_MAX_IOUT, TRIGGER_SPEED_PID_KP, TRIGGER_SPEED_PID_KI, TRIGGER_SPEED_PID_KD); // 3.0.10

//        PID_init(&Fire_left_motor_pid, PID_POSITION, Fire_left_speed_pid, FIRE_left_SPEED_PID_MAX_OUT, FIRE_left_SPEED_PID_MAX_IOUT);
//        PID_init(&Fire_right_motor_pid, PID_POSITION, Fire_right_speed_pid, FIRE_right_SPEED_PID_MAX_OUT, FIRE_right_SPEED_PID_MAX_IOUT);
//        PID_init(&FireLeftMoment, PID_POSITION, FireLeftMomentPID, FIRE_MOMENT_PID_MAX_OUT, FIRE_MOMENT_PID_MAX_IOUT);
//        PID_init(&FireRightMoment, PID_POSITION, FireRightMomentPID, FIRE_MOMENT_PID_MAX_OUT, FIRE_MOMENT_PID_MAX_IOUT);
//        /*未调参*/
//        Low_Pass_Filter_Init(&ShootMotor_Left, FilterTest[0], FilterTest[1]);
//        Low_Pass_Filter_Init(&ShootMotor_Right, FilterTest[0], FilterTest[1]);
//    }
    /*Chassis*/
    {
        SystemPIDCalc.ChassisPIDControl = GetChassisData();
        // chassis motor speed PID
        const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
        // chassis angle PID
        const static fp32 chassis_yaw_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};
        for (uint8_t i = 0; i < 4; i++)
        {
            SystemPIDCalc.ChassisPIDControl->motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
            PID_init(&SystemPIDCalc.ChassisPIDControl->motor_speed_pid[i], PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
        }
        // initialize angle PID
        PID_init(&SystemPIDCalc.ChassisPIDControl->chassis_angle_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
    }
    /*Gimbal*/
    {
        SystemPIDCalc.GimbalPIDControl = get_gimlal_data();

    }
}

int yyf = 0 ,delay = 2; 
uint8_t ChassisFrequency = 0;
int shoot_time = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

    if (htim->Instance == TIM6)
    {
//        SystemPIDCalc.ShootPIDControl = GetShootData();
//        if (SystemPIDCalc.ShootPIDControl->Fire_Data.poke_pos_ref < SystemPIDCalc.ShootPIDControl->shoot_motor_measure->total_angle - 5000)
//        {
//            SystemPIDCalc.ShootPIDControl->Turn_Back_Data.block_time++;
//        }
//        else
//        {
//            SystemPIDCalc.ShootPIDControl->Turn_Back_Data.block_time = 0;
//            SystemPIDCalc.ShootPIDControl->HeatLimitData.flag = 1;//ShootControl.HeatLimitData.flag = 1;
//        }
//        if (((SystemPIDCalc.ShootPIDControl->Control_FALG.SHOOT_SWITCH_KEYBOARD||switch_is_down(SystemPIDCalc.ShootPIDControl->RC_ctrl->rc.s[Shoot_RC_Channel_left])
//					||Follow_Vision) && (shoot_mode == SHOOT_READY)))
//        {
//            SystemPIDCalc.ShootPIDControl->Fire_Data.ShootFTime++;
//        }
    }
    if(htim->Instance == TIM2)
    {
//        /*Shoot*/
//        {
//            SystemPIDCalc.ShootPIDControl = GetShootData();
//            Fire_Left = -SystemPIDCalc.ShootPIDControl->LeftMotorData->speed_rpm;
//            Fire_Right = SystemPIDCalc.ShootPIDControl->RightMotorData->speed_rpm;
//            Low_Pass_Filter_OUT(&ShootMotor_Left, SystemPIDCalc.ShootPIDControl->LeftMotorData->speed_rpm);
//            Low_Pass_Filter_OUT(&ShootMotor_Right, -SystemPIDCalc.ShootPIDControl->RightMotorData->speed_rpm); 
//            Fire_Left_Low = ShootMotor_Left.out;
//            Fire_Right_Low = -ShootMotor_Right.out;
//#if Normal_Mode
//            PID_calc(&Fire_left_motor_pid, SystemPIDCalc.ShootPIDControl->LeftMotorData->speed_rpm, SystemPIDCalc.ShootPIDControl->Fire_Data.Fire_left_speed_set);
//            PID_calc(&Fire_right_motor_pid, SystemPIDCalc.ShootPIDControl->RightMotorData->speed_rpm, SystemPIDCalc.ShootPIDControl->Fire_Data.Fire_right_speed_set);
//#else 
//            PID_calc(&Fire_left_motor_pid, ShootMotor_Left.out, SystemPIDCalc.ShootPIDControl->Fire_Data.Fire_left_speed_set);
//            PID_calc(&Fire_right_motor_pid, ShootMotor_Right.out, SystemPIDCalc.ShootPIDControl->Fire_Data.Fire_right_speed_set);
//#endif

//            pid_calc_old(&pid_poke, SystemPIDCalc.ShootPIDControl->shoot_motor_measure->total_angle, SystemPIDCalc.ShootPIDControl->Fire_Data.poke_pos_ref);
//            pid_calc_old(&pid_poke_omg, SystemPIDCalc.ShootPIDControl->shoot_motor_measure->speed_rpm, pid_poke.pos_out);

//            /*OUT*/
//            SystemPIDCalc.ShootPIDControl->Fire_Left_OUT = (int16_t)Fire_left_motor_pid.out;
//            SystemPIDCalc.ShootPIDControl->Fire_Right_OUT = (int16_t)Fire_right_motor_pid.out;
//            SystemPIDCalc.ShootPIDControl->shoot_CAN_Set_Current = pid_poke_omg.pos_out; 

//        }
        /*Chassis*/
        {
            ChassisFrequency++;
            if(ChassisFrequency == 2)
            {
                yyf = 1;
                SystemPIDCalc.ChassisPIDControl = GetChassisData();
                ChassisPowerLimit();
//                PowerLimit();
                ChassisFrequency = 0;
            }
            if (0)
            {
                CAN_cmd_chassis(0, 0, 0, 0);
            }
            else
            {
                CAN_cmd_chassis(SystemPIDCalc.ChassisPIDControl->motor_chassis[0].give_current, SystemPIDCalc.ChassisPIDControl->motor_chassis[1].give_current,
                                SystemPIDCalc.ChassisPIDControl->motor_chassis[2].give_current, SystemPIDCalc.ChassisPIDControl->motor_chassis[3].give_current);
            }
        }
    }
		
			if(htim ->Instance == TIM3)//定时器3,定时时间10ms
	{
//		if(Hurt_Flag == 1)
//		{
//			Rotation_Timing++;
//			Reset_Timing++;
//			if(Rotation_Timing == 3000)
//			{
//				TopMode_Stop = 1;
//				Hurt_Flag = 0;
//				Rotation_Timing = 0;
//			}
//		}
//		else if(Hurt_Flag > 1)
//		{
//			Hurt_Flag = 1;
//			Rotation_Timing = 0;
//		}
		delay--;
		
		if(gimbal_mode)
		{
			Vision_Timing++;
			if(reset_yaw)
			Err_angle = Vision_Timing*0.0141;
		}
		else
			Vision_Timing = 0;
		if(Shoot_AllNum >= 9)
			shoot_time ++;
		
		if(VisionRecvData.IsFindTarget == 0)
		{
			Off_Vision ++;
		}
		else
			Off_Vision = 0;
		
		if(VisionRecvData.IsFindTarget == 1)
		{
			shoot_findflag ++;
		}
		else
			shoot_findflag = 0;
	}
}


//// 计算全向轮底盘云台PID输出
//void calculate_chassis_pid_output(chassis_pid_t *pid, float error) {
//    pid->integral += error;
//    pid->derivative = error - pid->last_error;
//    pid->last_error = error;
//    pid->error = error;
//    pid->out = pid->Kp * pid->error + 
//                pid->Ki * pid->integral +
//                pid->Kd * pid->derivative;
//}


//void control_chassis_stable_rotation(chassis_measure_t *chassis_measure, chassis_pid_controller_t *pid_controller, float rotation_rate, float swing_rate) {
//    float error[4] = {0};
//    float given_current[4] = {0};
//    float sum = 0;
//    for (int i = 0; i < 4; i++) {
//        error[i] = rotation_rate - chassis_measure[i].speed_rpm;
//        calculate_chassis_pid_output(&pid_controller->pid[i], error[i]);
//        given_current[i] = pid_controller->pid[i].out;
//        sum += given_current[i];
//    }
//    for (int i = 0; i < 4; i++) {
//        given_current[i] += (sum > 0 ? -swing_rate : swing_rate);
//    }
//    CAN_cmd_chassis(given_current[0], given_current[1], given_current[2], given_current[3]);
//}





