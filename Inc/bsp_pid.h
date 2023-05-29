#ifndef BSP_PID_H
#define BSP_PID_H
#include "main.h"
#include "gimbal_task.h"
#include "chassis_behaviour.h"
#include "shoot.h"
#include "my_pid.h"
#include "Filter.h"
#include "chassis_power_control.h"
#include "CAN_receive.h"
#include "Vision.h"
//#include "Fuzzy.h"


///*shoot*/
//#define FIRE_left_SPEED_PID_MAX_OUT 15000.0f
//#define FIRE_left_SPEED_PID_MAX_IOUT 500.0f

//#define FIRE_right_SPEED_PID_MAX_OUT 15000.0f
//#define FIRE_right_SPEED_PID_MAX_IOUT 500.0f

//#define FIRE_MOMENT_PID_MAX_OUT 15000.0f
//#define FIRE_MOMENT_PID_MAX_IOUT 1000.0f


//#define TRIGGER_ANGLE_PID_KP 0.3f
//#define TRIGGER_ANGLE_PID_KI 0.0f
//#define TRIGGER_ANGLE_PID_KD 0.9f

//#define TRIGGER_SPEED_PID_KP 9.0f
//#define TRIGGER_SPEED_PID_KI 0.0f
//#define TRIGGER_SPEED_PID_KD 1.0f



//#define Fire_left_SPEED_PID_KP 13
//#define Fire_left_SPEED_PID_KI 0.18
//#define Fire_left_SPEED_PID_KD 0

//#define Fire_right_SPEED_PID_KP 13
//#define Fire_right_SPEED_PID_KI 0.18
//#define Fire_right_SPEED_PID_KD 0



//#define BULLET_ANG_PID_MAX_OUT 10000.0f
//#define BULLET_ANG_PID_MAX_IOUT 0.0f
//#define BULLET_SPEED_PID_MAX_OUT 10000.0f
//#define BUFFET_SPEED_PID_MAX_IOUT 1000.0f
///*shoot end*/

/*Chassis*/

//chassis motor speed PID
#define M3505_MOTOR_SPEED_PID_KP 13000.0f
#define M3505_MOTOR_SPEED_PID_KI 0.11f
#define M3505_MOTOR_SPEED_PID_KD 9500.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

//chassis follow angle PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 7.8f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.0001f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 6.5f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 13.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.0f
/*Chassis end*/


typedef struct 
{
    gimbal_control_t *GimbalPIDControl;
    chassis_move_t *ChassisPIDControl;
    shoot_control_t *ShootPIDControl;
}SystemPIDCalc_t;



extern void SystemPIDInit(void);


#endif//BSP_PID_H

