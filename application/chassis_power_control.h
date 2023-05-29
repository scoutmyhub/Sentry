#ifndef CHASSIS_POWER_CONTROL_H
#define CHASSIS_POWER_CONTROL_H
#include "chassis_task.h"
#include "main.h"

#define AccAble KEY_PRESSED_OFFSET_F

typedef struct
{
    float Limit_k;
    float Real_PowerBuffer;
    float Max_PowerBuffer;
    float CHAS_TotalOutput;
    float CHAS_LimitOutput;
} CHASSIS_PowerLimit_t;

typedef struct
{
    const motor_measure_t *ChassisMotorData[4];
    CHASSIS_PowerLimit_t CHASSIS_PowerLimit;
    chassis_move_t *ChassisData;
	  const RC_ctrl_t* RCData;
}ChassisPowerControl_t;




void Chassis_VAL_LIMIT(int X);

extern void ChassisPowerLimit(void);
extern void PowerLimit(void);
extern void ChassisGainPID_Init(void);


#endif
