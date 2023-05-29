#ifndef VISION_H
#define VISION_H


#include "main.h"
#include "user_lib.h"
#include "arm_math.h"
#include "stm32f4xx_hal.h"
#include "gimbal_behaviour.h"
#include "remote_control.h"
#include "gimbal_task.h"
#include "referee.h"
#include "referee_usart_task.h"
#include "my_pid.h"
#include "struct_typedef.h"
#include "BMI088driver.h"
#include "BMI088reg.h"
#include "BMI088Middleware.h"

#define VISION_HUART huart1
#define USE_NEWCODE 1
typedef union
{
    float f_data;
    uint8_t byte[4];
} FloatUChar_t;



typedef __packed struct
{
#if USE_NEWCODE
	int Mode;
	float BulletSpeed;
	int Colour;
	float INSQuat[4];
//	float INSGyro[3];
//    float INSAcc[3];
#else
    char frame_head ;
	uint8_t a;
	char b;
	char c;

	float yaw_angle;
	float pitch_angle;
	uint8_t state;
	uint8_t mark;
	uint8_t anti_top;
	uint8_t color;
	uint8_t shoot;	
	int delta_x;
	int delta_y;
    uint8_t frame_tail;
#endif
} Vision_send_t;

typedef __packed struct
{
    char frame_head;
    float yaw_angle;
    float pitch_angle;
    float distance;
    float time;
    char shot_flag;
    char frame_tail;
    uint8_t CmdID;
    uint8_t statusHead;
    uint8_t statusTail;
    uint8_t IsSwitch;
    uint8_t IsFindTarget;
    uint8_t isspinning;
    uint8_t ismiddle;
} VisionRecvData_t;

typedef union
{
    float f_data;
    uint8_t byte[4];
} TX;

typedef union
{
    uint8_t byte[4];
    float f_data;
} RX;


extern void LanggoUartFrameIRQHandler(UART_HandleTypeDef *huart);
extern VisionRecvData_t VisionRecvData;
extern void Vision_task(void const *pvParameters);
extern uint8_t Follow_Vision;
extern void Kalman_Filter(void);
extern float kalman_targetYaw, kalman_targetPitch;
extern void Vision_Kalman_Init(void);
extern void vision_send(void);
extern  VisionRecvData_t* AimDataUpdate(void);
extern void GetIMUData(const float Acc[3], const float Gyro[3], const float Qart[4]);
extern int gimbal_mode;

// extern void GetIMUData(const float Acc[3], const float Gyro[3]);

#endif


