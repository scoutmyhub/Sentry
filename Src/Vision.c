#include "referee_lib.h"
#include "kalman.h"
#include "referee_lib.h"
#include "Vision.h"
#include "bsp_usart.h"
#include "stdlib.h"
#include "INS_task.h"
#include "usart.h"
#include "cmsis_os.h"
#include "math.h"
#include "CRC8_CRC16.h"
#include "CAN_receive.h"

Vision_send_t Vision_send;
TX TXData;
TX TX_yaw_ecd, TX_pitch_ecd;

RX RxSet;
int x = 0;

VisionRecvData_t InitZreo = {0};
VisionRecvData_t VisionRecvData;
extKalman_t kalman_Yaw, kalman_Pitch;
float kalman_targetYaw, kalman_targetPitch;
float targetYaw, targetPitch;
void vision_send(void);
extern int Off_Vision;

uint8_t Follow_Vision = 0;
uint8_t sv_buff[60];

//void vision_send(void)
//{
//	uint8_t length = 26;
//	uint8_t *x_delta, *y_delta;

//	Vision_send.frame_head = 0xD4;
//	Vision_send.a = 1;
//	Vision_send.b = 0;
//	Vision_send.c = 0;
//	Vision_send.state = 0;
//	Vision_send.mark = shoot_data_t.bullet_speed;
//	Vision_send.anti_top = 0;
//	Vision_send.color = 1;
//	Vision_send.delta_x = 32;
//	Vision_send.delta_y = 0;
//	Vision_send.frame_tail = 0xD5;

//	x_delta = (uint8_t *)&Vision_send.delta_x;
//	y_delta = (uint8_t *)&Vision_send.delta_y;
//	TX_yaw_ecd.f_data = (float)(motor_chassis[4].ecd);
//	TX_pitch_ecd.f_data = (float)(MOTOR_PITCH.ecd);

//	sv_buff[0] = Vision_send.frame_head;
//	sv_buff[1] = Vision_send.a;
//	sv_buff[2] = Vision_send.b;
//	sv_buff[3] = Vision_send.c;
//	sv_buff[4] = *TX_yaw_ecd.byte;
//	sv_buff[5] = *(TX_yaw_ecd.byte + 1);
//	sv_buff[6] = *(TX_yaw_ecd.byte + 2);
//	sv_buff[7] = *(TX_yaw_ecd.byte + 3);
//	sv_buff[8] = *TX_pitch_ecd.byte;
//	sv_buff[9] = *(TX_pitch_ecd.byte + 1);
//	sv_buff[10] = *(TX_pitch_ecd.byte + 2);
//	sv_buff[11] = *(TX_pitch_ecd.byte + 3);
//	sv_buff[12] = Vision_send.state;
//	sv_buff[13] = Vision_send.mark;
//	sv_buff[14] = Vision_send.anti_top;
//	sv_buff[15] = Vision_send.color;
//	sv_buff[16] = *x_delta;
//	sv_buff[17] = *(x_delta + 1);
//	sv_buff[18] = *(x_delta + 2);
//	sv_buff[19] = *(x_delta + 3);
//	sv_buff[20] = *y_delta;
//	sv_buff[21] = *(y_delta + 1);
//	sv_buff[22] = *(y_delta + 2);
//	sv_buff[23] = *(y_delta + 3);
//	sv_buff[24] = Vision_send.shoot;
//	sv_buff[25] = Vision_send.frame_tail;

//	HAL_UART_Transmit(&VISION_HUART, (uint8_t *)sv_buff, length, 100);
//}

void vision_send(void)
{
	Vision_send.BulletSpeed = 16.5; // ID1_speed_limit();
	Vision_send.Mode = 0x01;
	Vision_send.Colour = MyColour();
	sv_buff[0] = 0xD4;
	memcpy(sv_buff + 1, &Vision_send, 2 * sizeof(int) + 5 * sizeof(float));
	sv_buff[2 * sizeof(int) + 5 * sizeof(float) + 1] = 0xD5;
	HAL_UART_Transmit(&VISION_HUART, (uint8_t *)sv_buff, 30, 0xff);
}

uint16_t test = 0;
	uint8_t time = 0;	float LastPitchPrediction = 0., LastYawPrediction = 0.;

void LanggoUartFrameIRQHandler(UART_HandleTypeDef *huart)
{

if (huart == &huart1)
	{
		if (Vision_Data[0] == 0xA5)
		{
			VisionRecvData.frame_head = Vision_Data[0];
			VisionRecvData.CmdID = Vision_Data[1];
			//			if(get_CRC8_check_sum(Vision_Data, 3, 0xFF) == Vision_Data[2])
			//			{
			VisionRecvData.statusHead = 1;
			//			}
			//			else
			//			{
			//				VisionRecvData.statusHead = 0;
			//			}
		}
		if (VisionRecvData.statusHead)
		{
			RxSet.byte[0] = Vision_Data[3];
			RxSet.byte[1] = Vision_Data[4];
			RxSet.byte[2] = Vision_Data[5];
			RxSet.byte[3] = Vision_Data[6];
			VisionRecvData.pitch_angle = RxSet.f_data;

			RxSet.byte[0] = Vision_Data[7];
			RxSet.byte[1] = Vision_Data[8];
			RxSet.byte[2] = Vision_Data[9];
			RxSet.byte[3] = Vision_Data[10];
			VisionRecvData.yaw_angle = RxSet.f_data;

			RxSet.byte[0] = Vision_Data[11];
			RxSet.byte[1] = Vision_Data[12];
			RxSet.byte[2] = Vision_Data[13];
			RxSet.byte[3] = Vision_Data[14];
			VisionRecvData.distance = RxSet.f_data;

			VisionRecvData.IsSwitch = Vision_Data[15];
			VisionRecvData.IsFindTarget = Vision_Data[16];

			VisionRecvData.isspinning = Vision_Data[17];
			VisionRecvData.ismiddle = Vision_Data[18];

			VisionRecvData.frame_tail = Vision_Data[19];

			test = (uint16_t)Vision_Data[20] << 8 || Vision_Data[21];
			if (VisionRecvData.frame_tail == 0x00 && test == get_CRC16_check_sum(Vision_Data, 22, 0xFFFF))
			{
				VisionRecvData.statusTail = 1;
			}
			else
			{
				VisionRecvData.statusTail = 0;
			}
		}
	}

//				if (Vision_Data[0] == 0x73)
//				{
//					VisionRecvData.frame_head = Vision_Data[0];

//					VisionRecvData.yaw_angle = (short)(Vision_Data[1] << 8 | Vision_Data[2]);

//					VisionRecvData.pitch_angle = (short)(Vision_Data[3] << 8 | Vision_Data[4]);

//					VisionRecvData.distance = (short)(Vision_Data[5] << 8 | Vision_Data[6]);

//					VisionRecvData.time = (short)(Vision_Data[7] << 8 | Vision_Data[8]);

//					VisionRecvData.shot_flag = Vision_Data[9];

//					VisionRecvData.frame_tail = Vision_Data[10];

//					memset(Vision_Data, 0, 100);
//				}
//				VisionRecvData.pitch_angle = (int)(((VisionRecvData.pitch_angle * 100) / 32767) * 100000);
//				VisionRecvData.yaw_angle = (int)(((VisionRecvData.yaw_angle * 100) / 32767) * 100000);

//				VisionRecvData.pitch_angle = (float)(VisionRecvData.pitch_angle / 1000) / 100.0f;
//				VisionRecvData.yaw_angle = (float)(VisionRecvData.yaw_angle / 1000) / 100.0f;

				if(VisionRecvData.IsFindTarget && gimbal_mode)
					Follow_Vision = 1;
				else if(Off_Vision >= 50)
				{
					Follow_Vision = 0;		 
					Off_Vision = 0;
				}					


				MY_UART_callback(huart);
}

void USART1_IRQHandler(void)
{
	LanggoUartFrameIRQHandler(&huart1);
}

void Vision_Kalman_Init(void)
{
	//	KalmanCreate(&kalman_Yaw, YAW_Q, YAW_R);
	//	KalmanCreate(&kalman_Pitch, PITCH_Q, PITCH_R);
}

VisionRecvData_t *AimDataUpdate(void)
{
	return &VisionRecvData;
}

void GetIMUData(const float Acc[3], const float Gyro[3], const float Qart[4])
{
	// for (uint8_t i = 0; i < 3; i++)
	// {
	// 	Vision_send.INSAcc[i] = Acc[i];
	// 	Vision_send.INSGyro[i] = Gyro[i];
	// }
	 for (uint8_t count = 0; count < 4; count++)
	 {
	 	Vision_send.INSQuat[count] = Qart[count];
	 }
}

// void GetIMUData(const float Acc[3], const float Gyro[3])
// {
// 	for(uint8_t i=0; i<3; i++)
// 	{
// 		Vision_send.INSAcc[i] = Acc[i];
// 		Vision_send.INSGyro[i] = Gyro[i];
// 	}

// }
