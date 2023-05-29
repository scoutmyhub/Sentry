#include "referee.h"

void get_chassis_power_and_buffer(fp32 *power, fp32 *buffer, fp32 *powermax)
{
    *power = power_heat_data_t.chassis_power;//底盘输出功率
    *buffer = power_heat_data_t.chassis_power_buffer;//底盘缓冲功率
	*powermax = robot_state.chassis_power_limit;//底盘功率限制
}

void  get_chassis_power_and_buffer_OLD(fp32 *power, fp32 *buffer)
{
    *power = power_heat_data_t.chassis_power;
    *buffer = power_heat_data_t.chassis_power_buffer;
}


uint8_t get_robot_id(void)
{
	return robot_state.robot_id;
}

uint8_t get_robot_level(void)
{
	return robot_state.robot_level;
}
uint16_t get_robot_remain_HP(void)
{
	return robot_state.remain_HP;
}


uint16_t get_robot_max_HP(void)
{
	return robot_state.max_HP;
}


uint16_t ID1_cooling_rate(void)
{
	return robot_state.shooter_id1_17mm_cooling_rate;
}
uint16_t ID1_cooling_limit(void)
{
	return robot_state.shooter_id1_17mm_cooling_limit;
}
uint16_t ID1_speed_limit(void)
{
	return robot_state.shooter_id1_17mm_speed_limit;
}
uint16_t ID1CoolingHeat(void)
{
	return power_heat_data_t.shooter_id1_17mm_cooling_heat;
}


uint16_t ID2_cooling_rate(void)
{
	return robot_state.shooter_id2_17mm_cooling_rate;
}
uint16_t ID2_cooling_limit(void)
{
	return robot_state.shooter_id2_17mm_cooling_limit;
}
uint16_t ID2_speed_limit(void)
{
	return robot_state.shooter_id2_17mm_speed_limit;
}
uint16_t ID2CoolingHeat(void)
{
	return power_heat_data_t.shooter_id2_17mm_cooling_heat;
}



uint8_t robot_buff(void)
{
	return buff_musk_t.power_rune_buff;
}

uint16_t Chassis_Power_Limit(void)
{
	return robot_state.chassis_power_limit;
}


int MyColour(void)
{
	if(get_robot_id() < 10)
		return 0;
	else
		return 1;
}


//int Get_outpost(void)
//{
//	uint32_t outpost;
//	outpost = 0x00000400 & field_event.event_type;
//	if(outpost)
//		return 0x01;
//	else
//		return 0x00;
//}


int Get_outpost(void)
{
    uint32_t outpost;
    outpost = (field_event.event_type >> 10) & 0x01;
    if (outpost)
        return 0x01;
    else
        return 0x00;
}

