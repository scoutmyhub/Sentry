#ifndef USER_LIB_H
#define USER_LIB_H
#include "main.h"

typedef __packed struct
{
    fp32 input;        //输入数据
    fp32 out;          //输出数据
    fp32 min_value;    //限幅最小值
    fp32 max_value;    //限幅最大值
    fp32 frame_period; //时间间隔
} ramp_function_source_t;


//快速开方
extern fp32 invSqrt(fp32 num);

//斜波函数初始化
void ramp_init(ramp_function_source_t *ramp_source_type, fp32 frame_period, fp32 max, fp32 min);

//斜波函数计算
void ramp_calc(ramp_function_source_t *ramp_source_type, fp32 input);

void ramp_calc1(ramp_function_source_t *ramp_source_type, fp32 input);


//弧度格式化为-PI~PI
#define rad_format(Ang) loop_fp32_constrain((Ang), -PI, PI)

#endif
