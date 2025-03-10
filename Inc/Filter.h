#ifndef __FILTER_H
#define __FILTER_H

#include "main.h"
#include "struct_typedef.h"
#include "chassis_task.h"


typedef struct
{
    fp32 input;        //System input
    fp32 out;          //System out
    fp32 Trust;        //Trust
    fp32 frame_period; //sampling time
}Low_Pass_Filter_t;


extern void Low_Pass_Filter_Init(Low_Pass_Filter_t *Low_Pass_Filter, fp32 frame_period, const fp32 Trust);
extern void Low_Pass_Filter_OUT(Low_Pass_Filter_t *Low_Pass_Filter, float input);
extern void fix_GravityCenter_rotation(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set);

#endif
