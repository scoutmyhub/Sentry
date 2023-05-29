#include <stdio.h>
#include "Fuzzy.h"
#include "INS_task.h"

#define MAX_SAMPLES 1000   // ???????
#define MAX_STD 10000      // ????????????
#define SAMPLE_INTERVAL 10 // ??????,?????
#define THRESHOLD 0.1      // ??????????

int GET(void)
{
    int i;
    float ins_accel_history[MAX_SAMPLES][3]; // ????MAX_SAMPLES???
    float stddev_history[MAX_STD]; // ???????

    int idx = 0; // ????????
    int sample_count; // ???????????
    float stddev; // ????????

    while (1)
    {

        ins_accel_history[idx][0] = INS_accel[0];
        ins_accel_history[idx][1] = INS_accel[1];
        ins_accel_history[idx][2] = INS_accel[2];

        idx++;
        if (idx >= MAX_SAMPLES)
        {
            idx = 0;
        }


        sample_count = MAX_SAMPLES;
        if (sample_count > MAX_STD)
        {
            sample_count = MAX_STD;
        }

        float stdsum = 0.0;
        for (i = 0; i < sample_count; i++)
        {
            float xdiff = ins_accel_history[(idx - i - 1 + MAX_SAMPLES) % MAX_SAMPLES][0] - ins_accel_history[idx][0];
            float ydiff = ins_accel_history[(idx - i - 1 + MAX_SAMPLES) % MAX_SAMPLES][1] - ins_accel_history[idx][1];
            float zdiff = ins_accel_history[(idx - i - 1 + MAX_SAMPLES) % MAX_SAMPLES][2] - ins_accel_history[idx][2];
            stdsum += xdiff * xdiff + ydiff * ydiff + zdiff * zdiff;
        }
        stddev = sqrtf(stdsum / (float)sample_count);


        for (i = MAX_STD - 1; i > 0; i--)
        {
            stddev_history[i] = stddev_history[i - 1];
        }
        stddev_history[0] = stddev;


        int stable = 1;
        for (i = 1; i < 10; i++)
        {
            if (stddev_history[i] > THRESHOLD)
            {
                stable = 0;
                break;
            }
        }

        // TODO: ???????????????
        // ...

        // ?????????
        // ...
    }

    return 0;
}





