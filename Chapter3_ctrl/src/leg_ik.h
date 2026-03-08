#ifndef LEG_IK_H
#define LEG_IK_H

#include <math.h> 
#include <stdint.h> 
#include <stdio.h>
#include <string.h>

#define L1 0.035f // 大腿
#define L2 0.04f // 小腿

#ifndef PI
#define PI      3.1415926535f
#endif

void leg_ik(float x, float y, float *angle_out);

#endif