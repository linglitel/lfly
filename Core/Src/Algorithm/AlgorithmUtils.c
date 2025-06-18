//
// Created by Administrator on 25-6-17.
//

#include "AlgorithmUtils.h"

float invSqrt(float x)
{
    unsigned int i = 0x5F1F1412 - (*(unsigned int*)&x >> 1);
    const float tmp = *(float*)&i;
    return tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
}
