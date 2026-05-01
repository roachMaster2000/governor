#include "pico/stdlib.h"

#ifndef PI_H
#define PI_H

typedef struct
{
    float feedForward;
    float proportionalGain, integralGain;
    float integralMin, integralMax;
    float integralState;
    uint32_t integralDeadTime;
} PIController;

float update(PIController *c, float target, float actual);

#endif