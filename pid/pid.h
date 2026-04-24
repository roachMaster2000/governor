#ifndef PID_H
#define PID_H

typedef struct
{
    float proportionalGain, integralGain, derivativeGain;
    float integralMin, integralMax;
    float integralState;
    float derivativeState;
} Controller;

float update(Controller *c, float target, float actual);

#endif