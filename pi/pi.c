#include "pi.h"

float update(PIController *c, float target, float actual)
{
    float proportionalTerm, integralTerm;
    float error = target - actual;

    proportionalTerm = c->proportionalGain * error;

    if (c->integralDeadTime > 0)
    {
        c->integralDeadTime--;
    }
    else
    {
        c->integralState += error;
        if (c->integralState > c->integralMax)
        {
            c->integralState = c->integralMax;
        }
        else if (c->integralState < c->integralMin)
        {
            c->integralState = c->integralMin;
        }
    }
    integralTerm = c->integralGain * c->integralState;

    return c->feedForward + proportionalTerm + integralTerm;
}
