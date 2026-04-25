#include "pid.h"

float update(Controller *c, float target, float actual)
{
    float proportionalTerm, integralTerm, derivativeTerm;
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

    derivativeTerm = c->derivativeGain * (c->derivativeState - actual);
    c->derivativeState = actual;

    return proportionalTerm + integralTerm + derivativeTerm;
}
