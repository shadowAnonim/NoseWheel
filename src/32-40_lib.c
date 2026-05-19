#include "32-40_defs.h"
float nws_abs(float v)
{
    if (v < 0.0f)
    {
        return -v;
    }
    return v;
}

float nws_limit(
    float value,
    float min,
    float max
    )
    {
    if (value < min)
    {
        return min;
    }
    if (value > max)
    {
        return max;
    }
        return value;
    }

float nws_integrator(
    float input,
    float* state,
    float dt
    )
    {
    *state += input * dt;
    return *state;
}
