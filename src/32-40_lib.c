#include "32-40_defs.h"
float supk_abs(float v)
{
    if (v < 0.0f)
    {
        return -v;
    }
    return v;
}

float supk_limit(
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

float supk_integrator(
    float input,
    float* state,
    float dt
    )
    {
    *state += input * dt;
    return *state;
}
