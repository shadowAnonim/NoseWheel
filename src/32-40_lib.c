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

float nws_interp(
    float x,
    float x1,
    float y1,
    float x2,
    float y2
)
{
    if (x2 <= x1)
    {
        return y1;
    }
    
    float t = (x - x1) / (x2 - x1);
    
    if (t < 0.0f)
    {
        t = 0.0f;
    }
    if (t > 1.0f)
    {
        t = 1.0f;
    }
    
    return y1 + (y2 - y1) * t;
}

void nws_delay(
    float input,
    float* buffer,
    int size,
    float* output
)
{
    static int idx = 0;
    buffer[idx] = input;
    idx = (idx + 1) % size;
    *output = buffer[idx];
}

float nws_latch(
    float input,
    float trigger,
    float* state
)
{
    if (trigger > 0.5f)
    {
        *state = input;
    }
    return *state;
}