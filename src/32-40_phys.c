#include "32-40_defs.h"
void nws_phys_step(
    Input_t* in,
    Bus_t* bus,
    Output_t* out
)
{
    static float wheel_angle = 0.0f;
    float error;
    float rate;
    if (bus->mode == STATE_FREE_CASTORING)
    {
        rate = 0.0f;
    }
    else
    {
        error = bus->target_angle - wheel_angle;
        rate = error * 2.0f;
        if (in->fail_servo_jam)
        {
            rate = 0.0f;
        }
    }
    
    wheel_angle += rate * DT;
    if (bus->mode == STATE_TAKEOFF_MODE)
    {
        wheel_angle = nws_limit(
        wheel_angle,
        -MAX_PEDAL_ANGLE,
        MAX_PEDAL_ANGLE
    );
    }
    else 
    {
        wheel_angle = nws_limit(
        wheel_angle,
        -MAX_TILLER_ANGLE,
        MAX_TILLER_ANGLE
    );
    }

    out->wheel_angle_deg = wheel_angle;
    out->wheel_rate_deg_s = rate;
    out->servo_current = nws_abs(rate) * 0.1f;
    if (in->fail_hydraulic_leak)
    {
        out->hydraulic_consumption = 0.0f;
    }
    else
    {
        out->hydraulic_consumption =
        nws_abs(rate) * 0.05f;
    }
    if (nws_abs(wheel_angle) <= 2.0f)
    {
        out->gear_retract_enable = 1;
    }
    else
    {
        out->gear_retract_enable = 0;
    }
        out->steering_mode = bus->mode;
        out->active_channel = bus->active_channel;
    if (in->fail_angle_sensor || !in->sensor_power)
    {
        out->angle_word.data = INVALID_DATA;
        out->angle_word.ssm =
        SSM_NO_COMPUTED_DATA;
    }
    else
    {
        out->angle_word.data =
            (unsigned int)(wheel_angle * 100.0f);
        out->angle_word.ssm = SSM_NORMAL;
    }
    out->cas_fault = 0;
    if (bus->active_channel == 0)
    {
        out->cas_fault = 1;
    }
}

unsigned int Arinc429_BuildWord(Arinc429Word_t word)
{
    unsigned int result = 0;

    // LABEL -> биты 1-8
    result |= ((unsigned int)(word.label) & 0xFF);

    // SDI -> биты 9-10
    result |= (((unsigned int)(word.sdi) & 0x03) << 8);

    // DATA -> биты 11-29
    result |= (((unsigned int)(word.data) & 0x7FFFF) << 10);

    // SSM -> биты 30-31
    result |= (((unsigned int)(word.ssm) & 0x03) << 29);
    
    int temp = result;
    int ones = 0;

    for (int i = 0; i < 31; i++)
    {
        if (temp & (1u << i))
        {
            ones++;
        }
    }

    // Если количество единиц чётное —
    // ставим parity bit = 1
    if ((ones % 2) == 0)
    {
        result |= (1u << 31);
    }

    return result;
}