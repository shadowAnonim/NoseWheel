#include "32-40_defs.h"

Arinc429Word_t Arinc429_ParseWord(unsigned int rawWord)
{
    Arinc429Word_t result;

    // LABEL -> биты 1-8
    result.label = (unsigned char)(rawWord & 0xFF);

    // SDI -> биты 9-10
    result.sdi = (unsigned char)((rawWord >> 8) & 0x03);

    // DATA -> биты 11-29
    result.data = (int)((rawWord >> 10) & 0x7FFFF);

    // SSM -> биты 30-31
    result.ssm = (unsigned char)((rawWord >> 29) & 0x03);

    return result;
}

void nws_phys_step(
    Input_t* in,
    Bus_t* bus,
    Output_t* out
)
{
    int mode = Arinc429_ParseWord(bus->mode).data;
    int target_angle = Arinc429_ParseWord(bus->target_angle).data / 100;
    int valve_open = Arinc429_ParseWord(bus->mode).data;
    int centering_cmd = Arinc429_ParseWord(bus->centering_cmd).data;
    int active_channel = Arinc429_ParseWord(bus->active_channel).data;

    static float wheel_angle = 0.0f;
    float error;
    float rate;
    if (mode == STATE_FREE_CASTORING)
    {
        rate = 0.0f;
    }
    else
    {
        error = target_angle - wheel_angle;
        rate = error * 2.0f;
        if (in->fail_servo_jam)
        {
            rate = 0.0f;
        }
    }
    
    wheel_angle += rate * DT;
    if (mode == STATE_TAKEOFF_MODE)
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
    out->steering_mode = mode;
    out->active_channel = active_channel;
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
    if (active_channel == 0)
    {
        out->cas_fault = 1;
    }
}

