#include <stdio.h>
#include "32-40_defs.h"

int read_scenario(
    FILE* file,
    Input_t* in
)
{
    return fscanf(
        file,
        "%*f %f %f %f %f %d %d",
        &in->aircraft_speed,
        &in->tiller_cmd,
        &in->rudder_pedal_cmd,
        &in->hyd_pressure,
        &in->gear_lever_up
    );
}

void write_log(
    FILE* file,
    float time,
    Output_t* out
)
{
    fprintf(
        file,
        "%6.2f  %d   %7.2f %7.2f    %d      %d    %d   0x%08X\n",
        time,
        out->steering_mode,
        out->wheel_angle_deg,
        out->wheel_rate_deg_s,
        out->gear_retract_enable,
        out->active_channel,
        out->angle_word.ssm,
        out->angle_word.data
    );
}