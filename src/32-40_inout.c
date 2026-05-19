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
        &in->gear_lever_up,
        &in->wow_nlg
    );
}
