#include "32-40_defs.h"

 static void determine_active_channel(
    Input_t* in,
    Bus_t* bus,
    CU_Channel_t* ch1,
    CU_Channel_t* ch2
)
{
    ch1->healthy = !in->fail_cu_ch1;
    ch2->healthy = !in->fail_cu_ch2;
    if (ch1->healthy)
    {
        bus->active_channel = 1;
    }
    else if (ch2->healthy)
    {
        bus->active_channel = 2;
    }
    else
    {
        bus->active_channel = 0;
    }
}

void nws_cu_step(
    Input_t* in,
    Bus_t* bus,
    CU_Channel_t* ch1,
    CU_Channel_t* ch2
    )
    {
    determine_active_channel(in, bus, ch1, ch2);
    if (!in->dc_bus_power)
    {
        bus->mode = STATE_FREE_CASTORING;
        bus->target_angle = 0.0f;
        bus->valve_open = 0;
        return;
    }
    if (in->hyd_pressure < HYD_MIN_PRESSURE)
    {
        bus->mode = STATE_FREE_CASTORING;
        bus->target_angle = 0.0f;
        bus->valve_open = 0;
        return;
    }
    if (in->aircraft_speed < 50.0f)
    {
        bus->mode = STATE_TAXI_MODE;
        bus->target_angle =
        in->tiller_cmd * MAX_TILLER_ANGLE;
    }
    else
    {
        bus->mode = STATE_TAKEOFF_MODE;
        bus->target_angle = in->rudder_pedal_cmd * MAX_PEDAL_ANGLE;
    }
    if (in->gear_lever_up)
    {
        bus->centering_cmd = 1;
        bus->target_angle = 0.0f;
    }
    else
    {
        bus->centering_cmd = 0;
    }
    bus->valve_open = 1;
}

