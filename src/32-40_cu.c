#include "32-40_defs.h"

unsigned int Arinc429_BuildWord(Arinc429IntWord_t word)
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

Arinc429IntWord_t Arinc429_ParseWord(unsigned int rawWord)
{
    Arinc429IntWord_t result;

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

 static void determine_active_channel(
    Input_t* in,
    Bus_t* bus,
    CU_Channel_t* ch1,
    CU_Channel_t* ch2
)
{
    ch1->healthy = !in->fail_cu_ch1;
    ch2->healthy = !in->fail_cu_ch2;

    Arinc429IntWord_t arinc_channel;
    arinc_channel.label = LABEL_ACTIVE_CHANNEL;
    arinc_channel.sdi = 0;
    arinc_channel.ssm = 0;
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
    bus -> active_channel = Arinc429_BuildWord(arinc_channel);
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



