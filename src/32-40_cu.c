#include "32-40_defs.h"

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

 static void determine_active_channel(
    Input_t* in,
    Bus_t* bus,
    CU_Channel_t* ch1,
    CU_Channel_t* ch2
)
{
    ch1->healthy = !in->fail_cu_ch1;
    ch2->healthy = !in->fail_cu_ch2;

    Arinc429Word_t arinc_channel;
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

void write_to_bus(Bus_t* bus,
     Arinc429Word_t arinc_mode,
     Arinc429Word_t arinc_angle,
     Arinc429Word_t arinc_valve,
    Arinc429Word_t arinc_centering)
{
    bus->mode = Arinc429_BuildWord(arinc_mode);
    bus->target_angle = Arinc429_BuildWord(arinc_angle);
    bus->valve_open = Arinc429_BuildWord(arinc_valve);
    bus->centering_cmd = Arinc429_BuildWord(arinc_centering);
}

void nws_cu_step(
    Input_t* in,
    Bus_t* bus,
    CU_Channel_t* ch1,
    CU_Channel_t* ch2
    )
    {
    Arinc429Word_t arinc_mode;
    arinc_mode.label = LABEL_MODE;
    arinc_mode.sdi = 0;
    arinc_mode.ssm = 0;
    Arinc429Word_t arinc_angle;
    arinc_angle.label = LABEL_MODE;
    arinc_angle.sdi = 0;
    arinc_angle.ssm = 0;
    Arinc429Word_t arinc_valve;
    arinc_valve.label = LABEL_VALVE;
    arinc_valve.sdi = 0;
    arinc_valve.ssm = 0;
    determine_active_channel(in, bus, ch1, ch2);
    if (!in->dc_bus_power)
    {
        arinc_mode.data = STATE_FREE_CASTORING;
        arinc_angle.data = 0.0f;
        arinc_valve.data= 0;
        write_to_bus(bus, arinc_mode, arinc_angle, arinc_valve);
        return;
    }
    if (in->hyd_pressure < HYD_MIN_PRESSURE)
    {
        arinc_mode.data = STATE_FREE_CASTORING;
        arinc_angle.data = 0.0f;
        arinc_valve.data= 0;
        write_to_bus(bus, arinc_mode, arinc_angle, arinc_valve);
        return;
    }
    if (in->aircraft_speed < 50.0f)
    {
        arinc_mode.data = STATE_TAXI_MODE;
        arinc_valve.data =
            in->tiller_cmd * MAX_TILLER_ANGLE;
    }
    else
    {
        arinc_mode.data = STATE_TAKEOFF_MODE;
        arinc_angle.data =
            in->rudder_pedal_cmd * MAX_PEDAL_ANGLE;
    }
    Arinc429Word_t arinc_centering;
    arinc_centering.label = LABEL_CENTERING;
    arinc_centering.sdi = 0;
    arinc_centering.ssm = 0;
    if (in->gear_lever_up)
    {
        arinc_centering.data = 1;
        arinc_angle.data = 0.0f;
    }
    else
    {
        arinc_centering.data = 0;
    }
    arinc_valve.data = 1;
    write_to_bus(bus, arinc_mode, arinc_angle, arinc_valve, arinc_centering);
}



