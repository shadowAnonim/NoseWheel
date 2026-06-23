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

    if ((ones % 2) == 0)
    {
        result |= (1u << 31);
    }

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
    static float dual_input_timer = 0.0f;
    static int dual_input_active = 0;
    static int use_pilot1 = 1;
    static int locked_pilot = 0;
    static float lock_timer = 0.0f;
    static int dual_input_warning_active = 0;
    
    Arinc429Word_t arinc_mode;
    arinc_mode.label = LABEL_MODE;
    arinc_mode.sdi = 0;
    arinc_mode.ssm = 0;
    arinc_mode.data = 0;
    
    Arinc429Word_t arinc_angle;
    arinc_angle.label = LABEL_TARGET_ANGLE;
    arinc_angle.sdi = 0;
    arinc_angle.ssm = 0;
    arinc_angle.data = 0;
    
    Arinc429Word_t arinc_valve;
    arinc_valve.label = LABEL_VALVE;
    arinc_valve.sdi = 0;
    arinc_valve.ssm = 0;
    arinc_valve.data = 0;
    
    Arinc429Word_t arinc_centering;
    arinc_centering.label = LABEL_CENTERING;
    arinc_centering.sdi = 0;
    arinc_centering.ssm = 0;
    arinc_centering.data = 0;
    
    determine_active_channel(in, bus, ch1, ch2);
    
    // Проверка внешних условий (питание и гидравлика)
    // Если давление гидравлики ниже минимума - система переходит в FREE_CASTORING
    if (!in->dc_bus_power || in->hyd_pressure < HYD_MIN_PRESSURE)
    {
        arinc_mode.data = STATE_FREE_CASTORING;
        arinc_angle.data = 0;
        arinc_valve.data = 0;
        arinc_centering.data = 0;
        write_to_bus(bus, arinc_mode, arinc_angle, arinc_valve, arinc_centering);
        return;
    }
    
    // Проверка Dual Input
    float tiller_diff = nws_abs(in->tiller_cmd_1 - in->tiller_cmd_2);
    float pedal_diff = nws_abs(in->rudder_pedal_cmd_1 - in->rudder_pedal_cmd_2);
    int dual_input_detected = 0;
    
    if (tiller_diff > DUAL_INPUT_THRESHOLD || pedal_diff > DUAL_INPUT_THRESHOLD)
    {
        dual_input_detected = 1;
    }
    
    // Логика обработки Dual Input и Reset
    if (in->gear_reset)
    {
        dual_input_timer = 0.0f;
        dual_input_active = 0;
        use_pilot1 = 1;
        locked_pilot = 0;
        lock_timer = 0.0f;
        dual_input_warning_active = 0;
    }
    
    // Обработка сигнала приоритета пилота
    if (in->pilot_priority && !dual_input_active)
    {
        dual_input_active = 1;
        dual_input_warning_active = 1;
        use_pilot1 = 1;
        locked_pilot = 2;
        lock_timer = 0.0f;
        dual_input_timer = 0.0f;
    }
    
    if (dual_input_detected && !dual_input_active)
    {
        dual_input_active = 1;
        dual_input_warning_active = 1;
        dual_input_timer = 0.0f;
        use_pilot1 = 1;
        locked_pilot = 0;
        lock_timer = 0.0f;
    }
    
    float combined_tiller;
    float combined_pedal;
    int dual_input_error = 0;
    
    if (dual_input_active)
    {
        dual_input_timer += DT;
        
        if (locked_pilot == 2)
        {
            use_pilot1 = 1;
            lock_timer += DT;
            
            if (lock_timer >= DUAL_INPUT_LOCK_TIME)
            {
                locked_pilot = 0;
                lock_timer = 0.0f;
            }
        }
        else if (locked_pilot == 1)
        {
            use_pilot1 = 0;
            lock_timer += DT;
            
            if (lock_timer >= DUAL_INPUT_LOCK_TIME)
            {
                locked_pilot = 0;
                lock_timer = 0.0f;
            }
        }
        else
        {
            if (dual_input_timer >= DUAL_INPUT_LOCK_TIME)
            {
                use_pilot1 = !use_pilot1;
                dual_input_timer = 0.0f;
            }
        }
        
        if (use_pilot1)
        {
            combined_tiller = in->tiller_cmd_1;
            combined_pedal = in->rudder_pedal_cmd_1;
        }
        else
        {
            combined_tiller = in->tiller_cmd_2;
            combined_pedal = in->rudder_pedal_cmd_2;
        }
        
        if (!dual_input_detected && dual_input_timer >= 1.0f && locked_pilot == 0)
        {
            dual_input_active = 0;
            dual_input_warning_active = 0;
            dual_input_timer = 0.0f;
            dual_input_error = 0;
        }
        else
        {
            dual_input_error = 1;
        }
    }
    else
    {
        combined_tiller = (in->tiller_cmd_1 + in->tiller_cmd_2) / 2.0f;
        combined_pedal = (in->rudder_pedal_cmd_1 + in->rudder_pedal_cmd_2) / 2.0f;
        dual_input_warning_active = 0;
        
        if (dual_input_detected)
        {
            dual_input_active = 1;
            dual_input_warning_active = 1;
            dual_input_timer = 0.0f;
            use_pilot1 = 1;
            locked_pilot = 0;
            lock_timer = 0.0f;
            dual_input_error = 1;
        }
    }
    
    int raw_angle;
    unsigned char ssm;
    
    // Вычисляем угол в зависимости от режима
    if (in->aircraft_speed < SPEED_THRESHOLD_TAKEOFF)
    {
        arinc_mode.data = STATE_TAXI_MODE;
        raw_angle = (int)(combined_tiller * MAX_TILLER_ANGLE * 100);
    }
    else
    {
        arinc_mode.data = STATE_TAKEOFF_MODE;
        raw_angle = (int)(combined_pedal * MAX_PEDAL_ANGLE * 100);
    }
    
    // Определяем SSM (приоритет у ошибок Dual Input)
    if (dual_input_warning_active)
    {
        ssm = SSM_DUAL_INPUT_ERROR;
        arinc_angle.ssm = ssm;
    }
    else if (dual_input_error)
    {
        ssm = SSM_DUAL_INPUT_ERROR;
        arinc_angle.ssm = ssm;
    }
    else
    {
        ssm = SSM_NORMAL;
        arinc_angle.ssm = ssm;
    }
    
    if (raw_angle < 0)
    {
        arinc_angle.sdi = 1;
        arinc_angle.data = -raw_angle;
    }
    else
    {
        arinc_angle.sdi = 0;
        arinc_angle.data = raw_angle;
    }
    
    if (in->gear_lever_up)
    {
        arinc_centering.data = 1;
        arinc_angle.data = 0;
        arinc_angle.sdi = 0;
    }
    else
    {
        arinc_centering.data = 0;
    }
    
    arinc_valve.data = 1;
    
    write_to_bus(bus, arinc_mode, arinc_angle, arinc_valve, arinc_centering);
    
    // Сохраняем SSM для передачи в физику
    bus->angle_ssm = ssm;
}