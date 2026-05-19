#include "32-40_defs.h"

Arinc429Word_t Arinc429_ParseWord(unsigned int rawWord)
{
    Arinc429Word_t result;

    result.label = (unsigned char)(rawWord & 0xFF);
    result.sdi = (unsigned char)((rawWord >> 8) & 0x03);
    result.data = (int)((rawWord >> 10) & 0x7FFFF);
    result.ssm = (unsigned char)((rawWord >> 29) & 0x03);

    return result;
}

void nws_phys_step(
    Input_t* in,
    Bus_t* bus,
    Output_t* out
)
{
    Arinc429Word_t mode_word = Arinc429_ParseWord(bus->mode);
    Arinc429Word_t angle_word = Arinc429_ParseWord(bus->target_angle);
    Arinc429Word_t valve_word = Arinc429_ParseWord(bus->valve_open);
    Arinc429Word_t centering_word = Arinc429_ParseWord(bus->centering_cmd);
    
    int mode = mode_word.data;
    int valve_open = valve_word.data;
    
    // Восстанавливаем знак угла из SDI
    int target_angle_raw = angle_word.data;
    int target_angle_sign = (angle_word.sdi == 1) ? -1 : 1;
    float target_angle = (float)(target_angle_raw * target_angle_sign) / 100.0f;
    
    int centering_cmd = centering_word.data;
    int active_channel = bus->active_channel;

    static float wheel_angle = 0.0f;
    static float servo_pos = 0.0f;
    static float servo_vel = 0.0f;
    static float integral = 0.0f;
    
    float error;
    float cmd_rate;
    float rate = 0.0f;
    
    // Режим свободного касторинга ИЛИ клапан закрыт
    if (mode == STATE_FREE_CASTORING || valve_open == 0)
    {
        rate = 0.0f;
        servo_pos = 0.0f;
        servo_vel = 0.0f;
        integral = 0.0f;
    }
    else
    {
        // Команда центрирования
        if (centering_cmd == 1)
        {
            target_angle = 0.0f;
        }
        
        // Ошибка управления
        error = target_angle - wheel_angle;
        
        // Выбор коэффициента усиления по режиму
        float kp = (mode == STATE_TAXI_MODE) ? KP_TAXI : KP_TAKEOFF;
        
        // PI-регулятор
        float P = kp * error;
        nws_integrator(error, &integral, DT);
        integral = nws_limit(integral, -MAX_INTEGRAL, MAX_INTEGRAL);
        float I = KI * integral;
        
        // Команда на золотник
        float spool_cmd = P + I;
        spool_cmd = nws_limit(spool_cmd, -MAX_SERVO_CMD, MAX_SERVO_CMD);
        
        // Отказ сервопривода (заклинивание клапана)
        if (in->fail_servo_jam)
        {
            spool_cmd = 0.0f;
            servo_pos = 0.0f;
            servo_vel = 0.0f;
        }
        
        // Динамика золотника
        float spool_accel = SERVO_NATURAL_FREQ * SERVO_NATURAL_FREQ * (spool_cmd - servo_pos) 
                          - 2.0f * SERVO_DAMPING * SERVO_NATURAL_FREQ * servo_vel;
        servo_vel += spool_accel * DT;
        servo_pos += servo_vel * DT;
        servo_pos = nws_limit(servo_pos, -1.0f, 1.0f);
        
        // Фактор давления гидравлики
        float pressure_factor = in->hyd_pressure / HYD_NOMINAL_PRESSURE;
        pressure_factor = nws_limit(pressure_factor, 0.0f, 1.0f);
        
        // Аэродинамический демпфер
        float aero_factor = 1.0f;
        if (mode == STATE_TAKEOFF_MODE && in->aircraft_speed > SPEED_THRESHOLD_TAKEOFF)
        {
            aero_factor = 1.0f - (in->aircraft_speed - SPEED_THRESHOLD_TAKEOFF) / AERO_DEMPER_DENOMINATOR;
            aero_factor = nws_limit(aero_factor, AERO_DEMPER_MAX_FACTOR, 1.0f);
        }
        
        // Утечка гидравлики
        float leak_factor = 1.0f;
        if (in->fail_hydraulic_leak)
        {
            leak_factor = HYD_LEAK_FACTOR;
        }
        
        // Скорость поворота колеса
        cmd_rate = servo_pos * MAX_WHEEL_RATE * pressure_factor * aero_factor * leak_factor;
        cmd_rate = nws_limit(cmd_rate, -MAX_WHEEL_RATE, MAX_WHEEL_RATE);
        
        // Фильтр инерции колеса
        static float filtered_rate = 0.0f;
        float delta = (cmd_rate - filtered_rate) * WHEEL_INERTIA_FILTER;
        nws_integrator(delta, &filtered_rate, DT);
        rate = filtered_rate;
    }
    
    // Интегрирование угла
    nws_integrator(rate, &wheel_angle, DT);
    
    // Ограничение угла по режиму
    if (mode == STATE_TAKEOFF_MODE)
    {
        wheel_angle = nws_limit(wheel_angle, -MAX_PEDAL_ANGLE, MAX_PEDAL_ANGLE);
    }
    else 
    {
        wheel_angle = nws_limit(wheel_angle, -MAX_TILLER_ANGLE, MAX_TILLER_ANGLE);
    }

    // Выходные данные
    out->wheel_angle_deg = wheel_angle;
    out->wheel_rate_deg_s = rate;
    out->gear_retract_enable = (nws_abs(wheel_angle) <= GEAR_RETRACT_TOLERANCE) ? 1 : 0;
    out->steering_mode = mode;
    out->active_channel = active_channel;
    
    // ARINC слово угла (датчик с учётом питания)
    if (in->fail_angle_sensor || !in->sensor_power)
    {
        out->angle_word.data = INVALID_DATA;
        out->angle_word.ssm = SSM_NO_COMPUTED_DATA;
    }
    else
    {
        Arinc429Word_t arinc_wheel_angle;
        arinc_wheel_angle.label = LABEL_WHEEL_ANGLE;
        arinc_wheel_angle.sdi = 0;
        arinc_wheel_angle.ssm = 0;
        arinc_wheel_angle.data = (nws_abs(wheel_angle) * 100.0f);
        out->angle_word.data = Arinc429_BuildWord(arinc_wheel_angle);
        out->angle_word.sdi = (wheel_angle < 0.0f) ? 1 : 0;
        out->angle_word.ssm = SSM_NORMAL;
    }
}