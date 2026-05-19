#include <stdio.h>
#include "32-40_defs.h"

#define SIM_TIME 20.0f

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


/*
    Сценарий работы системы.

    Все входы обновляются
    на каждом цикле DT = 0.02 сек.
*/

static void scenario_step(
    float time,
    Input_t* in
)
{
    /*
        Значения по умолчанию
    */

    in->dc_bus_power = 1;
    in->sensor_power = 1;

    in->hyd_pressure = 3000.0f;
    in->hyd_low_pressure = 0;

    in->wow_nlg = 1;

    in->gear_reset = 0;

    in->fail_cu_ch1 = 0;
    in->fail_cu_ch2 = 0;

    in->fail_servo_jam = 0;
    in->fail_hydraulic_leak = 0;
    in->fail_angle_sensor = 0;

    in->tiller_cmd = 0.0f;
    in->rudder_pedal_cmd = 0.0f;

    in->gear_lever_up = 0;

    /*
        Этап 1
        Стоянка
    */

    if (time < 2.0f)
    {
        in->aircraft_speed = 0.0f;
    }

    /*
        Этап 2
        Руление
    */

    else if (time < 6.0f)
    {
        in->aircraft_speed = 15.0f;

        /*
            Поворот tiller
            влево
        */

        in->tiller_cmd = -0.6f;
    }

    /*
        Этап 3
        Руление вправо
    */

    else if (time < 10.0f)
    {
        in->aircraft_speed = 20.0f;

        in->tiller_cmd = 0.8f;
    }

    /*
        Этап 4
        Разбег

        Переход в режим педалей
    */

    else if (time < 14.0f)
    {
        in->aircraft_speed = 90.0f;

        in->rudder_pedal_cmd = 0.4f;
    }

    /*
        Этап 5
        Уборка шасси

        Команда центрирования
    */

    else if (time < 16.0f)
    {
        in->aircraft_speed = 160.0f;

        in->gear_lever_up = 1;
    }

    /*
        Этап 6
        Отказ датчика угла
    */

    else if (time < 18.0f)
    {
        in->aircraft_speed = 180.0f;

        in->gear_lever_up = 1;

        in->fail_angle_sensor = 1;
    }

    /*
        Этап 7
        Отказ канала 1
    */

    else
    {
        in->aircraft_speed = 180.0f;

        in->gear_lever_up = 1;

        in->fail_cu_ch1 = 1;
    }
}

int main(void)
{
    float sim_time;

    Input_t in;
    Output_t out;

    sim_time = 0.0f;

    printf(
        "TIME   MODE  ANGLE   RATE   RETRACT  "
        "CH  SSM  DATA\n"
    );

    while (sim_time <= SIM_TIME)
    {
        /*
            Обновление сценария
        */

        scenario_step(
            sim_time,
            &in
        );

        /*
            Один цикл модели
        */

        nws_manager_step(
            &in,
            &out
        );

        /*
            Вывод состояния
            на каждом шаге
        */

        printf(
            "%6.2f  %d   %7.2f %7.2f    %d      "
            "%d    %d   0x%08X\n",

            sim_time,

            out.steering_mode,

            out.wheel_angle_deg,
            out.wheel_rate_deg_s,

            out.gear_retract_enable,

            out.active_channel,

            out.angle_word.ssm,
            out.angle_word.data
        );

        /*
            Следующий цикл
        */

        sim_time += DT;
    }

    return 0;
}