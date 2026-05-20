#include <stdio.h>
#include "32-40_defs.h"

#define SIM_TIME 160.0f
#define MAX_SCENARIO_POINTS 100

static ScenarioPoint_t scenario[MAX_SCENARIO_POINTS];
static int scenario_size = 0;

static void scenario_step(float time, Input_t* in)
{
    static int initialized = 0;
    if (!initialized)
    {
        in->dc_bus_power = 1;
        in->sensor_power = 1;
        in->gear_reset = 0;
        in->fail_cu_ch1 = 0;
        in->fail_cu_ch2 = 0;
        in->fail_servo_jam = 0;
        in->fail_hydraulic_leak = 0;
        in->fail_angle_sensor = 0;
        initialized = 1;
    }
    
    // Поиск текущего интервала сценария
    int idx = 1;
    while (idx < scenario_size && time >= scenario[idx].time)
    {
        idx++;
    }
    
    int prev_idx = idx - 1;
    int next_idx = idx;
    
    if (prev_idx < 0) prev_idx = 0;
    if (next_idx >= scenario_size) next_idx = scenario_size - 1;
    
    float t1 = scenario[prev_idx].time;
    float t2 = scenario[next_idx].time;
    float ratio = (t2 > t1) ? (time - t1) / (t2 - t1) : 0.0f;
    if (ratio < 0.0f) ratio = 0.0f;
    if (ratio > 1.0f) ratio = 1.0f;
    
    in->aircraft_speed = scenario[prev_idx].speed * (1.0f - ratio) + scenario[next_idx].speed * ratio;
    in->tiller_cmd = scenario[prev_idx].tiller * (1.0f - ratio) + scenario[next_idx].tiller * ratio;
    in->rudder_pedal_cmd = scenario[prev_idx].pedal * (1.0f - ratio) + scenario[next_idx].pedal * ratio;
    in->hyd_pressure = scenario[prev_idx].hyd * (1.0f - ratio) + scenario[next_idx].hyd * ratio;
    
    if (ratio > 0.5f)
    {
        in->gear_lever_up = scenario[next_idx].gear_up;
    }
    else
    {
        in->gear_lever_up = scenario[prev_idx].gear_up;
    }
    
    // Отказы по времени
    if (time >= 100.0f && time < 110.0f)
    {
        in->fail_angle_sensor = 1;
    }
    else
    {
        in->fail_angle_sensor = 0;
    }
    
    if (time >= 110.0f)
    {
        in->fail_cu_ch1 = 1;
    }
    else
    {
        in->fail_cu_ch1 = 0;
    }
}

int main(void)
{
    float sim_time = 0.0f;
    Input_t in;
    Output_t out;
    
    // Загрузка сценария через inout
    scenario_size = read_scenario("scenario.txt", scenario, MAX_SCENARIO_POINTS);
    if (scenario_size == 0)
    {
        printf("Ошибка: не удалось загрузить сценарий\n");
        return 1;
    }
    
    printf("Загружено %d точек сценария\n", scenario_size);
    printf("TIME   MODE  ANGLE   RATE   RETRACT  CH  SSM  DATA\n");
    
    while (sim_time <= SIM_TIME)
    {
        scenario_step(sim_time, &in);
        nws_manager_step(&in, &out);
        
        write_log(LOG_FILENAME, sim_time, &out);
        
        // Вывод в консоль (каждые 0.5 секунды)
        if ((int)(sim_time * 50) % 25 == 0)
        {
            printf("%6.2f  %d   %7.2f %7.2f    %d      %d    %d   0x%08X\n",
                sim_time, out.steering_mode, out.wheel_angle_deg, out.wheel_rate_deg_s,
                out.gear_retract_enable, out.active_channel, out.angle_word.ssm, out.angle_word.data);
        }
        
        sim_time += DT;
    }
    
    printf("\nЛог сохранён в файл: %s\n", LOG_FILENAME);
    return 0;
}