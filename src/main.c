#include <stdio.h>
#include "32-40_defs.h"

#define SIM_TIME 360.0f
#define MAX_SCENARIO_POINTS 100

static ScenarioPoint_t scenario[MAX_SCENARIO_POINTS];
static int scenario_size = 0;

void read_fails(int fails, Input_t* in)
{
    in->fail_cu_ch1 = (int)(fails & 0x1);
    in->fail_cu_ch2 = (int)((fails >> 1) & 0x1);
    in->fail_servo_jam = (int)((fails >> 2) & 0x1);
    in->fail_hydraulic_leak = (int)((fails >> 3) & 0x1);
    in->fail_angle_sensor = (int)((fails >> 4) & 0x1);
}

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
        in->pilot_priority = 0;
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
    in->tiller_cmd_1 = scenario[prev_idx].tiller1 * (1.0f - ratio) + scenario[next_idx].tiller1 * ratio;
    in->tiller_cmd_2 = scenario[prev_idx].tiller2 * (1.0f - ratio) + scenario[next_idx].tiller2 * ratio;
    in->rudder_pedal_cmd_1 = scenario[prev_idx].pedal1 * (1.0f - ratio) + scenario[next_idx].pedal1 * ratio;
    in->rudder_pedal_cmd_2 = scenario[prev_idx].pedal2 * (1.0f - ratio) + scenario[next_idx].pedal2 * ratio;
    // Давление гидравлики приходит как внешний сигнал от "гидросистемы"
    in->hyd_pressure = scenario[prev_idx].hyd * (1.0f - ratio) + scenario[next_idx].hyd * ratio;
    
    if (ratio > 0.5f)
    {
        in->gear_lever_up = scenario[next_idx].gear_up;
    }
    else
    {
        in->gear_lever_up = scenario[prev_idx].gear_up;
    }

    read_fails(scenario[prev_idx].fails, in);
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
        printf("Error: cannot load scenery.\n");
        return 1;
    }
    
    printf("Loaded %d points of scenery.\n", scenario_size);
    printf("Starting NWS Simulation...\n");
    printf("============================================================\n");
    printf("Scenario Parameters:\n");
    printf(" - Simulation Time: %.1f seconds\n", SIM_TIME);
    printf(" - Time Step (DT): %.3f seconds\n", DT);
    printf(" - Number of Scenario Points: %d\n", scenario_size);
    printf(" - First Point: Time=%.1f, Speed=%.1f, Hyd=%.0f\n",
           scenario[0].time, scenario[0].speed, scenario[0].hyd);
    printf(" - Last Point: Time=%.1f, Speed=%.1f, Hyd=%.0f\n",
           scenario[scenario_size-1].time, scenario[scenario_size-1].speed,
           scenario[scenario_size-1].hyd);
    printf("============================================================\n");
    printf("Logging to: %s\n\n", LOG_FILENAME);
    
    while (sim_time <= SIM_TIME)
    {
        scenario_step(sim_time, &in);
        nws_manager_step(&in, &out);
        
        write_log(LOG_FILENAME, sim_time, &out, &in);
        
        sim_time += DT;
    }
    
    printf("\nLog saved successful. Filename: %s\n", LOG_FILENAME);
    return 0;
}