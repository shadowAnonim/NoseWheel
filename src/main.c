#include <stdio.h>
#include "32-40_defs.h"

#define SIM_TIME 160.0f
#define MAX_SCENARIO_POINTS 100

static void scenario_step(float time, Input_t* in, ScenarioPoint_t* scenario, int scenario_size)
{
    // Значения по умолчанию (не меняющиеся)
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
    
    // Линейная интерполяция
    in->aircraft_speed = scenario[prev_idx].speed * (1.0f - ratio) + scenario[next_idx].speed * ratio;
    in->tiller_cmd = scenario[prev_idx].tiller * (1.0f - ratio) + scenario[next_idx].tiller * ratio;
    in->rudder_pedal_cmd = scenario[prev_idx].pedal * (1.0f - ratio) + scenario[next_idx].pedal * ratio;
    in->hyd_pressure = scenario[prev_idx].hyd * (1.0f - ratio) + scenario[next_idx].hyd * ratio;
    
    // Дискретный сигнал - переключаем когда больше половины интервала
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
    FILE* scenario_file;
    FILE* log_file;
    ScenarioPoint_t scenario[MAX_SCENARIO_POINTS];
    int scenario_size = 0;
    char line[256];
    
    // Открываем файл сценария
    scenario_file = fopen("scenario.txt", "r");
    if (scenario_file == NULL)
    {
        printf("Ошибка: не удалось открыть файл сценария scenario.txt\n");
        return 1;
    }
    
    // Пропускаем заголовки (строки начинающиеся с #)
    while (fgets(line, sizeof(line), scenario_file))
    {
        if (line[0] != '#')
        {
            break;
        }
    }
    
    // Читаем сценарий
    do
    {
        if (sscanf(line, "%f %f %f %f %f %d",
            &scenario[scenario_size].time,
            &scenario[scenario_size].speed,
            &scenario[scenario_size].tiller,
            &scenario[scenario_size].pedal,
            &scenario[scenario_size].hyd,
            &scenario[scenario_size].gear_up) == 6)
        {
            scenario_size++;
        }
    } while (fgets(line, sizeof(line), scenario_file) && scenario_size < MAX_SCENARIO_POINTS);
    
    fclose(scenario_file);
    
    if (scenario_size == 0)
    {
        printf("Ошибка: файл сценария пуст или имеет неверный формат\n");
        return 1;
    }
    
    // Открываем файл для записи лога
    log_file = fopen(LOG_FILENAME, "w");
    if (log_file == NULL)
    {
        printf("Ошибка: не удалось создать файл лога %s\n", LOG_FILENAME);
        return 1;
    }
    
    // Заголовок лога
    fprintf(log_file, "TIME   MODE  ANGLE   RATE   RETRACT  CH  SSM  DATA\n");
    printf("TIME   MODE  ANGLE   RATE   RETRACT  CH  SSM  DATA\n");
    
    while (sim_time <= SIM_TIME)
    {
        scenario_step(sim_time, &in, scenario, scenario_size);
        nws_manager_step(&in, &out);
        
        write_log(log_file, sim_time, &out);
        
        // Вывод в консоль (каждые 0.5 секунды)
        if ((int)(sim_time * 50) % 25 == 0)
        {
            printf("%6.2f  %d   %7.2f %7.2f    %d      %d    %d   0x%08X\n",
                sim_time, out.steering_mode, out.wheel_angle_deg, out.wheel_rate_deg_s,
                out.gear_retract_enable, out.active_channel, out.angle_word.ssm, out.angle_word.data);
        }
        
        sim_time += DT;
    }
    
    fclose(log_file);
    printf("\nЛог сохранён в файл: %s\n", LOG_FILENAME);
    return 0;
}