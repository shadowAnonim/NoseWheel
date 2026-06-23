// 32-40_inout.c
#include <stdio.h>
#include "32-40_defs.h"

int read_scenario(
    const char* filename,
    ScenarioPoint_t* scenario,
    int max_points
)
{
    FILE* file = fopen(filename, "r");
    if (file == NULL)
    {
        return 0;
    }
    
    char line[256];
    int count = 0;
    
    while (fgets(line, sizeof(line), file) && count < max_points)
    {
        // Ручная обработка концов строки
        int len = 0;
        while (line[len] != '\0' && line[len] != '\r' && line[len] != '\n')
        {
            len++;
        }
        line[len] = 0;
        
        // Проверка на пустую строку или комментарий
        int is_empty = 1;
        int i = 0;
        while (line[i] != '\0')
        {
            if (line[i] != ' ' && line[i] != '\t' && line[i] != '\r' && line[i] != '\n')
            {
                is_empty = 0;
                break;
            }
            i++;
        }
        
        if (is_empty || line[0] == '#')
        {
            continue;
        }
        
        if (sscanf(line, "%f %f %f %f %f %f %f %d %d",
            &scenario[count].time,
            &scenario[count].speed,
            &scenario[count].tiller1,
            &scenario[count].tiller2,
            &scenario[count].pedal1,
            &scenario[count].pedal2,
            &scenario[count].hyd,
            &scenario[count].gear_up,
            &scenario[count].fails) == 9)
        {
            count++;
        }
    }
    
    fclose(file);
    return count;
}

void write_log(
    const char* filename,
    float time,
    Output_t* out,
    Input_t* in
)
{
    static FILE* file = NULL;
    static int header_written = 0;
    
    if (file == NULL)
    {
        file = fopen(filename, "w");
        if (file == NULL)
        {
            return;
        }
        header_written = 0;
    }
    
    if (!header_written)
    {
        fprintf(file, "TIME   MODE  ANGLE   RATE   RETRACT  CH  SSM  DATA(hex)  SPEED TILLER1 TILLER2 PEDAL1 PEDAL2 HYD GEAR\n");
        header_written = 1;
    }
    
    fprintf(
        file,
        "%6.2f  %d   %7.2f %7.2f    %d      %d    %d   0x%08X  %5.1f  %5.2f  %5.2f  %5.2f  %5.2f  %5.0f  %d\n",
        time,
        out->steering_mode,
        out->wheel_angle_deg,
        out->wheel_rate_deg_s,
        out->gear_retract_enable,
        out->active_channel,
        out->angle_word.ssm,
        out->angle_word.data,
        in->aircraft_speed,
        in->tiller_cmd_1,
        in->tiller_cmd_2,
        in->rudder_pedal_cmd_1,
        in->rudder_pedal_cmd_2,
        in->hyd_pressure,
        in->gear_lever_up
    );
}