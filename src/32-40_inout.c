#include <stdio.h>
#include <string.h>
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
        line[strcspn(line, "\r\n")] = 0;
        
        if (strlen(line) == 0 || line[0] == '#')
        {
            continue;
        }
        
        if (sscanf(line, "%f %f %f %f %f %d",
            &scenario[count].time,
            &scenario[count].speed,
            &scenario[count].tiller,
            &scenario[count].pedal,
            &scenario[count].hyd,
            &scenario[count].gear_up) == 6)
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
    Output_t* out
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
        fprintf(file, "TIME   MODE  ANGLE   RATE   RETRACT  CH  SSM  DATA\n");
        header_written = 1;
    }
    
    int signed_data = out->angle_word.data;
    if (out->angle_word.sdi == 1)
    {
        signed_data = -signed_data;
    }
    
    fprintf(
        file,
        "%6.2f  %d   %7.2f %7.2f    %d      %d    %d   %+7d\n",
        time,
        out->steering_mode,
        out->wheel_angle_deg,
        out->wheel_rate_deg_s,
        out->gear_retract_enable,
        out->active_channel,
        out->angle_word.ssm,
        signed_data
    );
}