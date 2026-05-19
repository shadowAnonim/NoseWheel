#ifndef nws_DEFS_H
#define nws_DEFS_H

#define DT 0.02f

#define STATE_FREE_CASTORING 0
#define STATE_TAXI_MODE 1
#define STATE_TAKEOFF_MODE 2

#define MAX_TILLER_ANGLE 90.0f
#define MAX_PEDAL_ANGLE 7.0f

#define HYD_MIN_PRESSURE 1800.0f

#define SSM_NORMAL 0
#define SSM_NO_COMPUTED_DATA 3

#define INVALID_DATA 0xFFFFFFFFu

typedef struct
{
    char label;
    char sdi;
    int data;
    unsigned char ssm;
} Arinc429Word_t;

typedef struct
{
    int dc_bus_power;
    int sensor_power;

    float hyd_pressure;
    int hyd_low_pressure;
    
    float aircraft_speed;

    float rudder_pedal_cmd;
    float tiller_cmd;

    int gear_lever_up;
    int wow_nlg;

    int gear_reset;

    int fail_cu_ch1;
    int fail_cu_ch2;

    int fail_servo_jam;
    int fail_hydraulic_leak;
    int fail_angle_sensor;
} Input_t;

typedef struct
{
    Arinc429Word_t mode;
    Arinc429Word_t target_angle;

    Arinc429Word_t valve_open;
    Arinc429Word_t centering_cmd;

    Arinc429Word_t active_channel;
} Bus_t;

typedef struct
{
    float wheel_angle_deg;
    float wheel_rate_deg_s;

    float servo_current;
    float hydraulic_consumption;

    int gear_retract_enable;
    int steering_mode;
    int cas_fault;

    int active_channel;

    Arinc429Word_t angle_word;
} Output_t;

typedef struct
{
    int healthy;
    int state;
} CU_Channel_t;

void nws_manager_step(
    Input_t* in,
    Output_t* out
);

void nws_cu_step(
    Input_t* in,
    Bus_t* bus,
    CU_Channel_t* ch1,
    CU_Channel_t* ch2
);

void nws_phys_step(
    Input_t* in,
    Bus_t* bus,
    Output_t* out
);

float nws_limit(
    float value,
    float min,
    float max
);

float nws_abs(float v);

float nws_integrator(
    float input,
    float* state,
    float dt
);
#endif
