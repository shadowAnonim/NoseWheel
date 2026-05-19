#ifndef nws_DEFS_H
#define nws_DEFS_H

#include <stdio.h>

#define DT 0.02f

// Режимы работы
#define STATE_FREE_CASTORING 0
#define STATE_TAXI_MODE 1
#define STATE_TAKEOFF_MODE 2

// Пороги скорости
#define SPEED_THRESHOLD_TAKEOFF 50.0f    // скорость перехода в режим взлёта
#define AERO_DEMPER_DENOMINATOR 200.0f   // знаменатель для расчёта аэродинамического демпфера

// Ограничения углов
#define MAX_TILLER_ANGLE 90.0f
#define MAX_PEDAL_ANGLE 7.0f

// Гидравлика
#define HYD_MIN_PRESSURE 1800.0f
#define HYD_NOMINAL_PRESSURE 3000.0f

// ARINC429
#define SSM_NORMAL 0
#define SSM_NO_COMPUTED_DATA 3
#define INVALID_DATA 0xFFFFFFFFu

// Метки ARINC429
#define LABEL_MODE 1
#define LABEL_TARGET_ANGLE 2
#define LABEL_VALVE 3
#define LABEL_CENTERING 4
#define LABEL_ACTIVE_CHANNEL 5

// Параметры PI-регулятора
#define KP_TAXI 1.2f
#define KP_TAKEOFF 0.8f
#define KI 0.05f
#define MAX_INTEGRAL 5.0f

// Параметры гидравлического привода
#define SERVO_NATURAL_FREQ 25.0f      // рад/сек
#define SERVO_DAMPING 0.7f
#define MAX_WHEEL_RATE 25.0f          // град/сек
#define MAX_SERVO_CMD 1.0f

// Коэффициенты влияния
#define PRESSURE_NOMINAL 3000.0f
#define AERO_DEMPER_START_SPEED 50.0f
#define AERO_DEMPER_MAX_FACTOR 0.2f
#define HYD_LEAK_FACTOR 0.3f
#define WHEEL_INERTIA_FILTER 15.0f

// Расход гидравлики и ток
#define SERVO_CURRENT_FACTOR 15.0f
#define HYD_FLOW_IDLE 0.5f
#define HYD_FLOW_FACTOR 2.0f

// Допуск для уборки шасси
#define GEAR_RETRACT_TOLERANCE 2.0f

// Параметры логирования
#define LOG_FILENAME "nws_sim.log"

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
    unsigned int mode;
    unsigned int target_angle;

    unsigned int valve_open;
    unsigned int centering_cmd;

    unsigned int active_channel;
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

// Функции
void nws_manager_step(Input_t* in, Output_t* out);
void nws_cu_step(Input_t* in, Bus_t* bus, CU_Channel_t* ch1, CU_Channel_t* ch2);
void nws_phys_step(Input_t* in, Bus_t* bus, Output_t* out);

float nws_limit(float value, float min, float max);
float nws_abs(float v);
float nws_integrator(float input, float* state, float dt);

int read_scenario(FILE* file, Input_t* in);
void write_log(FILE* file, float time, Output_t* out);

#endif