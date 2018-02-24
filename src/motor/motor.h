#ifndef MOTOR_H_
#define MOTOR_H_

#include <asf.h>
#include "cph_millis.h"

#define NUM_MOTORS 4

#define AP_ANGLE_MIN -90
#define AP_ANGLE_MAX 90
#define AP_ANGLE_MID 0
#define MOTOR_PWM_CLOCKSOURCE_FREQ 1000000
#define MOTOR_PWM_FREQ 50
#define MOTOR_PWM_PERIOD_TICKS MOTOR_PWM_CLOCKSOURCE_FREQ/MOTOR_PWM_FREQ

// #define PWM_MIN 900
// #define PWM_MID 1500
// #define PWM_MAX 2100


// #define PWM_MIN 		700
#define MOTOR_PWM_MIN 		1150
#define MOTOR_PWM_MID 		1350
#define MOTOR_PWM_MAX 		2000
#define MOTOR_PWM_STEP		10

pwm_clock_t motor_clock_setting;

typedef struct t_pwm_motor
{
    Pwm *pwm_channel;
    pwm_channel_t pwm_motor_channel;
    pwm_clock_t clock_setting;
    uint32_t pwm_pin;
    uint32_t pwm_peripheral;
    uint32_t pwm_clock_id;
    uint32_t duty_cycle;
    clock_time_t timeout;
    long angle_min;
    long angle_max;
} t_motor;

t_motor motors[NUM_MOTORS];

void motor_init(void);
void motor_tick(void);
void motor_set_angle(float angle);
void motor_min(t_motor motor);
void motor_max(t_motor motor);
void motor_mid(t_motor motor);
void motor_increment(void);
void motor_decrement(void);
void motor_set_power(t_motor motor, uint32_t power);






#endif