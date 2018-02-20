#ifndef PID_H_
#define PID_H_

#include <asf.h>
#include "cph_millis.h"

#define AUTOMATIC	1
#define MANUAL	0
#define DIRECT  0
#define REVERSE  1

double kp;
double ki;
double kd;

int8_t controller_direction;

double *pid_input;
double *pid_output;
double *pid_setpoint;

clock_time_t last_time;
clock_time_t sample_time;
double output_sum;
double last_input;

double output_min;
double output_max;
bool in_auto;

void pid_init(double *input, double *output, double *setpoint, double kp, double ki, double kd);
void pid_tick(void);
void pid_set_output_limits(double min, double max);
bool pid_compute(void);
void pid_set_mode(int mode);



#endif