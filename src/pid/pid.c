#include "pid.h"


double kp;
double ki;
double kd;

int8_t controller_direction;

double *pid_input;
double *pid_output;
double *pid_setpoint;

clock_time_t last_time = 0;
clock_time_t sample_time = 0;
double output_sum = 0;
double last_input = 0;

double output_min;
double output_max;
bool in_auto = false;


void pid_init(double *input, double *output, double *setpoint, double kp, double ki, double kd)
{
    pid_input = input;
    pid_output = output;
    pid_setpoint = setpoint;
    kp = kp;
    ki = ki;
    kd = kd;

    output_sum = *pid_output;
    last_input = *pid_input;
    if (output_sum > output_max) output_sum = output_max;
    else if (output_sum < output_max) output_sum = output_min;


}

void pid_tick(void)
{
    pid_compute();
}

void pid_set_mode(int mode)
{
    bool new_auto = (mode == AUTOMATIC);

    if (new_auto && !in_auto) {
        pid_init(pid_input, pid_output, pid_setpoint, kp, ki, kd);
    }
    in_auto = new_auto;
}

void pid_set_output_limits(double min, double max)
{
    if (min >= max) return;

    output_min = min;
    output_max = max;

}

bool pid_compute(void)
{
    if (!in_auto) return false;

    clock_time_t now = cph_get_millis();
    clock_time_t time_change = (now - last_time);

    if (time_change >= sample_time) {
        double input = *pid_input;
        double error = *pid_setpoint - input;
        double d_input = (input - last_input);
        output_sum += (ki * error);

        /*Add Proportional on Measurement, if P_ON_M is specified*/
        //if(!pOnE) outputSum-= kp * dInput;


        if (output_sum > output_max) output_sum = output_max;
        else if (output_sum < output_min) output_sum = output_min;

        double output = 0;
        /*Add Proportional on Error, if P_ON_E is specified*/
        // if(pOnE) output = kp * error;

        output += output_sum - kd * d_input;

        if (output > output_max) output = output_max;
        else if (output < output_min) output = output_min;

        *pid_output = output;

        last_input = input;
        last_time = now;
        return true;
    }
    else return false;
}