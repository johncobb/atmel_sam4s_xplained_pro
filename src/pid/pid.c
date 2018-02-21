#include "pid.h"
#include "imu.h"


clock_time_t time = 0;
clock_time_t elapsed_time = 0;
clock_time_t previous_time = 0;
float pid = 0.0f;
float error = 0.0f;
float previous_error = 0.0f;
float pid_p = 0.0f;
float pid_i = 0.0f;
float pid_d = 0.0f;
// float kp = 3.55f;
// float ki = 0.005f;
// float kd = 2.05f;
float kp = 1.0f;
float ki = 0.0f;
float kd = 0.0;
float desired_angle = 0.0f;


void pid_tick(void)
{
    previous_error = time;
    time = cph_get_millis();
    elapsed_time = (time - previous_time)/1000;

    error = ap.imu.y_axis - desired_angle;
    pid_p = kp*error;

    // if (-3.0f < error < 3.0f) {
    //     pid_i = pid_i +(ki*error);
    // }

    pid_i = pid_i +(ki*error);

    pid_d = kd * ((error-previous_error)/elapsed_time);

    pid = pid_p + pid_i + pid_d;

    previous_error = error;
}