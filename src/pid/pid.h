#ifndef PID_H_
#define PID_H_

#include <asf.h>
#include "cph_millis.h"

float pid;
float error;

float pid_tick(void);



#endif