#ifndef SERVO_H_
#define SERVO_H_

#include <asf.h>

void servo_init(void);
void servo_tick(void);
void servo_set_angle(float angle);

#endif
