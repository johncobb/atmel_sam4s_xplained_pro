#ifndef SERVO_H_
#define SERVO_H_

#include <asf.h>

void servo_init(void);
void servo_tick(void);
void servo_set_angle(float angle);
void servo_min(void);
void servo_max(void);
void servo_mid(void);
void servo_increment(void);
void servo_decrement(void);

#endif
