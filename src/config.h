#ifndef CONFIG_H_
#define CONFIG_H_

#include <stdint.h>
#include <stdbool.h>


typedef struct {
    bool gyro_calibrate;
    bool accel_calibrate;
    int16_t gyro_offset_x;
    int16_t gyro_offset_y;
    int16_t gyro_offset_z;
    int16_t accel_offset_x;
    int16_t accel_offset_y;
    int16_t accel_offset_z;
} config_t;

extern config_t config;


void config_init(void);


#endif /* CONFIG_H_ */