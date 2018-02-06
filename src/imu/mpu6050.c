#include <asf.h>

#include <stdio.h>
#include <string.h>
#include "mpu6050.h"

/*
https://github.com/jarzebski/Arduino-MPU6050/blob/master/MPU6050.cpp
http://community.atmel.com/forum/samc21-printf-not-printing-float-values

*/

static uint8_t imu_buffer[16] = {0};

void write_register8(uint8_t reg, uint8_t value);
void write_register16(uint8_t reg, int16_t value);
uint8_t read_register8(uint8_t reg);
void write_register_bit(uint8_t reg, uint8_t pos, bool state);
bool read_register_bit(uint8_t reg, uint8_t pos);
int16_t read_register16(uint8_t reg);


void write_register8(uint8_t reg, uint8_t value)
{

    twi_options_t opt;
    twi_packet_t packet_tx;

	/* Configure the options of TWI driver */
	opt.master_clk = sysclk_get_peripheral_hz();
    opt.speed      = TWI_CLK;

    memset(imu_buffer, 0, sizeof(imu_buffer));

    packet_tx.chip = IMU_ADDRESS;
    packet_tx.addr[0] = reg;
    packet_tx.addr_length = sizeof(uint8_t);
    packet_tx.buffer = &value;
    // packet_tx.length = 1;
    packet_tx.length = sizeof(uint8_t);


    uint32_t status = twi_master_write(IMU_TWI, &packet_tx);

    // printf("write_register status: %d\r\n", status);
    
    if (status == TWI_SUCCESS) {
        puts("write_register: success\r\n");
    }

    delay_ms(TWI_WAIT_TIME);
}


void write_register16(uint8_t reg, int16_t value)
{

    twi_options_t opt;
    twi_packet_t packet_tx;

	/* Configure the options of TWI driver */
	opt.master_clk = sysclk_get_peripheral_hz();
    opt.speed      = TWI_CLK;

    memset(imu_buffer, 0, sizeof(imu_buffer));

    packet_tx.chip = IMU_ADDRESS;
    packet_tx.addr[0] = reg;
    packet_tx.addr_length = sizeof(uint8_t);
    packet_tx.buffer = &value;
    // packet_tx.length = 1;
    packet_tx.length = sizeof(int16_t);


    uint32_t status = twi_master_write(IMU_TWI, &packet_tx);

    // printf("write_register status: %d\r\n", status);
    
    if (status == TWI_SUCCESS) {
        puts("write_register: success\r\n");
    }

    delay_ms(TWI_WAIT_TIME);
}

uint8_t read_register8(uint8_t reg)
{
    uint8_t value;
    twi_packet_t packet_rx;

    memset(imu_buffer, 0, sizeof(imu_buffer));

    packet_rx.chip = IMU_ADDRESS;
    packet_rx.addr[0] = reg;
    packet_rx.addr_length = sizeof(uint8_t);
    packet_rx.buffer = &imu_buffer;

    // packet_rx.length = 1;
    packet_rx.length = sizeof(uint8_t);

    uint32_t status = twi_master_read(IMU_TWI, &packet_rx);

    // printf("read_register status: %d\r\n", status);
    
    if (status == TWI_SUCCESS) {
        // puts("read_register: success\r\n");
        value = imu_buffer[0];
    }

    delay_ms(TWI_WAIT_TIME);

    return value;
}

int16_t read_register16(uint8_t reg)
{
    int16_t value;
    twi_packet_t packet_rx;

    memset(imu_buffer, 0, sizeof(imu_buffer));

    packet_rx.chip = IMU_ADDRESS;
    packet_rx.addr[0] = reg;
    packet_rx.addr_length = sizeof(uint8_t);
    packet_rx.buffer = &imu_buffer;

    // packet_rx.length = 1;
    packet_rx.length = sizeof(uint16_t);

    uint32_t status = twi_master_read(IMU_TWI, &packet_rx);

    // printf("read_register status: %d\r\n", status);
    
    if (status == TWI_SUCCESS) {
        // puts("read_register: success\r\n");
        uint8_t value_high = imu_buffer[0];
        uint8_t value_low = imu_buffer[1];

        value = value_high << 8 | value_low;
    }

    delay_ms(TWI_WAIT_TIME);

    return value;
}


void write_register_bit(uint8_t reg, uint8_t pos, bool state)
{
    uint8_t value;
    value = read_register8(reg);

    if (state) {
        value |= (1 << pos);
    } else {
        value &= ~(1 << pos);
    }

    write_register8(reg, value);
}

bool read_register_bit(uint8_t reg, uint8_t pos)
{
    uint8_t value;
    value = read_register8(reg);
    return ((value >> pos) & 1);
}

void imu_init(void)
{
    twi_options_t opt;

	/* Configure the options of TWI driver */
	opt.master_clk = sysclk_get_peripheral_hz();
    opt.speed      = TWI_CLK;

    if (twi_master_init(IMU_TWI, &opt) != TWI_SUCCESS) {
        puts("twi_master_init: failed\r\n");
        
    } else {
        puts("twi_master_init: success\r\n");
    }
    // Give I2C time to settle
    delay_ms(TWI_WAIT_TIME);
}

void imu_probe(void)
{
    if (twi_probe(IMU_TWI, IMU_ADDRESS) != TWI_SUCCESS) {
        puts("twi_probe: failed\r\n");
        
    } else {
        puts("twi_probe: success\r\n");
    }
}

void imu_who_am_i(void)
{
    uint8_t value = read_register8(MPU6050_RA_WHO_AM_I);

    printf("who_am_i: 0x%1x\r\n", value);

    if (value == 0x68) {
        puts("who_am_i: success\r\n");
    }
}

int16_t imu_get_temperature(void)
{
    int16_t T;
    T = read_register16(MPU6050_RA_TEMP_OUT_H);
    return T;
}



void imu_set_clock_source(uint8_t source)
{
    uint8_t value;

    value = read_register8(MPU6050_RA_PWR_MGMT_1);
    value &= 0b11111000; // mask
    value |= source;

    write_register8(MPU6050_RA_PWR_MGMT_1, value);
}

uint8_t imu_get_clock_source(void)
{
    uint8_t value;

    value = read_register8(MPU6050_RA_PWR_MGMT_1);
    value &= 0b00000111; // mask

    return (uint8_t)value;
}

void imu_set_gyro_scale(uint8_t scale)
{
    uint8_t value;

    value = read_register8(MPU6050_RA_GYRO_CONFIG);
    value &= 0b11100111; // mask
    value |= (scale << 3);

    write_register8(MPU6050_RA_GYRO_CONFIG, value);
}

uint8_t imu_get_gyro_scale(void)
{
    uint8_t value;

    value = read_register8(MPU6050_RA_GYRO_CONFIG);
    value &= 0b00011000; // mask
    value >>= 3;

    return (uint8_t)value;
}

void imu_set_accel_range(uint8_t range)
{
    uint8_t value;

    value = read_register8(MPU6050_RA_ACCEL_CONFIG);
    value &= 0b11100111; // mask
    value |= (range << 3);

    write_register8(MPU6050_RA_ACCEL_CONFIG, value);
}

uint8_t imu_get_accel_range(void)
{
    uint8_t value;

    value = read_register8(MPU6050_RA_ACCEL_CONFIG);
    value &= 0b00011000; // mask
    value >>= 3;

    return (uint8_t)value;
}

void imu_set_dlpf_mode(uint8_t mode)
{
    uint8_t value;

    value = read_register8(MPU6050_RA_ACCEL_CONFIG);
    value &= 0b11111000; // mask
    value |= (mode << 3);

    write_register8(MPU6050_RA_ACCEL_CONFIG, value);
}

uint8_t imu_get_dlpf_mode(void)
{
    uint8_t value;

    value = read_register8(MPU6050_RA_ACCEL_CONFIG);
    value &= 0b00000111; // mask
    value >>= 3;

    return (uint8_t)value;
}

void imu_set_sleep_enabled(bool state)
{
    write_register_bit(MPU6050_RA_PWR_MGMT_1, 6, state);
}

bool imu_get_sleep_enabled(void)
{
    read_register_bit(MPU6050_RA_PWR_MGMT_1, 6);
}

void imu_set_int_zero_motion_enabled(bool state)
{
    write_register_bit(MPU6050_RA_INT_ENABLE, 5, state);
}

bool imu_get_int_zero_motion_enabled(void)
{
    read_register_bit(MPU6050_RA_INT_ENABLE, 5);
}

void imu_set_int_motion_enabled(bool state)
{
    write_register_bit(MPU6050_RA_INT_ENABLE, 6, state);
}

bool imu_get_int_motion_enabled(void)
{
    read_register_bit(MPU6050_RA_INT_ENABLE, 6);
}

void imu_set_int_freefall_enabled(bool state)
{
    write_register_bit(MPU6050_RA_INT_ENABLE, 7, state);
}

bool imu_get_int_freefall_enabled(void)
{
    read_register_bit(MPU6050_RA_INT_ENABLE, 7);
}

void imu_set_motion_detection_threshold(uint8_t threshold)
{
    write_register8(MPU6050_RA_MOT_THR, threshold);
}

uint8_t imu_get_motion_detection_threshold(void)
{
    return read_register8(MPU6050_RA_MOT_THR);
}

void imu_set_motion_detection_duration(uint8_t duration)
{
    write_register8(MPU6050_RA_MOT_DUR, duration);
}

uint8_t imu_get_motion_detection_duration(void)
{
    return read_register8(MPU6050_RA_MOT_DUR);
}

void imu_set_zero_motion_detection_threshold(uint8_t threshold)
{
    write_register8(MPU6050_RA_ZRMOT_THR, threshold);
}

uint8_t imu_get_zero_motion_detection_threshold(void)
{
    return read_register8(MPU6050_RA_ZRMOT_THR);
}

void imu_set_zero_motion_detection_duration(uint8_t duration)
{
    write_register8(MPU6050_RA_ZRMOT_DUR, duration);
}

uint8_t imu_get_zero_motion_detection_duration(void)
{
    return read_register8(MPU6050_RA_ZRMOT_DUR);
}

void imu_set_freefall_detection_threshold(uint8_t threshold)
{
    write_register8(MPU6050_RA_FF_THR, threshold);
}

uint8_t imu_get_freefall_detection_threshold(void)
{
    return read_register8(MPU6050_RA_FF_THR);
}

void imu_set_freefall_detection_duration(uint8_t duration)
{
    write_register8(MPU6050_RA_FF_DUR, duration);
}

uint8_t imu_get_freefall_detection_duration(void)
{
    return read_register8(MPU6050_RA_FF_DUR);
}

bool imu_set_i2c_master_mode_enabled(bool state)
{
    write_register_bit(MPU6050_RA_USER_CTRL, 5, state);
}
void imu_get_i2c_master_mode_enabled(void)
{
    read_register_bit(MPU6050_RA_USER_CTRL, 5);
}

bool imu_set_i2c_bypass_enabled(bool state)
{
    write_register_bit(MPU6050_RA_INT_PIN_CFG, 1, state);
}
void imu_get_i2c_bypass_enabled(void)
{
    read_register_bit(MPU6050_RA_INT_PIN_CFG, 1);
}

uint8_t imu_get_accel_power_on_delay(void)
{
    uint8_t value;

    value = read_register8(MPU6050_RA_MOT_DETECT_CTRL);
    value &= 0b00110000; // mask
    value >>= 4;

    return (uint8_t)value;
}

void imu_set_accel_power_on_delay(uint8_t delay)
{
    uint8_t value;

    value = read_register8(MPU6050_RA_MOT_DETECT_CTRL);
    value &= 0b11001111; // mask
    value |= (delay << 4);

    write_register8(MPU6050_RA_MOT_DETECT_CTRL, value);
}

uint8_t imu_get_int_status(void)
{
    return read_register8(MPU6050_RA_INT_STATUS);
}


// void imu_set_clock_source(uint8_t source)
// {
//     uint8_t value;
    
//     twi_options_t opt;
//     twi_packet_t packet_tx;
//     twi_packet_t packet_rx;

// 	/* Configure the options of TWI driver */
// 	opt.master_clk = sysclk_get_peripheral_hz();
//     opt.speed      = TWI_CLK;

//      memset(imu_buffer, 0, sizeof(imu_buffer));

//     packet_rx.chip = IMU_ADDRESS;
//     packet_rx.addr[0] = MPU6050_RA_PWR_MGMT_1;
//     packet_rx.addr_length = sizeof(uint8_t);
//     packet_rx.buffer = &value;
//     packet_rx.length = 1;

//     uint32_t status = twi_master_read(IMU_TWI, &packet_rx);

//     if (status == TWI_SUCCESS) {
//         value &= 0b11111000;
//         value |= source;
//     }


//     packet_tx.chip = IMU_ADDRESS;
//     packet_tx.addr[0] = MPU6050_RA_PWR_MGMT_1;
//     packet_tx.addr_length = sizeof(uint8_t);
//     packet_tx.buffer = &value;
//     packet_tx.length = 1;


//     status = twi_master_write(IMU_TWI, &packet_tx);

//     printf("twi_master_write status: %d\r\n", status);
    
//     if (status == TWI_SUCCESS) {

//         puts("imu_set_clock_source: success\r\n");
//     }

// }

// void imu_set_full_scale_gyro_range(uint8_t range)
// {
//            // uint32_t i;
//     twi_options_t opt;
//     twi_packet_t packet_tx;
//     twi_packet_t packet_rx;

// 	/* Configure the options of TWI driver */
// 	opt.master_clk = sysclk_get_peripheral_hz();
//     opt.speed      = TWI_CLK;

//      memset(imu_buffer, 0, sizeof(imu_buffer));

//     packet_tx.chip = IMU_ADDRESS;
//     packet_tx.addr[0] = MPU6050_RA_PWR_MGMT_1;
//     packet_tx.addr_length = sizeof(uint8_t);
//     packet_tx.buffer = &range;
//     packet_tx.length = 1;


//     uint32_t status = twi_master_write(IMU_TWI, &packet_tx);

//     printf("twi_master_write status: %d\r\n", status);
    
//     if (status == TWI_SUCCESS) {

//         puts("imu_set_clock_source: success\r\n");
//     } 
// }