#ifndef MPU6050_H_
#define MPU6050_H_




/** EEPROM Wait Time */
#define TWI_WAIT_TIME   10
/** TWI Bus Clock 400kHz */
#define TWI_CLK     400000
// #define TWI_CLK     100000L

#define IMU_TWI_ID                      ID_TWI0
#define IMU_ADDRESS                     0x68
#define MPU6050_RA_WHO_AM_I             0x75
#define MPU6050_WHO_AM_I_BIT            6
#define MPU6050_WHO_AM_I_LENGTH         6

#define MPU6050_RA_PWR_MGMT_1           0x6B
#define MPU6050_RA_GYRO_CONFIG          0x1B
#define MPU6050_RA_ACCEL_CONFIG         0x1C
#define MPU6050_RA_INT_ENABLE           0x38


#define MPU6050_PWR1_CLKSEL_BIT         2
#define MPU6050_PWR1_CLKSEL_LENGTH      3
#define MPU6050_CLOCK_PLL_XGYRO         0x01
#define MPU6050_GYRO_FS_250             0x00

#define MPU6050_CLOCK_PLL_XGYRO         0x01

#define MPU6050_GYRO_FS_250         0x00
#define MPU6050_GYRO_FS_500         0x01
#define MPU6050_GYRO_FS_1000        0x02
#define MPU6050_GYRO_FS_2000        0x03

#define MPU6050_ACCEL_FS_2          0x00
#define MPU6050_ACCEL_FS_4          0x01
#define MPU6050_ACCEL_FS_8          0x02
#define MPU6050_ACCEL_FS_16         0x03

#define MPU6050_DLPF_BW_256         0x00
#define MPU6050_DLPF_BW_188         0x01
#define MPU6050_DLPF_BW_98          0x02
#define MPU6050_DLPF_BW_42          0x03
#define MPU6050_DLPF_BW_20          0x04
#define MPU6050_DLPF_BW_10          0x05
#define MPU6050_DLPF_BW_5           0x06

#define MPU6050_RA_TEMP_OUT_H       0x41
#define MPU6050_RA_TEMP_OUT_L       0x42

#define IMU_TWI  TWI0

void imu_init(void);
void imu_probe(void);
void imu_who_am_i(void);
void imu_set_clock_source(uint8_t source);
int16_t imu_get_temperature(void);
uint8_t imu_get_clock_source(void);
void imu_set_full_scale_gyro_range(uint8_t range);
void imu_set_gyro_scale(uint8_t scale);
uint8_t imu_get_gyro_scale(void);
void imu_set_accel_range(uint8_t range);
uint8_t imu_get_accel_range(void);

void imu_set_dlpf_mode(uint8_t mode);
uint8_t imu_get_dlpf_mode(void);

void imu_set_sleep_enabled(bool state);
bool imu_get_sleep_enabled(void);

void imu_set_int_zero_motion_enabled(bool state);
bool imu_get_int_zero_motion_enabled(void);

void imu_set_int_motion_enabled(bool state);
bool imu_get_int_motion_enabled(void);

void imu_set_int_freefall_enabled(bool state);
bool imu_get_int_freefall_enabled(void);


#endif