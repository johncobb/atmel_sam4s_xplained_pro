#ifndef MPU6050_H_
#define MPU6050_H_




/** EEPROM Wait Time */
#define WAIT_TIME   10
/** TWI Bus Clock 400kHz */
#define TWI_CLK     400000
// #define TWI_CLK     100000L

#define IMU_TWI_ID                      ID_TWI0
#define IMU_ADDRESS                     0x68
#define MPU6050_RA_WHO_AM_I             0x75
#define MPU6050_WHO_AM_I_BIT            6
#define MPU6050_WHO_AM_I_LENGTH         6

#define MPU6050_RA_PWR_MGMT_1           0x6B
#define MPU6050_PWR1_CLKSEL_BIT         2
#define MPU6050_PWR1_CLKSEL_LENGTH      3
#define MPU6050_CLOCK_PLL_XGYRO         0x01

#define MPU6050_CLOCK_PLL_XGYRO         0x01

#define IMU_TWI  TWI0

void imu_init(void);
void imu_probe(void);
void imu_who_am_i(void);
void imu_set_clock_source(uint8_t source);


#endif