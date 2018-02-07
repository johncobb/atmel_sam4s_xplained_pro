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

#define MPU6050_RA_MOT_DETECT_STATUS    0x61
#define MPU6050_RA_MOT_DETECT_CTRL      0x69
#define MPU6050_RA_USER_CTRL            0x6A
#define MPU6050_RA_PWR_MGMT_1           0x6B
#define MPU6050_RA_GYRO_CONFIG          0x1B
#define MPU6050_RA_ACCEL_CONFIG         0x1C
#define MPU6050_RA_INT_PIN_CFG          0x37
#define MPU6050_RA_INT_ENABLE           0x38
#define MPU6050_RA_FF_THR               0x1D
#define MPU6050_RA_FF_DUR               0x1E
#define MPU6050_RA_MOT_THR              0x1F
#define MPU6050_RA_MOT_DUR              0x20
#define MPU6050_RA_ZRMOT_THR            0x21
#define MPU6050_RA_ZRMOT_DUR            0x22
#define MPU6050_RA_INT_STATUS           0x3A
#define MPU6050_RA_ACCEL_XOUT_H         0x3B
#define MPU6050_RA_GYRO_XOUT_H          0x43



#define MPU6050_PWR1_CLKSEL_BIT         2
#define MPU6050_PWR1_CLKSEL_LENGTH      3
#define MPU6050_CLOCK_PLL_XGYRO         0x01
#define MPU6050_GYRO_FS_250             0x00

#define MPU6050_CLOCK_INTERNAL          0x00
#define MPU6050_CLOCK_PLL_XGYRO         0x01
#define MPU6050_CLOCK_PLL_YGYRO         0x02
#define MPU6050_CLOCK_PLL_ZGYRO         0x03
#define MPU6050_CLOCK_PLL_EXT32K        0x04
#define MPU6050_CLOCK_PLL_EXT19M        0x05
#define MPU6050_CLOCK_KEEP_RESET        0x07

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
bool imu_begin(uint8_t scale, uint8_t range);
uint8_t imu_who_am_i(void);
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

void imu_set_motion_detection_threshold(uint8_t threshold);
uint8_t imu_get_motion_detection_threshold(void);

void imu_set_motion_detection_duration(uint8_t duration);
uint8_t imu_get_motion_detection_duration(void);

void imu_set_zero_motion_detection_threshold(uint8_t threshold);
uint8_t imu_get_zero_motion_detection_threshold(void);

void imu_set_zero_motion_detection_duration(uint8_t duration);
uint8_t imu_get_zero_motion_detection_duration(void);

void imu_set_freefall_detection_threshold(uint8_t threshold);
uint8_t imu_get_freefall_detection_threshold(void);

void imu_set_freefall_detection_duration(uint8_t duration);
uint8_t imu_get_freefall_detection_duration(void);

void imu_set_i2c_master_mode_enabled(bool state);
bool imu_get_i2c_master_mode_enabled(void);

void imu_set_i2c_bypass_enabled(bool state);
bool imu_get_i2c_bypass_enabled(void);

void imu_set_accel_power_on_delay(uint8_t delay);
uint8_t imu_get_accel_power_on_delay(void);



uint8_t imu_get_int_status(void);



typedef struct fp_vector
{
    float x_axis;
    float y_axis;
    float z_axis;
} t_fp_vector;

typedef struct bool_activity
{
    bool is_overflow;
    bool is_freefall;
    bool is_inactivity;
    bool is_activity;
    bool is_pos_activity_on_x;
    bool is_pos_activity_on_y;
    bool is_pos_activity_on_z;
    bool is_neg_activity_on_x;
    bool is_neg_activity_on_y;
    bool is_neg_activity_on_z;
    bool is_data_ready;
} t_bool_activity;

t_fp_vector xyz;

t_fp_vector raw_accel;
t_fp_vector raw_gyro;

t_fp_vector normalized_accel;
t_fp_vector normalized_gyro;

t_bool_activity imu_activities;

void imu_read_activities(t_bool_activity *a);
void imu_log_settings(void);
void imu_read_rotation(int16_t *x, int16_t *y, int16_t *z);
// void imu_read_rotation(t_fp_vector *vect);
void imu_read_acceleration(int16_t *x, int16_t *y, int16_t *z);

#endif