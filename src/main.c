#include <asf.h>

#include <stdio.h>
#include <string.h>
#include "config.h"
#include "gyro.h"
#include "mpu6050.h"




static void configure_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
#ifdef CONF_UART_CHAR_LENGTH
		.charlength = CONF_UART_CHAR_LENGTH,
#endif
		.paritytype = CONF_UART_PARITY,
#ifdef CONF_UART_STOP_BITS
		.stopbits = CONF_UART_STOP_BITS,
#endif
	};

	/* Configure console UART. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);
}


typedef uint32_t clock_time_t;
extern volatile clock_time_t clock_millis;

volatile clock_time_t clock_millis = 0;
#define clock_time()	clock_millis

void SysTick_Handler(void) {
    clock_millis++;
}

int main(void)
{
    sysclk_init();
    board_init();
    delay_init();
    pmc_enable_periph_clk(IMU_TWI_ID);

    configure_console();
    puts("\r\n\r\nsam4d32c imu demo...\r\n");

    for (int i=0; i<5; i++) {
        puts(".");
        delay_ms(250);
    }
    puts("\r\n");

    SysTick_Config(sysclk_get_cpu_hz() / 1000);


    imu_init();
    imu_probe();
    
    // imu_who_am_i();

    // float temperature = (float)imu_get_temperature()/340 + 36.53;
    // printf("temperature: %f\r\n", temperature);

    // imu_set_clock_source(MPU6050_CLOCK_PLL_XGYRO);
    // printf("clock_source: %d\r\n", imu_get_clock_source());
    // imu_set_gyro_scale(MPU6050_GYRO_FS_250);
    // printf("imu_gyro_scale: %d\r\n", imu_get_gyro_scale());

    // imu_set_accel_range(MPU6050_ACCEL_FS_2);
    // printf("imu_accel_range: %d\r\n", imu_get_accel_range());

    // imu_set_sleep_enabled(false);
    // printf("imu_sleep_enabled: %d\r\n", imu_get_sleep_enabled());

    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    float pitch = 0.0f;
    float roll = 0.0f;
    float yaw = 0.0f;

    config.gyro_calibrate = true;

    clock_time_t last_time_read = 0;
    clock_time_t f_timeout = 0;
    clock_time_t f_log_timeout = 0;

    const float alpha = 0.96f;
    float last_angle_x = 0.0f;
    float last_angle_y = 0.0f;
    float last_angle_z = 0.0f;

    float last_gyro_angle_x = 0.0f;
    float last_gyro_angle_y = 0.0f;
    float last_gyro_angle_z = 0.0f;

    if (imu_begin(MPU6050_GYRO_FS_250, MPU6050_ACCEL_FS_2)) {
        

    
        imu_calibrate_gyro(GYRO_SAMPLES);

        imu_set_threshold(0);

        imu_log_settings();


        while(true) {

            clock_time_t t_now = clock_time();

            imu_read_normalized_gyro();
            imu_read_normalized_acceleration();



            float gyro_x = (norm_gyro.x_axis*M_PI)/180.0f;
            float gyro_y = (norm_gyro.y_axis*M_PI)/180.0f;
            float gyro_z = (norm_gyro.z_axis*M_PI)/180.0f;



            float accel_angle_y = atan2(norm_accel.x_axis, sqrt( pow(norm_accel.y_axis, 2) + pow(norm_accel.z_axis, 2))) * 180.0f / M_PI;
            float accel_angle_x = atan2(norm_accel.y_axis, sqrt( pow(norm_accel.x_axis, 2) + pow(norm_accel.z_axis, 2))) * 180.0f / M_PI;
            float accel_angle_z = 0;

            // imu_read_raw_gyro();
            // imu_read_raw_acceleration();

            // float gyro_x = raw_gyro.x_axis/131*M_PI/180.0f;
            // float gyro_y = raw_gyro.y_axis/131*M_PI/180.0f;
            // float gyro_z = raw_gyro.z_axis/131*M_PI/180.0f;

            // float accel_angle_y = atan2(raw_accel.x_axis, sqrt( pow(raw_accel.y_axis, 2) + pow(raw_accel.z_axis, 2))) * 180.0f / M_PI;
            // float accel_angle_x = atan2(raw_accel.y_axis, sqrt( pow(raw_accel.x_axis, 2) + pow(raw_accel.z_axis, 2))) * 180.0f / M_PI;
            // float accel_angle_z = 0;



            // Compute filtered angles
            clock_time_t delta_t = (t_now-last_time_read);
            float dt = ((float)delta_t/1000.0f);

            float gyro_angle_x = gyro_x * dt + last_angle_x;
            float gyro_angle_y = gyro_y * dt + last_angle_y;
            float gyro_angle_z = gyro_z * dt + last_angle_z;

            // Compute drifting gyro angles
            float unfiltered_gyro_angle_x = gyro_x * dt + last_gyro_angle_x;
            float unfiltered_gyro_angle_y = gyro_y * dt + last_gyro_angle_y;
            float unfiltered_gyro_angle_z = gyro_z * dt + last_gyro_angle_z;

            float angle_x = alpha * gyro_angle_x + (1.0f - alpha) * accel_angle_x;
            float angle_y = alpha * gyro_angle_y + (1.0f - alpha) * accel_angle_y;
            float angle_z = alpha * gyro_angle_z + (1.0f - alpha) * accel_angle_z;

    
            last_time_read = t_now;
            last_angle_x = angle_x;
            last_angle_y = angle_y;
            last_angle_z = angle_z;

            if (clock_time() >= f_log_timeout) {
                f_log_timeout = clock_time() + 100;
                printf("roll/pitch/yaw: %f %f %f\r\n", angle_x, angle_y, angle_z);
            }

            
            
        
        }



        // if (config.gyro_calibrate == true) {
        //     gyro_calibrate_offset();
        // }
        // t_bool_activity a;
        // imu_read_activities(&a);

        // printf("overflow, freefall, inactivity, activity, dataready\r\n");
        // printf("%d,%d,%d,%d,%d\r\n", a.is_overflow, a.is_freefall, a.is_inactivity, a.is_activity, a.is_data_ready);
        // printf("negX, posX, negY, posY, negZ, posZ\r\n");
        // printf("%d,%d,%d,%d,%d,%d\r\n", a.is_neg_activity_on_x, a.is_pos_activity_on_x, a.is_neg_activity_on_y, a.is_neg_activity_on_y, a.is_neg_activity_on_z, a.is_pos_activity_on_z);

        while(true) {


            imu_read_gyro(&gx, &gy, &gz);
            imu_read_acceleration(&ax, &ay, &az);

            float gyro_x = (float)gx;
            float gyro_y = (float)gy;
            float gyro_z = (float)gz;
            float accel_x = (float)ax;
            float accel_y = (float)ay;
            float accel_z = (float)az;

            gyro_x = ((float) gx)/131*M_PI/180.0f;
            gyro_y = ((float) gy)/131*M_PI/180.0f;
            gyro_z = ((float) gz)/131*M_PI/180.0f;

            float accel_angle_y = atan2(accel_x, sqrt( pow(accel_y, 2) + pow(accel_z, 2))) * 180.0f / M_PI;
            float accel_angle_x = atan2(accel_y, sqrt( pow(accel_x, 2) + pow(accel_z, 2))) * 180.0f / M_PI;
            float accel_angle_z = 0;
    
            printf("gyro x/y/z: %d %d %d\r\n", gyro_x, gyro_y, gyro_z);
            delay_ms(100);
        }
    }


    


    while(1) {

        if(ioport_get_pin_level(BUTTON_0_PIN) == BUTTON_0_ACTIVE) {
            ioport_toggle_pin_level(LED0_GPIO);
            delay_ms(500); 
        }

    }
}






// void test_twi(void)
// {
//     uint32_t i;
//     twi_options_t opt;
//     twi_packet_t packet_tx;
//     twi_packet_t packet_rx;

// 	/* Configure the options of TWI driver */
// 	opt.master_clk = sysclk_get_peripheral_hz();
//     opt.speed      = TWI_CLK;
    

//     	/* Configure the data packet to be transmitted */
// 	packet_tx.chip        = IMU_ADDRESS;
// 	// packet_tx.addr[0]     = EEPROM_MEM_ADDR >> 8;
//     // packet_tx.addr[1]     = EEPROM_MEM_ADDR;
//     packet_tx.addr[0]     = MPU6050_RA_WHO_AM_I;
// 	packet_tx.addr_length = 1;
// 	packet_tx.buffer      = (uint8_t *) 1;
// 	packet_tx.length      = 1;

// 	/* Configure the data packet to be received */
// 	// packet_rx.chip        = packet_tx.chip;
// 	// packet_rx.addr[0]     = packet_tx.addr[0];
// 	// packet_rx.addr[1]     = packet_tx.addr[1];
// 	// packet_rx.addr_length = packet_tx.addr_length;
// 	// packet_rx.buffer      = gs_uc_test_data_rx;
//     // packet_rx.length      = packet_tx.length;
    

//     if (twi_master_init(IMU_TWI, &opt) != TWI_SUCCESS) {

//         ioport_toggle_pin_level(LED0_GPIO);
            
//         while (1) {
//             /* Capture error */
//             delay_ms(100); 
//         }
//     }
// }