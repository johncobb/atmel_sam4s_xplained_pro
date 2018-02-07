#include <asf.h>

#include <stdio.h>
#include <string.h>

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
    // t_fp_vector gyro;
    // t_fp_vector accel;

    if (imu_begin(MPU6050_GYRO_FS_250, MPU6050_ACCEL_FS_2)) {
        imu_log_settings();
        // t_bool_activity a;
        // imu_read_activities(&a);

        // printf("overflow, freefall, inactivity, activity, dataready\r\n");
        // printf("%d,%d,%d,%d,%d\r\n", a.is_overflow, a.is_freefall, a.is_inactivity, a.is_activity, a.is_data_ready);
        // printf("negX, posX, negY, posY, negZ, posZ\r\n");
        // printf("%d,%d,%d,%d,%d,%d\r\n", a.is_neg_activity_on_x, a.is_pos_activity_on_x, a.is_neg_activity_on_y, a.is_neg_activity_on_y, a.is_neg_activity_on_z, a.is_pos_activity_on_z);

        while(true) {


            imu_read_rotation(&gx, &gy, &gz);
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