#include <asf.h>

#include <stdio.h>
#include <string.h>
#include "config.h"
#include "cph_millis.h"
#include "cph_util.h"
#include "cph_console.h"
#include "cph_cli.h"
#include "imu.h"
#include "servo.h"
#include "motor.h"
#include "pid.h"


clock_time_t f_log_timeout = 0;

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
    pmc_enable_periph_clk(ID_PWM);
    config_init();
    cph_millis_init();
    cli_init();
    configure_console();

    puts("\r\n\r\nsam4d32c imu demo...\r\n");

    for (int i=0; i<10; i++) {
        printf(".");
        delay_ms(100);
    }
    printf("\r\n");

    
    if (imu_init()) {

        motor_init();

        // while (true) {
        //     cli_tick();
        //     delay_ms(100);
        // }

        // Calibrate the imu
        printf("calibrating imu...\r\n");
        imu_calibrate();
        printf("calibration complete.\r\n");

        while(true) {
            if(ioport_get_pin_level(BUTTON_0_PIN) == BUTTON_0_ACTIVE) {
                ioport_toggle_pin_level(LED0_GPIO);
                break;
                delay_ms(100); 
            }
        }

        while(true) {
            cli_tick();
            imu_tick();
            // pid_tick();
            motor_tick();

            // long y = (long) ap.imu.x_axis;

            long y = (long) pid_tick();

            long power_left = map(y, ANGLE_MID, ANGLE_MAX, PWM_MIN, PWM_MAX);
            motor_set_power(motors[1], power_left + 200);

            long power_right = map(y, ANGLE_MID, ANGLE_MIN, PWM_MIN, PWM_MAX);
            motor_set_power(motors[0], power_right + 200);


            if (config.log_imu) {
                if (cph_get_millis() >= f_log_timeout) {
                    f_log_timeout = cph_get_millis() + 50;
                    printf("roll/pitch/yaw error/pid: %f %f %f %f %f\r\n", ap.imu.x_axis, ap.imu.y_axis, ap.imu.z_axis, error, pid);
                }
            }
            
        }

    }


    
    // while(1) {

    //     if(ioport_get_pin_level(BUTTON_0_PIN) == BUTTON_0_ACTIVE) {
    //         ioport_toggle_pin_level(LED0_GPIO);
    //         delay_ms(500); 
    //     }


    // }
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