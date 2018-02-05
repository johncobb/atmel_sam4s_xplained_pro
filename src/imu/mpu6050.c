#include <asf.h>

#include <stdio.h>
#include <string.h>
#include "mpu6050.h"

static uint8_t imu_buffer[32] = {0};

void imu_init(void)
{
    // uint32_t i;
    twi_options_t opt;
    twi_packet_t packet_tx;
    twi_packet_t packet_rx;

	/* Configure the options of TWI driver */
	opt.master_clk = sysclk_get_peripheral_hz();
    opt.speed      = TWI_CLK;

    if (twi_master_init(IMU_TWI, &opt) != TWI_SUCCESS) {
        puts("twi_master_init: failed\r\n");
        
    } else {
        puts("twi_master_init: success\r\n");
    }
}

void imu_probe(void)
{
    // delay_ms(20);

    if (twi_probe(IMU_TWI, IMU_ADDRESS) != TWI_SUCCESS) {
        puts("twi_probe: failed\r\n");
        
    } else {
        puts("twi_probe: success\r\n");
    }
}


void imu_who_am_i(void)
{
        // uint32_t i;
    twi_options_t opt;
    twi_packet_t packet_tx;
    twi_packet_t packet_rx;

	/* Configure the options of TWI driver */
	opt.master_clk = sysclk_get_peripheral_hz();
    opt.speed      = TWI_CLK;

     memset(imu_buffer, 0, sizeof(imu_buffer));

    packet_rx.chip = IMU_ADDRESS;
    packet_rx.addr[0] = MPU6050_RA_WHO_AM_I;
    packet_rx.addr_length = sizeof(uint8_t);
    packet_rx.buffer = &imu_buffer;
    packet_rx.length = 1;


    uint32_t status = twi_master_read(IMU_TWI, &packet_rx);

    printf("twi_master_read status: %d\r\n", status);
    
    if (status == TWI_SUCCESS) {

        uint8_t device_id = imu_buffer[0];
        printf("twi_master_read buffer: 0x%1x\r\n", imu_buffer[0]);

        if (device_id == 0x68) {
            puts("who_am_i: success\r\n");

        }

    }
}

void imu_set_clock_source(uint8_t source)
{
        // uint32_t i;
    twi_options_t opt;
    twi_packet_t packet_tx;
    twi_packet_t packet_rx;

	/* Configure the options of TWI driver */
	opt.master_clk = sysclk_get_peripheral_hz();
    opt.speed      = TWI_CLK;

     memset(imu_buffer, 0, sizeof(imu_buffer));

    packet_tx.chip = IMU_ADDRESS;
    packet_tx.addr[0] = MPU6050_RA_PWR_MGMT_1;
    packet_tx.addr_length = sizeof(uint8_t);
    packet_tx.buffer = &source;
    packet_tx.length = 1;


    uint32_t status = twi_master_write(IMU_TWI, &packet_tx);

    printf("twi_master_write status: %d\r\n", status);
    
    if (status == TWI_SUCCESS) {

        puts("imu_set_clock_source: success\r\n");
    }

}