#include <asf.h>


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
#define IMU_TWI  TWI0

#include <stdio.h>
#include <string.h>

void test_twi(void);
void twi_probe_imu(void);

int main(void)
{
    sysclk_init();
    board_init();
    delay_init();
    pmc_enable_periph_clk(IMU_TWI_ID);


    twi_probe_imu();

    while(1) {

        if(ioport_get_pin_level(BUTTON_0_PIN) == BUTTON_0_ACTIVE) {
            ioport_toggle_pin_level(LED0_GPIO);
            delay_ms(500); 
        }

    }
}


void twi_probe_imu(void)
{

    // uint8_t imu_buffer[8];
    // memset(imu_buffer, 0, sizeof(imu_buffer));

    // uint32_t i;
    twi_options_t opt;
    twi_packet_t packet_tx;
    twi_packet_t packet_rx;

	/* Configure the options of TWI driver */
	opt.master_clk = sysclk_get_peripheral_hz();
    opt.speed      = TWI_CLK;

    if (twi_master_init(IMU_TWI, &opt) != TWI_SUCCESS) {
        
        while(true) {
            delay_ms(25);
            ioport_toggle_pin_level(LED0_GPIO);
        }
    }

    // delay_ms(20);

    if (twi_probe(IMU_TWI, IMU_ADDRESS) != TWI_SUCCESS) {
        ioport_toggle_pin_level(LED0_GPIO);
        while(true) {
            delay_ms(50);
            ioport_toggle_pin_level(LED0_GPIO);
        }
    }

    // TODO: build out reading registers below
    uint8_t data_received = 0;

    packet_rx.chip = IMU_ADDRESS;
    packet_rx.addr[0] = MPU6050_WHO_AM_I_BIT;
    packet_rx.addr_length = sizeof(uint8_t);
    packet_rx.buffer = &data_received;
    packet_rx.length = sizeof(uint8_t);

    if (twi_master_read(IMU_TWI, &packet_rx) != TWI_SUCCESS) {

        while(true) {
            delay_ms(25);
            ioport_toggle_pin_level(LED0_GPIO);
        }
    }

    // }
    // if(packet_rx.buffer == 0x34) {
    //     ioport_toggle_pin_level(LED0_GPIO);
    //     while(true) {
    //         delay_ms(25);
    //         ioport_toggle_pin_level(LED0_GPIO);
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