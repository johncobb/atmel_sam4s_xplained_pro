#include "imu.h"
#include "cph_millis.h"
#include "motor.h"

uint32_t motor_current_duty = 0;
void motor_config_pins(t_motor_config motor_cfg);


void motor_init(void)
{
	printf("motor_init\r\n");

	pwm_clock_t clock_setting = {
        //.ul_clka = PWM_FREQUENCY * PERIOD_VALUE,
        .ul_clka = MOTOR_PWM_CLOCKSOURCE_FREQ,
		.ul_clkb = 0,
		.ul_mck = sysclk_get_cpu_hz()
	};

	t_motor_config motor_cfg = {
		.p_pwm = PWM,
		.ul_pin = EXT1_PIN_PWM_0,
		.ul_flag = PIO_TYPE_PIO_PERIPH_B,
		.ul_periph_clkid = ID_PWM,
		.ul_channel = EXT1_PWM_CHANNEL,
		.clock_setting = clock_setting,
		.pwm_channel.alignment = PWM_ALIGN_LEFT,
		.pwm_channel.polarity = PWM_HIGH,
		.pwm_channel.ul_prescaler = PWM_CMR_CPRE_CLKA,
		.pwm_channel.ul_period = MOTOR_PWM_PERIOD_TICKS,
		.pwm_channel.ul_duty = MOTOR_PWM_MIN,
		.pwm_channel.channel = EXT1_PWM_CHANNEL
	};

	motor_config_pins(motor_cfg);

	motors[0].config = motor_cfg;
	motors[0].angle_min = AP_ANGLE_MIN;
	motors[0].angle_max = AP_ANGLE_MAX;
	motors[0].timeout = 1000;
}

void motor_config_pins(t_motor_config motor_cfg)
{
	pio_configure_pin(motor_cfg.ul_pin, motor_cfg.ul_flag);

	pmc_enable_periph_clk(motor_cfg.ul_periph_clkid);

	pwm_channel_disable(motor_cfg.p_pwm, motor_cfg.ul_channel);
	pwm_init(motor_cfg.p_pwm, &motor_cfg.clock_setting);

	pwm_channel_init(motor_cfg.p_pwm, &motor_cfg.pwm_channel.channel);
	pwm_channel_enable(motor_cfg.p_pwm, motor_cfg.ul_channel);
}

void motor_tick(void)
{

}

void motor_set_power(t_motor motor, uint32_t power)
{
	motor.config.pwm_channel.ul_duty = power;
	printf("motor_output: %d\r\n", motor.config.pwm_channel.ul_duty);
	pwm_channel_update_duty(motor.config.p_pwm, &motor.config.pwm_channel, motor.config.pwm_channel.ul_duty);
}

void motor_min(t_motor motor)
{
    motor_current_duty = MOTOR_PWM_MIN;
    motor_set_power(motor, MOTOR_PWM_MIN);
}

void motor_mid(t_motor motor)
{
    motor_current_duty = MOTOR_PWM_MID;
	motor_set_power(motor, motor_current_duty);
}

void motor_max(t_motor motor)
{
    motor_current_duty = MOTOR_PWM_MAX;
    motor_set_power(motor, motor_current_duty);
}

void motor_increment(t_motor motor)
{
    if (motor_current_duty == MOTOR_PWM_MAX)
        return;
	motor_current_duty += MOTOR_PWM_STEP;
	motor_set_power(motor, motor_current_duty);
}

void motor_decrement(t_motor motor)
{
    if (motor_current_duty == MOTOR_PWM_MIN)
        return;
	motor_current_duty -= MOTOR_PWM_STEP;
	motor_set_power(motor, motor_current_duty);
}