#include "imu.h"
#include "cph_millis.h"
#include "motor.h"

// pwm_channel_update_duty(motors[0].pwm_channel, &motors[0].pwm_motor_channel, motors[0].duty_cycle);

void motor_cfg_pwm(Pwm *p_pwm, \
                    pwm_channel_t *pwm_channel, \
                    pwm_align_t pwm_align, \
                    pwm_level_t pwm_level, \
                    uint32_t prescalar, \
                    uint32_t period, \
                    uint32_t duty, \
                    uint32_t channel, \
                    uint32_t pin, \
                    uint32_t peripheral, \
                    uint32_t clock_id);

void motor_init(void)
{
	motor_clock_setting.ul_clka = MOTOR_PWM_CLOCKSOURCE_FREQ;
	motor_clock_setting.ul_clkb = 0;
	motor_clock_setting.ul_mck = sysclk_get_cpu_hz();

	motor_cfg_pwm(PWM, &motors[0], PWM_ALIGN_LEFT, PWM_HIGH, PWM_CMR_CPRE_CLKA, MOTOR_PWM_PERIOD_TICKS, MOTOR_PWM_MIN, EXT1_PWM_CHANNEL, EXT1_PIN_PWM_0, PIO_TYPE_PIO_PERIPH_B, ID_PWM);
}

void motor_cfg_pwm(Pwm *p_pwm, pwm_channel_t *pwm_channel, pwm_align_t pwm_align, pwm_level_t pwm_level, uint32_t prescalar, uint32_t period, uint32_t duty, uint32_t channel, uint32_t pin, uint32_t peripheral, uint32_t clock_id)
{
	/* Initialize PWM channel for LED0 */
	/* Period is left-aligned */
	pwm_channel->alignment = pwm_align;
	/* Output waveform starts at a low level */
	// g_pwm_channel_servo.polarity = PWM_LOW;
	pwm_channel->polarity = pwm_level;
	/* Use PWM clock A as source clock */
	pwm_channel->ul_prescaler = prescalar;
	/* Period value of output waveform */
	pwm_channel->ul_period = period;
	/* Duty cycle value of output waveform */
	pwm_channel->ul_duty = duty;
	pwm_channel->channel = channel;

	pio_configure_pin(pin, peripheral);

	pmc_enable_periph_clk(clock_id);

	pwm_channel_disable(p_pwm, channel);

	pwm_init(p_pwm, &motor_clock_setting);
	pwm_channel_enable(p_pwm, channel);

}


void motor_set_power(t_motor motor, uint32_t power)
{
    motor.duty_cycle = power;
	printf("current_duty_cycle: %d\r\n", motor.duty_cycle);
	pwm_channel_update_duty(motor.pwm_channel, &motor.pwm_motor_channel, motor.duty_cycle);
}

void motor_min(t_motor motor)
{
    motor_set_power(motor, MOTOR_PWM_MIN);
}

void motor_mid(t_motor motor)
{
	motor_set_power(motor, MOTOR_PWM_MID);
}

void motor_max(t_motor motor)
{
    motor_set_power(motor, MOTOR_PWM_MAX);
}