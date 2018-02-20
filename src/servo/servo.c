#include "imu.h"
#include "cph_millis.h"
#include "servo.h"


#define ANGLE_MIN -90
#define ANGLE_MAX 90
#define PWM_CLOCKSOURCE_FREQ 1000000
#define PWM_FREQ 50
#define PWM_PERIOD_TICKS PWM_CLOCKSOURCE_FREQ/PWM_FREQ

#define PWM_MIN 1000
#define PWM_MAX 2000
#define PWM_MID 1500

#define PULSE_WIDTH_FULL_LEFT_TICKS 1000
#define PULSE_WIDTH_CENTER_TICKS 1500
#define PULSE_WIDTH_FULL_RIGHT_TICKS 2000

/** PWM channel instance for Servos */
pwm_channel_t g_pwm_channel_servo;

clock_time_t f_servo_timeout = 0;

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void servo_init(void)
{
    
    printf("servo_init\r\n");
    pio_configure_pin(EXT1_PIN_PWM_0, PIO_TYPE_PIO_PERIPH_B);

	/* Enable PWM peripheral clock */

	pmc_enable_periph_clk(ID_PWM);


	/* Disable PWM channels for LEDs */

	pwm_channel_disable(PWM, EXT1_PWM_CHANNEL);

	/* Set PWM clock A as PWM_FREQUENCY*PERIOD_VALUE (clock B is not used) */
	pwm_clock_t clock_setting = {
        //.ul_clka = PWM_FREQUENCY * PERIOD_VALUE,
        .ul_clka = PWM_CLOCKSOURCE_FREQ,
		.ul_clkb = 0,
		.ul_mck = sysclk_get_cpu_hz()
	};

	pwm_init(PWM, &clock_setting);


	/* Initialize PWM channel for LED0 */
	/* Period is left-aligned */
	g_pwm_channel_servo.alignment = PWM_ALIGN_LEFT;
	/* Output waveform starts at a low level */
	// g_pwm_channel_servo.polarity = PWM_LOW;
	g_pwm_channel_servo.polarity = PWM_HIGH;
	/* Use PWM clock A as source clock */
	g_pwm_channel_servo.ul_prescaler = PWM_CMR_CPRE_CLKA;
	/* Period value of output waveform */
	g_pwm_channel_servo.ul_period = PWM_PERIOD_TICKS;
	/* Duty cycle value of output waveform */
	g_pwm_channel_servo.ul_duty = PULSE_WIDTH_CENTER_TICKS;
	g_pwm_channel_servo.channel = EXT1_PWM_CHANNEL;

	pwm_channel_init(PWM, &g_pwm_channel_servo);

    pwm_channel_enable(PWM, EXT1_PWM_CHANNEL);
	
}

void servo_tick(void)
{
	/* Stay within update range of servo */
	if (cph_get_millis() >= f_servo_timeout) {
		f_servo_timeout = cph_get_millis() + 50;
		// servo_set_angle(imu_complementary.y_axis);
		servo_set_angle(ap.imu.y_axis);
	}
}

void servo_set_angle(float angle)
{

	long x = (long) angle;

	long duty = map(x, ANGLE_MIN, ANGLE_MAX, PWM_MIN, PWM_MAX);

	// printf("servo_duty: %d\r\n", duty);

	pwm_channel_update_duty(PWM, &g_pwm_channel_servo, duty);
}