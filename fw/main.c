/* Testing/experimental code */

#include "ch.h"
#include "hal.h"
#include "chprintf.h"

#define PWM_MAX 64

struct adc_meas_res {
	int16_t solar_u_mv;
	int16_t solar_i_ma;

	int16_t vbat_u_mv;
	int16_t vbat_i_ma;

	int16_t vout_u_mv;
	int16_t vout_i_ma;
};


SerialConfig console_uart_config = {
	115200,
	0,
	USART_CR2_STOP1_BITS | USART_CR2_LINEN,
	0
};

int pwm_enable(void) {

	TIM_TypeDef *tim = (TIM_TypeDef *)STM32_TIM3;
	rccEnableTIM3(FALSE);
	rccResetTIM3();
	tim->CCR4 = 0;
	tim->ARR  = PWM_MAX;
	tim->CNT  = 0;
	tim->SR   = 0;
	tim->PSC  = 0;

	tim->CCMR1 = 0;
	tim->CCMR2 = TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4PE;

	tim->CCER = TIM_CCER_CC4E;

	tim->CR2 = 0;
	tim->CR1 = STM32_TIM_CR1_ARPE | STM32_TIM_CR1_CEN;

}

int pwm_set(uint8_t pwm) {
	TIM_TypeDef *tim = (TIM_TypeDef *)STM32_TIM3;
	tim->CCR4 = pwm;
}

static adcsample_t adc_val[7];


static void adcerrorcallback(ADCDriver *adcp, adcerror_t err) {
	(void)adcp;
	(void)err;
}

static const ADCConversionGroup adc_group1 = {
	FALSE,
	7,
	NULL,
	adcerrorcallback,
	ADC_CFGR1_CONT | ADC_CFGR1_RES_12BIT,             /* CFGR1 */
	ADC_TR(0, 0),                                     /* TR */
	ADC_SMPR_SMP_55P5,                                 /* SMPR */
	ADC_CHSELR_CHSEL0 | ADC_CHSELR_CHSEL1 | ADC_CHSELR_CHSEL2 |
	ADC_CHSELR_CHSEL3 | ADC_CHSELR_CHSEL4 | ADC_CHSELR_CHSEL5 |
	ADC_CHSELR_CHSEL17 /* CHSELR */
};


int32_t adc_meas(struct adc_meas_res *res) {
	adcConvert(&ADCD1, &adc_group1, adc_val, 1);
	chThdSleepMilliseconds(1);

	/* read Vref reading at 3.3V Vdda (calibration values, see the datasheet)
	 * and compute its voltage */
	uint16_t vref_cal = *(uint16_t *)0x1ffff7ba;
	int32_t vref = vref_cal * 3300 / 4096;

	/* compute Vdda using computed value of the internal Vref */
	int32_t vdda = vref * 4096 / adc_val[6];

	/* all voltages are sensed using 1:1 resistor divider */
	res->solar_u_mv = (adc_val[0] * vdda * 2 / 4096);
	res->vbat_u_mv = (adc_val[2] * vdda * 2 / 4096);
	res->vout_u_mv = (adc_val[4] * vdda * 2 / 4096);

	/* solar current is sensed through 0R2 resistor and amplified 10x,
	 * hence its range is 0-500mA giving adc readings 0-1V + vdda/2 */
	res->solar_i_ma = ((adc_val[1] - 2069) * vdda / 4096) / 2;

	/* vbat current, 1R, 10x amplified, 0-100mA, 0-1V + vdda/2 */
	res->vbat_i_ma = ((adc_val[3] - 2040) * vdda / 4096) / 5;

	/* vbat current, 1R, 10x amplified, 0-100mA, 0-1V + vdda/2 */
	res->vout_i_ma = ((adc_val[5] - 2048) * vdda / 4096) / 10;

	return 0;
}


int32_t adc_print_res(struct adc_meas_res *res) {
	chprintf(&SD1,
		"solar_u=%dmV solar_i=%dmA solar_p=%dmW vbat_u=%dmV vbat_i=%dmA vout_u=%dmV vout_i=%dmA\r\n",
		res->solar_u_mv,
		res->solar_i_ma,
		(res->solar_u_mv * res->solar_i_ma / 1000),
		res->vbat_u_mv,
		res->vbat_i_ma,
		res->vout_u_mv,
		res->vout_i_ma
	);

	return 0;
}


int32_t meas_available_power(int16_t *max_power, uint8_t *max_power_pwm) {
	struct adc_meas_res a;

	pwm_set(1);
	chThdSleepMilliseconds(100);

	*max_power = 0;
	*max_power_pwm = 0;

	for (uint8_t i = 0; i < (PWM_MAX - (PWM_MAX / 10)); i++) {
		pwm_set(i);
		chThdSleepMilliseconds(1);
		adc_meas(&a);
		int16_t curr_power = (a.solar_i_ma * a.solar_u_mv / 1000);

		if (curr_power > *max_power) {
			*max_power = curr_power;
			*max_power_pwm = i;
		}
	}
	pwm_set(0);

	return 0;
}


int32_t perturb_observe(int16_t *max_power, uint8_t *max_power_pwm) {
	struct adc_meas_res a;

	uint8_t pwm_a = 0;
	uint8_t pwm_b = 0;

	if (*max_power_pwm >= 2) {
		pwm_a = *max_power_pwm - 2;
	} else {
		pwm_a = 0;
	}

	if (*max_power_pwm <= (PWM_MAX - (PWM_MAX / 10) - 2)) {
		pwm_b = *max_power_pwm + 2;
	} else {
		pwm_b = (PWM_MAX - (PWM_MAX / 10));
	}

	//~ *max_power = 0;
	//~ (*max_power)--;

	for (uint8_t i = pwm_a; i <= pwm_b; i++) {
		//~ chprintf(&SD1, "try pwm %d\r\n", i);
		pwm_set(i);
		chThdSleepMilliseconds(1);
		adc_meas(&a);
		//~ adc_print_res(&a);
		int16_t curr_power = (a.solar_i_ma * a.solar_u_mv / 1000);

		if (curr_power > *max_power) {
			*max_power = curr_power;
			*max_power_pwm = i;
		}
	}

	return 0;
}


static WORKING_AREA(waThread1, 512);
static msg_t Thread1(void *arg) {
	(void)arg;
	chRegSetThreadName("blinker");

	int16_t max_power = 0;
	uint8_t max_power_pwm = 0;
	uint16_t cnt = 0;

	meas_available_power(&max_power, &max_power_pwm);
	pwm_set(max_power_pwm);

	while (1) {
		perturb_observe(&max_power, &max_power_pwm);
		pwm_set(max_power_pwm);
		chprintf(&SD1, "changing pwm to %d for max power %dmW\r\n", max_power_pwm, max_power);

		if ((cnt % 100) == 0) {
			meas_available_power(&max_power, &max_power_pwm);
			pwm_set(max_power_pwm);
		}

		chThdSleepMilliseconds(100);
		cnt++;
	}
}


int main(void) {
	chSysInit();
	halInit();

	/* boost converter gate drive */
	palSetPadMode(GPIOB, 1, PAL_MODE_ALTERNATE(1));

	/* analog inputs for voltage & current measurement */
	palSetPadMode(GPIOA, 0, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 1, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 2, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 3, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 4, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 5, PAL_MODE_INPUT_ANALOG);
	adcStart(&ADCD1, NULL);
	adcSTM32SetCCR(ADC_CCR_VREFEN);

	/* usart1 TX */
	palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(1));
	sdStart(&SD1, &console_uart_config);

	/* vddo enable */
	palSetPadMode(GPIOA, 7, PAL_MODE_OUTPUT_PUSHPULL);
	palSetPad(GPIOA, 7);

	chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

	pwm_enable();
	pwm_set(0);

	while (1) {
		chThdSleepMilliseconds(200);
	}
}
