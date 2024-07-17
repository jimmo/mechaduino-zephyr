#include <stdio.h>
#include <stdlib.h>

#include <soc.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/sys/util.h>
#include <zephyr/shell/shell.h>
#include "app/drivers/sensor/ams_as5048a.h"
#include "zephyr/drivers/sensor.h"

static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(DT_NODELABEL(led0), gpios);

static const struct pwm_dt_spec pwm_vref[] = {
	PWM_DT_SPEC_GET_BY_IDX(DT_NODELABEL(stepper), 0),
	PWM_DT_SPEC_GET_BY_IDX(DT_NODELABEL(stepper), 1),
};

static const struct gpio_dt_spec gpio_in[] = {
	GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(stepper), gpios, 0),
	GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(stepper), gpios, 1),
	GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(stepper), gpios, 2),
	GPIO_DT_SPEC_GET_BY_IDX(DT_NODELABEL(stepper), gpios, 3),
};

static gpio_port_pins_t gpio_in_mask = 0;

static const struct device *const dev = DEVICE_DT_GET_ANY(ams_as5048a);

static const int16_t sin_lookup[] = {
	// [round((2**15-1)*math.sin(2*math.pi*i/1024)) for i in range(0, 1024)]
	0, 201, 402, 603, 804, 1005, 1206, 1407, 1608, 1809, 2009, 2210, 2410, 2611, 2811, 3012, 3212, 3412, 3612, 3811, 4011, 4210, 4410, 4609, 4808, 5007, 5205, 5404, 5602, 5800, 5998, 6195, 6393, 6590, 6786, 6983, 7179, 7375, 7571, 7767, 7962, 8157, 8351, 8545, 8739, 8933, 9126, 9319, 9512, 9704, 9896, 10087, 10278, 10469, 10659, 10849, 11039, 11228, 11417, 11605, 11793, 11980, 12167, 12353, 12539, 12725, 12910, 13094, 13279, 13462, 13645, 13828, 14010, 14191, 14372, 14553, 14732, 14912, 15090, 15269, 15446, 15623, 15800, 15976, 16151, 16325, 16499, 16673, 16846, 17018, 17189, 17360, 17530, 17700, 17869, 18037, 18204, 18371, 18537, 18703, 18868, 19032, 19195, 19357, 19519, 19680, 19841, 20000, 20159, 20317, 20475, 20631, 20787, 20942, 21096, 21250, 21403, 21554, 21705, 21856, 22005, 22154, 22301, 22448, 22594, 22739, 22884, 23027, 23170, 23311, 23452, 23592, 23731, 23870, 24007, 24143, 24279, 24413, 24547, 24680, 24811, 24942, 25072, 25201, 25329, 25456, 25582, 25708, 25832, 25955, 26077, 26198, 26319, 26438, 26556, 26674, 26790, 26905, 27019, 27133, 27245, 27356, 27466, 27575, 27683, 27790, 27896, 28001, 28105, 28208, 28310, 28411, 28510, 28609, 28706, 28803, 28898, 28992, 29085, 29177, 29268, 29358, 29447, 29534, 29621, 29706, 29791, 29874, 29956, 30037, 30117, 30195, 30273, 30349, 30424, 30498, 30571, 30643, 30714, 30783, 30852, 30919, 30985, 31050, 31113, 31176, 31237, 31297, 31356, 31414, 31470, 31526, 31580, 31633, 31685, 31736, 31785, 31833, 31880, 31926, 31971, 32014, 32057, 32098, 32137, 32176, 32213, 32250, 32285, 32318, 32351, 32382, 32412, 32441, 32469, 32495, 32521, 32545, 32567, 32589, 32609, 32628, 32646, 32663, 32678, 32692, 32705, 32717, 32728, 32737, 32745, 32752, 32757, 32761, 32765, 32766, 32767, 32766, 32765, 32761, 32757, 32752, 32745, 32737, 32728, 32717, 32705, 32692, 32678, 32663, 32646, 32628, 32609, 32589, 32567, 32545, 32521, 32495, 32469, 32441, 32412, 32382, 32351, 32318, 32285, 32250, 32213, 32176, 32137, 32098, 32057, 32014, 31971, 31926, 31880, 31833, 31785, 31736, 31685, 31633, 31580, 31526, 31470, 31414, 31356, 31297, 31237, 31176, 31113, 31050, 30985, 30919, 30852, 30783, 30714, 30643, 30571, 30498, 30424, 30349, 30273, 30195, 30117, 30037, 29956, 29874, 29791, 29706, 29621, 29534, 29447, 29358, 29268, 29177, 29085, 28992, 28898, 28803, 28706, 28609, 28510, 28411, 28310, 28208, 28105, 28001, 27896, 27790, 27683, 27575, 27466, 27356, 27245, 27133, 27019, 26905, 26790, 26674, 26556, 26438, 26319, 26198, 26077, 25955, 25832, 25708, 25582, 25456, 25329, 25201, 25072, 24942, 24811, 24680, 24547, 24413, 24279, 24143, 24007, 23870, 23731, 23592, 23452, 23311, 23170, 23027, 22884, 22739, 22594, 22448, 22301, 22154, 22005, 21856, 21705, 21554, 21403, 21250, 21096, 20942, 20787, 20631, 20475, 20317, 20159, 20000, 19841, 19680, 19519, 19357, 19195, 19032, 18868, 18703, 18537, 18371, 18204, 18037, 17869, 17700, 17530, 17360, 17189, 17018, 16846, 16673, 16499, 16325, 16151, 15976, 15800, 15623, 15446, 15269, 15090, 14912, 14732, 14553, 14372, 14191, 14010, 13828, 13645, 13462, 13279, 13094, 12910, 12725, 12539, 12353, 12167, 11980, 11793, 11605, 11417, 11228, 11039, 10849, 10659, 10469, 10278, 10087, 9896, 9704, 9512, 9319, 9126, 8933, 8739, 8545, 8351, 8157, 7962, 7767, 7571, 7375, 7179, 6983, 6786, 6590, 6393, 6195, 5998, 5800, 5602, 5404, 5205, 5007, 4808, 4609, 4410, 4210, 4011, 3811, 3612, 3412, 3212, 3012, 2811, 2611, 2410, 2210, 2009, 1809, 1608, 1407, 1206, 1005, 804, 603, 402, 201, 0, -201, -402, -603, -804, -1005, -1206, -1407, -1608, -1809, -2009, -2210, -2410, -2611, -2811, -3012, -3212, -3412, -3612, -3811, -4011, -4210, -4410, -4609, -4808, -5007, -5205, -5404, -5602, -5800, -5998, -6195, -6393, -6590, -6786, -6983, -7179, -7375, -7571, -7767, -7962, -8157, -8351, -8545, -8739, -8933, -9126, -9319, -9512, -9704, -9896, -10087, -10278, -10469, -10659, -10849, -11039, -11228, -11417, -11605, -11793, -11980, -12167, -12353, -12539, -12725, -12910, -13094, -13279, -13462, -13645, -13828, -14010, -14191, -14372, -14553, -14732, -14912, -15090, -15269, -15446, -15623, -15800, -15976, -16151, -16325, -16499, -16673, -16846, -17018, -17189, -17360, -17530, -17700, -17869, -18037, -18204, -18371, -18537, -18703, -18868, -19032, -19195, -19357, -19519, -19680, -19841, -20000, -20159, -20317, -20475, -20631, -20787, -20942, -21096, -21250, -21403, -21554, -21705, -21856, -22005, -22154, -22301, -22448, -22594, -22739, -22884, -23027, -23170, -23311, -23452, -23592, -23731, -23870, -24007, -24143, -24279, -24413, -24547, -24680, -24811, -24942, -25072, -25201, -25329, -25456, -25582, -25708, -25832, -25955, -26077, -26198, -26319, -26438, -26556, -26674, -26790, -26905, -27019, -27133, -27245, -27356, -27466, -27575, -27683, -27790, -27896, -28001, -28105, -28208, -28310, -28411, -28510, -28609, -28706, -28803, -28898, -28992, -29085, -29177, -29268, -29358, -29447, -29534, -29621, -29706, -29791, -29874, -29956, -30037, -30117, -30195, -30273, -30349, -30424, -30498, -30571, -30643, -30714, -30783, -30852, -30919, -30985, -31050, -31113, -31176, -31237, -31297, -31356, -31414, -31470, -31526, -31580, -31633, -31685, -31736, -31785, -31833, -31880, -31926, -31971, -32014, -32057, -32098, -32137, -32176, -32213, -32250, -32285, -32318, -32351, -32382, -32412, -32441, -32469, -32495, -32521, -32545, -32567, -32589, -32609, -32628, -32646, -32663, -32678, -32692, -32705, -32717, -32728, -32737, -32745, -32752, -32757, -32761, -32765, -32766, -32767, -32766, -32765, -32761, -32757, -32752, -32745, -32737, -32728, -32717, -32705, -32692, -32678, -32663, -32646, -32628, -32609, -32589, -32567, -32545, -32521, -32495, -32469, -32441, -32412, -32382, -32351, -32318, -32285, -32250, -32213, -32176, -32137, -32098, -32057, -32014, -31971, -31926, -31880, -31833, -31785, -31736, -31685, -31633, -31580, -31526, -31470, -31414, -31356, -31297, -31237, -31176, -31113, -31050, -30985, -30919, -30852, -30783, -30714, -30643, -30571, -30498, -30424, -30349, -30273, -30195, -30117, -30037, -29956, -29874, -29791, -29706, -29621, -29534, -29447, -29358, -29268, -29177, -29085, -28992, -28898, -28803, -28706, -28609, -28510, -28411, -28310, -28208, -28105, -28001, -27896, -27790, -27683, -27575, -27466, -27356, -27245, -27133, -27019, -26905, -26790, -26674, -26556, -26438, -26319, -26198, -26077, -25955, -25832, -25708, -25582, -25456, -25329, -25201, -25072, -24942, -24811, -24680, -24547, -24413, -24279, -24143, -24007, -23870, -23731, -23592, -23452, -23311, -23170, -23027, -22884, -22739, -22594, -22448, -22301, -22154, -22005, -21856, -21705, -21554, -21403, -21250, -21096, -20942, -20787, -20631, -20475, -20317, -20159, -20000, -19841, -19680, -19519, -19357, -19195, -19032, -18868, -18703, -18537, -18371, -18204, -18037, -17869, -17700, -17530, -17360, -17189, -17018, -16846, -16673, -16499, -16325, -16151, -15976, -15800, -15623, -15446, -15269, -15090, -14912, -14732, -14553, -14372, -14191, -14010, -13828, -13645, -13462, -13279, -13094, -12910, -12725, -12539, -12353, -12167, -11980, -11793, -11605, -11417, -11228, -11039, -10849, -10659, -10469, -10278, -10087, -9896, -9704, -9512, -9319, -9126, -8933, -8739, -8545, -8351, -8157, -7962, -7767, -7571, -7375, -7179, -6983, -6786, -6590, -6393, -6195, -5998, -5800, -5602, -5404, -5205, -5007, -4808, -4609, -4410, -4210, -4011, -3811, -3612, -3412, -3212, -3012, -2811, -2611, -2410, -2210, -2009, -1809, -1608, -1407, -1206, -1005, -804, -603, -402, -201,
};

static void drive_coils(uint32_t micro, int32_t umax) {
	gpio_port_value_t value = 0;
	int32_t u12 = (umax * sin_lookup[(micro + 256) % 1024]) >> 15;
	if (u12 >= 0) {
		value |= (1 << gpio_in[0].pin);
	} else {
		value |= (1 << gpio_in[1].pin);
		u12 = -u12;
	}
	int32_t u34 = (umax * sin_lookup[micro]) >> 15;
	if (u34 >= 0) {
		value |= (1 << gpio_in[2].pin);
	} else {
		value |= (1 << gpio_in[3].pin);
		u34 = -u34;
	}

	// printf("%d: %08x %d %d (%d %d)\n", micro, value, u12, u34, PWM_NSEC(u12), PWM_NSEC(u34));

	pwm_set_pulse_dt(&pwm_vref[0], PWM_NSEC(u12));
	pwm_set_pulse_dt(&pwm_vref[1], PWM_NSEC(u34));
	gpio_port_set_masked_raw(gpio_in[0].port, gpio_in_mask, value);
}

static int32_t get_raw_angle(void) {
	struct sensor_value sensor_raw_angle;
	sensor_sample_fetch_chan(dev, (enum sensor_channel)SENSOR_CHAN_RAW_ANGLE);
	sensor_channel_get(dev, (enum sensor_channel)SENSOR_CHAN_RAW_ANGLE, &sensor_raw_angle);
	return sensor_raw_angle.val1;
}

static uint32_t raw_angle_to_microstep(int32_t raw_angle) {
	int32_t cal_angle = (raw_angle + 16384 - 287) % 16384;
	return cal_angle * (200 * 256) / 16384;
}

static int32_t microstep_to_raw_angle(uint32_t micro) {
	return ((micro * 16384 / (200 * 256)) + 287) % 16384;
}

enum stepper_mode {
	STEPPER_MODE_OFF,
	STEPPER_MODE_CALIBRATE,
	STEPPER_MODE_CALIBRATING,
	STEPPER_MODE_RUN,
	STEPPER_MODE_POSITION,
	STEPPER_MODE_CURVE,
};

struct stepper_state_t {
	enum stepper_mode mode;
	uint32_t micro;
	uint32_t step;
	uint32_t target;
	uint32_t n;
	int64_t start;
	bool repeat;
	double poly[5];
};

const uint32_t CALIBRATE_UMAX = 3000;
const uint32_t CALIBRATE_SLEEP_US = 15000;

const uint32_t RUN_UMAX = 4000;
const uint32_t RUN_SLEEP_US = 500;

const uint32_t POSITION_UMIN = 2000;
const uint32_t POSITION_UMAX = 6000;
const uint32_t POSITION_SLEEP_US = 250;

volatile struct stepper_state_t state = { .mode = STEPPER_MODE_OFF, .micro = 0, .step = 0, .n = 0, .target = 0, };

// volatile int32_t dstep = 256;
// volatile int32_t dt_us = 5000;
// volatile int32_t umax = 3000;
// volatile int32_t angle = 100;

static void position_hold(void) {
	uint32_t current = raw_angle_to_microstep(get_raw_angle());
	uint32_t err = (state.target + (200 * 256) - current) % (200 * 256);
	uint32_t u = POSITION_UMIN;
	if (err < 100 * 256) {
		u += (POSITION_UMAX - POSITION_UMIN) * MIN(50 * 256, err) / (50 * 256);
		err = MIN(256, err);
	} else {
		err = 200 * 256 - 1 - err;
		u += (POSITION_UMAX - POSITION_UMIN) * MIN(50 * 256, err) / (50 * 256);
		err = MIN(256, err);
		err = 1024 - err;
	}
	drive_coils((raw_angle_to_microstep(get_raw_angle()) + err) % 1024, u);
	k_usleep(POSITION_SLEEP_US);
}

int main(void)
{
	if (!gpio_is_ready_dt(&led0)) {
		return 0;
	}
	if (gpio_pin_configure_dt(&led0, GPIO_OUTPUT_ACTIVE) < 0) {
		return 0;
	}

	for (int i = 0; i < ARRAY_SIZE(pwm_vref); ++i) {
		if (!pwm_is_ready_dt(&pwm_vref[i])) {
			return 0;
		}
	}

	for (int i = 0; i < ARRAY_SIZE(gpio_in); ++i) {
		if (!gpio_is_ready_dt(&gpio_in[i]) || gpio_pin_configure_dt(&gpio_in[i], GPIO_OUTPUT_ACTIVE) < 0) {
			return 0;
		}
		gpio_in_mask |= (1 << gpio_in[i].pin);
	}

	if (dev == NULL) {
		/* No such node, or the node does not have status "okay". */
		printk("\nError: no device found.\n");
		return 0;
	}

	if (!device_is_ready(dev)) {
		printk("\nError: Device \"%s\" is not ready; "
		       "check the driver initialization logs for errors.\n",
		       dev->name);
		return 0;
	}

	printk("Found device \"%s\", getting sensor data\n", dev->name);

	/*pwm_set_pulse_dt(&pwm_vref[0], PWM_USEC(3));
	pwm_set_pulse_dt(&pwm_vref[1], PWM_USEC(3));
	uint32_t step = 0;

	while (1) {
		gpio_port_value_t value = 0;
		value |= (((step + 0) & 2) >> 1) << gpio_in[0].pin;
		value |= (((step + 2) & 2) >> 1) << gpio_in[1].pin;
		value |= (((step + 1) & 2) >> 1) << gpio_in[2].pin;
		value |= (((step + 3) & 2) >> 1) << gpio_in[3].pin;
		gpio_port_set_masked_raw(gpio_in[0].port, mask, value);
		k_msleep(SLEEP_TIME_MS);
		gpio_pin_toggle_dt(&led0);
		++step;
	}*/

	/*pwm_set_pulse_dt(&pwm_vref[0], PWM_USEC(3));
	pwm_set_pulse_dt(&pwm_vref[1], PWM_USEC(3));
	uint32_t step = 0;

	uint32_t duty[] = {PWM_USEC(0), PWM_USEC(0), PWM_USEC(0), PWM_USEC(0), PWM_USEC(0), PWM_USEC(0), PWM_USEC(1), PWM_USEC(2), PWM_USEC(3), PWM_USEC(3), PWM_USEC(2), PWM_USEC(1), PWM_USEC(0), PWM_USEC(0), PWM_USEC(0), PWM_USEC(0), };

	while (1) {
		gpio_port_value_t value = 0;
		uint32_t micro = step % 16;
		// 0:  00 00 00 12 33 21 00 00
		// 1:  33 21 00 00 00 00 00 12
		// 2:  00 12 33 21 00 00 00 00
		// 3:  00 00 00 00 00 12 33 21

		// 0: 0 1 2 3 3 2 1 0 0 1 2 3 3 2 1 0
		value |= (micro >= 6 && micro <= 11) << gpio_in[0].pin;
		value |= (micro <= 3 || micro >= 14) << gpio_in[1].pin;
		value |= (micro >= 2 && micro <= 7) << gpio_in[2].pin;
		value |= (micro >= 10 && micro <= 15) << gpio_in[3].pin;
		gpio_port_set_masked_raw(gpio_in[0].port, mask, value);

		pwm_set_pulse_dt(&pwm_vref[0], duty[(micro + 0) % 16]);
		pwm_set_pulse_dt(&pwm_vref[1], duty[(micro + 4) % 16]);

		k_msleep(SLEEP_TIME_MS);
		gpio_pin_toggle_dt(&led0);
		++step;
	}*/

	while (1) {
		switch (state.mode) {
		case STEPPER_MODE_OFF:
			drive_coils(0, 0);
			k_msleep(5);
			break;

		case STEPPER_MODE_CALIBRATE:
			state.micro = 0;
			state.step = 32;
			state.n = 200;
			state.mode = STEPPER_MODE_CALIBRATING;
			break;

		case STEPPER_MODE_CALIBRATING:
			drive_coils(state.micro, CALIBRATE_UMAX);
			k_usleep(CALIBRATE_SLEEP_US);

			if (state.micro % 256 == 0) {
				int32_t raw_angle = get_raw_angle();
				printf("%d %d (-> %d -> %d)\n", state.micro, raw_angle, raw_angle_to_microstep(raw_angle), microstep_to_raw_angle(raw_angle_to_microstep(raw_angle)));

				--state.n;
				if (state.n == 0) {
					state.mode = STEPPER_MODE_OFF;
					break;
				}
			}

			state.micro = (state.micro + state.step) % 1024;
			break;

		case STEPPER_MODE_RUN:
			drive_coils((raw_angle_to_microstep(get_raw_angle()) + 256) % 1024, RUN_UMAX);
			k_usleep(RUN_SLEEP_US);
			break;

		case STEPPER_MODE_POSITION:
			position_hold();
			break;

		case STEPPER_MODE_CURVE:
			int64_t s = state.start;
			double t = k_uptime_delta(&s);
			if (t >= state.n) {
				if (state.repeat) {
					state.start += state.n;
				} else {
					state.mode = STEPPER_MODE_POSITION;
				}
			}
			state.target = (int32_t)(state.poly[4] * t * t * t * t + state.poly[3] * t * t * t + state.poly[2] * t * t + state.poly[1] * t + state.poly[0]) % (256 * 200);
			position_hold();
			break;
		}
	}

	return 0;
}

// stops at
// 00300000 3000 19 (3000 19)
// 00208000 3000 0 (3000 0)
// 00208000 3000 18 (3000 18)

// 00208000 19 2999 (19 2999)
// 00008040 0 2999 (0 2999)
// 00008040 18 2999 (18 2999)

// starts at
// 00300000 871 2871

#define DBL_TAP_MAGIC_LOADER 0xf01669ef

static int cmd_bootloader(const struct shell *sh, size_t argc, char **argv) {
	// Functionally the same as bossa_reset()
	uint32_t *top = (uint32_t *)(DT_REG_ADDR(DT_NODELABEL(sram0)) + DT_REG_SIZE(DT_NODELABEL(sram0)));
	top[-1] = DBL_TAP_MAGIC_LOADER;
	NVIC_SystemReset();
}

SHELL_CMD_REGISTER(bootloader, NULL, "Jump to bootloader", cmd_bootloader);

// static int cmd_stepper_dstep(const struct shell *sh, size_t argc, char **argv) {
// 	if (argc == 2) {
// 		dstep = atoi(argv[1]);
// 	}
// 	shell_print(sh, "dstep = %d", dstep);
//     return 0;
// }

// static int cmd_stepper_dt_us(const struct shell *sh, size_t argc, char **argv) {
// 	if (argc == 2) {
// 		dt_us = atoi(argv[1]);
// 	}
// 	shell_print(sh, "dt_us = %d", dt_us);
//     return 0;
// }

// static int cmd_stepper_umax(const struct shell *sh, size_t argc, char **argv) {
// 	if (argc == 2) {
// 		umax = atoi(argv[1]);
// 	}
// 	shell_print(sh, "umax = %d", umax);
//     return 0;
// }

// static int cmd_stepper_angle(const struct shell *sh, size_t argc, char **argv) {
// 	if (argc == 2) {
// 		angle = atoi(argv[1]);
// 	}
// 	shell_print(sh, "angle = %d", umax);
//     return 0;
// }

static int cmd_stepper_off(const struct shell *sh, size_t argc, char **argv) {
	state.mode = STEPPER_MODE_OFF;
	return 0;
}

static int cmd_stepper_calibrate(const struct shell *sh, size_t argc, char **argv) {
	state.mode = STEPPER_MODE_CALIBRATE;
	return 0;
}

static int cmd_stepper_run(const struct shell *sh, size_t argc, char **argv) {
	state.mode = STEPPER_MODE_RUN;
	return 0;
}

static int cmd_stepper_position(const struct shell *sh, size_t argc, char **argv) {
	if (argc == 2) {
		state.target = atoi(argv[1]);
		state.mode = STEPPER_MODE_POSITION;
		return 0;
	} else {
		return 1;
	}
}

static int cmd_stepper_curve(const struct shell *sh, size_t argc, char **argv) {
	if (argc == 8) {
		for (int i = 0; i < 5; ++i) {
			state.poly[i] = atof(argv[i + 1]);
		}
		state.n = atoi(argv[6]);
		state.repeat = !!atoi(argv[7]);
		state.start = k_uptime_get();
		state.mode = STEPPER_MODE_CURVE;
		return 0;
	} else {
		return 1;
	}
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_stepper,
        // SHELL_CMD(dstep, NULL, "Set dstep.", cmd_stepper_dstep),
        // SHELL_CMD(dt_us, NULL, "Set dt_us.", cmd_stepper_dt_us),
        // SHELL_CMD(umax, NULL, "Set umax.", cmd_stepper_umax),
        // SHELL_CMD(angle, NULL, "Set angle.", cmd_stepper_angle),
        SHELL_CMD(off, NULL, "Turn off driver.", cmd_stepper_off),
        SHELL_CMD(calibrate, NULL, "Calibrate angle sensor.", cmd_stepper_calibrate),
        SHELL_CMD(run, NULL, "Run.", cmd_stepper_run),
        SHELL_CMD(position, NULL, "Position hold.", cmd_stepper_position),
        SHELL_CMD(curve, NULL, "Curve follow.", cmd_stepper_curve),
        SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(stepper, &sub_stepper, "Stepper commands", NULL);
