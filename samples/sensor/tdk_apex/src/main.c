/*
 * Copyright (c) 2024 TDK Invensense
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/tdk_apex.h>
#include <stdio.h>

static struct sensor_trigger data_trigger;
static struct sensor_trigger data_rd_trigger;


/* Flag set from IMU device irq handler */
static volatile int irq_from_device;

static volatile int motion_trig;
static volatile int data_trig;

/*
 * Get a device structure from a devicetree node from alias
 * "tdk_apex_sensor0".
 */
static const struct device *get_tdk_apex_device(void)
{
	const struct device *const dev = DEVICE_DT_GET(DT_ALIAS(tdk_apex_sensor0));

	if (!device_is_ready(dev)) {
		printk("\nError: Device \"%s\" is not ready; "
		       "check the driver initialization logs for errors.\n",
		       dev->name);
		return NULL;
	}

	printk("Found device \"%s\", getting sensor data\n", dev->name);
	return dev;
}

static const char *now_str(void)
{
	static char buf[16]; /* ...HH:MM:SS.MMM */
	uint32_t now = k_uptime_get_32();
	unsigned int ms = now % MSEC_PER_SEC;
	unsigned int s;
	unsigned int min;
	unsigned int h;

	now /= MSEC_PER_SEC;
	s = now % 60U;
	now /= 60U;
	min = now % 60U;
	now /= 60U;
	h = now;

	snprintf(buf, sizeof(buf), "%u:%02u:%02u.%03u", h, min, s, ms);
	return buf;
}

static void handle_tdk_apex_drdy(const struct device *dev, const struct sensor_trigger *trig)
{
	if (trig->type == SENSOR_TRIG_MOTION) {
		int rc = sensor_sample_fetch_chan(dev, trig->chan);

		if (rc < 0) {
			printf("sample fetch failed: %d\n", rc);
			printf("cancelling trigger due to failure: %d\n", rc);
			(void)sensor_trigger_set(dev, trig, NULL);
			return;
		} else if (rc == 0) {
			irq_from_device = 1;
			motion_trig = 1;
		}
	}
}

static void handle_6dof_motion_drdy(const struct device *dev, const struct sensor_trigger *trig)
{
        if (trig->type == SENSOR_TRIG_DATA_READY) {
                int rc = sensor_sample_fetch_chan(dev, trig->chan);

                if (rc < 0) {
                        printf("sample fetch failed: %d\n", rc);
                        printf("cancelling trigger due to failure: %d\n", rc);
                        (void)sensor_trigger_set(dev, trig, NULL);
                        return;
                } else if (rc == 0) {
                        irq_from_device = 1;
			data_trig = 1;
                }
        }
}


int main(void)
{
	const struct device *dev = get_tdk_apex_device();
	struct sensor_value apex_mode;
	struct sensor_value accel[3];
	struct sensor_value gyro[3];
	struct sensor_value temperature;
	struct sensor_value accel_odr, gyro_odr;


	if (dev == NULL) {
		return 0;
	}

	sensor_attr_get(dev, SENSOR_CHAN_APEX_MOTION, SENSOR_ATTR_CONFIGURATION, &apex_mode);
	if (apex_mode.val1 == TDK_APEX_PEDOMETER) {
		printf("Pedometer data sample.\n");
	} else if (apex_mode.val1 == TDK_APEX_TILT) {
		printf("Tilt data sample.\n");
	} else if (apex_mode.val1 == TDK_APEX_WOM) {
		printf("WOM data sample.\n");
	} else if (apex_mode.val1 == TDK_APEX_SMD) {
		printf("SMD data sample.\n");
	} else if (apex_mode.val1 == TDK_APEX_TAP) {
		printf("TAP data sample.\n");
	}
	apex_mode.val2 = 0;
	sensor_attr_set(dev, SENSOR_CHAN_APEX_MOTION, SENSOR_ATTR_CONFIGURATION, &apex_mode);

	data_trigger = (struct sensor_trigger){
		.type = SENSOR_TRIG_MOTION,
		.chan = SENSOR_CHAN_APEX_MOTION,
	};
	if (sensor_trigger_set(dev, &data_trigger, handle_tdk_apex_drdy) < 0) {
		printf("Cannot configure data trigger!!!\n");
		return 0;
	}

	printf("Configured for APEX data collecting.\n");

	
	data_rd_trigger = (struct sensor_trigger){
		.type = SENSOR_TRIG_DATA_READY,
			.chan = SENSOR_CHAN_ALL,
	};

	if (sensor_trigger_set(dev, &data_rd_trigger, handle_6dof_motion_drdy) < 0) {
		printf("Cannot configure data trigger!!!\n");
		return 0;
	}

	accel_odr.val1 = 12;
        accel_odr.val2 = 0;
        gyro_odr.val1 = 12;
        gyro_odr.val2 = 0;

        sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &accel_odr);
        sensor_attr_set(dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &gyro_odr);


	k_sleep(K_MSEC(1000));

	while (1) {
		if (irq_from_device) {
			if (motion_trig){
				printf("Motion Triggered\n");
				if (apex_mode.val1 == TDK_APEX_PEDOMETER) {
					struct sensor_value apex_pedometer[3];

					sensor_channel_get(dev, SENSOR_CHAN_APEX_MOTION, apex_pedometer);

					printf("[%s]: STEP_DET     count: %d steps  cadence: %.1f steps/s  "
							"activity: %s\n",
							now_str(), apex_pedometer[0].val1,
							sensor_value_to_double(&apex_pedometer[2]),
							apex_pedometer[1].val1 == 1   ? "Walk"
							: apex_pedometer[1].val1 == 2 ? "Run"
							: "Unknown");
				} else if (apex_mode.val1 == TDK_APEX_TILT) {
					struct sensor_value apex_tilt;

					sensor_channel_get(dev, SENSOR_CHAN_APEX_MOTION, &apex_tilt);

					printf("[%s]: %s\n", now_str(),
							apex_tilt.val1 ? "TILT" : "Unknown trig");

				} else if (apex_mode.val1 == TDK_APEX_WOM) {
					struct sensor_value apex_wom[3];

					sensor_channel_get(dev, SENSOR_CHAN_APEX_MOTION, apex_wom);

					printf("[%s]: WOM x=%d y=%d z=%d\n", now_str(), apex_wom[0].val1,
							apex_wom[1].val1, apex_wom[2].val1);

				} else if (apex_mode.val1 == TDK_APEX_SMD) {
					struct sensor_value apex_smd;

					sensor_channel_get(dev, SENSOR_CHAN_APEX_MOTION, &apex_smd);

					printf("[%s]: %s\n", now_str(),
							apex_smd.val1 ? "SMD" : "Unknown trig");

				} else if (apex_mode.val1 == TDK_APEX_TAP) {
					struct sensor_value apex_tap;

					sensor_channel_get(dev, SENSOR_CHAN_APEX_MOTION, &apex_tap);

					printf("[%s]: %s\n", now_str(),
							apex_tap.val1 ? "TAP" : "Unknown trig");

				} 
				motion_trig = 0;
			}
			if (data_trig) {
				sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);
                        sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);
                        sensor_channel_get(dev, SENSOR_CHAN_DIE_TEMP, &temperature);
/*
                        printf("[%s]: temp %.2f Cel "
                               "  accel %f %f %f m/s/s "
                               "  gyro  %f %f %f rad/s\n",
                               now_str(), sensor_value_to_double(&temperature),
                               sensor_value_to_double(&accel[0]), sensor_value_to_double(&accel[1]),
                               sensor_value_to_double(&accel[2]), sensor_value_to_double(&gyro[0]),
                               sensor_value_to_double(&gyro[1]), sensor_value_to_double(&gyro[2]));
				data_trig = 0;
*/
			}


			irq_from_device = 0;
		}
	}
	return 0;
}
