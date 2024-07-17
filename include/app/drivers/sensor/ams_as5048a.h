/*
 * Copyright 2022 Google LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MECHADUINO_INCLUDE_APP_DRIVERS_SENSOR_AMS_AS5048A_H_
#define MECHADUINO_INCLUDE_APP_DRIVERS_SENSOR_AMS_AS5048A_H_

#include <zephyr/drivers/sensor.h>

enum sensor_channel_ams_as5048a {
    SENSOR_CHAN_RAW_ANGLE = SENSOR_CHAN_PRIV_START,
};

#endif /* MECHADUINO_INCLUDE_APP_DRIVERS_SENSOR_AMS_AS5048A_H_ */
