/*
 * Copyright (c) 2024, Jim Mussared
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ams_as5048a

#include <errno.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>

#include <app/drivers/sensor/ams_as5048a.h>

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#warning "AS5048A driver enabled without any devices"
#endif

LOG_MODULE_REGISTER(as5048a, CONFIG_SENSOR_LOG_LEVEL);

struct as5048a_dev_cfg {
    struct spi_dt_spec bus;
};

struct as5048a_dev_data {
    uint16_t raw_angle;
};

static int as5048a_fetch(const struct device *dev, enum sensor_channel chan)
{
    struct as5048a_dev_data *const data = dev->data;
    const struct as5048a_dev_cfg *const cfg = dev->config;

    uint16_t tx_data = 0xffff;
    uint16_t rx_data = 0;

    const struct spi_buf tx_buf = {
        .buf = &tx_data,
        .len = sizeof(tx_data),
    };
    const struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count = 1,
    };
    const struct spi_buf rx_buf = {
        .buf = &rx_data,
        .len = sizeof(rx_data),
    };
    const struct spi_buf_set rx = {
        .buffers = &rx_buf,
        .count = 1,
    };
    int ret = spi_transceive_dt(&cfg->bus, &tx, &rx);
    if (ret) {
        LOG_DBG("spi_transceive_dt FAIL %d\n", ret);
        return ret;
    }

    data->raw_angle = sys_be16_to_cpu(rx_data) & 0x3fff;

    return 0;
}

static int as5048a_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val)
{
    const struct as5048a_dev_data *const data = dev->data;

    if ((enum sensor_channel_ams_as5048a)chan == SENSOR_CHAN_RAW_ANGLE) {
        val->val1 = data->raw_angle;
        val->val2 = 0;
    } else if (chan == SENSOR_CHAN_ROTATION) {
        val->val1 = (data->raw_angle * 45) / 2048; // (r * 360) / 16384
        val->val2 = ((data->raw_angle * 45 * 15625) / 32) % 1000000; // ((r * 360 * 1000000) / 16384) % 1000000
    } else {
        return -ENOTSUP;
    }

    return 0;
}

static int as5048a_init(const struct device *dev)
{
    struct as5048a_dev_data *const data = dev->data;
    data->raw_angle = 0;

    const struct as5048a_dev_cfg *const cfg = dev->config;

    if (!spi_is_ready_dt(&cfg->bus)) {
        LOG_ERR("SPI dev %s not ready", cfg->bus.bus->name);
        return -ENODEV;
    }

    LOG_INF("Device %s initialized", dev->name);

    return 0;
}

static const struct sensor_driver_api as5048a_driver_api = {
    .sample_fetch = as5048a_fetch,
    .channel_get = as5048a_get,
};

#define AS5048A_INIT(inst)                                    \
    static struct as5048a_dev_data as5048a_data##inst;        \
    static const struct as5048a_dev_cfg as5048a_cfg##inst = { \
        .bus = SPI_DT_SPEC_INST_GET(inst,                     \
                                    SPI_WORD_SET(8)          \
                                        | SPI_TRANSFER_MSB    \
                                        | SPI_MODE_CPHA,      \
                                    0)                        \
    };                                                        \
                                                              \
    SENSOR_DEVICE_DT_INST_DEFINE(inst, as5048a_init, NULL,    \
                &as5048a_data##inst, &as5048a_cfg##inst,      \
                POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,     \
                &as5048a_driver_api);

DT_INST_FOREACH_STATUS_OKAY(AS5048A_INIT)


                                        // | SPI_MODE_CPOL
                                        // | SPI_MODE_CPHA,
