/*
 * Copyright (c) 2021 Technosystem sp. z o.o.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT max6642

#include <device.h>
#include <devicetree.h>
#include <drivers/i2c.h>
#include <drivers/sensor.h>
#include <logging/log.h>

LOG_MODULE_REGISTER(MAX6642, CONFIG_SENSOR_LOG_LEVEL);


#define MAX6642_REG_TEMP   0x00

struct max6642_data {
	int8_t local_temp;
	int8_t remote_temp;
};

struct max6642_config {
	const struct device *i2c_dev;
	uint8_t i2c_addr;
};

static inline int max6642_reg_read(struct max6642_config *cfg, uint8_t reg,
				uint8_t *buf, uint32_t size)
{
	return i2c_burst_read(cfg->i2c_dev, cfg->i2c_addr, reg, buf, size);
}

static inline int max6642_fetch_temp(struct max6642_config *cfg, struct max6642_data *data)
{
	int ret;
	uint8_t temp_read[2];

	ret = max6642_reg_read(cfg, MAX6642_REG_TEMP, temp_read, sizeof(temp_read));
	if (ret) {
		LOG_ERR("Could not fetch temperature [%d]", ret);
		return -EIO;
	}

	data->local_temp = temp_read[0];
	data->remote_temp = temp_read[1];

	return 0;
}

static int max6642_sample_fetch(const struct device *dev,
			     enum sensor_channel chan)
{
	struct max6642_data *data = (struct max6642_data *)dev->data;
	struct max6642_config *cfg = (struct max6642_config *)dev->config;

	switch (chan) {
	case SENSOR_CHAN_ALL:
	case SENSOR_CHAN_AMBIENT_TEMP:
	case SENSOR_CHAN_GAUGE_TEMP:
		return max6642_fetch_temp(cfg, data);
	default:
		return -ENOTSUP;
	}
}

static int max6642_channel_get(const struct device *dev,
			    enum sensor_channel chan,
			    struct sensor_value *val)
{
	struct max6642_data *data = (struct max6642_data *)dev->data;

	switch (chan) {
	// Local temperature
	case SENSOR_CHAN_AMBIENT_TEMP:
		val->val1 = data->local_temp;
		val->val2 = 0;
		return 0;
	// Remote temperature
	case SENSOR_CHAN_GAUGE_TEMP:
		val->val1 = data->remote_temp;
		val->val2 = 0;
		return 0;
	default:
		return -ENOTSUP;
	}
}

static const struct sensor_driver_api max6642_driver_api = {
	.sample_fetch = max6642_sample_fetch,
	.channel_get = max6642_channel_get,
};

int max6642_init(const struct device *dev)
{
	struct max6642_config *cfg = (struct max6642_config *)dev->config;

	if (device_is_ready(cfg->i2c_dev)) {
		return 0;
	}

	LOG_ERR("I2C dev not ready");
	return -ENODEV;
}

#define MAX6642_INST(inst)                                             \
static struct max6642_data max6642_data_##inst;                           \
static const struct max6642_config max6642_config_##inst = {              \
	.i2c_dev = DEVICE_DT_GET(DT_INST_BUS(inst)),                \
	.i2c_addr = DT_INST_REG_ADDR(inst),                         \
};                                                                  \
DEVICE_DT_INST_DEFINE(inst, max6642_init, NULL, &max6642_data_##inst,     \
		      &max6642_config_##inst, POST_KERNEL,             \
		      CONFIG_SENSOR_INIT_PRIORITY, &max6642_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MAX6642_INST)
