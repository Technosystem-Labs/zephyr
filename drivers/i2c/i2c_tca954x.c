/*
 * Copyright (c) 2020 Innoseis BV
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/i2c.h>
#include <logging/log.h>
#include <stdint.h>

LOG_MODULE_REGISTER(tca954x, CONFIG_I2C_LOG_LEVEL);

struct tca954x_root_config {
	const struct device *bus;
	uint16_t slave_addr;
	uint8_t nchans;
};

struct tca954x_root_data {
	struct k_mutex lock;
	uint8_t selected_chan;
};

struct tca954x_channel_config {
	const struct device *root;
	uint8_t chan_mask;
};

static inline struct tca954x_root_data *
get_root_data_from_channel(const struct device *dev)
{
	const struct tca954x_channel_config *channel_config = dev->config;

	return channel_config->root->data;
}

static inline const struct tca954x_root_config *
get_root_config_from_channel(const struct device *dev)
{
	const struct tca954x_channel_config *channel_config = dev->config;

	return channel_config->root->config;
}

static int tca954x_configure(const struct device *dev, uint32_t dev_config)
{
	const struct tca954x_root_config *cfg =
			get_root_config_from_channel(dev);

	return i2c_configure(cfg->bus, dev_config);
}

static int tca954x_set_channel(const struct device *dev, uint8_t select_mask)
{
	int res = 0;
	struct tca954x_root_data *data = dev->data;
	const struct tca954x_root_config *cfg = dev->config;

	/* Only select the channel if its different from the last channel */
	if (data->selected_chan != select_mask) {
		res = i2c_write(cfg->bus, &select_mask, 1, cfg->slave_addr);
		if (res == 0) {
			data->selected_chan = select_mask;
		} else {
			LOG_DBG("tca954x: failed to set channel");
		}
	}
	return res;
}

static int tca954x_transfer(const struct device *dev,
			     struct i2c_msg *msgs,
			     uint8_t num_msgs,
			     uint16_t addr)
{
	struct tca954x_root_data *data = get_root_data_from_channel(dev);
	const struct tca954x_root_config *config =
			get_root_config_from_channel(dev);
	const struct tca954x_channel_config *down_cfg = dev->config;
	int res;

	res = k_mutex_lock(&data->lock, K_MSEC(5000));
	if (res != 0) {
		return res;
	}

	res = tca954x_set_channel(down_cfg->root, down_cfg->chan_mask);
	if (res != 0) {
		goto end_trans;
	}

	res = i2c_transfer(config->bus, msgs, num_msgs, addr);

end_trans:
	k_mutex_unlock(&data->lock);
	return res;
}

static int tca954x_root_init(const struct device *dev)
{
	struct tca954x_root_data *i2c_tca954x = dev->data;
	const struct tca954x_root_config *config = dev->config;

	if (!device_is_ready(config->bus)) {
		LOG_ERR("I2C bus %s not ready", config->bus->name);
		return -ENODEV;
	}

	i2c_tca954x->selected_chan = 0;

	return 0;
}

static int tca954x_channel_init(const struct device *dev)
{
	const struct tca954x_channel_config *chan_cfg = dev->config;
	const struct tca954x_root_config *root_cfg =
			get_root_config_from_channel(dev);

	if (!device_is_ready(chan_cfg->root)) {
		LOG_ERR("I2C mux root %s not ready", chan_cfg->root->name);
		return -ENODEV;
	}

	if (chan_cfg->chan_mask >= BIT(root_cfg->nchans)) {
		LOG_ERR("Wrong DTS address provided for %s", dev->name);
		return -EINVAL;
	}

	return 0;
}

const struct i2c_driver_api tca954x_api_funcs = {
	.configure = tca954x_configure,
	.transfer = tca954x_transfer,
};

#define TCA954x_CHILD_DEFINE(node_id)					    \
	static const struct tca954x_channel_config			    \
		tca954x_down_config_##node_id = {			    \
		.chan_mask = BIT(DT_REG_ADDR(node_id)),			    \
		.root = DEVICE_DT_GET(DT_PARENT(node_id)),		    \
	};								    \
	DEVICE_DT_DEFINE(node_id,					    \
			 tca954x_channel_init,				    \
			 NULL,						    \
			 NULL,						    \
			 &tca954x_down_config_##node_id,		    \
			 POST_KERNEL, CONFIG_I2C_TCA954X_CHANNEL_INIT_PRIO, \
			 &tca954x_api_funcs);

#define TCA954x_ROOT_DEFINE(inst, ch, compat)				      \
	static const struct tca954x_root_config tca954x_cfg_##inst = {        \
		.slave_addr = DT_REG_ADDR_BY_IDX(DT_INST(inst, compat), 0),   \
		.bus = DEVICE_DT_GET(DT_BUS(DT_INST(inst, compat))),		      \
		.nchans = ch,			      \
	};								      \
	static struct tca954x_root_data tca954x_data_##inst = {	      \
		.lock = Z_MUTEX_INITIALIZER(tca954x_data_##inst.lock),	      \
	};								      \
	DEVICE_DT_DEFINE(DT_INST(inst, compat),					      \
			      tca954x_root_init, NULL,			      \
			      &tca954x_data_##inst, &tca954x_cfg_##inst,    \
			      POST_KERNEL, CONFIG_I2C_TCA954X_ROOT_INIT_PRIO, \
			      NULL);					      \
	DT_FOREACH_CHILD(DT_INST(inst, compat), TCA954x_CHILD_DEFINE);

#define TCA954X_FOREACH_STATUS_OKAY(compat, fn)		\
	COND_CODE_1(DT_HAS_COMPAT_STATUS_OKAY(compat),		\
		    (UTIL_CAT(DT_FOREACH_OKAY_INST_,		\
			      compat)(fn)),			\
		    ())

/*
 * TCA9546A: 4 channels
 */
#define TCA9546A_INIT(n) TCA954x_ROOT_DEFINE(n, 4, ti_tca9546a)
TCA954X_FOREACH_STATUS_OKAY(ti_tca9546a, TCA9546A_INIT)
