/*
 * Copyright (c) 2022 Badgerd Technologies B.V.
 * Copyright (c) 2023 Metratec GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>

#include "bmp581.h"

LOG_MODULE_REGISTER(bmp581, CONFIG_SENSOR_LOG_LEVEL);

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#warning "BMP581 driver enabled without any devices"
#endif

static int power_up_check(struct bmp581_data *drv);
static int get_nvm_status(uint8_t *nvm_status, struct bmp581_data *drv);
static int get_interrupt_status(uint8_t *int_status, struct bmp581_data *drv);
static int validate_chip_id(struct bmp581_data *drv);
static int get_osr_odr_press_config(struct bmp581_osr_odr_press_config *osr_odr_press_cfg,
				    struct bmp581_data *drv);
static int set_osr_config(const struct sensor_value *osr, enum sensor_channel chan,
			  struct bmp581_data *drv);
static int set_odr_config(const struct sensor_value *odr, struct bmp581_data *drv);
static int soft_reset(struct bmp581_data *drv);
static int set_iir_config(const struct sensor_value *iir, struct bmp581_data *drv);
static int get_power_mode(enum bmp5_powermode *powermode, struct bmp581_data *drv);
static int set_power_mode(enum bmp5_powermode powermode, struct bmp581_data *drv);

#if IS_ENABLED(CONFIG_BMP581_ALTITUDE_CALC_LUT)
struct altutide_lut {
	/* in Pa */
	double pressure;
	/* in mm */
	int32_t altitude;
};

struct altutide_lut altitude_lut[] = {
	{30000, 9160758},   {32500, 8621298},   {35000, 8114456},   {37500, 7636129},
	{40000, 7182964},   {42500, 6752181},   {45000, 6341450},   {47500, 5948800},
	{50000, 5572541},   {52500, 5211219},   {55000, 4863571},   {57500, 4528491},
	{60000, 4205007},   {62500, 3892259},   {65000, 3589482},   {67500, 3295992},
	{70000, 3011175},   {72500, 2734479},   {75000, 2465404},   {77500, 2203496},
	{80000, 1948342},   {82500, 1699564},   {85000, 1456818},   {87500, 1219785},
	{90000, 988174},    {92500, 761716},    {95000, 540159},    {97500, 323275},
	{100000, 110848},   {102500, -97321},   {105000, -301420},  {107500, -501621},
	{110000, -698087},  {112500, -890970},  {115000, -1080413}, {117500, -1266550},
	{120000, -1449508}, {122500, -1629405}, {125000, -1806354}};

static double _interpolate(double x, double x0, double x1, int32_t y0, int32_t y1)
{
	return y0 + (y1 - y0) * (x - x0) / (x1 - x0);
}
#endif

int reg_read(uint8_t reg, uint8_t *data, uint16_t length, struct bmp581_data *drv)
{
	return i2c_burst_read_dt(drv->i2c, reg, data, length);
}

int reg_write(uint8_t reg, const uint8_t *data, uint16_t length, struct bmp581_data *drv)
{
	return i2c_burst_write_dt(drv->i2c, reg, data, length);
}

static int set_power_mode(enum bmp5_powermode powermode, struct bmp581_data *drv)
{
	int ret = BMP5_OK;
	uint8_t odr = 0;
	enum bmp5_powermode current_powermode;

	ret = get_power_mode(&current_powermode, drv);
	if (ret != BMP5_OK) {
		LOG_DBG("Couldnt set the power mode because something went wrong when getting the "
			"current power mode.");
		return ret;
	}

	if (current_powermode != BMP5_POWERMODE_STANDBY) {
		/*
		 * Device should be set to standby before transitioning to forced mode or normal
		 * mode or continuous mode.
		 */

		ret = reg_read(BMP5_REG_ODR_CONFIG, &odr, 1, drv);
		if (ret == BMP5_OK) {
			/* Setting deep_dis = 1(BMP5_DEEP_DISABLED) disables the deep standby mode
			 */
			odr = BMP5_SET_BITSLICE(odr, BMP5_DEEP_DISABLE, BMP5_DEEP_DISABLED);
			odr = BMP5_SET_BITS_POS_0(odr, BMP5_POWERMODE, BMP5_POWERMODE_STANDBY);
			ret = reg_write(BMP5_REG_ODR_CONFIG, &odr, 1, drv);
			if (ret != BMP5_OK) {
				LOG_DBG("Failed to set power mode to BMP5_POWERMODE_STANDBY.");
				return ret;
			}
		}
	}

	/* lets update the power mode */
	switch (powermode) {
	case BMP5_POWERMODE_STANDBY:
		/* this change is already done so we can just return */
		ret = BMP5_OK;
		break;
	case BMP5_POWERMODE_DEEP_STANDBY:
		LOG_DBG("Setting power mode to DEEP STANDBY is not supported, current power mode "
			"is BMP5_POWERMODE_STANDBY.");
		ret = -ENOTSUP;
		break;
	case BMP5_POWERMODE_NORMAL:
	case BMP5_POWERMODE_FORCED:
	case BMP5_POWERMODE_CONTINUOUS:
		odr = BMP5_SET_BITSLICE(odr, BMP5_DEEP_DISABLE, BMP5_DEEP_DISABLED);
		odr = BMP5_SET_BITS_POS_0(odr, BMP5_POWERMODE, powermode);
		ret = reg_write(BMP5_REG_ODR_CONFIG, &odr, 1, drv);
		break;
	default:
		ret = BMP5_E_INVALID_POWERMODE;
		break;
	}

	return ret;
}

static int get_power_mode(enum bmp5_powermode *powermode, struct bmp581_data *drv)
{
	int ret = BMP5_OK;

	if (powermode != NULL) {
		uint8_t reg = 0;
		uint8_t raw_power_mode = 0;

		ret = reg_read(BMP5_REG_ODR_CONFIG, &reg, 1, drv);
		if (ret != BMP5_OK) {
			LOG_DBG("Failed to read odr config to get power mode!");
			return ret;
		}

		raw_power_mode = BMP5_GET_BITS_POS_0(reg, BMP5_POWERMODE);

		switch (raw_power_mode) {
		case BMP5_POWERMODE_STANDBY: {
			/* Getting deep disable status */
			uint8_t deep_dis = BMP5_GET_BITSLICE(reg, BMP5_DEEP_DISABLE);

			/* Checking deepstandby status only when powermode is in standby mode */

			/* If deep_dis = 0(BMP5_DEEP_ENABLED) then deepstandby mode is enabled.
			 * If deep_dis = 1(BMP5_DEEP_DISABLED) then deepstandby mode is disabled
			 */
			if (deep_dis == BMP5_DEEP_ENABLED) {
				/* TODO: check if it is really deep standby */
				*powermode = BMP5_POWERMODE_DEEP_STANDBY;
			} else {
				*powermode = BMP5_POWERMODE_STANDBY;
			}

			break;
		}
		case BMP5_POWERMODE_NORMAL:
			*powermode = BMP5_POWERMODE_NORMAL;
			break;
		case BMP5_POWERMODE_FORCED:
			*powermode = BMP5_POWERMODE_FORCED;
			break;
		case BMP5_POWERMODE_CONTINUOUS:
			*powermode = BMP5_POWERMODE_CONTINUOUS;
			break;
		default:
			ret = BMP5_E_INVALID_POWERMODE;
			LOG_DBG("Something went wrong invalid powermode!");
			break;
		}
	} else {
		ret = BMP5_E_NULL_PTR;
	}

	return ret;
}

static int power_up_check(struct bmp581_data *drv)
{
	int8_t rslt = 0;
	uint8_t nvm_status = 0;

	rslt = get_nvm_status(&nvm_status, drv);

	if (rslt == BMP5_OK) {
		/* Check if nvm_rdy status = 1 and nvm_err status = 0 to proceed */
		if ((nvm_status & BMP5_INT_NVM_RDY) && (!(nvm_status & BMP5_INT_NVM_ERR))) {
			rslt = BMP5_OK;
		} else {
			rslt = BMP5_E_POWER_UP;
		}
	}

	return rslt;
}

static int get_interrupt_status(uint8_t *int_status, struct bmp581_data *drv)
{
	int8_t rslt = 0;

	if (int_status != NULL) {
		rslt = reg_read(BMP5_REG_INT_STATUS, int_status, 1, drv);
	} else {
		rslt = BMP5_E_NULL_PTR;
	}

	return rslt;
}

static int get_nvm_status(uint8_t *nvm_status, struct bmp581_data *drv)
{
	int8_t rslt = 0;

	if (nvm_status != NULL) {
		rslt = reg_read(BMP5_REG_STATUS, nvm_status, 1, drv);
	} else {
		rslt = BMP5_E_NULL_PTR;
	}

	return rslt;
}

static int validate_chip_id(struct bmp581_data *drv)
{
	int8_t rslt = 0;

	if ((drv->chip_id == BMP5_CHIP_ID_PRIM) || (drv->chip_id == BMP5_CHIP_ID_SEC)) {
		rslt = BMP5_OK;
	} else {
		drv->chip_id = 0;
		rslt = BMP5_E_DEV_NOT_FOUND;
	}

	return rslt;
}

/*!
 *  This API gets the configuration for oversampling of temperature, oversampling of
 *  pressure and ODR configuration along with pressure enable.
 */
static int get_osr_odr_press_config(struct bmp581_osr_odr_press_config *osr_odr_press_cfg,
				    struct bmp581_data *drv)
{
	/* Variable to store the function result */
	int8_t rslt = 0;

	/* Variable to store OSR and ODR config */
	uint8_t reg_data[2] = {0};

	if (osr_odr_press_cfg != NULL) {
		/* Get OSR and ODR configuration in burst read */
		rslt = reg_read(BMP5_REG_OSR_CONFIG, reg_data, 2, drv);

		if (rslt == BMP5_OK) {
			osr_odr_press_cfg->osr_t = BMP5_GET_BITS_POS_0(reg_data[0], BMP5_TEMP_OS);
			osr_odr_press_cfg->osr_p = BMP5_GET_BITSLICE(reg_data[0], BMP5_PRESS_OS);
			osr_odr_press_cfg->press_en = BMP5_GET_BITSLICE(reg_data[0], BMP5_PRESS_EN);
			osr_odr_press_cfg->odr = BMP5_GET_BITSLICE(reg_data[1], BMP5_ODR);
		}
	} else {
		rslt = BMP5_E_NULL_PTR;
	}

	return rslt;
}

static int set_osr_config(const struct sensor_value *osr, enum sensor_channel chan,
			  struct bmp581_data *drv)
{
	int ret = 0;

	if (osr != NULL) {
		uint8_t oversampling = osr->val1;
		uint8_t press_en = osr->val2 != 0; /* if it is not 0 then pressure is enabled */
		uint8_t osr_val = 0;

		ret = reg_read(BMP5_REG_OSR_CONFIG, &osr_val, 1, drv);
		if (ret == BMP5_OK) {
			switch (chan) {
			case SENSOR_CHAN_ALL:
				osr_val = BMP5_SET_BITS_POS_0(osr_val, BMP5_TEMP_OS, oversampling);
				osr_val = BMP5_SET_BITSLICE(osr_val, BMP5_PRESS_OS, oversampling);
				osr_val = BMP5_SET_BITSLICE(osr_val, BMP5_PRESS_EN, press_en);
				break;
			case SENSOR_CHAN_PRESS:
				osr_val = BMP5_SET_BITSLICE(osr_val, BMP5_PRESS_OS, oversampling);
				osr_val = BMP5_SET_BITSLICE(osr_val, BMP5_PRESS_EN, press_en);
				break;
			case SENSOR_CHAN_AMBIENT_TEMP:
				osr_val = BMP5_SET_BITS_POS_0(osr_val, BMP5_TEMP_OS, oversampling);
				break;
			default:
				ret = -ENOTSUP;
				break;
			}

			if (ret == BMP5_OK) {
				ret = reg_write(BMP5_REG_OSR_CONFIG, &osr_val, 1, drv);
				get_osr_odr_press_config(&drv->osr_odr_press_config, drv);
			}
		}
	} else {
		ret = BMP5_E_NULL_PTR;
	}

	return ret;
}

static int set_odr_config(const struct sensor_value *odr, struct bmp581_data *drv)
{
	int ret = 0;

	if (odr != NULL) {
		uint8_t odr_val = 0;

		ret = reg_read(BMP5_REG_ODR_CONFIG, &odr_val, 1, drv);
		if (ret != BMP5_OK) {
			return ret;
		}
		odr_val = BMP5_SET_BITSLICE(odr_val, BMP5_ODR, odr->val1);
		ret = reg_write(BMP5_REG_ODR_CONFIG, (const uint8_t *)&odr_val, 1, drv);
		get_osr_odr_press_config(&drv->osr_odr_press_config, drv);
	} else {
		ret = BMP5_E_NULL_PTR;
	}

	return ret;
}

static int soft_reset(struct bmp581_data *drv)
{
	int ret = 0;
	const uint8_t reset_cmd = BMP5_SOFT_RESET_CMD;
	uint8_t int_status = 0;

	ret = reg_write(BMP5_REG_CMD, &reset_cmd, 1, drv);

	if (ret == BMP5_OK) {
		k_usleep(BMP5_DELAY_US_SOFT_RESET);
		ret = get_interrupt_status(&int_status, drv);
		if (ret == BMP5_OK) {
			if (int_status & BMP5_INT_ASSERTED_POR_SOFTRESET_COMPLETE) {
				ret = BMP5_OK;
			} else {
				ret = BMP5_E_POR_SOFTRESET;
			}
		} else {
			ret = BMP5_E_NULL_PTR;
		}
	} else {
		LOG_DBG("Failed perform soft-reset.");
	}

	return ret;
}

static int bmp581_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct bmp581_data *drv = (struct bmp581_data *)dev->data;
	uint8_t data[6];
	int ret = 0;

	if (chan != SENSOR_CHAN_ALL) {
		return -ENOTSUP;
	}

	ret = reg_read(BMP5_REG_TEMP_DATA_XLSB, data, 6, drv);
	if (ret == BMP5_OK) {
		int32_t raw_temperature =
			(int32_t)((uint32_t)(data[2] << 16) | (uint16_t)(data[1] << 8) | data[0]);
		/*
		 * Division by 2^16(whose equivalent value is 65536) is performed to get temperature
		 * data in deg C
		 */

		sensor_value_from_double(&drv->last_sample.temperature,
					 (raw_temperature / 65536.0));

		if (drv->osr_odr_press_config.press_en == BMP5_ENABLE) {
			uint32_t raw_pressure = (uint32_t)((uint32_t)(data[5] << 16) |
							   (uint16_t)(data[4] << 8) | data[3]);
			/*
			 * Division by 2^6(whose equivalent value is 64) is performed to get
			 * pressure data in Pa
			 */
			sensor_value_from_double(&drv->last_sample.pressure, (raw_pressure / 64.0));
		} else {
			drv->last_sample.pressure.val1 = 0;
			drv->last_sample.pressure.val2 = 0;
		}
	}

	return ret;
}

static int bmp581_channel_get(const struct device *dev, enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct bmp581_data *drv = (struct bmp581_data *)dev->data;

	if (val == NULL) {
		return BMP5_E_NULL_PTR;
	}

	switch (chan) {
	case SENSOR_CHAN_ALTITUDE: {
#if IS_ENABLED(CONFIG_BMP581_ALTITUDE_CALC_MATH)
		/*
		 *	Whitepaper regarding calculation of pressure altitude can be found in:
		 *	https://www.weather.gov/media/epz/wxcalc/pressureAltitude.pdf
		 *
		 *	Following formula calculates the altitude in meters from pressure
		 *      and temperature.
		 */
		double last_pressure = sensor_value_to_double(&drv->last_sample.pressure);
		double last_temperature = sensor_value_to_double(&drv->last_sample.temperature);
		double sealevel_pressure = 101325.0;
		double deg_c_to_kelvin = 273.15;
		double altitude = ((deg_c_to_kelvin + last_temperature) / 0.0065) *
				  (1.0 - pow(last_pressure / sealevel_pressure, 0.1903));

		sensor_value_from_double(val, altitude);

		return BMP5_OK;
#elif IS_ENABLED(CONFIG_BMP581_ALTITUDE_CALC_LUT)
		double last_pressure = sensor_value_to_double(&drv->last_sample.pressure);

		/* Check that the pressure is within the range defined in the LUT  */
		if (!IN_RANGE(last_pressure, altitude_lut[0].pressure,
			      altitude_lut[ARRAY_SIZE(altitude_lut) - 1].pressure)) {
			/* No matching range found */
			LOG_WRN("Pressure out of range: %f", last_pressure);
			return -EINVAL;
		}

		/* Find the correct altitude range in the look-up table */
		size_t idx = 0;

		for (size_t i = 0; i < ARRAY_SIZE(altitude_lut) - 1; i++) {
			if (last_pressure < altitude_lut[i + 1].pressure) {
				idx = i;
				break;
			}
		}

		/* Get the interpolated measurement multiplier */
		double altitude = _interpolate(
			last_pressure, altitude_lut[idx].pressure, altitude_lut[idx + 1].pressure,
			altitude_lut[idx].altitude, altitude_lut[idx + 1].altitude);

		/* convert altitude from mm to m */
		altitude /= 1000;

		sensor_value_from_double(val, altitude);

		return BMP5_OK;
#else /* CONFIG_BMP581_ALTITUDE_CALC_OFF */
		return -ENOTSUP;
#endif
	}
	case SENSOR_CHAN_PRESS:
		/* returns pressure in Pa */
		*val = drv->last_sample.pressure;
		return BMP5_OK;
	case SENSOR_CHAN_AMBIENT_TEMP:
		/* returns temperature in Celcius */
		*val = drv->last_sample.temperature;
		return BMP5_OK;
	default:
		return -ENOTSUP;
	}
}

static int set_iir_config(const struct sensor_value *iir, struct bmp581_data *drv)
{
	int ret = BMP5_OK;

	if (iir != NULL) {
		/* Variable to store existing powermode */
		enum bmp5_powermode prev_powermode;

		ret = get_power_mode(&prev_powermode, drv);
		if (ret != BMP5_OK) {
			LOG_DBG("Not able to get current power mode.");
			return ret;
		}
		/* IIR configuration is writable only during STANDBY mode(as per datasheet) */
		set_power_mode(BMP5_POWERMODE_STANDBY, drv);

		/* update IIR config */
		uint8_t dsp_config[2];

		ret = reg_read(BMP5_REG_DSP_CONFIG, dsp_config, 2, drv);
		if (ret != BMP5_OK) {
			LOG_DBG("Failed to read dsp config register.");
			return ret;
		}
		/* Put IIR filtered values in data registers */
		dsp_config[0] =
			BMP5_SET_BITSLICE(dsp_config[0], BMP5_SHDW_SET_IIR_TEMP, BMP5_ENABLE);
		dsp_config[0] =
			BMP5_SET_BITSLICE(dsp_config[0], BMP5_SHDW_SET_IIR_PRESS, BMP5_ENABLE);

		/* Configure IIR filter */
		dsp_config[1] = iir->val1;
		dsp_config[1] = BMP5_SET_BITSLICE(dsp_config[1], BMP5_SET_IIR_PRESS, iir->val2);

		/* Set IIR configuration */
		ret = reg_write(BMP5_REG_DSP_CONFIG, dsp_config, 2, drv);

		if (ret != BMP5_OK) {
			LOG_DBG("Failed to configure IIR filter.");
			return ret;
		}

		/* Restore previous power mode if it is not standby already */
		if (prev_powermode != BMP5_POWERMODE_STANDBY) {
			ret = set_power_mode(prev_powermode, drv);
		}
	} else {
		ret = BMP5_E_NULL_PTR;
	}

	return ret;
}

static int bmp581_attr_set(const struct device *dev, enum sensor_channel chan,
			   enum sensor_attribute attr, const struct sensor_value *val)
{
	struct bmp581_data *drv = (struct bmp581_data *)dev->data;
	int ret = -ENOTSUP;

	switch ((int)attr) {
	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		ret = set_odr_config(val, drv);
		break;
	case SENSOR_ATTR_OVERSAMPLING:
		ret = set_osr_config(val, chan, drv);
		break;
	case BMP5_ATTR_POWER_MODE: {
		enum bmp5_powermode powermode = (enum bmp5_powermode)val->val1;

		ret = set_power_mode(powermode, drv);
		break;
	}
	case BMP5_ATTR_IIR_CONFIG:
		ret = set_iir_config(val, drv);
		break;
	default:
		ret = -ENOTSUP;
		break;
	}
	return ret;
}

static int bmp581_init(const struct device *dev)
{
	const struct bmp581_config *cfg = (struct bmp581_config *)dev->config;
	struct bmp581_data *drv = (struct bmp581_data *)dev->data;
	int ret = -1;

	/* Reset the chip id. */
	drv->chip_id = 0;
	memset(&drv->osr_odr_press_config, 0, sizeof(drv->osr_odr_press_config));
	memset(&drv->last_sample, 0, sizeof(drv->last_sample));

	drv->i2c = &cfg->i2c;

	soft_reset(drv);

	ret = reg_read(BMP5_REG_CHIP_ID, &drv->chip_id, 1, drv);
	if (ret != BMP5_OK) {
		return ret;
	}

	if (drv->chip_id != 0) {
		ret = power_up_check(drv);
		if (ret == BMP5_OK) {
			ret = validate_chip_id(drv);
			if (ret != BMP5_OK) {
				LOG_ERR("Unexpected chip id (%x). Expected (%x or %x)",
					drv->chip_id, BMP5_CHIP_ID_PRIM, BMP5_CHIP_ID_SEC);
			}
		}
	} else {
		/* that means something went wrong */
		LOG_ERR("Unexpected chip id (%x). Expected (%x or %x)", drv->chip_id,
			BMP5_CHIP_ID_PRIM, BMP5_CHIP_ID_SEC);
		return BMP5_E_INVALID_CHIP_ID;
	}

	return ret;
}

static const struct sensor_driver_api bmp581_driver_api = {
	.sample_fetch = bmp581_sample_fetch,
	.channel_get = bmp581_channel_get,
	.attr_set = bmp581_attr_set,
};

#define BMP581_CONFIG(i)                                                                           \
	static const struct bmp581_config bmp581_config_##i = {                                    \
		.i2c = I2C_DT_SPEC_INST_GET(i),                                                    \
	}

#define BMP581_INIT(i)                                                                             \
	static struct bmp581_data bmp581_data_##i;                                                 \
	BMP581_CONFIG(i);                                                                          \
                                                                                                   \
	SENSOR_DEVICE_DT_INST_DEFINE(i, bmp581_init, NULL, &bmp581_data_##i, &bmp581_config_##i,   \
				     POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,                     \
				     &bmp581_driver_api);

DT_INST_FOREACH_STATUS_OKAY(BMP581_INIT)
