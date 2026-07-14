/*
 * Copyright (c) 2020 Andreas Sandberg
 * Copyright (c) 2020 Grinn
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SX12XX_COMMON_H_
#define ZEPHYR_DRIVERS_SX12XX_COMMON_H_

#include <zephyr/types.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/device.h>

int __sx12xx_configure_pin(const struct gpio_dt_spec *gpio, gpio_flags_t flags);

#define sx12xx_configure_pin(_name, _flags)				\
	COND_CODE_1(DT_INST_NODE_HAS_PROP(0, _name##_gpios),		\
		    (__sx12xx_configure_pin(&dev_config._name, _flags)),\
		    (0))

bool sx12xx_modem_acquire(void);

bool sx12xx_modem_release(void);

const struct lora_modem_config *sx12xx_get_tx_config(void);

int sx12xx_lora_send(const struct device *dev, uint8_t *data,
		     uint32_t data_len);

int sx12xx_lora_send_async(const struct device *dev, uint8_t *data,
			   uint32_t data_len, struct k_poll_signal *async);

int sx12xx_lora_recv(const struct device *dev, uint8_t *data, uint8_t size,
		     k_timeout_t timeout, int16_t *rssi, int8_t *snr);

int sx12xx_lora_recv_async(const struct device *dev, lora_recv_cb cb, void *user_data);

uint32_t sx12xx_airtime(const struct device *dev, uint32_t data_len);

int sx12xx_lora_config(const struct device *dev,
		       const struct lora_modem_config *config);

/**
 * Perform a sync channel active detection.
 *
 * @param dev The LoRa device
 * @param timeout Timeout for the detection
 * @param acquire Set to @c true to run a modem_acquire.
 * 				  Set to @c false if modem is already acquired.
 * @return 0 if no activity detected (channel free)
 * @return 1 if activity detected (channel busy)
 * @return negative on other errors
 */
int sx126x_lora_cad_common(const struct device *dev, k_timeout_t timeout, bool acquire);

/**
 * Perform a async channel active detection.
 *
 * @param dev The LoRa device
 * @param cb CAD user callback
 * @param user_data CAD callback user data
 * @param acquire Set to @c true to run a modem_acquire.
 * 				  Set to @c false if modem is already acquired.
 * @return 0 if no activity detected (channel free)
 * @return 1 if activity detected (channel busy)
 * @return negative on other errors
 */
int sx12xx_lora_cad_async_common(const struct device *dev, lora_cad_cb cb, void *user_data, bool acquire);

int sx12xx_lora_test_cw(const struct device *dev, uint32_t frequency,
			int8_t tx_power,
			uint16_t duration);

int sx12xx_init(const struct device *dev);

#endif /* ZEPHYR_DRIVERS_SX12XX_COMMON_H_ */
