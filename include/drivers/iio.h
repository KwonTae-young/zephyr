/*
 * Copyright (c) 2019 Kwon Tae-young <tykwon@m2i.co.kr>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_IIO_H_
#define ZEPHYR_INCLUDE_DRIVERS_IIO_H_

/**
 * @file
 * @brief Public IIO APIs
 */

#include <zephyr/types.h>
#include <device.h>

/**
 * @typedef iio_api_write()
 * @brief Callback API for sending data over IIO
 *
 * @see iio_write() for argument descriptions.
 */
typedef int (*iio_api_write)(struct device *dev, u8_t *data, u8_t size);

/**
 * @typedef iio_api_read()
 * @brief Callback API for receiving data over IIO
 *
 * @see iio_read() for argument descriptions.
 */
typedef int (*iio_api_read)(struct device *dev, u8_t *data, u8_t size);

/**
 * @typedef iio_api_status()
 * @brief Callback API for status of IIO
 *
 * @see iio_status() for argument descriptions.
 */
typedef int (*iio_api_status)(struct device *dev, u8_t *data, u8_t size);

struct iio_driver_api {
	iio_api_write	write;
	iio_api_read		read;
	iio_api_status	status;
};

/**
 * @brief Write data over IIO
 *
 * @note This is a non-blocking call.
 *
 * @param dev       IIO device
 * @param data      Data to be written
 * @param size	 Length of the data to be written
 * @return 0 on success, negative on error
 */
static inline int iio_write(struct device *dev, u8_t *data, u8_t size)
{
	const struct iio_driver_api *api = dev->driver_api;

	return api->write(dev, data, size);
}

/**
 * @brief Read data over IIO
 *
 * @note This is a non-blocking call.
 *
 * @param dev       IIO device
 * @param data      Buffer to read data
 * @param size	 Length of the data to be read
 * @return Length of the data read on success, negative on error
 */
static inline int iio_read(struct device *dev, u8_t *data, u8_t size)
{
	const struct iio_driver_api *api = dev->driver_api;

	return api->read(dev, data, size);
}

/**
 * @brief Status of IIO
 *
 * @note This is a non-blocking call.
 *
 * @param dev       IIO device
 * @param data      Buffer to read data
 * @param size	 Length of the data to be read
 * @return Length of the data read on success, negative on error
 */
static inline int iio_status(struct device *dev, u8_t *data, u8_t size)
{
	const struct iio_driver_api *api = dev->driver_api;

	return api->status(dev, data, size);
}

#endif	/* ZEPHYR_INCLUDE_DRIVERS_IIO_H_ */
