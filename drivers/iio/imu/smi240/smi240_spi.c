// SPDX-License-Identifier: BSD-3-Clause OR GPL-2.0
/**
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE
 * Copyright (c) 2023 Robert Bosch GmbH. All rights reserved.
 *
 * This file is free software licensed under the terms of version 2
 * of the GNU General Public License, available from the file LICENSE-GPL
 * in the main directory of this source tree.
 *
 * BSD LICENSE
 * Copyright (c) 2023 Robert Bosch GmbH. All rights reserved.
 *
 * BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 **/

#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/types.h>

#include "smi240.h"

#define SMI240_SPI_MAX_BUFFER_SIZE 32

static uint8_t *rx_buf;
static uint8_t *tx_buf;
static struct spi_device *smi240_spi_dev;
static struct smi240_device smi240_dev;

static int8_t smi240_spi_transfer(uint32_t request, uint32_t *response)
{
	int8_t ret;
	struct spi_message msg;
	struct spi_transfer xfer = {
		.tx_buf = tx_buf, .rx_buf = rx_buf, .len = 4,
		//.bits_per_word = 32,
	};

	if (smi240_spi_dev == NULL)
		return -ENODEV;

	tx_buf[0] = request >> 24;
	tx_buf[1] = request >> 16;
	tx_buf[2] = request >> 8;
	tx_buf[3] = request;

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);
	ret = spi_sync(smi240_spi_dev, &msg);

	if (ret)
		return ret;

	if (response != NULL)
		*response = (rx_buf[0] << 24) | (rx_buf[1] << 16) |
			    (rx_buf[2] << 8) | rx_buf[3];

	return ret;
}

static int smi240_spi_probe(struct spi_device *device)
{
	int err;

	device->bits_per_word = 8;
	device->max_speed_hz = 10000000;
	device->mode = SPI_MODE_0;

	err = spi_setup(device);
	if (err < 0) {
		pr_err("spi_setup err!\n");
		return err;
	}

	if (rx_buf == NULL)
		rx_buf = kmalloc(4, GFP_KERNEL);
	if (!rx_buf)
		return -ENOMEM;

	if (tx_buf == NULL)
		tx_buf = kmalloc(4, GFP_KERNEL);
	if (!tx_buf)
		return -ENOMEM;

	smi240_spi_dev = device;

	err = smi240_probe(&device->dev, &smi240_dev);
	if (err) {
		kfree(rx_buf);
		rx_buf = NULL;
		kfree(tx_buf);
		tx_buf = NULL;
		smi240_spi_dev = NULL;
		dev_err(&device->dev,
			"Bosch Sensor Device %s initialization failed %d",
			SENSOR_NAME, err);
	} else
		pr_info("Bosch Sensor Device %s initialized", SENSOR_NAME);

	return err;
}

static int smi240_spi_remove(struct spi_device *device)
{
	if (rx_buf != NULL) {
		kfree(rx_buf);
		rx_buf = NULL;
	}

	if (tx_buf != NULL) {
		kfree(tx_buf);
		tx_buf = NULL;
	}
	return smi240_remove(&device->dev);
}

static const struct spi_device_id smi240_id[] = { { SENSOR_NAME, 0 }, {} };

MODULE_DEVICE_TABLE(spi, smi240_id);

static const struct of_device_id smi240_of_match[] = {
	{
		.compatible = SENSOR_NAME,
	},
	{}
};

MODULE_DEVICE_TABLE(of, smi240_of_match);

static struct spi_driver smi240_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = SENSOR_NAME,
		   .of_match_table = smi240_of_match,
		    },
	.id_table = smi240_id,
	.probe = smi240_spi_probe,
	.remove = smi240_spi_remove,
};

static int __init smi240_module_init(void)
{
	int err = 0;

	smi240_dev.delay_ms = smi240_delay;
	smi240_dev.xfer = smi240_spi_transfer;

	err |= spi_register_driver(&smi240_driver);
	return err;
}

static void __exit smi240_module_exit(void)
{
	spi_unregister_driver(&smi240_driver);
}

module_init(smi240_module_init);
module_exit(smi240_module_exit);

MODULE_DESCRIPTION("SMI240 IMU SENSOR DRIVER");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_VERSION(DRIVER_VERSION);
