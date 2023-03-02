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

#include <linux/delay.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#include "smi240.h"

enum {
	SMI240_ACC_X,
	SMI240_ACC_Y,
	SMI240_ACC_Z,
	SMI240_GYRO_X,
	SMI240_GYRO_Y,
	SMI240_GYRO_Z,
	SMI240_TEMP_OBJECT,
	SMI240_TIMESTAMP,
};

#define SMI240_CRC_INIT 0x05
#define SMI240_CRC_POLY 0x0B
#define SMI240_BUS_ID	0x00

#define SMI240_WRITE_ADDR_MASK 0x3FC00000
#define SMI240_WRITE_ADDR_POS  22
#define SMI240_WRITE_BIT_MASK  0x00200000
#define SMI240_WRITE_BIT_POS   21
#define SMI240_WRITE_DATA_MASK 0x0007FFF8
#define SMI240_WRITE_DATA_POS  3
#define SMI240_CAP_BIT_MASK    0x00100000
#define SMI240_CAP_BIT_POS     20
#define SMI240_READ_DATA_MASK  0x000FFFF0
#define SMI240_READ_DATA_POS   4

#define SMI240_CHIP_ID_REG     0x00
#define SMI240_SOFT_CONFIG_REG 0x0A
#define SMI240_TEMP_CUR_REG    0x10
#define SMI240_ACCEL_X_CUR_REG 0x11
#define SMI240_ACCEL_Y_CUR_REG 0x12
#define SMI240_ACCEL_Z_CUR_REG 0x13
#define SMI240_GYRO_X_CUR_REG  0x14
#define SMI240_GYRO_Y_CUR_REG  0x15
#define SMI240_GYRO_Z_CUR_REG  0x16

#define SMI240_TEMP_CAP_REG    0x17
#define SMI240_ACCEL_X_CAP_REG 0x18
#define SMI240_ACCEL_Y_CAP_REG 0x19
#define SMI240_ACCEL_Z_CAP_REG 0x1A
#define SMI240_GYRO_X_CAP_REG  0x1B
#define SMI240_GYRO_Y_CAP_REG  0x1C
#define SMI240_GYRO_Z_CAP_REG  0x1D

#define SMI240_SOFT_CONFIG_DELAY 500
#define SMI240_TEMPERATURE_BASE	 25
#define SMI240_TEMPERATURE_SHIFT 8

#define SMI240_CHANNEL(_type, _axis, _index)                                   \
	{                                                                      \
		.type = _type, .modified = 1, .channel2 = IIO_MOD_##_axis,     \
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),                  \
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),       \
		.scan_index = _index,                                          \
		.scan_type = {                                                 \
			.sign = 's',                                           \
			.realbits = 16,                                        \
			.storagebits = 16,                                     \
			.endianness = IIO_LE,                                  \
		},                                                             \
	}

static const struct iio_chan_spec smi240_channels[] = {
	SMI240_CHANNEL(IIO_ACCEL, X, SMI240_ACC_X),
	SMI240_CHANNEL(IIO_ACCEL, Y, SMI240_ACC_Y),
	SMI240_CHANNEL(IIO_ACCEL, Z, SMI240_ACC_Z),
	SMI240_CHANNEL(IIO_ANGL_VEL, X, SMI240_GYRO_X),
	SMI240_CHANNEL(IIO_ANGL_VEL, Y, SMI240_GYRO_Y),
	SMI240_CHANNEL(IIO_ANGL_VEL, Z, SMI240_GYRO_Z),
	SMI240_CHANNEL(IIO_TEMP, TEMP_OBJECT, SMI240_TEMP_OBJECT),
	IIO_CHAN_SOFT_TIMESTAMP(SMI240_TIMESTAMP),
};

static uint8_t smi240_crc3(uint32_t data, uint8_t init, uint8_t poly)
{
	uint8_t crc = init;
	uint8_t do_xor;
	int8_t i = 31;

	do {
		do_xor = crc & 0x04;
		crc <<= 1;
		crc |= 0x01 & (data >> i);
		if (do_xor)
			crc ^= poly;

		crc &= 0x07;
	} while (--i >= 0);

	return crc;
}

static int8_t smi240_get_regs(uint8_t reg_addr, uint16_t *reg_data,
			      uint16_t len, uint8_t capture,
			      const struct smi240_device *dev)
{
	int ret, i;
	uint8_t cap;
	uint32_t request, response;

	for (i = 0; i < len; i++) {
		cap = capture && (i == 0);
		request = SMI240_BUS_ID << 30;
		request = SET_BITS(request, SMI240_CAP_BIT, cap);
		request = SET_BITS(request, SMI240_WRITE_ADDR, reg_addr + i);
		request |=
			smi240_crc3(request, SMI240_CRC_INIT, SMI240_CRC_POLY);

		ret = dev->xfer(request, &response);

		if (i > 0) {
			if (smi240_crc3(response, SMI240_CRC_INIT,
					SMI240_CRC_POLY))
				return -EIO;

			reg_data[i - 1] = GET_BITS(response, SMI240_READ_DATA);
		}
	}

	ret = dev->xfer(0x0, &response);

	if (smi240_crc3(response, SMI240_CRC_INIT, SMI240_CRC_POLY))
		return -EIO;

	reg_data[i - 1] = GET_BITS(response, SMI240_READ_DATA);

	return ret;
}

static int8_t smi240_set_regs(uint8_t reg_addr, uint16_t *reg_data,
			      uint16_t len, const struct smi240_device *dev)
{
	int ret;
	int i;
	uint32_t data;

	for (i = 0; i < len; i++) {
		data = SMI240_BUS_ID << 30;
		data = SET_BITS(data, SMI240_WRITE_BIT, 1);
		data = SET_BITS(data, SMI240_WRITE_ADDR, reg_addr + i);
		data = SET_BITS(data, SMI240_WRITE_DATA, reg_data[i]);
		data |= smi240_crc3(data, SMI240_CRC_INIT, SMI240_CRC_POLY);
		ret = dev->xfer(data, NULL);
	}
	return ret;
}

void smi240_delay(uint32_t msec)
{
	if (msec <= 100)
		mdelay(msec);
	else
		msleep(msec);
}

static int smi240_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan, int *val,
			   int *val2, long mask)
{
	int ret;
	int16_t data;
	struct smi240_device *dev = iio_device_get_drvdata(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		switch (chan->channel2) {
		case IIO_MOD_X:
			if (chan->type == IIO_ACCEL)
				ret = smi240_get_regs(SMI240_ACCEL_X_CUR_REG,
						      &data, 1, 0, dev);
			else if (chan->type == IIO_ANGL_VEL)
				ret = smi240_get_regs(SMI240_GYRO_X_CUR_REG,
						      &data, 1, 0, dev);
			break;
		case IIO_MOD_Y:
			if (chan->type == IIO_ACCEL)
				ret = smi240_get_regs(SMI240_ACCEL_Y_CUR_REG,
						      &data, 1, 0, dev);
			else if (chan->type == IIO_ANGL_VEL)
				ret = smi240_get_regs(SMI240_GYRO_Y_CUR_REG,
						      &data, 1, 0, dev);
			break;
		case IIO_MOD_Z:
			if (chan->type == IIO_ACCEL)
				ret = smi240_get_regs(SMI240_ACCEL_Z_CUR_REG,
						      &data, 1, 0, dev);
			else if (chan->type == IIO_ANGL_VEL)
				ret = smi240_get_regs(SMI240_GYRO_Z_CUR_REG,
						      &data, 1, 0, dev);
			break;
		case IIO_MOD_TEMP_OBJECT:
			ret = smi240_get_regs(SMI240_TEMP_CUR_REG, &data, 1, 0,
					      dev);
			data >>= SMI240_TEMPERATURE_SHIFT;
			data += SMI240_TEMPERATURE_BASE;
			break;
		default:
			return -EINVAL;
		}

		if (ret)
			return -EIO;

		*val = data;
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = 0;
		*val2 = 0;
		return IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}

	return ret;
}

static int smi240_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan, int val, int val2,
			    long mask)
{
	int ret;
	// struct smi240_device *dev = iio_device_get_drvdata(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		pr_info("Write sampling frequency.");
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static int smi240_init(struct smi240_device *dev)
{
	int ret;
	uint16_t request, response;

	request = 0x000F; // soft config

	ret = smi240_set_regs(SMI240_SOFT_CONFIG_REG, &request, 1, dev);
	if (ret) {
		pr_info("Soft Config failed.");
		return ret;
	}

	dev->delay_ms(SMI240_SOFT_CONFIG_DELAY);

	ret = smi240_get_regs(SMI240_CHIP_ID_REG, &response, 1, 0, dev);
	if (ret) {
		pr_info("Read chip id failed.");
		return ret;
	}

	pr_info("SMI240 Chip ID: 0x%04x", response);
	return ret;
}

static int in_temp_accel_anglvel_capture_show(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	int ret;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi240_device *smi240_dev = iio_device_get_drvdata(indio_dev);
	int16_t data[7];

	ret = smi240_get_regs(SMI240_TEMP_CAP_REG, data, 7, 1, smi240_dev);

	data[0] >>= SMI240_TEMPERATURE_SHIFT;
	data[0] += SMI240_TEMPERATURE_BASE;

	return snprintf(buf, PAGE_SIZE, "%hd %hd %hd %hd %hd %hd %hd\n",
			data[0], data[1], data[2], data[3], data[4], data[5],
			data[6]);
}

static IIO_CONST_ATTR(sampling_frequency_available, "100 200 400 1000 2000");
static IIO_DEVICE_ATTR_RO(in_temp_accel_anglvel_capture, 0);

static struct attribute *smi240_attrs[] = {
	&iio_const_attr_sampling_frequency_available.dev_attr.attr,
	&iio_dev_attr_in_temp_accel_anglvel_capture.dev_attr.attr,
	NULL,
};

static const struct attribute_group smi240_attrs_group = {
	.attrs = smi240_attrs,
};

static const struct iio_info smi240_info = {
	.read_raw = smi240_read_raw,
	.write_raw = smi240_write_raw,
	.attrs = &smi240_attrs_group,
};

int smi240_probe(struct device *dev, struct smi240_device *smi240_dev)
{
	int ret;
	struct iio_dev *indio_dev;

	ret = smi240_init(smi240_dev);
	if (ret)
		return ret;

	indio_dev = devm_iio_device_alloc(dev, 0);
	if (!indio_dev)
		return -ENOMEM;

	iio_device_set_drvdata(indio_dev, smi240_dev);
	dev_set_drvdata(dev, indio_dev);

	indio_dev->dev.parent = dev;
	indio_dev->channels = smi240_channels;
	indio_dev->num_channels = ARRAY_SIZE(smi240_channels);
	indio_dev->name = SENSOR_NAME;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &smi240_info;

	ret = devm_iio_device_register(dev, indio_dev);
	if (ret) {
		dev_err(dev, "Register IIO device failed");
		goto exit_failure;
	}

	return ret;

exit_failure:
	smi240_remove(dev);
	return ret;
}

int smi240_remove(struct device *dev)
{
	return 0;
}
