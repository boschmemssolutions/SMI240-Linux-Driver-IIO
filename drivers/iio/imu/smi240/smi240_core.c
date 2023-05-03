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
#include <linux/string.h>

#include "smi240.h"

enum {
	SMI240_ACC_X_AND_Y_AND_Z,
	SMI240_GYRO_X_AND_Y_AND_Z,
	SMI240_TEMP_OBJECT,
	SMI240_TIMESTAMP,
};

#define SMI240_CHIP_ID 0x0024

#define SMI240_CRC_INIT 0x05
#define SMI240_CRC_POLY 0x0B
#define SMI240_BUS_ID	0x00

#define SMI240_SD_BIT_MASK 0x80000000
#define SMI240_SD_BIT_POS  31
#define SMI240_CS_BIT_MASK 0x00000008
#define SMI240_CS_BIT_POS  3

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

#define SMI240_GYR_BW_MASK    0x0002
#define SMI240_GYR_BW_POS     1
#define SMI240_ACC_BW_MASK    0x0004
#define SMI240_ACC_BW_POS     2
#define SMI240_BITE_AUTO_MASK 0x0008
#define SMI240_BITE_AUTO_POS  3
#define SMI240_BITE_REP_MASK  0x0070
#define SMI240_BITE_REP_POS   4

#define SMI240_GYR_INVERTX_MASK 0x01
#define SMI240_GYR_INVERTX_POS	0
#define SMI240_GYR_INVERTY_MASK 0x02
#define SMI240_GYR_INVERTY_POS	1
#define SMI240_GYR_INVERTZ_MASK 0x04
#define SMI240_GYR_INVERTZ_POS	2
#define SMI240_ACC_INVERTX_MASK 0x08
#define SMI240_ACC_INVERTX_POS	3
#define SMI240_ACC_INVERTY_MASK 0x10
#define SMI240_ACC_INVERTY_POS	4
#define SMI240_ACC_INVERTZ_MASK 0x20
#define SMI240_ACC_INVERTZ_POS	5

#define SMI240_CHIP_ID_REG	0x00
#define SMI240_SOFT_CONFIG_REG	0x0A
#define SMI240_SIGN_SFT_CFG_REG 0x0B
#define SMI240_TEMP_CUR_REG	0x10
#define SMI240_ACCEL_X_CUR_REG	0x11
#define SMI240_ACCEL_Y_CUR_REG	0x12
#define SMI240_ACCEL_Z_CUR_REG	0x13
#define SMI240_GYRO_X_CUR_REG	0x14
#define SMI240_GYRO_Y_CUR_REG	0x15
#define SMI240_GYRO_Z_CUR_REG	0x16

#define SMI240_TEMP_CAP_REG    0x17
#define SMI240_ACCEL_X_CAP_REG 0x18
#define SMI240_ACCEL_Y_CAP_REG 0x19
#define SMI240_ACCEL_Z_CAP_REG 0x1A
#define SMI240_GYRO_X_CAP_REG  0x1B
#define SMI240_GYRO_Y_CAP_REG  0x1C
#define SMI240_GYRO_Z_CAP_REG  0x1D

#define SMI240_CMD_REG	    0x2F
#define SMI240_BITE_CMD_REG 0x36

#define SMI240_SOFT_RESET_CMD 0xB6
#define SMI240_BITE_CMD	      0xB17E

#define SMI240_BITE_SEQUENCE_DELAY   140
#define SMI240_FILTER_FLUSH_DELAY    60
#define SMI240_DIGITAL_STARTUP_DELAY 120
#define SMI240_MECH_STARTUP_DELAY    100

#define SMI240_MIN_BITE_REPS 1
#define SMI240_MAX_BITE_REPS 8

#define SMI240_TEMPERATURE_BASE	 25
#define SMI240_TEMPERATURE_SHIFT 8

#define SMI240_DATA_CHANNEL(_type, _axis, _index)                              \
	{                                                                      \
		.type = _type, .modified = 1, .channel2 = IIO_MOD_##_axis,     \
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),                  \
		.info_mask_shared_by_type =                                    \
			BIT(IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY),      \
		.info_mask_shared_by_type_available =                          \
			BIT(IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY),      \
		.scan_index = _index,                                          \
		.scan_type = {                                                 \
			.sign = 's',                                           \
			.realbits = 16,                                        \
			.storagebits = 16,                                     \
			.endianness = IIO_LE,                                  \
		},                                                             \
	}

#define SMI240_TEMP_CHANNEL(_index)                                            \
	{                                                                      \
		.type = IIO_TEMP, .modified = 1,                               \
		.channel2 = IIO_MOD_TEMP_OBJECT,                               \
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),                  \
		.scan_index = _index,                                          \
		.scan_type = {                                                 \
			.sign = 's',                                           \
			.realbits = 16,                                        \
			.storagebits = 16,                                     \
			.endianness = IIO_LE,                                  \
		},                                                             \
	}

static const int smi240_low_pass_freqs[] = { 50, 400 };

static const struct iio_chan_spec smi240_channels[] = {
	SMI240_DATA_CHANNEL(IIO_ACCEL, X_AND_Y_AND_Z, SMI240_ACC_X_AND_Y_AND_Z),
	SMI240_DATA_CHANNEL(IIO_ANGL_VEL, X_AND_Y_AND_Z,
			    SMI240_GYRO_X_AND_Y_AND_Z),
	SMI240_TEMP_CHANNEL(SMI240_TEMP_OBJECT),
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

static bool smi240_sensor_data_is_valid(uint32_t data)
{
	if (smi240_crc3(data, SMI240_CRC_INIT, SMI240_CRC_POLY))
		return false;

	if (GET_BITS(data, SMI240_SD_BIT) & GET_BITS(data, SMI240_CS_BIT))
		return false;

	return true;
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
			if (!smi240_sensor_data_is_valid(response))
				return -EIO;

			reg_data[i - 1] = GET_BITS(response, SMI240_READ_DATA);
		}
	}

	ret = dev->xfer(0x0, &response);
	if (!smi240_sensor_data_is_valid(response))
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

static void smi240_delay(uint32_t msec)
{
	if (msec <= 100)
		mdelay(msec);
	else
		msleep(msec);
}

static int smi240_self_test(struct smi240_device *dev)
{
	int ret;
	uint16_t response[7];
	uint16_t request = SMI240_BITE_CMD;

	ret = smi240_set_regs(SMI240_BITE_CMD_REG, &request, 1, dev);
	smi240_delay(dev->bite_reps * SMI240_BITE_SEQUENCE_DELAY +
		     SMI240_FILTER_FLUSH_DELAY);
	if (ret) {
		pr_err("Sending BITE command failed.");
		return -EIO;
	}

	/* Reading from all 7 sensor data capture registers w/o error
	 * makes sure all channels are valid.
	 */
	return smi240_get_regs(SMI240_TEMP_CAP_REG, response, 7, 1, dev);
}

static int smi240_soft_reset(struct smi240_device *dev)
{
	int ret;
	uint16_t data = SMI240_SOFT_RESET_CMD;

	ret = smi240_set_regs(SMI240_CMD_REG, &data, 1, dev);
	smi240_delay(SMI240_DIGITAL_STARTUP_DELAY);
	return ret;
}

static int smi240_soft_config(struct smi240_device *dev)
{
	int ret;
	uint8_t acc_bw, gyr_bw;
	uint16_t request = 0x1;

	switch (dev->accel_filter_freq) {
	case 50:
		acc_bw = 0x1;
		break;
	case 400:
		acc_bw = 0x0;
		break;
	default:
		pr_err("Soft Config: invalid ACC_BW.");
		return -EINVAL;
	}

	switch (dev->anglvel_filter_freq) {
	case 50:
		gyr_bw = 0x1;
		break;
	case 400:
		gyr_bw = 0x0;
		break;
	default:
		pr_err("Soft Config: invalid GYR_BW.");
		return -EINVAL;
	}

	request = SET_BITS(request, SMI240_GYR_BW, gyr_bw);
	request = SET_BITS(request, SMI240_ACC_BW, acc_bw);
	request = SET_BITS(request, SMI240_BITE_AUTO, 1);
	request = SET_BITS(request, SMI240_BITE_REP, dev->bite_reps - 1);

	ret = smi240_set_regs(SMI240_SIGN_SFT_CFG_REG, &(dev->sign_of_channels),
			      1, dev);
	ret |= smi240_set_regs(SMI240_SOFT_CONFIG_REG, &request, 1, dev);
	if (ret)
		pr_err("Soft Config: IO error.");

	smi240_delay(SMI240_MECH_STARTUP_DELAY +
		     dev->bite_reps * SMI240_BITE_SEQUENCE_DELAY +
		     SMI240_FILTER_FLUSH_DELAY);
	return ret;
}

static int smi240_read_raw_multi(struct iio_dev *indio_dev,
				 struct iio_chan_spec const *chan, int max_len,
				 int *vals, int *val_len, long mask)
{
	int ret;
	int16_t data[3];
	struct smi240_device *dev = iio_device_get_drvdata(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (chan->channel2 == IIO_MOD_X_AND_Y_AND_Z) {
			if (chan->type == IIO_ACCEL)
				ret = smi240_get_regs(SMI240_ACCEL_X_CAP_REG,
						      data, 3, 1, dev);
			else if (chan->type == IIO_ANGL_VEL)
				ret = smi240_get_regs(SMI240_GYRO_X_CAP_REG,
						      data, 3, 1, dev);

			if (ret)
				return -EIO;

			*val_len = 3;
			vals[0] = data[0];
			vals[1] = data[1];
			vals[2] = data[2];
		} else if (chan->channel2 == IIO_MOD_TEMP_OBJECT) {
			ret = smi240_get_regs(SMI240_TEMP_CUR_REG, data, 1, 0,
					      dev);

			if (ret)
				return -EIO;

			data[0] >>= SMI240_TEMPERATURE_SHIFT;
			data[0] += SMI240_TEMPERATURE_BASE;

			*val_len = 1;
			vals[0] = data[0];
		} else
			return -EINVAL;

		return IIO_VAL_INT_MULTIPLE;

	case IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY:
		ret = smi240_get_regs(SMI240_SOFT_CONFIG_REG, data, 1, 0, dev);
		if (ret)
			return -EIO;

		switch (chan->type) {
		case IIO_ACCEL:
			switch (GET_BITS(data[0], SMI240_ACC_BW)) {
			case 0:
				dev->accel_filter_freq = 400;
				break;
			case 1:
				dev->accel_filter_freq = 50;
				break;
			}

			vals[0] = dev->accel_filter_freq;
			break;
		case IIO_ANGL_VEL:
			switch (GET_BITS(data[0], SMI240_GYR_BW)) {
			case 0:
				dev->anglvel_filter_freq = 400;
				break;
			case 1:
				dev->anglvel_filter_freq = 50;
				break;
			}

			vals[0] = dev->anglvel_filter_freq;
			break;
		default:
			return -EINVAL;
		}

		*val_len = 1;
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}

	return ret;
}

static int smi240_read_avail(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan, const int **vals,
			     int *type, int *length, long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY:
		*vals = smi240_low_pass_freqs;
		*length = ARRAY_SIZE(smi240_low_pass_freqs);
		*type = IIO_VAL_INT;
		return IIO_AVAIL_LIST;
	default:
		return -EINVAL;
	}
}

static int smi240_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan, int val, int val2,
			    long mask)
{
	int ret, i;
	bool valid = false;
	struct smi240_device *dev = iio_device_get_drvdata(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY:
		for (i = 0; i < ARRAY_SIZE(smi240_low_pass_freqs); ++i) {
			if (val == smi240_low_pass_freqs[i]) {
				valid = true;
				break;
			}
		}

		if (!valid)
			return -EINVAL;

		switch (chan->type) {
		case IIO_ACCEL:
			dev->accel_filter_freq = val;
			break;
		case IIO_ANGL_VEL:
			dev->anglvel_filter_freq = val;
			break;
		default:
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}

	ret = smi240_soft_reset(dev);
	ret |= smi240_soft_config(dev);
	if (ret)
		ret = -EIO;

	return ret;
}

static int smi240_init(struct smi240_device *dev)
{
	int ret;

	dev->accel_filter_freq = 400;
	dev->anglvel_filter_freq = 400;
	dev->sign_of_channels = 0x00;
	dev->bite_reps = 3;

	ret = smi240_soft_config(dev);
	if (ret)
		pr_info("Soft Config failed.");

	return ret;
}

static ssize_t in_temp_accel_anglvel_capture_show(struct device *dev,
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

static ssize_t self_test_show(struct device *dev, struct device_attribute *atrr,
			      char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi240_device *smi240_dev = iio_device_get_drvdata(indio_dev);

	if (smi240_self_test(smi240_dev))
		return snprintf(buf, PAGE_SIZE, "self test fail.\n");
	else
		return snprintf(buf, PAGE_SIZE, "self test success.\n");
}

static ssize_t soft_reset_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int ret;
	bool success = true;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi240_device *smi240_dev = iio_device_get_drvdata(indio_dev);

	ret = smi240_soft_reset(smi240_dev);
	if (ret) {
		dev_err(dev, "Soft reset failed.");
		success = false;
	}

	ret = smi240_init(smi240_dev);
	if (ret) {
		dev_err(dev, "Device initialization failed.");
		success = false;
	}

	if (!success)
		return snprintf(buf, PAGE_SIZE, "soft reset failed.\n");
	else
		return snprintf(buf, PAGE_SIZE, "soft reset performed.\n");
}

static ssize_t sign_of_channels_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	int ret;
	uint16_t data;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi240_device *smi240_dev = iio_device_get_drvdata(indio_dev);

	ret = smi240_get_regs(SMI240_SIGN_SFT_CFG_REG, &data, 1, 0, smi240_dev);
	if (ret)
		return -EIO;

	smi240_dev->sign_of_channels = data;

	return snprintf(
		buf, PAGE_SIZE, "ax:%d,ay:%d,az:%d,gx:%d,gy:%d,gz:%d\n",
		GET_BITS(smi240_dev->sign_of_channels, SMI240_ACC_INVERTX),
		GET_BITS(smi240_dev->sign_of_channels, SMI240_ACC_INVERTY),
		GET_BITS(smi240_dev->sign_of_channels, SMI240_ACC_INVERTZ),
		GET_BITS(smi240_dev->sign_of_channels, SMI240_GYR_INVERTX),
		GET_BITS(smi240_dev->sign_of_channels, SMI240_GYR_INVERTY),
		GET_BITS(smi240_dev->sign_of_channels, SMI240_GYR_INVERTZ));
}

static uint16_t calculate_sign_of_channels(const char *buf,
					   const uint16_t register_val,
					   size_t count)
{
	uint16_t sign_of_channels = register_val;
	char *sep = ",";
	char *config;
	char data[32];

	char *input = data;
	char *sep2 = ":";
	char *value;
	char *channel;

	if (count <= 30) {
		memset(data, 0, sizeof(data));
		strncpy(data, buf, count);

		if (data[strlen(data) - 1] == '\n')
			data[strlen(data) - 1] = '\0';

		config = strsep(&input, sep);
		while (config != NULL) {
			channel = strsep(&config, sep2);
			if (channel != NULL && strcmp(channel, "ax") == 0) {
				value = strsep(&config, sep2);
				if (value != NULL && strcmp(value, "0") == 0) {
					sign_of_channels =
						SET_BITS(sign_of_channels,
							 SMI240_ACC_INVERTX, 0);
				}
				if (value != NULL && strcmp(value, "1") == 0) {
					sign_of_channels =
						SET_BITS(sign_of_channels,
							 SMI240_ACC_INVERTX, 1);
				}
			}
			if (channel != NULL && strcmp(channel, "ay") == 0) {
				value = strsep(&config, sep2);
				if (value != NULL && strcmp(value, "0") == 0) {
					sign_of_channels =
						SET_BITS(sign_of_channels,
							 SMI240_ACC_INVERTY, 0);
				}
				if (value != NULL && strcmp(value, "1") == 0) {
					sign_of_channels =
						SET_BITS(sign_of_channels,
							 SMI240_ACC_INVERTY, 1);
				}
			}
			if (channel != NULL && strcmp(channel, "az") == 0) {
				value = strsep(&config, sep2);
				if (value != NULL && strcmp(value, "0") == 0) {
					sign_of_channels =
						SET_BITS(sign_of_channels,
							 SMI240_ACC_INVERTZ, 0);
				}
				if (value != NULL && strcmp(value, "1") == 0) {
					sign_of_channels =
						SET_BITS(sign_of_channels,
							 SMI240_ACC_INVERTZ, 1);
				}
			}
			if (channel != NULL && strcmp(channel, "gx") == 0) {
				value = strsep(&config, sep2);
				if (value != NULL && strcmp(value, "0") == 0) {
					sign_of_channels =
						SET_BITS(sign_of_channels,
							 SMI240_GYR_INVERTX, 0);
				}
				if (value != NULL && strcmp(value, "1") == 0) {
					sign_of_channels =
						SET_BITS(sign_of_channels,
							 SMI240_GYR_INVERTX, 1);
				}
			}
			if (channel != NULL && strcmp(channel, "gy") == 0) {
				value = strsep(&config, sep2);
				if (value != NULL && strcmp(value, "0") == 0) {
					sign_of_channels =
						SET_BITS(sign_of_channels,
							 SMI240_GYR_INVERTY, 0);
				}
				if (value != NULL && strcmp(value, "1") == 0) {
					sign_of_channels =
						SET_BITS(sign_of_channels,
							 SMI240_GYR_INVERTY, 1);
				}
			}
			if (channel != NULL && strcmp(channel, "gz") == 0) {
				value = strsep(&config, sep2);
				if (value != NULL && strcmp(value, "0") == 0) {
					sign_of_channels =
						SET_BITS(sign_of_channels,
							 SMI240_GYR_INVERTZ, 0);
				}
				if (value != NULL && strcmp(value, "1") == 0) {
					sign_of_channels =
						SET_BITS(sign_of_channels,
							 SMI240_GYR_INVERTZ, 1);
				}
			}
			config = strsep(&input, sep);
		}
	}
	return sign_of_channels;
}

static ssize_t sign_of_channels_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	int ret;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi240_device *smi240_dev = iio_device_get_drvdata(indio_dev);

	if (count > 30)
		return -EINVAL;

	smi240_dev->sign_of_channels = calculate_sign_of_channels(
		buf, smi240_dev->sign_of_channels, count);

	ret = smi240_soft_reset(smi240_dev);
	if (ret) {
		pr_err("Soft Reset failed.");
		return -EIO;
	}

	ret = smi240_soft_config(smi240_dev);
	if (ret) {
		pr_err("Soft Config failed.");
		return -EIO;
	}

	return count;
}

static ssize_t bite_repetitions_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	int ret;
	uint16_t data;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi240_device *smi240_dev = iio_device_get_drvdata(indio_dev);

	ret = smi240_get_regs(SMI240_SOFT_CONFIG_REG, &data, 1, 0, smi240_dev);
	if (ret)
		return -EIO;

	smi240_dev->bite_reps = GET_BITS(data, SMI240_BITE_REP) + 1;

	return snprintf(buf, PAGE_SIZE, "%d\n", smi240_dev->bite_reps);
}

static ssize_t bite_repetitions_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	int ret;
	uint8_t bite_reps;
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct smi240_device *smi240_dev = iio_device_get_drvdata(indio_dev);

	ret = kstrtou8(buf, 10, &bite_reps);
	if (ret || bite_reps < SMI240_MIN_BITE_REPS ||
	    bite_reps > SMI240_MAX_BITE_REPS)
		return -EINVAL;

	smi240_dev->bite_reps = bite_reps;

	ret = smi240_soft_reset(smi240_dev);
	ret |= smi240_soft_config(smi240_dev);
	if (ret)
		return -EIO;

	return count;
}

static IIO_DEVICE_ATTR_RO(in_temp_accel_anglvel_capture, 0);
static IIO_DEVICE_ATTR_RO(self_test, 0);
static IIO_DEVICE_ATTR_RO(soft_reset, 0);
static IIO_DEVICE_ATTR_RW(sign_of_channels, 0);
static IIO_DEVICE_ATTR_RW(bite_repetitions, 0);

static struct attribute *smi240_attrs[] = {
	&iio_dev_attr_in_temp_accel_anglvel_capture.dev_attr.attr,
	&iio_dev_attr_self_test.dev_attr.attr,
	&iio_dev_attr_soft_reset.dev_attr.attr,
	&iio_dev_attr_sign_of_channels.dev_attr.attr,
	&iio_dev_attr_bite_repetitions.dev_attr.attr,
	NULL,
};

static const struct attribute_group smi240_attrs_group = {
	.attrs = smi240_attrs,
};

static const struct iio_info smi240_info = {
	.read_raw_multi = smi240_read_raw_multi,
	.read_avail = smi240_read_avail,
	.write_raw = smi240_write_raw,
	.attrs = &smi240_attrs_group,
};

int smi240_probe(struct device *dev, struct smi240_device *smi240_dev)
{
	int ret;
	int16_t response;
	struct iio_dev *indio_dev;

	ret = smi240_get_regs(SMI240_CHIP_ID_REG, &response, 1, 0, smi240_dev);
	if (ret) {
		pr_err("Read chip id failed.");
		return ret;
	}

	if (response == SMI240_CHIP_ID) {
		pr_info("SMI240 Chip ID: 0x%04x", response);
	} else {
		pr_err("Unexpected Chip ID for SMI240: 0x%04x", response);
		return -ENODEV;
	}

	ret = smi240_soft_reset(smi240_dev);
	if (ret) {
		pr_err("Soft Reset failed.");
		return ret;
	}

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
	dev_info(dev, "unregister SMI240");
	return 0;
}
