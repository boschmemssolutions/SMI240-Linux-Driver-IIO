# SMI240 IIO Sensor-API and Sensor Driver

## Table of Contents
 - [Introduction](#Intro)
 - [License](#License)
 - [Sensor interfaces](#interfaces)
 - [Architecture](#Architecture)
 - [Operation examples](#examples)

## Introduction <a name=Intro></a>

The SMI240 is a combined three axis angular rate and three axis acceleration sensor module with a measurement range of +/-300°/s and up to 16g. SMI240 is a 6 DoF (Degrees of Freedom) sensor module providing acceleration and angular rate signals via a digital interface (SPI). 

## Documentation <a name=Doc></a>

https://boschmemssolutions.github.io/iio/bosch_smi240_IIO.html

## License <a name=License></a>

See [LICENSE](drivers/iio/imu/smi240/LICENSE.md) file

## Sensor interfaces <a name=interfaces></a>
* SPI

## Architecture <a name=Architecture></a>
```
                  User space
-------------------------------------------------------
                 |          |
               sysfs       dev
                 \          /
                IMU-subsystem
	             |
                iio-subsystem
	             |
               smi240_driver --> smi240_SPI
                                       |
                                    SPI_bus
                                       |
-------------------------------------------------------
                  Hardware
```
## Operation examples <a name=examples></a>
1. Userspace
The driver exposes device file node under /dev/iio_device:0

2. Sysfs
The driver also exposes a set of sysfs nodes under /sys/bus/iio/devices/iio_device:0, where users can get information about the sensor and also control the sensor. Eg.:
```
# device number 0 is dynamically assingn to smi240, it can change if there are other iio devices available in the same system. 
# to find out which deveice you are working with, you can read the device name
cat /sys/bus/iio/devices/iio:device0/name

# read acc raw value
cat  /sys/bus/iio/devices/iio:device0/in_accel_x\&y\&z_raw

# read gyro raw value
cat  /sys/bus/iio/devices/iio:device0/in_anglvel_x\&y\&z_raw

# read temprature
cat  /sys/bus/iio/devices/iio:device0/in_temp_object_raw

# Synchronisation of sensor data is done by trigger and capture mechanism. Sensor measurement data and channel status of all channels (LF and HF) on one chip select line can be captured (stored sensor internally) at one point in time and read out at a later point in time
# read capture data
cat /sys/bus/iio/devices/iio:device0/in_temp_accel_anglvel_capture
