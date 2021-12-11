# Telescope-Orientation Syncronisation to Stellarium

Synchronize you telescope's current orientation into [Stellarium](http://stellarium.org/) where you can keep track what you are looking at in real-time and find objects much easier.

## Introduction

This readme will be my notebook during development and should provide detailled instructions to replicate the setup once it's finished.

## Architecture and Design

### Determining telescope orientation

Can be done using a high quality motion sensor (accelerometer, gyroscope and compass)

Name | Pros | Cons
---|---|---
MPU-9250 | | 
ADXL345 | |
ICM-20948 | |
[LSM6DSOX + LIS3MDL](https://github.com/adafruit/Adafruit_LSM6DS) |  |

### Connection to Stellarium Host

Bluetooth or Wifi.

### Interfacing with Stellarium

Can either be done using the [Remote Control Plug-in](http://stellarium.org/doc/head/remoteControlDoc.html) or the [Telescope Control plug-in](http://stellarium.sourceforge.net/wiki/index.php/Telescope_Control_plug-in).

## References

### Similar Projects

- [Control Your Telescope Using Stellarium & Arduino](https://www.instructables.com/Control-Your-Telescope-Using-Stellarium-Arduino/)

### Stellarium

- [RemoteControl plugin HTTP API description ](http://stellarium.org/doc/head/remoteControlApi.html)
- [Telescope Control plug-in](http://stellarium.sourceforge.net/wiki/index.php/Telescope_Control_plug-in)

### Networking

- [curl tutorial](https://curl.se/docs/manual.html)