# Telescope-Orientation Syncronisation to Stellarium

Synchronize you telescope's current orientation into [Stellarium](http://stellarium.org/) where you can keep track what you are looking at in real-time and find objects much easier.

This readme will be my notebook during development and should provide detailled instructions to replicate the setup once it's finished.

## Introduction

To achieve the goal of finding an object in the sky more easily, we need to link the telescope's current orientation to a star chart where we can compare the current orientation with the orientation needed to  

## Determining telescope orientation

Can be done using a high quality motion sensor (accelerometer, gyroscope and compass) or rotary encoder mounted on top of the two axes of rotation (az/alt). To avoid additional mechanical work, I chose the motion sensor approach.

After some research I selected the following list of possible candidates:

Rank | Name | Pros | Cons
---|---|---|---
1 | [BNO085](https://learn.adafruit.com/adafruit-9-dof-orientation-imu-fusion-breakout-bno085/) | Integrated Sensor Fusion | Out of Stock
2 | [BNO055](https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/overview) |  Integrated Sensor Fusion | 
3 | [NXP 9-DOF](https://learn.adafruit.com/nxp-precision-9dof-breakout) |  | Out of Stock
4 | [ICM-20948](https://learn.adafruit.com/adafruit-tdk-invensense-icm-20948-9-dof-imu) | ok-ish Zero-rate Offset | 
5 | [LSM6DSOX + LIS3MDL](https://github.com/adafruit/Adafruit_LSM6DS) |  | Out of Stock
6 | MPU-9250 | | High Zero-rate Offset

Stellarium expects the right ascension and declination of the telescope as input. To compute these values, we need the exact location and a timestamp. A simple GPS Receiver ([GP-20U7](docs/GP-20U7.pdf)) should provide this information with sufficient precision.

## Host-Controller

Either [ESP8266](http://esp8266.net/) or [ESP32](http://esp32.net/) where the ESP32 provides additional Bluetooth capabilities.

## Interfacing with Stellarium

Bluetooth or Wifi.

Can either be done using the [Remote Control Plug-in](http://stellarium.org/doc/head/remoteControlDoc.html) or the [Telescope Control plug-in](http://stellarium.sourceforge.net/wiki/index.php/Telescope_Control_plug-in).

Messaging via the [Stellarium Telescope Protocol](http://svn.code.sf.net/p/stellarium/code/trunk/telescope_server/stellarium_telescope_protocol.txt):

Name | Offset (Bytes) | Length (Bytes) | Type | content
---|---|---|---|---
LENGTH | 0 | 2 | int16 | length of the message including the LENGTH field
TYPE | 2 | 2 | int16 | always 0
TIME | 4 | 8 | int64 | server timestamp in microseconds since epoch
RA | 12 | 4 | int32 | right ascension of the telescope (J2000); 0x0 = 0h = 24h; 0x80000000 = 12h
DEC | 16 | 4 | int32 | declination of the telescope (J2000); 0x0 = 0°; 0x40000000 = +90°; -0x40000000 = 0xFFFFC0000000 = -90°
STATUS | 20 | 4 | int32 | status of the telescope, currently unused; 0=ok

Byte-order is little-endian. Least significiant bytes are stored first.

## References

### Similar Projects

- [Control Your Telescope Using Stellarium & Arduino](https://www.instructables.com/Control-Your-Telescope-Using-Stellarium-Arduino/)
- [Arduino Star-Finder for Telescopes](https://www.instructables.com/Arduino-Star-Finder-for-Telescopes/)
- [SpotNik - StepTo / PushTo / Digital Setting Circles for EQ Mounted Telescopes Based on Arduino](https://www.instructables.com/SpotNik-StepTo-PushTo-Digital-Setting-Circles-for-/)

### Hardware

- [Comparing Gyroscope Datasheets](https://learn.adafruit.com/comparing-gyroscope-datasheets)

### Stellarium

- [RemoteControl plugin HTTP API description ](http://stellarium.org/doc/head/remoteControlApi.html)
- [Telescope Control plug-in](http://stellarium.sourceforge.net/wiki/index.php/Telescope_Control_plug-in)

### Networking

- [curl tutorial](https://curl.se/docs/manual.html)