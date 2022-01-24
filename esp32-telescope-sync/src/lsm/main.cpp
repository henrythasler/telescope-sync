// Full orientation sensing using NXP/Madgwick/Mahony and a range of 9-DoF
// sensor sets.
// You *must* perform a magnetic calibration before this code will work.
//
// To view this data, use the Arduino Serial Monitor to watch the
// scrolling angles, or run the OrientationVisualiser example in Processing.
// Based on  https://github.com/PaulStoffregen/NXPMotionSense with adjustments
// to Adafruit Unified Sensor interface

#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>

Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;

#include "LSM6DS_LIS3MDL.h"

// pick your filter! slower == better quality output
Adafruit_NXPSensorFusion filter; // slowest
// Adafruit_Madgwick filter; // faster than NXP
// Adafruit_Mahony filter;  // fastest/smalleset

#if defined(ADAFRUIT_SENSOR_CALIBRATION_USE_EEPROM)
Adafruit_Sensor_Calibration_EEPROM cal;
#else
Adafruit_Sensor_Calibration_SDFat cal;
#endif

#define FILTER_UPDATE_RATE_HZ 100
#define PRINT_EVERY_N_UPDATES 10

#define IMU_PIN_VCC (4)

uint32_t timestamp;
sensors_event_t accel, gyro, mag;

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        yield();

    if (!cal.begin())
    {
        Serial.println("Failed to initialize calibration helper");
    }
    // else if (!cal.loadCalibration())
    // {
    //     Serial.println("No calibration loaded/found");
    // }

    // µT
    cal.mag_hardiron[0] = -64.86967343;
    cal.mag_hardiron[1] = 17.48569469;
    cal.mag_hardiron[2] = -0.59624021;

    cal.mag_softiron[0] = 1;
    cal.mag_softiron[1] = 0;
    cal.mag_softiron[2] = 0;
    cal.mag_softiron[3] = 0;
    cal.mag_softiron[4] = 1;
    cal.mag_softiron[5] = 0;
    cal.mag_softiron[6] = 0;
    cal.mag_softiron[7] = 0;
    cal.mag_softiron[8] = 1;

    // rad/s
    cal.gyro_zerorate[0] = 0.0651;
    cal.gyro_zerorate[1] = -0.1081;
    cal.gyro_zerorate[2] = -0.08;
  
    // m/s^2
    cal.accel_zerog[0] = 0.01452;
    cal.accel_zerog[1] = -0.01221;
    cal.accel_zerog[2] = -0.2102;

    pinMode(IMU_PIN_VCC, OUTPUT);
    digitalWrite(IMU_PIN_VCC, HIGH);
    delay(500); // wait for power up. Takes around 400ms (T_Sup).

    if (!init_sensors())
    {
        Serial.println("Failed to find sensors");
        while (1)
            delay(10);
    }

    accelerometer->printSensorDetails();
    gyroscope->printSensorDetails();
    magnetometer->printSensorDetails();

    setup_sensors();
    filter.begin(FILTER_UPDATE_RATE_HZ);
    timestamp = millis();

    Wire.setClock(400000); // 400KHz
}

void loop()
{
    float roll, pitch, heading;
    float gx, gy, gz;
    static uint8_t counter = 0;

    if ((millis() - timestamp) < (1000 / FILTER_UPDATE_RATE_HZ))
    {
        return;
    }
    timestamp = millis();
    // Read the motion sensors
    accelerometer->getEvent(&accel);
    gyroscope->getEvent(&gyro);
    magnetometer->getEvent(&mag);

    cal.calibrate(mag);
    cal.calibrate(accel);
    cal.calibrate(gyro);

    // Gyroscope needs to be converted from Rad/s to Degree/s
    // the rest are not unit-important
    gx = gyro.gyro.x * SENSORS_RADS_TO_DPS;
    gy = gyro.gyro.y * SENSORS_RADS_TO_DPS;
    gz = gyro.gyro.z * SENSORS_RADS_TO_DPS;

    // Update the SensorFusion filter
    filter.update(gx, gy, gz,
                  accel.acceleration.x, accel.acceleration.y, accel.acceleration.z,
                  mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);

    // only print the calculated output once in a while
    if (counter++ <= PRINT_EVERY_N_UPDATES)
    {
        return;
    }
    // reset the counter
    counter = 0;

    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();

    float qw, qx, qy, qz;
    filter.getQuaternion(&qw, &qx, &qy, &qz);
    Serial.printf("/*%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f*/\n",
                  heading, pitch, roll,
                  qw, qx, qy, qz,
                  accel.acceleration.x, accel.acceleration.y, accel.acceleration.z,
                  gyro.gyro.x, gyro.gyro.y, gyro.gyro.z,
                  mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);
}
