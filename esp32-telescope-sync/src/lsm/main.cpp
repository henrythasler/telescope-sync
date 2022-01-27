// Full orientation sensing using NXP/Madgwick/Mahony and a range of 9-DoF
// sensor sets.
// You *must* perform a magnetic calibration before this code will work.
//
// To view this data, use the Arduino Serial Monitor to watch the
// scrolling angles, or run the OrientationVisualiser example in Processing.
// Based on  https://github.com/PaulStoffregen/NXPMotionSense with adjustments
// to Adafruit Unified Sensor interface

#include <Arduino.h>

#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>

#include <orientationsensor.h>

#define IMU_PIN_VCC (4)
#define IMU_PIN_I2C_SCL (22)
#define IMU_PIN_I2C_SDA (21)
#define IMU_I2C_CLOCK (100000U)

#define IMU_BUS_ID (0)
#define IMU_ACC_ID (1)
#define IMU_MAG_ID (2)

TwoWire I2CBus = TwoWire(IMU_BUS_ID); // set up a new Wire-Instance for BNO055 Intelligent Absolute Orientation Sensor
OrientationSensor imu(IMU_ACC_ID, IMU_MAG_ID, LSM6DS_I2CADDR_DEFAULT, LIS3MDL_I2CADDR_DEFAULT, &I2CBus);

Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;

// pick your filter! slower == better quality output
Adafruit_NXPSensorFusion filter; // slowest
// Adafruit_Madgwick filter; // faster than NXP
// Adafruit_Mahony filter;  // fastest/smalleset

Adafruit_Sensor_Calibration_EEPROM cal;

#define FILTER_UPDATE_RATE_HZ 80
#define PRINT_EVERY_N_UPDATES 10

uint32_t timestamp;
sensors_event_t accel, gyro, mag;

float roll = 0, pitch = 0, heading = 0;
float gx = 0, gy = 0, gz = 0;

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

    cal.mag_softiron[0] = 1.00377056e+00;
    cal.mag_softiron[1] = 5.27418179e-02;
    cal.mag_softiron[2] = -9.83060614e-03;
    cal.mag_softiron[3] = 5.27418179e-02;
    cal.mag_softiron[4] = 9.93287432e-01;
    cal.mag_softiron[5] = 2.64077721e-05;
    cal.mag_softiron[6] = -9.83060614e-03;
    cal.mag_softiron[7] = 2.64077721e-05;
    cal.mag_softiron[8] = 1.00003403e+00;

    // cal.mag_softiron[0] = 1;
    // cal.mag_softiron[1] = 0;
    // cal.mag_softiron[2] = 0;
    // cal.mag_softiron[3] = 0;
    // cal.mag_softiron[4] = 1;
    // cal.mag_softiron[5] = 0;
    // cal.mag_softiron[6] = 0;
    // cal.mag_softiron[7] = 0;
    // cal.mag_softiron[8] = 1;

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

    // Initialize Environment Sensor
    if (I2CBus.begin(IMU_PIN_I2C_SDA, IMU_PIN_I2C_SCL, IMU_I2C_CLOCK))
    {
        if (imu.begin())
        {
            Serial.println("[  INIT  ] Found Orientation Sensor");
        }
        else
        {
            Serial.println("[! INIT  ] Could not find a sensor. Check wiring!");
        }
    }
    else
    {
        Serial.println("[! INIT  ] Could not setup I2C Interface!");
    }

    imu.printSensorDetails();

    filter.begin(FILTER_UPDATE_RATE_HZ);

    timestamp = millis();
}

static uint8_t counter = 0;

void loop()
{

    uint32_t dt = millis() - timestamp;

    if (dt < (1000 / FILTER_UPDATE_RATE_HZ))
    {
        return;
    }
    timestamp = millis();

    // Read the motion sensors
    imu.getEvent(&accel, &gyro, &mag);

    cal.calibrate(mag);
    cal.calibrate(accel);
    cal.calibrate(gyro);

    // clip noise
    gyro.gyro.x = abs(gyro.gyro.x) > 0.005 ? gyro.gyro.x : 0.;
    gyro.gyro.y = abs(gyro.gyro.y) > 0.005 ? gyro.gyro.y : 0.;
    gyro.gyro.z = abs(gyro.gyro.z) > 0.005 ? gyro.gyro.z : 0.;

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
