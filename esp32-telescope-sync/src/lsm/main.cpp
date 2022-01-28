// Full orientation sensing using NXP/Madgwick/Mahony and a range of 9-DoF
// sensor sets.
// You *must* perform a magnetic calibration before this code will work.
//
// To view this data, use the Arduino Serial Monitor to watch the
// scrolling angles, or run the OrientationVisualiser example in Processing.
// Based on  https://github.com/PaulStoffregen/NXPMotionSense with adjustments
// to Adafruit Unified Sensor interface

#include <Arduino.h>
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

    pinMode(IMU_PIN_VCC, OUTPUT);
    digitalWrite(IMU_PIN_VCC, HIGH);
    delay(500); // wait for power up. Takes around 400ms (T_Sup).

    // Initialize Environment Sensor
    if (I2CBus.begin(IMU_PIN_I2C_SDA, IMU_PIN_I2C_SCL, IMU_I2C_CLOCK))
    {
        if (imu.begin())
        {
            imu.setCalibration();
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
    imu.calibrate(&accel, &gyro, &mag);
    imu.clipGyroNoise(&gyro);

    // Gyroscope needs to be converted from Rad/s to Degree/s
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
