// Arduino base
#include <Arduino.h>

// external Libraries
#include <Adafruit_AHRS.h>

// own libraries
#include <ledmanager.h>
#include <orientationsensor.h>

#define IMU_PIN_VCC (4)
#define IMU_PIN_I2C_SCL (22)
#define IMU_PIN_I2C_SDA (21)
#define IMU_I2C_CLOCK (100000U)

#define IMU_BUS_ID (0)
#define IMU_ACC_ID (1)
#define IMU_MAG_ID (2)

TwoWire I2CBus = TwoWire(IMU_BUS_ID); // set up a new Wire-Instance
OrientationSensor imu(IMU_ACC_ID, IMU_MAG_ID, LSM6DS_I2CADDR_DEFAULT, LIS3MDL_I2CADDR_DEFAULT, &I2CBus);

Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;
sensors_event_t accel, gyro, mag;
float roll = 0, pitch = 0, heading = 0;
float gx = 0, gy = 0, gz = 0;
float qw = 0, qx = 0, qy = 0, qz = 0;

Adafruit_NXPSensorFusion filter; // slowest
// Adafruit_Madgwick filter; // faster than NXP
// Adafruit_Mahony filter;  // fastest/smalleset

#define FILTER_UPDATE_RATE_HZ 100

bool orientationSensorAvailable = false;

LEDManager ledmanager;

// Flow control, basic task scheduler
#define SCHEDULER_MAIN_LOOP_MS (10) // ms
uint32_t timestamp;
uint32_t initStage = 0;

uint32_t task10msTimer = 0;
uint32_t task100msTimer = 0;

void setup()
{
    // LED output
    pinMode(LED_BUILTIN, OUTPUT);
    ledmanager.setMode(LEDManager::LEDMode::ON);
    ledmanager.update();
    initStage++;

    // Setup serial connection for debugging
    Serial.begin(115200U);
    delay(500);
    Serial.println();
    Serial.println("[  INIT  ] Begin");
    initStage++;

    Serial.printf("[  INIT  ] ChipRevision: 0x%02X   CpuFreq: %uMHz   FlashChipSize: %uKiB   HeapSize: %uKiB   SdkVersion: %s\n",
                  ESP.getChipRevision(), ESP.getCpuFreqMHz(), ESP.getFlashChipSize() / 1024, ESP.getHeapSize() / 1024, ESP.getSdkVersion());
    initStage++;

    // Power-On Motion Sensor
    pinMode(IMU_PIN_VCC, OUTPUT);
    digitalWrite(IMU_PIN_VCC, HIGH);
    delay(500); // wait for power up. Takes around 400ms (T_Sup).
    initStage++;

    // Initialize Environment Sensor
    if (I2CBus.begin(IMU_PIN_I2C_SDA, IMU_PIN_I2C_SCL, IMU_I2C_CLOCK))
    {
        initStage++;
        if (imu.begin())
        {
            imu.setCalibration();
            filter.begin(FILTER_UPDATE_RATE_HZ);
            initStage++;
            orientationSensorAvailable = true;
            Serial.println("[  INIT  ] Found LSM6DS33+LIS3MDL Absolute Orientation Sensor Board");
        }
        else
        {
            Serial.println("[! INIT  ] Could not find orientation sensor. Check wiring!");
        }
    }
    else
    {
        Serial.println("[! INIT  ] Could not setup I2C Interface!");
    }

    Serial.printf("[  INIT  ] Completed at stage %u\n\n", initStage);
    ledmanager.setMode(LEDManager::LEDMode::BLINK_10HZ);
}

void loop()
{
    timestamp = millis();

    if ((timestamp - task10msTimer) > SCHEDULER_MAIN_LOOP_MS)
    {
        task10msTimer = timestamp;

        if (orientationSensorAvailable)
        {
            // Read the motion sensors
            imu.getEvent(&accel, &gyro, &mag);

            // Gyroscope needs to be converted from Rad/s to Degree/s
            gx = gyro.gyro.x * SENSORS_RADS_TO_DPS;
            gy = gyro.gyro.y * SENSORS_RADS_TO_DPS;
            gz = gyro.gyro.z * SENSORS_RADS_TO_DPS;

            // Update the SensorFusion filter
            filter.update(gx, gy, gz,
                          accel.acceleration.x, accel.acceleration.y, accel.acceleration.z,
                          mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);

            roll = filter.getRoll();
            pitch = filter.getPitch();
            heading = filter.getYaw();
        }

        ledmanager.update();
        return;
    }

    // 100ms Tasks
    if ((timestamp - task100msTimer) > 100)
    {
        task100msTimer = timestamp;

        // send data for SerialStudio
        Serial.printf("/*%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f*/\n",
                      heading, pitch, roll,
                      qw, qx, qy, qz,
                      accel.acceleration.x, accel.acceleration.y, accel.acceleration.z,
                      gyro.gyro.x, gyro.gyro.y, gyro.gyro.z,
                      mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);
    }
}
