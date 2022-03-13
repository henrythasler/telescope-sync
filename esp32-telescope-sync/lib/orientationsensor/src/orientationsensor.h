#ifndef ORIENTATIONSENSOR_H
#define ORIENTATIONSENSOR_H

#include <stdint.h>

#ifdef ARDUINO
#include <Adafruit_LSM6DSOX.h>

#define GYRO_AUTOCAL_SAMPLES (500)
#define ACC_FILTER_SAMPLES (50)

class LSM6Wrapper
{
public:
    LSM6Wrapper(int32_t accSensorID = -1, uint8_t accAddress = LSM6DS_I2CADDR_DEFAULT, TwoWire *theWire = &Wire);
    bool begin(void);
    void setCalibration(void);
    void getEvent(sensors_event_t *acc, sensors_event_t *gyr);
    void calibrate(sensors_event_t *acc, sensors_event_t *gyr);
    void clipGyroNoise(sensors_event_t *gyr);
    void gyroSample(float *samples, uint32_t offset);
    void accSample(void);
    bool getMeanAcc(sensors_event_t *accMean);
    bool meanVec3(float *samples, uint32_t num_samples, float *mean);
    void printSensorDetails(void);
    void printSensorOffsets(void);
    bool validateSensorOffsets(void);

    typedef struct
    {
    } SensorStatus;

    SensorStatus acc_gyr;
    sensor_t acc_properties;

    float acc_offset[3] = {0, 0, 0};                     // XYZ vector offsets for Accelerometer, [m/s^2]
    float gyr_offset[3] = {0, 0, 0};                     // XYZ vector offsets for Gyroscope [rad/s]

    int32_t sensorID, sensorAddress;
    int32_t sample_counter = 0;

private:
    Adafruit_I2CDevice *i2c_dev_acc = NULL; ///< Pointer to I2C bus interface

    sensors_event_t gyro_event, acc_event;  

    int32_t accAddress = 0;

    TwoWire *wire = NULL;

    Adafruit_LSM6DSOX lsm6; // Acc+Gyr

    Adafruit_Sensor *accelerometer, *gyroscope;

    int32_t accSensorID = 0;

    float accSamples[3 * ACC_FILTER_SAMPLES];
    int32_t accSampleCounter = 0;
};
#endif // ARDUINO
#endif // ORIENTATIONSENSOR_H