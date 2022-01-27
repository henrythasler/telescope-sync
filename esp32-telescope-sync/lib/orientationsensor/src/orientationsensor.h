#ifndef ORIENTATIONSENSOR_H
#define ORIENTATIONSENSOR_H

using namespace std;

#include <stdint.h>

#ifdef ARDUINO
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LSM6DS33.h>

class OrientationSensor
{
public:
    OrientationSensor(int32_t accSensorID = -1,
                      int32_t magSensorID = -1,
                      uint8_t accAddress = LSM6DS_I2CADDR_DEFAULT,
                      uint8_t magAddress = LIS3MDL_I2CADDR_DEFAULT,
                      TwoWire *theWire = &Wire);
    bool begin(void);
    void getEvent(sensors_event_t *acc, sensors_event_t *gyr, sensors_event_t *mag);
    void printSensorDetails(void);
    void printSensorOffsets(void);
    bool validateSensorOffsets(void);

    typedef struct
    {
    } SensorStatus;

    SensorStatus acc_gyr, mag;
    sensor_t acc_properties, mag_properties;

    /**! XYZ vector of offsets for zero-g, in m/s^2 */
    float acc_offset[3] = {0, 0, 0};

    /**! XYZ vector of offsets for zero-rate, in rad/s */
    float gyr_offset[3] = {0, 0, 0};

    /**! XYZ vector of offsets for hard iron calibration (in uT) */
    float mag_hardiron[3] = {0, 0, 0};

    /**! The 3x3 matrix for soft-iron calibration (unitless) */
    float mag_softiron[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};

    /**! The magnetic field magnitude in uTesla */
    float mag_field = 50;

    int32_t sensorID, sensorAddress;

private:
    Adafruit_I2CDevice *i2c_dev_acc = NULL, *i2c_dev_mag = NULL; ///< Pointer to I2C bus interface

    int32_t accAddress = 0;
    int32_t magAddress = 0;

    TwoWire *wire = NULL;

    Adafruit_LSM6DS33 lsm6ds;   // Acc+Gyr
    Adafruit_LIS3MDL lis3mdl;   // Mag

    Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;    

    int32_t accSensorID = 0;
    int32_t magSensorID = 0;
};
#endif // ARDUINO
#endif // ORIENTATIONSENSOR_H