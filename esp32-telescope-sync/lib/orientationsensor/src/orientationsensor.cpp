#include <orientationsensor.h>

#ifdef ARDUINO
OrientationSensor::OrientationSensor(int32_t accSensorID, uint8_t accAddress, TwoWire *theWire)
{
    this->accSensorID = accSensorID;
    this->accAddress = accAddress;

    this->wire = theWire;
}

bool OrientationSensor::begin(void)
{
    if (!this->lsm6ds.begin_I2C(accAddress, wire, accSensorID))
    {
        return false;
    }

    lsm6ds.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
    lsm6ds.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);

    lsm6ds.setAccelDataRate(LSM6DS_RATE_416_HZ);
    lsm6ds.setGyroDataRate(LSM6DS_RATE_416_HZ);

    accelerometer = lsm6ds.getAccelerometerSensor();
    gyroscope = lsm6ds.getGyroSensor();
    return true;
}

void OrientationSensor::setCalibration(void)
{
    // rad/s
    gyr_offset[0] = 0;
    gyr_offset[1] = 0;
    gyr_offset[2] = 0;

    // m/s^2
    acc_offset[0] = 0;
    acc_offset[1] = 0;
    acc_offset[2] = 0;
}

void OrientationSensor::getEvent(sensors_event_t *acc, sensors_event_t *gyr)
{
    accelerometer->getEvent(acc);
    gyroscope->getEvent(gyr);
}

void OrientationSensor::calibrate(sensors_event_t *acc, sensors_event_t *gyr)
{
    gyr->gyro.x -= gyr_offset[0];
    gyr->gyro.y -= gyr_offset[1];
    gyr->gyro.z -= gyr_offset[2];

    acc->acceleration.x -= acc_offset[0];
    acc->acceleration.y -= acc_offset[1];
    acc->acceleration.z -= acc_offset[2];
}

void OrientationSensor::clipGyroNoise(sensors_event_t *gyr)
{
    // clip noise
    gyr->gyro.x = abs(gyr->gyro.x) > 0.01 ? gyr->gyro.x : 0.;
    gyr->gyro.y = abs(gyr->gyro.y) > 0.01 ? gyr->gyro.y : 0.;
    gyr->gyro.z = abs(gyr->gyro.z) > 0.01 ? gyr->gyro.z : 0.;
}

void OrientationSensor::printSensorDetails()
{
    accelerometer->printSensorDetails();
    gyroscope->printSensorDetails();
}

void OrientationSensor::gyroSample(float *samples, uint32_t offset)
{
    gyroscope->getEvent(&gyro_event);
    samples[offset * 3] = gyro_event.gyro.x;
    samples[offset * 3 +1] = gyro_event.gyro.y;
    samples[offset * 3 + 2] = gyro_event.gyro.z;
}

bool OrientationSensor::meanVec3(float *samples, uint32_t num_samples, float *mean)
{
    float sum_x = 0, sum_y = 0,sum_z = 0;
    for (int32_t i = 0; i < num_samples; i++)
    {
        sum_x += samples[3 * i];
        sum_y += samples[3 * i + 1];
        sum_z += samples[3 * i + 2];
    }

    mean[0] = sum_x / num_samples;
    mean[1] = sum_y / num_samples;
    mean[2] = sum_z / num_samples;
    return true;
}

void OrientationSensor::printSensorOffsets()
{
    // Serial.printf("[ SENSOR ] Calibration data: Accel=[%i, %i, %i], Gyro=[%i, %i, %i], Mag=[%i, %i, %i], Accel Radius=%u, Mag Radius=%i\n",
    //               calibrationData.accel_offset_x, calibrationData.accel_offset_y, calibrationData.accel_offset_z,
    //               calibrationData.gyro_offset_x, calibrationData.gyro_offset_y, calibrationData.gyro_offset_z,
    //               calibrationData.mag_offset_x, calibrationData.mag_offset_y, calibrationData.mag_offset_z,
    //               calibrationData.accel_radius, calibrationData.mag_radius);
}

#endif