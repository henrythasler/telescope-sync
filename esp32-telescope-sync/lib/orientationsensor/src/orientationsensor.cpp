#include <orientationsensor.h>

#ifdef ARDUINO
LSM6Wrapper::LSM6Wrapper(int32_t accSensorID, uint8_t accAddress, TwoWire *theWire)
{
    this->accSensorID = accSensorID;
    this->accAddress = accAddress;

    this->wire = theWire;
}

bool LSM6Wrapper::begin(void)
{
    if (!this->lsm6.begin_I2C(accAddress, wire, accSensorID))
    {
        return false;
    }

    lsm6.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
    lsm6.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);

    lsm6.setAccelDataRate(LSM6DS_RATE_416_HZ);
    lsm6.setGyroDataRate(LSM6DS_RATE_416_HZ);

    accelerometer = lsm6.getAccelerometerSensor();
    gyroscope = lsm6.getGyroSensor();
    return true;
}

void LSM6Wrapper::setCalibration(void)
{
    // rad/s
    gyr_offset[0] = 0;
    gyr_offset[1] = 0;
    gyr_offset[2] = 0;

    // m/s^2
    acc_offset[0] = 0.;
    acc_offset[1] = 0.19;
    acc_offset[2] = -0.18;
}

void LSM6Wrapper::getEvent(sensors_event_t *acc, sensors_event_t *gyr)
{
    accelerometer->getEvent(acc);
    gyroscope->getEvent(gyr);
}

void LSM6Wrapper::calibrate(sensors_event_t *acc, sensors_event_t *gyr)
{
    gyr->gyro.x -= gyr_offset[0];
    gyr->gyro.y -= gyr_offset[1];
    gyr->gyro.z -= gyr_offset[2];

    acc->acceleration.x -= acc_offset[0];
    acc->acceleration.y -= acc_offset[1];
    acc->acceleration.z -= acc_offset[2];
}

void LSM6Wrapper::clipGyroNoise(sensors_event_t *gyr)
{
    // clip noise
    gyr->gyro.x = abs(gyr->gyro.x) > 0.01 ? gyr->gyro.x : 0.;
    gyr->gyro.y = abs(gyr->gyro.y) > 0.01 ? gyr->gyro.y : 0.;
    gyr->gyro.z = abs(gyr->gyro.z) > 0.01 ? gyr->gyro.z : 0.;
}

void LSM6Wrapper::printSensorDetails()
{
    accelerometer->printSensorDetails();
    gyroscope->printSensorDetails();
}

void LSM6Wrapper::gyroSample(float *samples, uint32_t offset)
{
    gyroscope->getEvent(&gyro_event);
    samples[offset * 3] = gyro_event.gyro.x;
    samples[offset * 3 +1] = gyro_event.gyro.y;
    samples[offset * 3 + 2] = gyro_event.gyro.z;
}


void LSM6Wrapper::accSample()
{
    accelerometer->getEvent(&acc_event);
    this->accSamples[accSampleCounter * 3] = acc_event.acceleration.x;
    this->accSamples[accSampleCounter * 3 +1] = acc_event.acceleration.y;
    this->accSamples[accSampleCounter * 3 + 2] = acc_event.acceleration.z;
    accSampleCounter = (accSampleCounter + 1) % ACC_FILTER_SAMPLES;
}


bool LSM6Wrapper::getMeanAcc(sensors_event_t *accMean)
{
    float mean[3] = {0, 0, 0};
    this->meanVec3(this->accSamples, ACC_FILTER_SAMPLES, mean);
    accMean->acceleration.x = mean[0];
    accMean->acceleration.y = mean[1];
    accMean->acceleration.z = mean[2];
    return true;
}


bool LSM6Wrapper::meanVec3(float *samples, uint32_t num_samples, float *mean)
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

void LSM6Wrapper::printSensorOffsets()
{
    // Serial.printf("[ SENSOR ] Calibration data: Accel=[%i, %i, %i], Gyro=[%i, %i, %i], Mag=[%i, %i, %i], Accel Radius=%u, Mag Radius=%i\n",
    //               calibrationData.accel_offset_x, calibrationData.accel_offset_y, calibrationData.accel_offset_z,
    //               calibrationData.gyro_offset_x, calibrationData.gyro_offset_y, calibrationData.gyro_offset_z,
    //               calibrationData.mag_offset_x, calibrationData.mag_offset_y, calibrationData.mag_offset_z,
    //               calibrationData.accel_radius, calibrationData.mag_radius);
}

#endif