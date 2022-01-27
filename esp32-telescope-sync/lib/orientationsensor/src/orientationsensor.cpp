#include <orientationsensor.h>

#ifdef ARDUINO
OrientationSensor::OrientationSensor(int32_t accSensorID, int32_t magSensorID, uint8_t accAddress, uint8_t magAddress, TwoWire *theWire)
{
    this->accSensorID = accSensorID;
    this->magSensorID = magSensorID;

    this->accAddress = accAddress;
    this->magAddress = magAddress;

    this->wire = theWire;
}

bool OrientationSensor::begin(void)
{
    if (!this->lsm6ds.begin_I2C(accAddress, wire, accSensorID) || !this->lis3mdl.begin_I2C(magAddress, wire))
    {
        return false;
    }

    lsm6ds.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
    lsm6ds.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
    lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);

    lsm6ds.setAccelDataRate(LSM6DS_RATE_104_HZ);
    lsm6ds.setGyroDataRate(LSM6DS_RATE_104_HZ);
    lis3mdl.setDataRate(LIS3MDL_DATARATE_1000_HZ);
    lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
    lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);

    accelerometer = lsm6ds.getAccelerometerSensor();
    gyroscope = lsm6ds.getGyroSensor();
    magnetometer = &lis3mdl;
}

void OrientationSensor::getEvent(sensors_event_t *acc, sensors_event_t *gyr, sensors_event_t *mag)
{
    accelerometer->getEvent(acc);
    gyroscope->getEvent(gyr);
    magnetometer->getEvent(mag);    
}

void OrientationSensor::printSensorDetails()
{
    accelerometer->printSensorDetails();
    gyroscope->printSensorDetails();
    magnetometer->printSensorDetails();
}

void OrientationSensor::printSensorOffsets()
{
    // Serial.printf("[ SENSOR ] Calibration data: Accel=[%i, %i, %i], Gyro=[%i, %i, %i], Mag=[%i, %i, %i], Accel Radius=%u, Mag Radius=%i\n",
    //               calibrationData.accel_offset_x, calibrationData.accel_offset_y, calibrationData.accel_offset_z,
    //               calibrationData.gyro_offset_x, calibrationData.gyro_offset_y, calibrationData.gyro_offset_z,
    //               calibrationData.mag_offset_x, calibrationData.mag_offset_y, calibrationData.mag_offset_z,
    //               calibrationData.accel_radius, calibrationData.mag_radius);
}

bool OrientationSensor::validateSensorOffsets()
{
    // return( (calibrationData.mag_radius >= 144) &&
    // (calibrationData.mag_radius <= 1280)
    // );
}

#endif