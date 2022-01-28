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

void OrientationSensor::setCalibration(void)
{
    // µT
    mag_hardiron[0] = -64.86967343;
    mag_hardiron[1] = 17.48569469;
    mag_hardiron[2] = -0.59624021;

    mag_softiron[0] = 1.00377056e+00;
    mag_softiron[1] = 5.27418179e-02;
    mag_softiron[2] = -9.83060614e-03;
    mag_softiron[3] = 5.27418179e-02;
    mag_softiron[4] = 9.93287432e-01;
    mag_softiron[5] = 2.64077721e-05;
    mag_softiron[6] = -9.83060614e-03;
    mag_softiron[7] = 2.64077721e-05;
    mag_softiron[8] = 1.00003403e+00;

    // rad/s
    gyr_offset[0] = 0.0651;
    gyr_offset[1] = -0.1081;
    gyr_offset[2] = -0.08;

    // m/s^2
    acc_offset[0] = 0.01452;
    acc_offset[1] = -0.01221;
    acc_offset[2] = -0.2102;
}

void OrientationSensor::getEvent(sensors_event_t *acc, sensors_event_t *gyr, sensors_event_t *mag)
{
    accelerometer->getEvent(acc);
    gyroscope->getEvent(gyr);
    magnetometer->getEvent(mag);
}

void OrientationSensor::calibrate(sensors_event_t *acc, sensors_event_t *gyr, sensors_event_t *mag)
{
    // hard iron cal
    float mx = mag->magnetic.x - mag_hardiron[0];
    float my = mag->magnetic.y - mag_hardiron[1];
    float mz = mag->magnetic.z - mag_hardiron[2];
    // soft iron cal
    mag->magnetic.x = mx * mag_softiron[0] + my * mag_softiron[1] + mz * mag_softiron[2];
    mag->magnetic.y = mx * mag_softiron[3] + my * mag_softiron[4] + mz * mag_softiron[5];
    mag->magnetic.z = mx * mag_softiron[6] + my * mag_softiron[7] + mz * mag_softiron[8];

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
    gyr->gyro.x = abs(gyr->gyro.x) > 0.005 ? gyr->gyro.x : 0.;
    gyr->gyro.y = abs(gyr->gyro.y) > 0.005 ? gyr->gyro.y : 0.;
    gyr->gyro.z = abs(gyr->gyro.z) > 0.005 ? gyr->gyro.z : 0.;
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