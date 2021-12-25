#include <bnodata.h>

BnoData::BnoData(int32_t id, uint8_t address)
{
    sensorID = id;
    sensorAddress = address;
}

void BnoData::printSensorOffsets()
{
    Serial.printf("[ SENSOR ] Calibration data: Accel=[%i, %i, %i], Gyro=[%i, %i, %i], Mag=[%i, %i, %i], Accel Radius=%u, Mag Radius=%i\n",
                  calibrationData.accel_offset_x, calibrationData.accel_offset_y, calibrationData.accel_offset_z,
                  calibrationData.gyro_offset_x, calibrationData.gyro_offset_y, calibrationData.gyro_offset_z,
                  calibrationData.mag_offset_x, calibrationData.mag_offset_y, calibrationData.mag_offset_z,
                  calibrationData.accel_radius, calibrationData.mag_radius);
}