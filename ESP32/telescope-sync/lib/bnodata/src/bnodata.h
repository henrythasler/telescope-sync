#ifndef BNODATA_H
#define BNODATA_H

using namespace std;

#include <stdint.h>

#ifdef ARDUINO
#include <Adafruit_BNO055.h>

class BnoData
{
public:
    BnoData(int32_t sensorID = -1, uint8_t address = BNO055_ADDRESS_A);
    void printSensorOffsets(void);
    bool validateSensorOffsets(void);

    typedef struct 
    {
        // System Status
        uint8_t statSystem = 0;   // SYS_STAT
        uint8_t statSelfTest = 0; // ST_RESULT
        uint8_t errSystem = 0;    // SYS_ERR

        // Calibration Status via CALIB_STAT register
        uint8_t calSystem = 0;
        uint8_t calGyro = 0;
        uint8_t calAccel = 0;
        uint8_t calMag = 0;
        bool fullyCalibrated = false;
        bool partlyCalibrated = false;
        bool calibrationDataAvailable = false;
        bool calibrationDataSaved = false;
    } BnoStatus;

    BnoStatus status;
    adafruit_bno055_offsets_t calibrationData;

    int32_t sensorID, sensorAddress;

private:    
};
#endif
#endif