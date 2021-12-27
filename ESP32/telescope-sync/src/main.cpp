#include <Arduino.h>
#include <WiFi.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SPIFFS.h>
// #include <AsyncTCP.h>

// own libraries
#include <persistency.h>
#include <bnodata.h>
#include <telescope.h>

/**
 * A file called 'secrets.h' must be placed in lib/secrets and contain the following content:
 * 
 * #ifndef SECRETS_H
 * #define SECRETS_H
 * struct Secrets
 * {
 *     const char *wifiSsid = "test";   // WiFi AP-Name
 *     const char *wifiPassword = "1234";
 * } secrets;
 * #endif
 * 
 **/
#include <secrets.h>

#define DEBUG (true)

#define BNO055_PIN_I2C_SCL (22)
#define BNO055_PIN_I2C_SDA (21)
#define BNO055_BUS_ID (0)
#define BNO055_PIN_VCC (4)

TwoWire I2CBus = TwoWire(BNO055_BUS_ID);                                     // set up a new Wire-Instance for BNO055 Intelligent Absolute Orientation Sensor
Adafruit_BNO055 bno = Adafruit_BNO055(BNO055_ID, BNO055_ADDRESS_A, &I2CBus); // use custom I2C-Instance
BnoData bnoData(BNO055_ID, BNO055_ADDRESS_A);

Persistency persistency;

bool orientationSensorAvailable = false;
bool gnssModuleAvailable = false;
bool filesystemAvailable = false;

#define TCP_SERVER_PORT (10001)
// AsyncServer server(TCP_SERVER_PORT);
WiFiServer server(TCP_SERVER_PORT);
WiFiClient remoteClient;

// Flow control, basic task scheduler
#define SCHEDULER_MAIN_LOOP_MS (10) // ms
uint32_t counterBase = 0;
uint32_t counter2s = 0;
uint32_t counter300s = 0;
uint32_t counter1h = 0;
uint32_t initStage = 0;

uint32_t calibrationStable = 0;

Telescope telescope(101.298, -16.724);

// uint8_t txBuffer[32] = {0x18, 0x00, 0x00, 0x00, 0x40, 0x7b, 0x0b, 0x16, 0xe5, 0xd3, 0x05, 0x00, 0xc3, 0xdd, 0x78, 0x29, 0xa8, 0x8b, 0x79, 0x0c, 0x00, 0x00, 0x00, 0x00};
uint8_t txBuffer[32] = {0x18, 0x00, 0x00, 0x00, 0xde, 0xad, 0xbe, 0xef, 0x00, 0x00, 0x00, 0x00, 0x74, 0x24, 0x07, 0x48, 0x3a, 0x7d, 0x1b, 0xf4, 0x00, 0x00, 0x00, 0x00};
uint8_t rxBuffer[32];

void setup()
{
    // LED output
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    initStage++;

    // Setup serial connection for debugging
    Serial.begin(115200U);
    delay(500);
    Serial.println();
    Serial.println("[  INIT  ] Begin");
    initStage++;

    Serial.printf("[  INIT  ] ChipRevision: 0x%02X    CpuFreq: %uMHz   FlashChipSize: %uKiB   HeapSize: %uKiB   MAC: %s   SdkVersion: %s\n",
                  ESP.getChipRevision(),
                  ESP.getCpuFreqMHz(),
                  ESP.getFlashChipSize() / 1024,
                  ESP.getHeapSize() / 1024,
                  WiFi.macAddress().c_str(),
                  ESP.getSdkVersion());
    initStage++;

    //connect to your local wi-fi network
    Serial.printf("[  INIT  ] Connecting to Wifi '%s'", secrets.wifiSsid);
    WiFi.begin(secrets.wifiSsid, secrets.wifiPassword);

    //check wi-fi is connected to wi-fi network
    int retries = 5;
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
        Serial.print(".");
        retries--;
        if (retries <= 0)
        {
            ESP.restart();
        }
    }
    Serial.print(" connected!");
    Serial.print(" (IP=");
    Serial.print(WiFi.localIP());
    Serial.println(")");
    initStage++;

    Serial.print("[  INIT  ] Mounting file system... ");
    if (SPIFFS.begin(true))
    {
        Serial.println("ok");
        Serial.printf("[  FILE  ] total: %u KiB  available: %u KiB\n", SPIFFS.totalBytes() / 1024, (SPIFFS.totalBytes() - SPIFFS.usedBytes()) / 1024);
        filesystemAvailable = true;
    }
    else
    {
        Serial.println("failed");
        Serial.println("[ ERROR  ] An Error has occurred while mounting SPIFFS");
    }
    initStage++;

    if (filesystemAvailable)
    {
        Serial.println("[  INIT  ] file list");
        File root = SPIFFS.open("/");
        File file = root.openNextFile();
        while (file)
        {
            Serial.printf("[  FILE  ] %s - %u Bytes\n", file.name(), file.size());
            file = root.openNextFile();
        }
        root.close();
        file.close();

        initStage++;
    }

    // Power-On Motion Sensor
    pinMode(BNO055_PIN_VCC, OUTPUT);
    digitalWrite(BNO055_PIN_VCC, HIGH);
    delay(500); // wait for power up. Takes around 400ms (T_Sup).
    initStage++;

    // Initialize Environment Sensor
    if (I2CBus.begin(BNO055_PIN_I2C_SDA, BNO055_PIN_I2C_SCL, 250000U)) // set I2C-Clock to 250kHz
    {
        initStage++;
        if (bno.begin(bno.OPERATION_MODE_NDOF))
        {
            initStage++;
            orientationSensorAvailable = true;
            Serial.printf("[  INIT  ] found BNO055 Intelligent Absolute Orientation Sensor (ID=%02X, Addr=%02X)\n", BNO055_ID, BNO055_ADDRESS_A);
        }
        else
        {
            Serial.println("[ ERROR  ] Could not find a BNO055 sensor. Check wiring!");
        }
    }
    else
    {
        Serial.println("[ ERROR  ] Could not setup I2C Interface!");
    }

    if (orientationSensorAvailable)
    {
        initStage++;

        sensor_t sensor;
        bno.getSensor(&sensor);
        Serial.print("           Sensor:        ");
        Serial.println(sensor.name);
        Serial.print("           Driver Ver:    ");
        Serial.println(sensor.version);
        Serial.print("           Unique ID:     ");
        Serial.println(sensor.sensor_id);

        /* Get the system status values (mostly for debugging purposes) */
        uint8_t system_status, self_test_results, system_error;
        system_status = self_test_results = system_error = 0;
        bno.getSystemStatus(&system_status, &self_test_results, &system_error);

        Serial.print("           System Status: 0x");
        Serial.println(system_status, HEX);
        Serial.print("           Self Test:     0x");
        Serial.println(self_test_results, HEX);
        Serial.print("           System Error:  0x");
        Serial.println(system_error, HEX);

        if (filesystemAvailable)
        {
            Serial.println("[ SENSOR ] restoring sensor calibration data... ");
            if (persistency.readBinaryData((uint8_t *)&bnoData.calibrationData, sizeof(bnoData.calibrationData), "/calibration.dat"))
            {
                bnoData.status.calibrationDataAvailable = true;
                if (DEBUG)
                    bnoData.printSensorOffsets();

                // Calibration data: Accel=[8, -38, -32], Gyro=[-1, -2, 1], Mag=[235, -394, -355], Accel Radius=1000, Mag Radius=1282
                bno.setSensorOffsets(bnoData.calibrationData);
                delay(100);

                bno.setExtCrystalUse(true);
                // bno.setMode(bno.OPERATION_MODE_NDOF);
                delay(1000);
                Serial.println("[ SENSOR ] ok");
            }
            else
            {
                Serial.println("[ SENSOR ] restoring calibration data failed");
            }

            initStage++;
        }
        Serial.println("[ SENSOR ] Orientation sensor enabled");
    }

    server.begin();
    Serial.printf("[ SOCKET ] Listening on port %u\n", TCP_SERVER_PORT);
    initStage++;

    Serial.printf("[  INIT  ] Completed at stage %u\n\n", initStage);
}

void checkForConnections()
{
    if (server.hasClient())
    {
        // If we are already connected to another computer,
        // then reject the new connection. Otherwise accept
        // the connection.
        if (remoteClient && remoteClient.connected())
        {
            uint32_t ip = server.available().remoteIP();
            Serial.printf("[ SOCKET ] Already connected. New connection from %u.%u.%u.%u rejected.\n", ip & 0xFF, (ip >> 8) & 0xFF, (ip >> 16) & 0xFF, (ip >> 24) & 0xFF);
            server.available().stop();
        }
        else
        {
            remoteClient = server.available();
            uint32_t ip = remoteClient.remoteIP();
            Serial.printf("[ SOCKET ] Connection accepted from %u.%u.%u.%u\n", ip & 0xFF, (ip >> 8) & 0xFF, (ip >> 16) & 0xFF, (ip >> 24) & 0xFF);
        }
    }
}

imu::Vector<3> toEuler(imu::Quaternion q1)
{
    imu::Vector<3> ret;

    double test = q1.x() * q1.y() + q1.z() * q1.w();
    if (test > 0.499)
    { // singularity at north pole
        ret.x() = 2 * atan2(q1.x(), q1.w());
        ret.y() = M_PI / 2;
        ret.z() = 0;
        return ret;
    }
    if (test < -0.499)
    { // singularity at south pole
        ret.x() = -2 * atan2(q1.x(), q1.w());
        ret.y() = -M_PI / 2;
        ret.z() = 0;
        return ret;
    }
    double sqx = q1.x() * q1.x();
    double sqy = q1.y() * q1.y();
    double sqz = q1.z() * q1.z();
    ret.x() = atan2(2 * q1.y() * q1.w() - 2 * q1.x() * q1.z(), 1 - 2 * sqy - 2 * sqz);
    ret.y() = asin(2 * test);
    ret.z() = atan2(2 * q1.x() * q1.w() - 2 * q1.y() * q1.z(), 1 - 2 * sqx - 2 * sqz);

    return ret;
}

void loop()
{
    // base tasks
    checkForConnections();

    // 100ms Tasks
    if (!(counterBase % (100L / SCHEDULER_MAIN_LOOP_MS)))
    {
        digitalWrite(LED_BUILTIN, HIGH); // regularly turn on LED again

        if (remoteClient && remoteClient.connected())
        {
            int received = remoteClient.read(rxBuffer, sizeof(rxBuffer));
            if (received > 0)
            {
                Serial.print("[ SOCKET ] Rx: ");
                for (int i = 0; i < received; i++)
                {
                    Serial.printf("%02X ", rxBuffer[i]);
                }
                Serial.println();
            }
        }

        bno.getCalibration(&bnoData.status.calSystem, &bnoData.status.calGyro, &bnoData.status.calAccel, &bnoData.status.calMag);

        bnoData.status.fullyCalibrated = bno.isFullyCalibrated();
        bnoData.status.partlyCalibrated = (bnoData.status.calSystem >= 1) && (bnoData.status.calGyro >= 1) && (bnoData.status.calAccel >= 1) && (bnoData.status.calMag >= 1);

        if (!bnoData.status.fullyCalibrated)
        {
            calibrationStable = 0;
        }
    }

    // 500ms Tasks
    if (!(counterBase % (500L / SCHEDULER_MAIN_LOOP_MS)))
    {
        if (!bnoData.status.fullyCalibrated)
        {
            digitalWrite(LED_BUILTIN, LOW);
        }

        if (bnoData.status.partlyCalibrated)
        {

            // see https://forums.adafruit.com/viewtopic.php?f=25&t=108290&p=541754#p541754
            imu::Quaternion q = bno.getQuat();
            q.normalize();

            float temp = q.x();
            q.x() = -q.y();
            q.y() = temp;
            q.z() = -q.z();

            // euler.x() = -90 * euler.x() * M_PI;
            // euler.y() = -90 * euler.y() * M_PI;
            // euler.z() = -90 * euler.z() * M_PI;

            // imu::Vector<3> euler = bno.getVector(bno.VECTOR_EULER);
            imu::Vector<3> euler = q.toEuler();
            // imu::Vector<3> euler = toEuler(imu::Quaternion(0.7071, 0.7071, 0, 0 ));
            // imu::Vector<3> euler = toEuler(q);

            Serial.printf("[ SENSOR ] Heading: %3.2f Attitude: %3.2f Bank: %3.2f (w=%3.2f x=%3.2f y=%3.2f z=%3.2f)\n",
                          euler.x() * 180 / M_PI, euler.y() * 180 / M_PI, euler.z() * 180 / M_PI,
                          q.w(), q.x(), q.y(), q.z());

            // imu::Vector<3> gravity = bno.getVector(bno.VECTOR_GRAVITY);
            // Serial.printf("[ SENSOR ] Gravity: (x=%3.2f y=%3.2f z=%3.2f)\n",
            //               gravity.x(), gravity.y(), gravity.z());

            sensors_event_t event;
            bno.getEvent(&event);
            Serial.printf("[ SENSOR ] Orientation: (x=%3.2f y=%3.2f z=%3.2f)\n",
                          event.orientation.x, event.orientation.y, event.orientation.z);

            telescope.fromHorizontalPosition(event.orientation.x, event.orientation.y, 48, 102.3379);
        }

        if (remoteClient && remoteClient.connected() && bnoData.status.partlyCalibrated)
        {
            uint32_t length = telescope.packPosition(txBuffer, sizeof(txBuffer));
            if (length == 24)
            {
                remoteClient.write(txBuffer, length);
            }
        }
    }

    // 2s Tasks
    if (!(counterBase % (2000L / SCHEDULER_MAIN_LOOP_MS)))
    {
        counter2s++;
        // indicate alive
        if (bnoData.status.fullyCalibrated)
        {
            calibrationStable++;
            digitalWrite(LED_BUILTIN, LOW);
        }

        // save calibration data once per lifecycle after the sensor is fully calibrated
        if (bnoData.status.fullyCalibrated && !bnoData.status.calibrationDataSaved && calibrationStable > 15)
        {
            Serial.println("[ SENSOR ] Saving calibration data... ");
            bool success = bno.getSensorOffsets(bnoData.calibrationData);
            if (success && DEBUG)
                bnoData.printSensorOffsets();

            if (success && persistency.writeBinaryData((uint8_t *)&bnoData.calibrationData, sizeof(bnoData.calibrationData), "/calibration.dat"))
            {
                Serial.println("[ SENSOR ] ok");
                bnoData.status.calibrationDataSaved = true;
            }
            else
            {
                Serial.println("[ SENSOR ] Saving calibration data failed");
            }
        }

        if (!bnoData.status.partlyCalibrated && DEBUG)
        {
            bno.getSystemStatus(&bnoData.status.statSystem, &bnoData.status.statSelfTest, &bnoData.status.errSystem);
            bno.getCalibration(&bnoData.status.calSystem, &bnoData.status.calGyro, &bnoData.status.calAccel, &bnoData.status.calMag);

            // show overall system state
            Serial.printf("[ SENSOR ] Sys: %X ST: %X Err: %X  Cal: %u%u%u%u\n",
                          bnoData.status.statSystem,
                          bnoData.status.statSelfTest,
                          bnoData.status.errSystem,
                          bnoData.status.calSystem, bnoData.status.calGyro, bnoData.status.calAccel, bnoData.status.calMag);
        }
    }

    // 30s Tasks
    if (!(counterBase % (30000L / SCHEDULER_MAIN_LOOP_MS)))
    {
        if (orientationSensorAvailable)
        {
            bno.getSystemStatus(&bnoData.status.statSystem, &bnoData.status.statSelfTest, &bnoData.status.errSystem);
            bno.getCalibration(&bnoData.status.calSystem, &bnoData.status.calGyro, &bnoData.status.calAccel, &bnoData.status.calMag);

            // show overall system state
            Serial.printf("[ STATUS ] Free: %u KiB (%u KiB)  RSSI: %i dBm  Uptime: %" PRIi64 "s  Sys: %u Err: %u  Cal: %u\n",
                          ESP.getFreeHeap() / 1024,
                          ESP.getMaxAllocHeap() / 1024,
                          WiFi.RSSI(),
                          (esp_timer_get_time() / 1000000LL),
                          bnoData.status.statSystem,
                          bnoData.status.errSystem,
                          bnoData.status.calSystem);
        }
    }

    // 300s Tasks
    if (!(counterBase % (300000L / SCHEDULER_MAIN_LOOP_MS)))
    {
        counter300s++;
    }

    // 1h Tasks
    if (!(counterBase % (3600000L / SCHEDULER_MAIN_LOOP_MS)))
    {
        counter1h++;
    }

    delay(SCHEDULER_MAIN_LOOP_MS);
    counterBase++;
}