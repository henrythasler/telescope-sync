#include <Arduino.h>
#include <WiFi.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SPIFFS.h>
#include <persistency.h>
#include <bnodata.h>
// #include <AsyncTCP.h>

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

TwoWire I2CBME = TwoWire(BNO055_BUS_ID); // set up a new Wire-Instance for BNO055 Intelligent Absolute Orientation Sensor
Adafruit_BNO055 bno = Adafruit_BNO055(BNO055_ID, BNO055_ADDRESS_A, &I2CBME);
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

uint8_t txBuffer[32] = {0x18, 0x00, 0x00, 0x00, 0x40, 0x7b, 0x0b, 0x16, 0xe5, 0xd3, 0x05, 0x00, 0xc3, 0xdd, 0x78, 0x29, 0xa8, 0x8b, 0x79, 0x0c, 0x00, 0x00, 0x00, 0x00};
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
    if (I2CBME.begin(BNO055_PIN_I2C_SDA, BNO055_PIN_I2C_SCL, 250000U)) // set I2C-Clock to 250kHz
    {
        initStage++;
        if (bno.begin(bno.OPERATION_MODE_CONFIG)) // use custom Wire-Instance to avoid interference with other libraries.
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

                bno.setSensorOffsets(bnoData.calibrationData);
                Serial.println("[ SENSOR ] ok");
            }
            else
            {
                Serial.println("[ SENSOR ] restoring calibration data failed");
            }

            initStage++;
        }
        delay(50);
        bno.setMode(bno.OPERATION_MODE_NDOF);
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

        bnoData.status.fullyCalibrated = bno.isFullyCalibrated();
    }

    // 500ms Tasks
    if (!(counterBase % (500L / SCHEDULER_MAIN_LOOP_MS)))
    {
        if (!bnoData.status.fullyCalibrated)
        {
            digitalWrite(LED_BUILTIN, LOW);
        }

        if (bnoData.status.fullyCalibrated)
        {
            imu::Vector<3> vec = bno.getVector(bno.VECTOR_EULER);
            Serial.printf("[ SENSOR ] Heading: %3.2f Roll: %3.2f Pitch: %3.2f\n", vec[0], vec[1], vec[2]);
        }
    }

    // 2s Tasks
    if (!(counterBase % (2000L / SCHEDULER_MAIN_LOOP_MS)))
    {
        counter2s++;
        // indicate alive
        if (bnoData.status.fullyCalibrated)
        {
            digitalWrite(LED_BUILTIN, LOW);
        }

        // save calibration data once per lifecycle after the sensor is fully calibrated
        if (bnoData.status.fullyCalibrated && !bnoData.status.calibrationDataSaved)
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

        if (!bnoData.status.fullyCalibrated && DEBUG)
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

        if (remoteClient && remoteClient.connected())
        {
            remoteClient.write(txBuffer, 24);
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