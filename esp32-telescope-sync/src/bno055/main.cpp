// Arduino base
#include <Arduino.h>
#include <WiFi.h>

// external Libraries
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SPIFFS.h>
// #include <AsyncTCP.h>

// own libraries
#include <persistency.h>
#include <bnodata.h>
#include <telescope.h>
#include <gnss.h>
#include <nexstar.h>
#include <mqttbroker.h>
#include <helper.h>
#include <ledmanager.h>

/**
 * A file called 'secrets.h' must be placed in lib/secrets and contain the following content:
 * 
 * #ifndef SECRETS_H
 * #define SECRETS_H
 * struct Secrets
 * {
 *     const char *wifiSsid = "test";   // WiFi AP-Name
 *     const char *wifiPassword = "1234";
 *     IPAddress mqttBroker = IPAddress(192, 168, 1, 1);    // your MQTT-Broker
 *     const char *accessPointSsid = "ESP32";
 *     const char *accessPointPassword = "12345678";    // must be 8+ characters
 *     IPAddress accessPointIP = IPAddress(192, 168, 1, 1); 
 * } secrets;
 * #endif
 * 
 **/
#include <secrets.h>

#define DEBUG (false)

#define BNO055_I2C_CLOCK (100000U)
#define BNO055_PIN_I2C_SCL (22)
#define BNO055_PIN_I2C_SDA (21)
#define BNO055_BUS_ID (0)
#define BNO055_PIN_VCC (4)

TwoWire I2CBus = TwoWire(BNO055_BUS_ID);                                     // set up a new Wire-Instance for BNO055 Intelligent Absolute Orientation Sensor
Adafruit_BNO055 bno = Adafruit_BNO055(BNO055_ID, BNO055_ADDRESS_A, &I2CBus); // use custom I2C-Instance
BnoData bnoData(BNO055_ID, BNO055_ADDRESS_A);

// global switches for connected devices
#define ENABLE_WIFI (true)
#define WIFIMODE_AUTO (true)
#define WIFIMODE_FORCE_AP (false)

#define ENABLE_MQTT (true)
#define ENABLE_GNSS (true)
#define ENABLE_ORIENTATION (true)
#define ENABLE_BROKER (true)

bool orientationSensorAvailable = false;
bool gnssModuleAvailable = false;
bool filesystemAvailable = false;
bool wifiClientMode = false;
bool wifiAccessPointMode = false;
bool gnssAvailable = false;
bool mqttAvailable = false;
bool localBrokerAvailable = false;
float headingOffset = 0;
bool nexStarMode = false;

#define TCP_SERVER_PORT (10001)
#define SSID "ESP32"
// AsyncServer server(TCP_SERVER_PORT);

WiFiServer server(TCP_SERVER_PORT);
WiFiClient remoteClient;
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient); // for connecting to a remote broker
MQTTBroker broker(&Serial);          // set-up own broker, needs a HardwareSerial for logging

Persistency persistency;            // load and save data in flash memory
GNSS gnss(48, 11);                  // GNSS data provider; set initial position to enable operation before GNSS fix
Telescope telescope;                // Telescope properties and related calculations
NexStar nexstar(&telescope, &gnss); // Communication to Stellarium-App
LEDManager ledmanager;

// Flow control, basic task scheduler
#define SCHEDULER_MAIN_LOOP_MS (10) // ms
uint32_t counterBase = 0;
uint32_t counter2s = 0;
uint32_t counter300s = 0;
uint32_t counter1h = 0;
uint32_t initStage = 0;

uint32_t calibrationStableCounter = 0;
uint32_t gnssTimeout = 0;

uint8_t txBuffer[265];
uint8_t rxBuffer[265];

void mqttCallback(char *topic, byte *payload, unsigned int length)
{
    Serial.print("[  MQTT  ] Message arrived [");
    Serial.print(topic);
    Serial.print("]: ");
    uint32_t i;
    for (i = 0; i < length; i++)
    {
        Serial.print((char)payload[i]);
    }
    Serial.println();
}

void onWifiDisconnect(WiFiEvent_t event, WiFiEventInfo_t info)
{
    Serial.printf("[  WIFI  ] Disconnected from `` (%i). Reconnecting...\n", info.disconnected.reason);
    WiFi.disconnect();
    WiFi.begin(secrets.wifiSsid, secrets.wifiPassword);
}

void onWifiConnected(WiFiEvent_t event, WiFiEventInfo_t info)
{
    Serial.println("[  WIFI  ] WiFi connected!");
}

void onWifiIPAddress(WiFiEvent_t event, WiFiEventInfo_t info)
{
    Serial.printf("[  WIFI  ] IP=%s\n", WiFi.localIP().toString().c_str());
}

void setup()
{
    // LED output
    pinMode(LED_BUILTIN, OUTPUT);
    ledmanager.setMode(LEDManager::LEDMode::ON);
    ledmanager.update();
    initStage++;

    // Setup serial connection for debugging
    Serial.begin(115200U);
    delay(500);
    Serial.println();
    Serial.println("[  INIT  ] Begin");
    initStage++;

    Serial.printf("[  INIT  ] ChipRevision: 0x%02X   CpuFreq: %uMHz   FlashChipSize: %uKiB   HeapSize: %uKiB   MAC: %s   SdkVersion: %s\n",
                  ESP.getChipRevision(), ESP.getCpuFreqMHz(), ESP.getFlashChipSize() / 1024, ESP.getHeapSize() / 1024, WiFi.macAddress().c_str(), ESP.getSdkVersion());
    initStage++;

    if (ENABLE_WIFI)
    {
        if (WIFIMODE_AUTO && !WIFIMODE_FORCE_AP)
        {
            //connect to your local wi-fi network
            Serial.printf("[  INIT  ] Connecting to Wifi '%s'", secrets.wifiSsid);
            WiFi.begin(secrets.wifiSsid, secrets.wifiPassword);

            //check wi-fi is connected to wi-fi network
            int retries = 5;
            while ((WiFi.status() != WL_CONNECTED) && ((retries--) > 0))
            {
                delay(1000);
                Serial.print(".");
            }

            if (WiFi.status() == WL_CONNECTED)
            {
                Serial.printf(" connected! (IP=%s)\n", WiFi.localIP().toString().c_str());
                WiFi.onEvent(onWifiDisconnect, SYSTEM_EVENT_STA_DISCONNECTED);
                WiFi.onEvent(onWifiConnected, SYSTEM_EVENT_STA_CONNECTED);
                WiFi.onEvent(onWifiIPAddress, SYSTEM_EVENT_STA_GOT_IP);
                wifiClientMode = true;
            }
            else
            {
                Serial.println(" failed!");
                WiFi.disconnect();
            }
        }

        if ((WIFIMODE_AUTO && !wifiClientMode) || WIFIMODE_FORCE_AP)
        {
            Serial.printf("[  INIT  ] Setting up Wifi access point '%s'...", SSID);
            if (WiFi.softAP(secrets.accessPointSsid, secrets.accessPointPassword))
            {
                WiFi.softAPConfig(secrets.accessPointIP, secrets.accessPointIP, IPAddress(255, 255, 255, 0));
                Serial.printf(" done (IP=%s)\n", WiFi.softAPIP().toString().c_str());
                wifiAccessPointMode = true;
            }
            else
            {
                Serial.println(" failed!");
            }
        }
        initStage++;
    }

    Serial.print("[  INIT  ] Mounting file system... ");
    if (SPIFFS.begin(true))
    {
        Serial.println("ok");
        Serial.printf("[  INIT  ] Flash total: %u KiB  available: %u KiB\n", SPIFFS.totalBytes() / 1024, (SPIFFS.totalBytes() - SPIFFS.usedBytes()) / 1024);
        filesystemAvailable = true;
    }
    else
    {
        Serial.println("failed");
        Serial.println("[! INIT  ] An Error has occurred while mounting SPIFFS");
    }
    initStage++;

    if (filesystemAvailable)
    {
        Serial.println("[  INIT  ] file list:");
        File root = SPIFFS.open("/");
        File file = root.openNextFile();
        while (file)
        {
            Serial.printf("[  INIT  ]       %s - %u Bytes\n", file.name(), file.size());
            file = root.openNextFile();
        }
        root.close();
        file.close();
        initStage++;
    }

    if (ENABLE_ORIENTATION)
    {
        // Power-On Motion Sensor
        pinMode(BNO055_PIN_VCC, OUTPUT);
        digitalWrite(BNO055_PIN_VCC, HIGH);
        delay(500); // wait for power up. Takes around 400ms (T_Sup).
        initStage++;

        // Initialize Environment Sensor
        if (I2CBus.begin(BNO055_PIN_I2C_SDA, BNO055_PIN_I2C_SCL, BNO055_I2C_CLOCK)) // set I2C-Clock to 100kHz
        {
            initStage++;
            // in NDOF, too many errors are introduced by the magentometer. Especially when the sensor is attached to the metal tube of the telescope
            // IMU-Mode has no absolute azimuth-orientation but less variations and better repeatability.
            if (bno.begin(bno.OPERATION_MODE_NDOF))
            {
                initStage++;
                orientationSensorAvailable = true;
                Serial.printf("[  INIT  ] Found BNO055 Intelligent Absolute Orientation Sensor (ID=%02X, Addr=%02X)\n", BNO055_ID, BNO055_ADDRESS_A);
            }
            else
            {
                Serial.println("[! INIT  ] Could not find a BNO055 sensor. Check wiring!");
            }
        }
        else
        {
            Serial.println("[! INIT  ] Could not setup I2C Interface!");
        }
    }

    if (orientationSensorAvailable)
    {
        initStage++;
        bno.setExtCrystalUse(true);
        delay(1000);

        if (filesystemAvailable)
        {
            Serial.println("[  INIT  ] restoring sensor calibration data... ");
            if (persistency.readBinaryData((uint8_t *)&bnoData.calibrationData, sizeof(bnoData.calibrationData), "/calibration.dat"))
            {
                bnoData.status.calibrationDataAvailable = true;
                if (DEBUG)
                    bnoData.printSensorOffsets();

                bno.setSensorOffsets((uint8_t *)&bnoData.calibrationData);
                delay(100);

                bool success = bno.getSensorOffsets((uint8_t *)&bnoData.calibrationData);
                if (success && DEBUG)
                    bnoData.printSensorOffsets();

                // Calibration data: Accel=[8, -38, -32], Gyro=[-1, -2, 1], Mag=[235, -394, -355], Accel Radius=1000, Mag Radius=1282

                Serial.println("[  INIT  ] ok");
            }
            else
            {
                Serial.println("[! INIT  ] restoring calibration data failed");
            }

            initStage++;
        }

        // read sensor properties and status
        bno.getSensor(&bnoData.properties);
        bno.getSystemStatus(&bnoData.status.statSystem, &bnoData.status.statSelfTest, &bnoData.status.errSystem);

        Serial.printf("[  INIT  ] Sensor: %s (v%i)   ID: 0x%02X   Status: 0x%02X   Self Test: 0x%02X   Error: 0x%02X\n",
                      bnoData.properties.name, bnoData.properties.version, bnoData.properties.sensor_id,
                      bnoData.status.statSystem, bnoData.status.statSelfTest, bnoData.status.errSystem);

        Serial.println("[  INIT  ] Orientation sensor enabled");
    }

    if (wifiClientMode || wifiAccessPointMode)
    {
        server.begin();
        Serial.printf("[  INIT  ] TCP-Server listening on port %u\n", TCP_SERVER_PORT);
        initStage++;

        if (ENABLE_BROKER)
        {
            broker.init(1883);
            localBrokerAvailable = true;
            Serial.printf("[  INIT  ] MQTT-Broker listening on port %u\n", 1883);
        }

        if (ENABLE_MQTT && wifiClientMode)
        {
            Serial.print("[  INIT  ] Connecting to remote MQTT-Broker... ");
            mqttClient.setServer(secrets.mqttBroker, 1883);
            mqttClient.setCallback(mqttCallback);
            Serial.println("ok");
            initStage++;
            mqttAvailable = true;
        }
    }

    if (ENABLE_GNSS)
    {
        Serial2.begin(9600u);
        Serial.print("[  INIT  ] GNSS-Module enabled\n");
    }

    Serial.printf("[  INIT  ] Completed at stage %u\n\n", initStage);
    ledmanager.setMode(LEDManager::LEDMode::READY);
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

void reconnectMQTTClient()
{
    Serial.print("[  MQTT  ] Attempting MQTT connection... ");
    if (mqttClient.connect(WiFi.getHostname()))
    {
        Serial.println("connected");
    }
    else
    {
        Serial.printf("failed! (0x%02X)\n", mqttClient.state());
    }
}

void loop()
{
    ledmanager.update(counterBase);

    // base tasks
    if (wifiClientMode || wifiAccessPointMode)
    {
        checkForConnections();
    }

    if (localBrokerAvailable)
    {
        broker.update();
    }

    // 100ms Tasks
    if (!(counterBase % (100L / SCHEDULER_MAIN_LOOP_MS)))
    {
        if (remoteClient && remoteClient.connected())
        {
            uint32_t received = remoteClient.read(rxBuffer, sizeof(rxBuffer));

            if (received > 0)
            {
                // Serial.print("[ SOCKET ] Rx: ");
                // for (int i = 0; i < received; i++)
                // {
                //     Serial.printf("%02X ", rxBuffer[i]);
                // }
                // Serial.println();

                int32_t nexstarResponseLength = nexstar.handleRequest(rxBuffer, sizeof(rxBuffer), txBuffer, sizeof(txBuffer));

                if (nexstarResponseLength > 0)
                {
                    remoteClient.write(txBuffer, nexstarResponseLength);
                    nexStarMode = true;
                }
                else
                {
                    Telescope::Equatorial reference;

                    // check if the packet could be decoded correctly before using it to calibrate the offset
                    if (telescope.unpackPosition(&reference, NULL, rxBuffer, received))
                    {
                        imu::Quaternion q = bno.getQuat();
                        imu::Vector<3> euler = q.toEuler() * 180. / PI;
                        euler.x() *= -1; // convert to Azimuth where north=0° and a rotation towards east (right) increases the value

                        telescope.setOrientation(euler.z(), euler.x() + headingOffset);
                        double localSiderealTimeDegrees = MathHelper::getLocalSiderealTimeDegrees(gnss.utcTimestamp, gnss.longitude);

                        Serial.printf("[ SENSOR ] Received Calibration Data  Ra: %.3f Dec: %.3f\n", reference.ra, reference.dec);

                        telescope.calibrate(reference, gnss.latitude, localSiderealTimeDegrees);
                    }
                }
            }
        }

        if (orientationSensorAvailable)
        {
            bno.getCalibration(&bnoData.status.calSystem, &bnoData.status.calGyro, &bnoData.status.calAccel, &bnoData.status.calMag);

            bnoData.status.fullyCalibrated = bno.isFullyCalibrated(); //(bnoData.status.calSystem == 3) && (bnoData.status.calGyro == 3) && (bnoData.status.calAccel == 3) && (bnoData.status.calMag == 3);
            bnoData.status.partlyCalibrated = (bnoData.status.calSystem >= 1) && (bnoData.status.calGyro >= 1) && (bnoData.status.calAccel >= 1) && (bnoData.status.calMag >= 1);

            if (!bnoData.status.fullyCalibrated)
            {
                calibrationStableCounter = 0;
            }
        }

        if (mqttAvailable)
        {
            mqttClient.loop();
        }

        if (!bnoData.status.fullyCalibrated && !bnoData.status.calibrationDataSaved)
        {
            ledmanager.setMode(LEDManager::LEDMode::BLINK_1HZ);
        }
        else if (!bnoData.status.fullyCalibrated && bnoData.status.calibrationDataSaved)
        {
            ledmanager.setMode(LEDManager::LEDMode::FLASH_4X_EVERY_5S);
        }
        else if (!gnss.valid)
        {
            ledmanager.setMode(LEDManager::LEDMode::FLASH_2X_EVERY_5S);
        }
        else
        {
            ledmanager.setMode(LEDManager::LEDMode::FLASH_1X_EVERY_5S);
        }
    }

    // 500ms Tasks
    if (!(counterBase % (500L / SCHEDULER_MAIN_LOOP_MS)))
    {
        if (orientationSensorAvailable)
        {
            // if (!bnoData.status.fullyCalibrated)
            // ledmanager.setMode(LEDManager::LEDMode::BLINK_1HZ);

            // ZYX-Order. Sensor xy-plane is aligned with earth's surface => x=azimuth; y=altitude
            imu::Quaternion q = bno.getQuat();
            imu::Vector<3> euler = q.toEuler() * 180. / PI;
            euler.x() *= -1; // convert to Azimuth where north=0° and a rotation towards east (right) increases the value

            telescope.setOrientation(euler.z(), euler.x() + headingOffset);

            Telescope::Horizontal corrected = telescope.getCalibratedOrientation();
            double localSiderealTimeDegrees = MathHelper::getLocalSiderealTimeDegrees(gnss.utcTimestamp, gnss.longitude);
            Telescope::Equatorial position = telescope.horizontalToEquatorial(corrected, gnss.latitude, localSiderealTimeDegrees);

            // if (remoteClient && remoteClient.connected() && bnoData.status.partlyCalibrated && gnssAvailable && (abs(gnss.longitude) > 0.01))
            if (remoteClient && remoteClient.connected() && gnssAvailable && !nexStarMode)
            {
                uint64_t timestamp = 0;
                uint32_t length = telescope.packPosition(position, timestamp, txBuffer, sizeof(txBuffer));
                if (length == 24)
                {
                    remoteClient.write(txBuffer, length);
                }
            }

            int32_t len = 0;
            len = snprintf((char *)txBuffer, sizeof(txBuffer), "%.3f", corrected.alt);
            if (mqttAvailable)
                mqttClient.publish("home/appliance/telescope/orientation/alt", txBuffer, len);
            if (localBrokerAvailable)
                broker.publish("home/appliance/telescope/orientation/alt", (char *)txBuffer);

            len = snprintf((char *)txBuffer, sizeof(txBuffer), "%.3f", corrected.az);
            if (mqttAvailable)
                mqttClient.publish("home/appliance/telescope/orientation/az", txBuffer, len);
            if (localBrokerAvailable)
                broker.publish("home/appliance/telescope/orientation/az", (char *)txBuffer);

            len = snprintf((char *)txBuffer, sizeof(txBuffer), "%.3f", position.ra);
            if (mqttAvailable)
                mqttClient.publish("home/appliance/telescope/orientation/ra", txBuffer, len);
            if (localBrokerAvailable)
                broker.publish("home/appliance/telescope/orientation/ra", (char *)txBuffer);

            len = snprintf((char *)txBuffer, sizeof(txBuffer), "%.3f", position.dec);
            if (mqttAvailable)
                mqttClient.publish("home/appliance/telescope/orientation/dec", txBuffer, len);
            if (localBrokerAvailable)
                broker.publish("home/appliance/telescope/orientation/dec", (char *)txBuffer);

            len = snprintf((char *)txBuffer, sizeof(txBuffer), "%.3f", localSiderealTimeDegrees / 15.);
            if (mqttAvailable)
                mqttClient.publish("home/appliance/telescope/orientation/lst", txBuffer, len);
            if (localBrokerAvailable)
                broker.publish("home/appliance/telescope/orientation/lst", (char *)txBuffer);

            len = snprintf((char *)txBuffer, sizeof(txBuffer), "%.3f", euler.x());
            if (mqttAvailable)
                mqttClient.publish("home/appliance/telescope/orientation/roll", txBuffer, len);
            if (localBrokerAvailable)
                broker.publish("home/appliance/telescope/orientation/roll", (char *)txBuffer);

            len = snprintf((char *)txBuffer, sizeof(txBuffer), "%.3f", euler.y());
            if (mqttAvailable)
                mqttClient.publish("home/appliance/telescope/orientation/pitch", txBuffer, len);
            if (localBrokerAvailable)
                broker.publish("home/appliance/telescope/orientation/pitch", (char *)txBuffer);

            len = snprintf((char *)txBuffer, sizeof(txBuffer), "%.3f", euler.z());
            if (mqttAvailable)
                mqttClient.publish("home/appliance/telescope/orientation/yaw", txBuffer, len);
            if (localBrokerAvailable)
                broker.publish("home/appliance/telescope/orientation/yaw", (char *)txBuffer);

            len = snprintf((char *)txBuffer, sizeof(txBuffer), "[%.4f, %.4f, %.4f, %.4f]", q.w(), q.x(), q.y(), q.z());
            if (mqttAvailable)
                mqttClient.publish("home/appliance/telescope/orientation/quat", txBuffer, len);
            if (localBrokerAvailable)
                broker.publish("home/appliance/telescope/orientation/quat", (char *)txBuffer);

            imu::Vector<3> acc, gyr, mag;
            acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
            gyr = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
            mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

            len = snprintf((char *)txBuffer, sizeof(txBuffer), "{\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,\"gx\":%.3f,\"gy\":%.3f,\"gz\":%.3f,\"mx\":%.3f,\"my\":%.3f,\"mz\":%.3f}",
                           acc.x(), acc.y(), acc.z(),
                           gyr.x(), gyr.y(), gyr.z(),
                           mag.x(), mag.y(), mag.z());

            if (mqttAvailable)
                mqttClient.publish("home/appliance/telescope/orientation/raw", txBuffer, len);
            if (localBrokerAvailable)
                broker.publish("home/appliance/telescope/orientation/raw", (char *)txBuffer);
        }
        if (ENABLE_GNSS)
        {
            int received = Serial2.read(rxBuffer, sizeof(rxBuffer) - 1);
            if (received > 0)
            {
                rxBuffer[received] = 0;
                if (gnss.fromBuffer(rxBuffer, received) > 0)
                {
                    // reset timeout counter if we received a valid NMEA sentence
                    gnssTimeout = 4 * 5; // 5s
                    gnssAvailable = true;
                }
                // Serial.printf("[  GNSS  ] %s\n", rxBuffer);
            }
            else
            {
                // check if there is no data from the gnss-unit for a while and set state accordingly
                gnssTimeout -= (gnssTimeout > 0) ? 1 : 0;
                gnssAvailable = gnssTimeout > 0;
            }
        }
    }

    // 2s Tasks
    if (!(counterBase % (2000L / SCHEDULER_MAIN_LOOP_MS)))
    {
        counter2s++;
        // indicate alive

        if (orientationSensorAvailable)
        {
            if (bnoData.status.fullyCalibrated)
            {
                calibrationStableCounter++;
            }

            // save calibration data once per lifecycle after the sensor is fully calibrated
            if (bnoData.status.fullyCalibrated && !bnoData.status.calibrationDataSaved && calibrationStableCounter > 15)
            {
                Serial.println("[ SENSOR ] Saving calibration data... ");
                bool success = bno.getSensorOffsets(bnoData.calibrationData);
                if (success && DEBUG)
                    bnoData.printSensorOffsets();

                if (bnoData.validateSensorOffsets())
                {
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
                else
                {
                    Serial.println("[ SENSOR ] Invalid calibration data!");
                }
            }

            bno.getSystemStatus(&bnoData.status.statSystem, &bnoData.status.statSelfTest, &bnoData.status.errSystem);
            bnoData.status.temp = bno.getTemp();

            if (!bnoData.status.fullyCalibrated && DEBUG)
            {
                // show overall system state
                if (DEBUG)
                {
                    Serial.printf("[ SENSOR ] Sys: %X ST: %X Err: %X  Cal: %u%u%u%u  Temp: %i°C\n",
                                  bnoData.status.statSystem,
                                  bnoData.status.statSelfTest,
                                  bnoData.status.errSystem,
                                  bnoData.status.calSystem, bnoData.status.calGyro, bnoData.status.calAccel, bnoData.status.calMag,
                                  bnoData.status.temp);
                }
            }
            int32_t len = 0;
            len = snprintf((char *)txBuffer, sizeof(txBuffer), "{\"system\":\"0x%X\",\"selftest\":\"0x%X\",\"error\":\"0x%X\",\"cal\":\"0x%04X\",\"temp\":%i}",
                           bnoData.status.statSystem,
                           bnoData.status.statSelfTest,
                           bnoData.status.errSystem,
                           (uint32_t(bnoData.status.calSystem) << 12) + (uint32_t(bnoData.status.calGyro) << 8) + (uint32_t(bnoData.status.calAccel) << 4) + uint32_t(bnoData.status.calMag),
                           bnoData.status.temp);

            if (mqttAvailable)
                mqttClient.publish("home/appliance/telescope/sensor", txBuffer, len);

            if (localBrokerAvailable)
                broker.publish("home/appliance/telescope/sensor", (char *)txBuffer);
        }

        if (gnssAvailable)
        {
            if (DEBUG)
            {
                Serial.printf("[  GNSS  ] %s Lat=%.2f, Lng=%.2f, Alt=%.0fm, Date=%04u-%02u-%02u Time=%02u:%02u:%02u (Sat %u of %u)\n",
                              gnss.valid ? "Fix" : "No fix",
                              gnss.latitude,
                              gnss.longitude,
                              gnss.altitude,
                              gnss.utcTimestamp.tm_year,
                              gnss.utcTimestamp.tm_mon,
                              gnss.utcTimestamp.tm_mday,
                              gnss.utcTimestamp.tm_hour,
                              gnss.utcTimestamp.tm_min,
                              gnss.utcTimestamp.tm_sec,
                              gnss.satUsed,
                              gnss.satView);
            }
            int32_t len = 0;
            len = snprintf((char *)txBuffer, sizeof(txBuffer), "{\"valid\":%s,\"lat\":%.3f,\"lng\":%.3f,\"alt\":%.0f,\"date\":\"%04u-%02u-%02u\",\"time\":\"%02u:%02u:%02u\",\"satUse\":%u,\"satView\":%u}",
                           gnss.valid ? "true" : "false",
                           gnss.latitude,
                           gnss.longitude,
                           gnss.altitude,
                           gnss.utcTimestamp.tm_year,
                           gnss.utcTimestamp.tm_mon,
                           gnss.utcTimestamp.tm_mday,
                           gnss.utcTimestamp.tm_hour,
                           gnss.utcTimestamp.tm_min,
                           gnss.utcTimestamp.tm_sec,
                           gnss.satUsed,
                           gnss.satView);
            if (localBrokerAvailable)
                broker.publish("home/appliance/telescope/gnss", (char *)txBuffer);
            if (mqttAvailable)
                mqttClient.publish("home/appliance/telescope/gnss", txBuffer, len);
        }

        if (mqttAvailable && !mqttClient.connected())
        {
            reconnectMQTTClient();
        }
    }

    // 30s Tasks
    if (!(counterBase % (30000L / SCHEDULER_MAIN_LOOP_MS)))
    {
        if (orientationSensorAvailable)
        {
            // show overall system state
            Serial.printf("[ STATUS ] Free: %u KiB (%u KiB)   RSSI: %i dBm   Uptime: %" PRIi64 "s   Sys: %u Err: %u   Cal: %u   Temp: %i°C   GNSS: %s\n",
                          ESP.getFreeHeap() / 1024,
                          ESP.getMaxAllocHeap() / 1024,
                          WiFi.RSSI(),
                          (esp_timer_get_time() / 1000000LL),
                          bnoData.status.statSystem,
                          bnoData.status.errSystem,
                          bnoData.status.calSystem,
                          bnoData.status.temp,
                          gnss.valid ? "fix" : "no fix");
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