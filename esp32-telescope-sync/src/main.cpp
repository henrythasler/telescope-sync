// own libraries
#include <persistency.h>
#include <telescope.h>
#include <gnss.h>
#include <nexstar.h>
#include <mqttbroker.h>
#include <helper.h>
#include <ledmanager.h>
#include <orientationsensor.h>

/**
 * A file called 'secrets.h' must be placed in 'lib/secrets' and contain the following content:
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

// external Libraries
#include <PubSubClient.h>
#include <SPIFFS.h>
// #include <AsyncTCP.h>

// Arduino base
#include <Arduino.h>
#include <WiFi.h>

constexpr bool DEBUG = false;

// global switches for connected devices
constexpr bool ENABLE_WIFI = true;
constexpr bool WIFIMODE_AUTO = true;
constexpr bool WIFIMODE_FORCE_AP = false; // don't try to connect to a wifi-network and set up an AP immediately

constexpr bool ENABLE_MQTT = true;
constexpr bool ENABLE_GNSS = true;
constexpr bool ENABLE_ORIENTATION = true;
constexpr bool ENABLE_ROTATION = true;
constexpr bool ENABLE_BROKER = true;

// I/O settings
constexpr int IMU_PIN_VCC = 4;
constexpr int IMU_PIN_I2C_SCL = 22;
constexpr int IMU_PIN_I2C_SDA = 21;
constexpr int IMU_I2C_CLOCK = 400000U;
constexpr int IMU_BUS_ID = 0;

TwoWire I2CBus = TwoWire(IMU_BUS_ID); // set up a new Wire-Instance
LSM6Wrapper imu(0, 0x6A, &I2CBus);

constexpr int AMT_SS = 15;
SPIClass AMT22(HSPI);

// stores the availability of certain modules
bool orientationSensorAvailable = false;
bool gnssModuleAvailable = false;
bool filesystemAvailable = false;
bool gnssAvailable = false;
bool mqttAvailable = false;
bool localBrokerAvailable = false;

// internal variables
Adafruit_Sensor *accelerometer, *gyroscope;
sensors_event_t accel, gyro, mag;
float roll = 0, pitch = 0, heading = 0;
float gx = 0, gy = 0, gz = 0;
float qw = 0, qx = 0, qy = 0, qz = 0;
bool wifiClientMode = false;
bool wifiAccessPointMode = false;
float headingOffset = 0;
bool nexStarMode = false;

// Network settings and components
#define TCP_SERVER_PORT (10001)
#define SSID "ESP32"
// AsyncServer server(TCP_SERVER_PORT);

WiFiServer server(TCP_SERVER_PORT);
WiFiClient remoteClient;
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient); // for connecting to a remote broker
MQTTBroker broker(&Serial);          // set-up own broker, needs a HardwareSerial for logging

// Library modules
Persistency persistency;            // load and save data in flash memory
GNSS gnss(48, 11);                  // GNSS data provider; set initial position to enable operation before GNSS fix
Telescope telescope;                // Telescope properties and related calculations
NexStar nexstar(&telescope, &gnss); // Communication to Stellarium-App
LEDManager ledmanager;

// Flow control, basic task scheduler
#define FILTER_UPDATE_RATE_HZ 50
#define SCHEDULER_MAIN_LOOP_US (1000000 / FILTER_UPDATE_RATE_HZ) // ms
float sensorUpdateRate = 0;

uint32_t timestamp;
uint32_t counter2s = 0;
uint32_t counter300s = 0;
uint32_t counter1h = 0;
uint32_t initStage = 0;

uint32_t taskFilterTimer = 0;
uint32_t task25msTimer = 0;
uint32_t task100msTimer = 0;
uint32_t task500msTimer = 0;
uint32_t task1sTimer = 0;
uint32_t task2sTimer = 0;
uint32_t task30sTimer = 0;

uint32_t gnssTimeout = 0;

// multi-purpose static bytearray
uint8_t txBuffer[256];
uint8_t rxBuffer[256];

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
            // connect to your local wi-fi network
            Serial.printf("[  INIT  ] Connecting to Wifi '%s'", secrets.wifiSsid);
            WiFi.begin(secrets.wifiSsid, secrets.wifiPassword);

            // check wi-fi is connected to wi-fi network
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
        pinMode(IMU_PIN_VCC, OUTPUT);
        digitalWrite(IMU_PIN_VCC, HIGH);
        delay(500); // wait for power up. Takes around 400ms (T_Sup).
        initStage++;

        // Initialize Environment Sensor
        if (I2CBus.begin(IMU_PIN_I2C_SDA, IMU_PIN_I2C_SCL, IMU_I2C_CLOCK))
        {
            initStage++;
            if (imu.begin())
            {
                imu.setCalibration();
                initStage++;
                orientationSensorAvailable = true;
                Serial.println("[  INIT  ] Found LSM6DS33+LIS3MDL Absolute Orientation Sensor Board");
            }
            else
            {
                Serial.println("[! INIT  ] Could not find orientation sensor. Check wiring!");
            }
        }
        else
        {
            Serial.println("[! INIT  ] Could not setup I2C Interface!");
        }
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
            mqttClient.subscribe("home/appliance/telescope/control");
            Serial.println("ok");
            initStage++;
            mqttAvailable = true;
        }
    }

    if (ENABLE_GNSS)
    {
        // The GNSS module streams out it's data every second via the serial interface; no further setup required
        Serial2.begin(9600u);
        Serial.print("[  INIT  ] GNSS-Module enabled\n");
    }

    if (ENABLE_ROTATION)
    {
        AMT22.begin();
        // SPI3.setClockDivider(SPI_CLOCK_DIV128);
        digitalWrite(AMT_SS, HIGH);
        pinMode(AMT_SS, OUTPUT);
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
    timestamp = micros();

    if ((timestamp - taskFilterTimer) > SCHEDULER_MAIN_LOOP_US)
    {
        sensorUpdateRate = 1000000. / float(timestamp - taskFilterTimer);
        taskFilterTimer = timestamp;

        if (orientationSensorAvailable)
        {
            imu.accSample();
        }
        return;
    }

    // 25ms Tasks
    if ((timestamp - task25msTimer) > 25000L)
    {
        task25msTimer = timestamp;

        ledmanager.update(timestamp);
        return;
    }

    // 100ms Tasks
    if ((timestamp - task100msTimer) > 100000L)
    {
        task100msTimer = timestamp;

        // read filtered accelerometer data and compute pitch from calibrated values
        imu.getMeanAcc(&accel);
        imu.calibrate(&accel, &gyro);
        pitch = atan2(accel.acceleration.x, accel.acceleration.y) * 180 / PI;

        // read rotation sensor data via SPI and verify checksum
        digitalWrite(AMT_SS, LOW);
        uint16_t heading_raw = (AMT22.transfer(0) << 8) | AMT22.transfer(0);
        digitalWrite(AMT_SS, HIGH);
        if (Checksum::verifyAmtCheckbits(heading_raw))
        {
            // update heading if checksum is ok
            heading = float(heading_raw & 0x3fff) * 360. / 16385.;
        }

        // update telescope properties with current measurements
        telescope.setOrientation(heading + headingOffset, pitch);

        if (remoteClient && remoteClient.connected())
        {
            uint32_t received = remoteClient.read(rxBuffer, sizeof(rxBuffer));

            if (received > 0)
            {
                int32_t nexstarResponseLength = nexstar.handleRequest(rxBuffer, sizeof(rxBuffer), txBuffer, sizeof(txBuffer));

                if (nexstarResponseLength > 0)
                {
                    remoteClient.write(txBuffer, nexstarResponseLength);
                    nexStarMode = true;
                }
                else
                {
                    Equatorial reference;

                    // check if the packet could be decoded correctly before using it to calibrate the offset
                    if (telescope.unpackPosition(&reference, NULL, rxBuffer, received))
                    {
                        double localSiderealTimeDegrees = MathHelper::getLocalSiderealTimeDegrees(gnss.utcTimestamp, gnss.longitude);
                        Serial.printf("[ SENSOR ] Received Calibration Data  Ra: %.3f Dec: %.3f\n", reference.ra, reference.dec);
                        telescope.addReferencePoint(reference, gnss.latitude, localSiderealTimeDegrees);
                    }
                }
            }
        }

        if (mqttAvailable)
        {
            mqttClient.loop();
        }

        // if (!gyroCalibrationDone)
        // {
        //     ledmanager.setMode(LEDManager::LEDMode::BLINK_10HZ);
        // }
        if (!gnss.valid)
        {
            ledmanager.setMode(LEDManager::LEDMode::BLINK_1HZ);
        }
        else if (remoteClient && remoteClient.connected())
        {
            ledmanager.setMode(LEDManager::LEDMode::FLASH_2X_EVERY_5S);
        }
        else
        {
            ledmanager.setMode(LEDManager::LEDMode::FLASH_1X_EVERY_5S);
        }

        // base tasks
        if (wifiClientMode || wifiAccessPointMode)
            checkForConnections();

        if (localBrokerAvailable)
            broker.update();

        if (DEBUG)
        {
            // send data for SerialStudio
            Serial.printf("/*%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f*/\n",
                          heading, pitch, roll,
                          qw, qx, qy, qz,
                          accel.acceleration.x, accel.acceleration.y, accel.acceleration.z,
                          gyro.gyro.x, gyro.gyro.y, gyro.gyro.z,
                          mag.magnetic.x, mag.magnetic.y, mag.magnetic.z,
                          sensorUpdateRate);
        }
        return;
    }

    // 500ms Tasks
    if ((timestamp - task500msTimer) > 500000L)
    {
        task500msTimer = timestamp;

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

        if (orientationSensorAvailable)
        {
            double localSiderealTimeDegrees = MathHelper::getLocalSiderealTimeDegrees(gnss.utcTimestamp, gnss.longitude);
            Equatorial position = telescope.getCalibratedOrientation(gnss.latitude, localSiderealTimeDegrees);
            Horizontal corrected = telescope.equatorialToHorizontal(position, gnss.latitude, localSiderealTimeDegrees);

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

            len = snprintf((char *)txBuffer, sizeof(txBuffer), "%.3f", pitch);
            if (mqttAvailable)
                mqttClient.publish("home/appliance/telescope/orientation/pitch", txBuffer, len);
            if (localBrokerAvailable)
                broker.publish("home/appliance/telescope/orientation/pitch", (char *)txBuffer);

            len = snprintf((char *)txBuffer, sizeof(txBuffer), "%.3f", heading);
            if (mqttAvailable)
                mqttClient.publish("home/appliance/telescope/orientation/heading", txBuffer, len);
            if (localBrokerAvailable)
                broker.publish("home/appliance/telescope/orientation/heading", (char *)txBuffer);

            auto matrix = telescope.alignment.getTransformationMatrix(telescope.horizontalToEquatorial(telescope.orientation, gnss.latitude, localSiderealTimeDegrees));
            len = snprintf((char *)txBuffer, sizeof(txBuffer), "[%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                           matrix(0, 0), matrix(0, 1), matrix(0, 2),
                           matrix(1, 0), matrix(1, 1), matrix(1, 2),
                           matrix(2, 0), matrix(2, 1), matrix(2, 2));
            if (mqttAvailable)
                mqttClient.publish("home/appliance/telescope/orientation/matrix", txBuffer, len);
            if (localBrokerAvailable)
                broker.publish("home/appliance/telescope/orientation/matrix", (char *)txBuffer);

            len = snprintf((char *)txBuffer, sizeof(txBuffer),
                           "{\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,\"gx\":%.3f,\"gy\":%.3f,\"gz\":%.3f,\"mx\":%.3f,\"my\":%.3f,\"mz\":%.3f}",
                           accel.acceleration.x, accel.acceleration.y, accel.acceleration.z,
                           gx, gy, gz,
                           mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);

            if (mqttAvailable)
                mqttClient.publish("home/appliance/telescope/orientation/raw", txBuffer, len);
            if (localBrokerAvailable)
                broker.publish("home/appliance/telescope/orientation/raw", (char *)txBuffer);
        }

        return;
    }

    // 1s Tasks
    if ((timestamp - task1sTimer) > 1000000L)
    {
        task1sTimer = timestamp;

        return;
    }

    // 2s Tasks
    if ((timestamp - task2sTimer) > 2000000L)
    {
        task2sTimer = timestamp;

        counter2s++;

        txBuffer[0] = 0;
        txBuffer[1] = 0;

        rxBuffer[0] = 1;
        rxBuffer[1] = 2;

        if (gnssAvailable)
        {
            if (!DEBUG)
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
            {
                broker.publish("home/appliance/telescope/gnss", (char *)txBuffer);
            }
            if (mqttAvailable)
            {
                mqttClient.publish("home/appliance/telescope/gnss", txBuffer, len);
            }
        }

        if (mqttAvailable && !mqttClient.connected())
        {
            reconnectMQTTClient();
        }
        return;
    }

    // 30s Tasks
    if ((timestamp - task30sTimer) > 30000000L)
    {
        task30sTimer = timestamp;

        // show overall system state
        if (!DEBUG)
        {
            Serial.printf("[ STATUS ] Free: %u KiB (%u KiB)   RSSI: %i dBm   Uptime: %" PRIi64 "s   GNSS: %s\n",
                          ESP.getFreeHeap() / 1024,
                          ESP.getMaxAllocHeap() / 1024,
                          WiFi.RSSI(),
                          (esp_timer_get_time() / 1000000LL),
                          gnss.valid ? "fix" : "no fix");
        }
    }
}
