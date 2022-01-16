#ifndef MQTTBROKER_H
#define MQTTBROKER_H

using namespace std;

#include <sMQTTBroker.h>
#include <Arduino.h>

class MQTTBroker : public sMQTTBroker
{
public:
    MQTTBroker(HardwareSerial *serial);
    bool onConnect(sMQTTClient *client, const std::string &username, const std::string &password);
    void onRemove(sMQTTClient *client);
    void onPublish(sMQTTClient *client, const std::string &topic, const std::string &payload);
private:
    HardwareSerial *serial;
};

#endif // MQTTBROKER_H