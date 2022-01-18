#include <mqttbroker.h>

#ifdef ARDUINO

MQTTBroker::MQTTBroker(HardwareSerial *serial)
{
    this->serial = serial;
}

bool MQTTBroker::onConnect(sMQTTClient *client, const std::string &username, const std::string &password)
{
    this->serial->printf("[ BROKER ] Client '%s' connected\n", client->getClientId().c_str());
    return true;
};

void MQTTBroker::onRemove(sMQTTClient *client)
{
    this->serial->printf("[ BROKER ] '%s' disconnected\n", client->getClientId().c_str());
};

void MQTTBroker::onPublish(sMQTTClient *client, const std::string &topic, const std::string &payload)
{
    this->serial->printf("[ BROKER ] Client '%s' published topic '%s': '%s'\n", client->getClientId().c_str(), topic.c_str(), payload.c_str());
}
#endif