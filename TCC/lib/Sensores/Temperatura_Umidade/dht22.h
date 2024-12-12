#ifndef DHT22_SENSOR_H
#define DHT22_SENSOR_H

#include <DHT.h>

class DHT22Sensor {
public:
    DHT22Sensor(int pin);
    void beginDHT22();
    bool readDHT22(float &temperature, float &humidity);
    bool isDHT22Connected();

private:
    int _pin;
    DHT _dht;
};

#endif // DHT22_SENSOR_H
