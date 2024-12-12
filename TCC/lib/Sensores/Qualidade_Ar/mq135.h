#ifndef MQ135_H
#define MQ135_H

#include <Arduino.h>

class MQ135Sensor {
public:
    MQ135Sensor(int pin);
    void beginMQ135();
    bool readAirQuality(float &ppm);
    bool isMQ135Connected();

private:
    int _pin;
};

#endif // MQ135_H