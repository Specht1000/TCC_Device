#ifndef SOIL_MOISTURE_H
#define SOIL_MOISTURE_H

#include <Arduino.h>

class SoilMoistureSensor {
public:
    SoilMoistureSensor(int analogPin, int digitalPin = -1);
    void beginSoilM();
    int readAnalog();
    bool readDigital();
    bool isSoilMConnected();

private:
    int _analogPin;
    int _digitalPin;
};

#endif // SOIL_MOISTURE_H
