#ifndef SOIL_PH_H
#define SOIL_PH_H

#include <Arduino.h>

class SoilPHSensor {
public:
    SoilPHSensor(uint8_t analogPin);
    void beginSoilPH();
    float readSoilPH();
    bool isSoilPHConnected(); 

private:
    uint8_t _analogPin;
};

#endif // SOIL_PH_H
