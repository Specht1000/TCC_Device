#ifndef BH1750_H
#define BH1750_H

#include <BH1750.h>
#include <Wire.h>

class BH1750Sensor {
public:
    BH1750Sensor();
    void beginBH1750();
    bool readBH1750(float &lux);
    bool isBH1750Connected();

private:
    BH1750 _lightMeter;
};

#endif // BH1750_H
