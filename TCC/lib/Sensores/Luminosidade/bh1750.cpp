#include "bh1750.h"
#include "main.h"
#include <Arduino.h>

BH1750Sensor::BH1750Sensor() : _lightMeter() {}

void BH1750Sensor::beginBH1750() 
{
    if (_lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) 
    {
        LOG("BH1750", "Sensor BH1750 inicializado com sucesso");
    } 
    else 
    {
        LOG("BH1750", "Falha ao inicializar o sensor BH1750");
    }
}

bool BH1750Sensor::readBH1750(float &lux) 
{
    lux = _lightMeter.readLightLevel();

    if (lux < 0) 
    {
        LOG("BH1750", "Falha na leitura do sensor");
        return false;
    } 
    else 
    {
        LOG("BH1750", "Luminosidade: %.2f lux", lux);
        return true;
    }
}

bool BH1750Sensor::isBH1750Connected() 
{
    Wire.beginTransmission(0x23);
    uint8_t error = Wire.endTransmission();

    if (error == 0) 
    {
        LOG("BH1750", "Sensor conectado e respondendo");
        return true;
    } 
    else 
    {
        LOG("BH1750", "Sensor desconectado ou com falha");
        return false;
    }
}
