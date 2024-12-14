#include "umidade_solo.h"
#include "main.h" 

SoilMoistureSensor::SoilMoistureSensor(int analogPin, int digitalPin) 
    : _analogPin(analogPin), _digitalPin(digitalPin) {}

void SoilMoistureSensor::beginSoilM() 
{
    pinMode(_analogPin, INPUT);
    if (_digitalPin != -1) {
        pinMode(_digitalPin, INPUT);
    }
    LOG("SOIL", "Sensor de umidade do solo inicializado nos pinos A%d e D%d", _analogPin, _digitalPin);
}

int SoilMoistureSensor::readAnalog() 
{
    int value = analogRead(_analogPin);
    LOG("SOIL", "Leitura analogica: %d", value);
    return value;
}

bool SoilMoistureSensor::readDigital() 
{
    if (_digitalPin == -1) 
    {
        LOG("SOIL", "Erro: pino digital nao definido.");
        return false;
    }
    bool state = digitalRead(_digitalPin);
    LOG("SOIL", "Leitura digital: %s", state ? "HIGH" : "LOW");
    return state;
}

bool SoilMoistureSensor::isSoilMConnected() 
{
    int value = analogRead(_analogPin);
    if (value == 0 || value == 1023) 
    {
        LOG("SOIL", "Sensor desconectado ou com falha.");
        return false;
    }
    LOG("SOIL", "Sensor conectado e funcional.");
    return true;
}
