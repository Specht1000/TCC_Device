#include "dht22.h"
#include "main.h"
#include <Arduino.h>

DHT22Sensor::DHT22Sensor(int pin) : _pin(pin), _dht(pin, DHT22) {}

void DHT22Sensor::beginDHT22() 
{
    _dht.begin();
}

bool DHT22Sensor::readDHT22(float &temperature, float &humidity) 
{
    temperature = _dht.readTemperature();
    humidity = _dht.readHumidity();

    if (isnan(temperature) || isnan(humidity)) 
    {
        LOG("DHT22", "Falha na leitura do sensor");
        return false;
    } 
    else 
    {
        LOG("DHT22", "Temperatura: %.2f C, Umidade: %.2f %%", temperature, humidity);
        return true;
    }
}

bool DHT22Sensor::isDHT22Connected() 
{
    float testTemperature = _dht.readTemperature();

    if (isnan(testTemperature)) 
    {
        LOG("DHT22", "Sensor desconectado ou com falha");
        return false;
    } 
    else 
    {
        LOG("DHT22", "Sensor conectado e operando corretamente");
        return true;
    }
}
