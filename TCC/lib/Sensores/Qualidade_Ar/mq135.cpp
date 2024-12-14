#include "mq135.h"
#include "main.h"

MQ135Sensor::MQ135Sensor(int pin) : _pin(pin) {}

void MQ135Sensor::beginMQ135()
{
    pinMode(_pin, INPUT);
    LOG("MQ135", "Sensor MQ135 inicializado no pino %d", _pin);
}

bool MQ135Sensor::readAirQuality(float &ppm)
{
    int sensorValue = analogRead(_pin);
    // Convertendo o valor lido para ppm (ajustar com base na calibração do sensor)
    ppm = sensorValue * (5.0 / 1024.0) * 100;

    if (sensorValue < 0)
    {
        LOG("MQ135", "Falha na leitura do sensor MQ135");
        return false;
    }
    else
    {
        LOG("MQ135", "Qualidade do ar: %.2f ppm", ppm);
        return true;
    }
}

bool MQ135Sensor::isMQ135Connected()
{
    int sensorValue = analogRead(_pin); // Leitura do valor analógico

    if (sensorValue < 0 || sensorValue > 1023)
    {
        LOG("MQ135", "Sensor desconectado ou com falha");
        return false;
    }
    else
    {
        LOG("MQ135", "Sensor conectado e operando corretamente");
        return true;
    }
}