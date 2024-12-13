#include "ph_solo.h"

SoilPHSensor::SoilPHSensor(uint8_t analogPin) : _analogPin(analogPin) {}

void SoilPHSensor::beginSoilPH() {
    pinMode(_analogPin, INPUT);
}

float SoilPHSensor::readSoilPH() {
    int analogValue = analogRead(_analogPin);
    // Converte o valor analógico para pH (exemplo genérico)
    float voltage = analogValue * (5.0 / 1023.0); // Para placas de 10 bits, ajuste se necessário
    float phValue = 3.5 * voltage; // Fórmula depende do sensor usado
    return phValue;
}

bool SoilPHSensor::isSoilPHConnected() {
    int analogValue = analogRead(_analogPin);
    return (analogValue > 0 && analogValue < 1023); // Sensor é considerado conectado se valores são válidos
}
