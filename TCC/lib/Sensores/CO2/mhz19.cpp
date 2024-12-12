#include "mhz19.h"
#include "main.h"

MHZ19Sensor::MHZ19Sensor(HardwareSerial &serialPort) : _serial(serialPort) {}

void MHZ19Sensor::beginMHZ19(int baudRate) {
    _serial.begin(baudRate);
    LOG("MHZ19", "Sensor MH-Z19 inicializado com taxa de baud %d", baudRate);
}

bool MHZ19Sensor::readCO2(int &co2) {
    uint8_t cmd[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
    uint8_t response[9];

    _serial.write(cmd, 9);
    LOG("MHZ19", "Comando enviado ao sensor MH-Z19.");
    delay(100);

    if (_serial.available() >= 9) {
        LOG("MHZ19", "Resposta recebida do sensor.");
        for (int i = 0; i < 9; i++) {
            response[i] = _serial.read();
        }

        if (response[0] == 0xFF && response[1] == 0x86) {
            co2 = (response[2] << 8) + response[3];
            LOG("MHZ19", "Concentração de CO2: %d ppm", co2);
            return true;
        } else {
            LOG("MHZ19", "Resposta inválida do sensor MH-Z19.");
            return false;
        }
    } else {
        LOG("MHZ19", "Nenhuma resposta do sensor MH-Z19.");
        return false;
    }
}

bool MHZ19Sensor::isMHZ19Connected() {
    int dummyCO2;
    return readCO2(dummyCO2);
}
