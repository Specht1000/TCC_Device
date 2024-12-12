#ifndef MHZ19_H
#define MHZ19_H

#include <HardwareSerial.h> // Use HardwareSerial para ESP32

class MHZ19Sensor {
public:
    MHZ19Sensor(HardwareSerial &serialPort);
    void beginMHZ19(int baudRate = 115200); // Agora aceita a taxa de baud
    bool readCO2(int &co2);
    bool isMHZ19Connected();

private:
    HardwareSerial &_serial; // Use HardwareSerial em vez de SoftwareSerial
};

#endif // MHZ19_H
