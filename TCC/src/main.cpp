/******************************************************************************
 * Projeto: Dispositivo de Telemetria para Agricultura de Precisão e 
 *          Monitoramento Ambiental
 * Autor: Guilherme Martins Specht
 * Orientador: 
 * Data: 
 * Descrição: Firmware de coleta e análise de dados
 *****************************************************************************/

#include <Arduino.h>
#include <WiFi.h>
#include "main.h"
#include "../lib/Sensores/Temperatura_Umidade/dht22.h"
#include "../lib/Sensores/Luminosidade/bh1750.h"
#include "../lib/Sensores/Qualidade_Ar/mq135.h"
#include "../lib/Sensores/CO2/mhz19.h"
#include "../lib/Sensores/Umidade_Solo/soil_moisture.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "tasks/task_monitor.h"
#include "rtc.h"

const char* ssid = "igoal_24G";
const char* password = "igoal@2021";

DHT22Sensor dhtSensor(15);              // Pino 15 para DHT22
BH1750Sensor bh1750Sensor;              // I2C (pinos 21 e 22)
MQ135Sensor mq135Sensor(34);            // Pino 34 para MQ135
MHZ19Sensor mhz19Sensor(Serial2);       // Serial2 para MH-Z19
SoilMoistureSensor soilSensor(32, 35);  // Pinos 32 (analógico) e 35 (digital) para sensor de umidade do solo
RTC_DS3231 rtcDs3231;                   // Objeto RTC no I2C

SemaphoreHandle_t i2cMutex;

void connectToWiFi();

void taskDHT22(void* parameter);
void taskBH1750(void* parameter);
void taskMQ135(void* parameter);
void taskMHZ19(void* parameter);
void taskSoilMoisture(void* parameter);

void setup() 
{
    Serial.begin(115200);
    Wire.begin(21, 22);
    Serial.print("/******************************************************************************\n");
    Serial.print(" * Projeto: Dispositivo de Telemetria para Agricultura de Precisao e\n" );
    Serial.print(" *          Monitoramento Ambiental\n");
    Serial.print(" * Autor: Guilherme Martins Specht\n");
    Serial.print(" * Orientador: \n");
    Serial.print(" * Data: \n");
    Serial.print(" * Descricao: Firmware de coleta e analise de dados\n");
    Serial.print("/******************************************************************************\n");
    LOG("SETUP", "Inicializando o sistema...");

    rtcDs3231.initRtc(); 

    mhz19Sensor.beginMHZ19(115200);
    LOG("MHZ19", "Sensor MH-Z19 configurado.");

    i2cMutex = xSemaphoreCreateMutex();
    if (i2cMutex == NULL) {
        LOG("I2C", "Falha ao criar o mutex.");
    } 
    else {
        LOG("I2C", "Mutex criado com sucesso.");
    }

    dhtSensor.beginDHT22();
    bh1750Sensor.beginBH1750();
    mq135Sensor.beginMQ135();
    soilSensor.beginSoilM();
    
    connectToWiFi();

    xTaskCreate(taskDHT22, "taskDHT22", 2048, NULL, 1, NULL);
    xTaskCreate(taskBH1750, "taskBH1750", 2048, NULL, 1, NULL);
    xTaskCreate(taskMQ135, "taskMQ135", 2048, NULL, 1, NULL);
    xTaskCreate(taskMHZ19, "taskMHZ19", 2048, &mhz19Sensor, 1, NULL);
    xTaskCreate(taskSoilMoisture, "taskSoil", 2048, NULL, 1, NULL);

    xTaskCreate(taskMonitorTasks, "taskMonitor", 4096, NULL, 1, NULL);
}

void loop() 
{
    static unsigned long lastWiFiCheck = 0;
    unsigned long currentMillis = millis();

    // Verifica o WiFi a cada 5 segundos
    if (currentMillis - lastWiFiCheck >= 5000) {
        lastWiFiCheck = currentMillis;

        if (WiFi.status() != WL_CONNECTED) {
            LOG("WIFI", "WiFi desconectado! Tentando reconectar...");
            connectToWiFi();
        }
    }
}

void connectToWiFi() 
{
    LOG("WIFI", "Tentando conectar a rede WiFi: %s", ssid);
    WiFi.begin(ssid, password);

    int retries = 0;
    int maxRetries = 10;

    while (WiFi.status() != WL_CONNECTED && retries < maxRetries) 
    {
        LOG("WIFI", "Tentativa %d de %d para conectar ao WiFi...", retries + 1, maxRetries);
        delay(1000);
        retries++;
    }

    if (WiFi.status() == WL_CONNECTED) 
    {
        LOG("WIFI", "Conectado com sucesso ao WiFi!");
        LOG("WIFI", "Endereco IP: %s", WiFi.localIP().toString().c_str());
        LOG("WIFI", "SSID: %s || Password: %s", ssid, password);
    } 
}

void taskDHT22(void* parameter) {
    LOG("DHT22", "Task DHT iniciada.");
    while (true) {
        startTaskTimer(MONITOR_DHT22); // Início do timer
        if (xSemaphoreTake(i2cMutex, portMAX_DELAY)) {
            float temperature = 0.0;
            float humidity = 0.0;

            if (dhtSensor.isDHT22Connected()) {
                dhtSensor.readDHT22(temperature, humidity);
            }
            xSemaphoreGive(i2cMutex);
        }
        endTaskTimer(MONITOR_DHT22); // Fim do timer
        vTaskDelay(pdMS_TO_TICKS(20000)); // Aguarda 20 segundos
    }
}

void taskBH1750(void* parameter) {
    LOG("BH1750", "Task BH1750 iniciada.");
    while (true) {
        startTaskTimer(MONITOR_BH1750); // Início do timer
        if (xSemaphoreTake(i2cMutex, portMAX_DELAY)) {
            float lux = 0.0;

            if (bh1750Sensor.isBH1750Connected()) {
                bh1750Sensor.readBH1750(lux);
            }
            xSemaphoreGive(i2cMutex);
        }
        endTaskTimer(MONITOR_BH1750); // Fim do timer
        vTaskDelay(pdMS_TO_TICKS(5000)); // Aguarda 5 segundos
    }
}

void taskMQ135(void* parameter) {
    LOG("MQ135", "Task MQ135 iniciada.");
    while (true) {
        startTaskTimer(MONITOR_MQ135); // Início do timer

        if (mq135Sensor.isMQ135Connected()) { // Verifica se o sensor está conectado
            float ppm = 0.0;

            if (mq135Sensor.readAirQuality(ppm)) {
                LOG("MQ135", "Qualidade do ar: %.2f ppm", ppm);
            } else {
                LOG("MQ135", "Falha na leitura do MQ135.");
            }
        }
        else {
            LOG("MQ135", "Sensor MQ135 desconectado!");
        }

        endTaskTimer(MONITOR_MQ135); // Fim do timer
        vTaskDelay(pdMS_TO_TICKS(15000)); // Aguarda 15 segundos
    }
}

void taskMHZ19(void* parameter) {
    LOG("MHZ19", "Task MH-Z19 iniciada.");
    MHZ19Sensor* sensor = static_cast<MHZ19Sensor*>(parameter);

    if (sensor == nullptr) {
        LOG("MHZ19", "Sensor não inicializado. Finalizando task.");
        vTaskDelete(NULL);
        return;
    }

    while (true) {
        startTaskTimer(MONITOR_MHZ19); // Início do timer

        if (sensor->isMHZ19Connected()) {
            int co2 = 0;
            if (sensor->readCO2(co2)) {
                LOG("MHZ19", "Concentração de CO2: %d ppm", co2);
            } else {
                LOG("MHZ19", "Falha na leitura do sensor MH-Z19.");
            }
        } else {
            LOG("MHZ19", "Sensor MH-Z19 desconectado!");
        }

        endTaskTimer(MONITOR_MHZ19); // Fim do timer
        vTaskDelay(pdMS_TO_TICKS(10000)); // Aguarda 10 segundos
    }
}

void taskSoilMoisture(void* parameter) {
    LOG("SOIL", "Task Soil Moisture iniciada.");
    while (true) {
        startTaskTimer(MONITOR_SOIL); // Início do timer
        
        if (soilSensor.isSoilMConnected()) {
            int analogValue = soilSensor.readAnalog();
            bool digitalValue = soilSensor.readDigital();
            LOG("SOIL", "Umidade analogica: %d, Digital: %s", analogValue, digitalValue ? "HIGH" : "LOW");
        } else {
            LOG("SOIL", "Sensor de umidade do solo desconectado!");
        }

        endTaskTimer(MONITOR_SOIL); // Fim do timer
        vTaskDelay(pdMS_TO_TICKS(5000)); // Aguarda 5 segundos
    }
}
