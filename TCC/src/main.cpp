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
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFiClient.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "main.h"
#include "../lib/Sensores/Temperatura_Umidade/dht22.h"
#include "../lib/Sensores/Luminosidade/bh1750.h"
#include "../lib/Sensores/Qualidade_Ar/mq135.h"
#include "../lib/Sensores/CO2/mhz19.h"
#include "../lib/Sensores/Umidade_Solo/umidade_solo.h"
#include "../lib/Sensores/pH_Solo/ph_solo.h"
#include "tasks/task_monitor.h"
#include "rtc.h"

const char* ssid = "igoal_24G";
const char* password = "igoal@2021";

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
const char* mqttBroker = "broker.hivemq.com";
const int mqttPort = 1883;
const char* mqttTopic = "telemetry/data";

DHT22Sensor dhtSensor(15);              // Pino 15 para DHT22
BH1750Sensor bh1750Sensor;              // I2C (pinos 21 e 22)
MQ135Sensor mq135Sensor(34);            // Pino 34 para MQ135
MHZ19Sensor mhz19Sensor(Serial2);       // Serial2 para MH-Z19
SoilMoistureSensor soilSensor(32, 35);  // Pinos 32 (analógico) e 35 (digital) para sensor de umidade do solo
SoilPHSensor soilPHSensor(36);          // Pino 36 analógico para o sensor de pH
RTC_DS3231 rtcDs3231;                   // Objeto RTC no I2C

SemaphoreHandle_t i2cMutex;

void connectToWiFi();

void taskDHT22(void* parameter);
void taskBH1750(void* parameter);
void taskMQ135(void* parameter);
void taskMHZ19(void* parameter);
void taskSoilMoisture(void* parameter);
void taskSoilPH(void* parameter);

void publishTelemetry();

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
    mhz19Sensor.beginMHZ19(115200);
    soilSensor.beginSoilM();
    soilPHSensor.beginSoilPH();
    
    connectToWiFi();

    mqttClient.setServer(mqttBroker, mqttPort);

    xTaskCreate(taskDHT22, "taskDHT22", 2048, NULL, 1, NULL);
    xTaskCreate(taskBH1750, "taskBH1750", 2048, NULL, 1, NULL);
    xTaskCreate(taskMQ135, "taskMQ135", 2048, NULL, 1, NULL);
    xTaskCreate(taskMHZ19, "taskMHZ19", 2048, &mhz19Sensor, 1, NULL);
    xTaskCreate(taskSoilMoisture, "taskSoil", 2048, NULL, 1, NULL);
    xTaskCreate(taskSoilPH, "taskSoilPH", 2048, NULL, 1, NULL);

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

    mqttClient.loop();
    publishTelemetry();
    delay(60000);
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

void publishTelemetry() {
    StaticJsonDocument<512> jsonDoc;

    // Adiciona dados do DHT22
    if (dhtSensor.isDHT22Connected()) {
        float temperature = 0.0, humidity = 0.0;
        dhtSensor.readDHT22(temperature, humidity);
        jsonDoc["temperature"] = temperature;
        jsonDoc["humidity"] = humidity;
    } else {
        jsonDoc["temperature"] = "NaN";
        jsonDoc["humidity"] = "NaN";
    }

    // Adiciona dados do BH1750
    if (bh1750Sensor.isBH1750Connected()) {
        float lux = 0.0;
        bh1750Sensor.readBH1750(lux);
        jsonDoc["lux"] = lux;
    } else {
        jsonDoc["lux"] = "NaN";
    }

    // Adiciona dados do MQ135
    if (mq135Sensor.isMQ135Connected()) {
        float airQuality = 0.0;
        mq135Sensor.readAirQuality(airQuality);
        jsonDoc["airQuality"] = airQuality;
    } else {
        jsonDoc["airQuality"] = "NaN";
    }

    // Adiciona dados do MH-Z19
    if (mhz19Sensor.isMHZ19Connected()) {
        int co2 = 0;
        mhz19Sensor.readCO2(co2);
        jsonDoc["co2"] = co2;
    } else {
        jsonDoc["co2"] = "NaN";
    }

    // Adiciona dados de umidade do solo
    if (soilSensor.isSoilMConnected()) {
        int soilAnalog = soilSensor.readAnalog();
        jsonDoc["soilMoisture"] = soilAnalog;
    } else {
        jsonDoc["soilMoisture"] = "NaN";
    }

    // Adiciona dados de pH do solo
    if (soilPHSensor.isSoilPHConnected()) {
        float ph = soilPHSensor.readSoilPH();
        jsonDoc["soilPH"] = ph;
    } else {
        jsonDoc["soilPH"] = "NaN";
    }

    // Adiciona data/hora do RTC
    RTCx::tm rtcTime = rtcDs3231.getTimeRTC(false);
    char timeBuffer[20];
    snprintf(timeBuffer, sizeof(timeBuffer), "%02d/%02d/%04d %02d:%02d:%02d", rtcTime.tm_mday, rtcTime.tm_mon + 1, rtcTime.tm_year + 1900, rtcTime.tm_hour, rtcTime.tm_min, rtcTime.tm_sec);
    jsonDoc["timestamp"] = String(timeBuffer);

    // Serializa JSON e publica
    char jsonBuffer[512];
    serializeJson(jsonDoc, jsonBuffer);

    if (mqttClient.publish(mqttTopic, jsonBuffer)) {
        LOG("MQTT", "Dados publicados com sucesso: ");
        Serial.println(jsonBuffer);
    } else {
        LOG("MQTT", "Falha ao publicar os dados MQTT.");
    }
}

void taskDHT22(void* parameter) {
    LOG("DHT22", "Task DHT22 iniciada.");
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
    LOG("MHZ19", "Task MHZ19 iniciada.");
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

void taskSoilPH(void* parameter) {
    LOG("PH", "Task de pH do solo iniciada.");
    while (true) {
        startTaskTimer(MONITOR_SOIL_PH); // Início do timer

        if (soilPHSensor.isSoilPHConnected()) {
            float phValue = soilPHSensor.readSoilPH();
            LOG("PH", "Valor de pH do solo: %.2f", phValue);
        } else {
            LOG("PH", "Sensor de pH do solo desconectado!");
        }

        endTaskTimer(MONITOR_SOIL_PH); // Fim do timer
        vTaskDelay(pdMS_TO_TICKS(10000)); // Aguarda 10 segundos
    }
}

