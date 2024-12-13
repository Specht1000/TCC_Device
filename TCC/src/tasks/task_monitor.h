#ifndef TASKS_MONITOR_H
#define TASKS_MONITOR_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "main.h"

// Enumerado para identificar as tasks
enum TASKS_TIMER {
    MONITOR_DHT22 = 1,
    MONITOR_BH1750,
    MONITOR_WIFI,
    MONITOR_MQ135,
    MONITOR_MHZ19,
    MONITOR_SOIL,
    MONITOR_SOIL_PH,
    MONITOR_COUNT   // Número total de tasks
};

// Estrutura para associar o valor do enumerado e o nome da task
typedef struct {
    TASKS_TIMER task;
    const char* name;
} TaskInfo;

// Array de TaskInfo para associar os valores do enumerado aos nomes das tasks
const TaskInfo taskInfoArray[MONITOR_COUNT] = {
    {MONITOR_DHT22,  "DHT22 "},
    {MONITOR_BH1750, "BH1750"},
    {MONITOR_WIFI,   "WIFI  "},
    {MONITOR_MQ135,  "MQ135 "},
    {MONITOR_MHZ19,  "MHZ19 "},
    {MONITOR_SOIL,   "SOIL  "},
    {MONITOR_SOIL_PH,"PH    "}
};

// Estrutura para armazenar o tempo de execução e o contador de execuções de cada task
typedef struct {
    TickType_t startTime;
    TickType_t endTime;
    TickType_t executionTime;
    uint32_t executionCount;
} TaskExecutionTime;

void taskMonitorTasks(void* pvParameters);
void startTaskTimer(TASKS_TIMER task);
void endTaskTimer(TASKS_TIMER task);

#endif // TASKS_MONITOR_H
