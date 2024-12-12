#include "rtc.h"
#include "main.h"

RTC_DS3231::RTC_DS3231() {}

void RTC_DS3231::initRtc() {
    Wire.begin(21, 22, 400000U); // Inicializa I2C
    if (rtc.autoprobe()) {
        LOG("RTC", "RTC conectado e inicializado.");
        rtc.startClock();
        rtc.setSQW(RTCx::freq4096Hz);
    } else {
        LOG("RTC", "RTC não conectado ou com falha.");
    }
}

RTCx::tm RTC_DS3231::getTimeRTC(bool showTime) {
    RTCx::tm tm;
    if (rtc.readClock(tm)) {
        if (showTime) {
            LOG("RTC", "Hora: %02d:%02d:%02d, Data: %02d/%02d/%04d",
                tm.tm_hour, tm.tm_min, tm.tm_sec,
                tm.tm_mday, tm.tm_mon + 1, tm.tm_year + 1900);
        }
    } else {
        LOG("RTC", "Erro ao ler o RTC.");
    }
    return tm;
}

void RTC_DS3231::setRtc(uint8_t day, uint8_t month, uint16_t year, uint8_t hour, uint8_t min, uint8_t sec) {
    RTCx::tm tm = {0};
    tm.tm_mday = day;
    tm.tm_mon = month - 1;
    tm.tm_year = year - 1900;
    tm.tm_hour = hour;
    tm.tm_min = min;
    tm.tm_sec = sec;
    rtc.setClock(&tm);

    LOG("RTC", "RTC atualizado: %02d/%02d/%04d %02d:%02d:%02d",
        day, month, year, hour, min, sec);
}
