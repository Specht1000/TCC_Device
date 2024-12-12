#ifndef RTC_DS3231_H
#define RTC_DS3231_H

#include <Wire.h>
#include <RTCx.h> // Biblioteca RTCx

class RTC_DS3231 {
public:
    RTC_DS3231();
    void initRtc();
    RTCx::tm getTimeRTC(bool showTime);
    void setRtc(uint8_t day, uint8_t month, uint16_t year, uint8_t hour, uint8_t min, uint8_t sec);
    time_t getTimestampRTC();
    void setTimestampRtc(time_t timestamp);

private:
    RTCx rtc; // Objeto interno RTCx
};

#endif // RTC_DS3231_H
