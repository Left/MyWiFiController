#pragma once

#include <stdint.h>

namespace date {
    const uint8_t leapYearMonth[] = {
        31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
    };
    const uint8_t regularYearMonth[] = {
        31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
    };
    const int cycle400year = 365*400 + (100 - 3);
    const int cycle100year = 365*100 + 25 - 1;
    const int cycle4year = 365*4 + 1;

    struct RTC {
        uint16_t dow;
        uint16_t day;
        uint16_t month;
        uint16_t year; // 
        bool leapYear;
        uint16_t doy; // 0-based
        uint32_t dayOfCycle;
    };

    static void epoc2rtc(uint32_t t, RTC &rtc) {
        rtc.dow = (t + 3) % 7; // Day of week
        rtc.dayOfCycle = (((t + 719162 - 365) % cycle400year) % cycle100year) % cycle4year;
        int yearOfCycle = 0;
        rtc.year = 1970 + 
            t / cycle400year * 400 + 
            t / cycle100year * 100 + 
            t / cycle4year * 4;

        for (;;) {
            // printf("%d \n", rtc.year);
            bool leapYear = false;
            int days = 365;
            if (rtc.year % 400 == 0 || (rtc.year % 100 != 0 && rtc.year % 4 == 0)) {
                days++; // Leap year
            }
            if (rtc.dayOfCycle >= days) {
                rtc.dayOfCycle -= days;
                rtc.year++;
                rtc.leapYear = rtc.year % 400 == 0 || (rtc.year % 100 != 0 && rtc.year % 4 == 0);
            } else {
                break;
            }
        }

        const uint8_t* dm = rtc.leapYear ? leapYearMonth : regularYearMonth;
        rtc.doy = rtc.dayOfCycle;
        rtc.month = 0;
        int d = rtc.doy;
        for (;d >= dm[rtc.month];) {
            d -= dm[rtc.month];
            rtc.month++;
        }
        rtc.day = d;
    }
}