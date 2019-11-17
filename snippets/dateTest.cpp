#include "pseudo_arduino.h"
#include "../dateutil.h"

int main(int argc, char const *argv[]) {   
    const char* weekd[] = { "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};

    for (int i = 10000; i <= 37000; i+=1) {
        std::time_t seconds = i * 24l * 60l * 60l;
        std::time_t t = seconds;
        date::RTC rtc;
        date::epoc2rtc(i, rtc);
        std::tm* tt = std::gmtime(&t);

        if (tt->tm_year != (rtc.year - 1900) ||
            tt->tm_mon != rtc.month ||
            tt->tm_mday != (rtc.day + 1) ||
            tt->tm_wday != ((rtc.dow + 1) % 7)  ||
            tt->tm_yday != rtc.doy) {
            
            printf("%d ::: %d%s %s %d %d (%d %d) -> %s \n", i, 
                rtc.year, rtc.leapYear ? "(leap)" : "", 
                weekd[rtc.dow],
                (rtc.day + 1),
                rtc.month, 
                rtc.doy,
                rtc.dayOfCycle,
                std::asctime(tt));
        }
    }

}