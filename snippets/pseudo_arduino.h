#include <stdio.h>
#include <unistd.h>
#include <term.h>
#include <string>

#include <sys/time.h>
#include <iomanip>
#include <sstream>
#include <unistd.h>

#include <string.h>
#include <stdlib.h>

#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2
#define BYTE 0

#define PROGMEM
#define HIGH 0x1
#define LOW  0x0

#define INPUT 0x0
#define OUTPUT 0x1

#define LSBFIRST 0 
#define MSBFIRST 1

#define CHANGE 1
#define FALLING 2
#define RISING 3

#define INTERNAL 3
#define DEFAULT 1
#define EXTERNAL 0

void delay(unsigned long) {}
void pinMode(int,int) {}
void digitalWrite(int pin, int val) {}
void shiftOut(int dataPin, int clockPin, int bitOrder, int value);

uint64_t micros64() {
    timeval tv1 = {0};
    struct timezone tz = {0};
    gettimeofday(&tv1, &tz);
    return (((uint64_t)tv1.tv_sec - tz.tz_minuteswest*60ULL) * 1000000ULL + tv1.tv_usec);
}

uint32_t micros() {
    return micros64() & 0xffffffff;
}

uint32_t millis() {
    return micros64() / 1000ull;
}