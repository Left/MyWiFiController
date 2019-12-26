#include <stdio.h>
#include <string>

#include <unistd.h>
#include <term.h>
#include <sys/time.h>

#include "ledStripe.h"

uint64_t micros64() {
    timeval tv1 = {0};
    struct timezone tz = {0};
    gettimeofday(&tv1, &tz);
    return (((uint64_t)tv1.tv_sec - tz.tz_minuteswest*60ULL) * 1000000ULL + tv1.tv_usec);
}

int throwErr(std::string&& err) {
    printf("%s\n", err.c_str());
    throw -1;
}

void printLed(const LedStripe& l) {
    putp(tigetstr((char *)"clear"));
    putp( tparm( tigetstr((char *)"cup" ), 0, 0, 0, 0, 0, 0, 0, 0, 0 ) );

    for (auto i = 0; i < l.getPixelCount(); ++i) {
        printf("%08x ", l.getPixel(i));
    }

    printf("\n");
}

int main( int argc, char **argv) {
    int result;
    setupterm( NULL, STDOUT_FILENO, &result );
    if (result <= 0) return -1;

    const uint32_t NUMPIXELS = 10;
    std::vector<uint32_t> leds(NUMPIXELS, 0x00203040);

    LedStripe stripe(NUMPIXELS, [] () { return micros64() / 1000l; });

    // stripe.set(leds, 10000);
    stripe.runLedStripeEffect(0x00101010, {0x00ff00ff, 0x00ff0000, 0x000000ff}, 10000);
    // printLed(stripe);

    // leds.at(0) = 0x000000ff;
    // stripe.set(leds, 100, 0);
    for (int i = 0; i <= 10000; i += 10) {
        stripe.update();
        printLed(stripe);
        printf("\n %d \n", i);
        usleep(100000l); // will sleep for 10 ms
    }
    
    // throwErr("Hey");
}