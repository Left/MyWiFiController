#include "pseudo_arduino.h"

#include "../src/protocol.pb.h"
#include "../src/lcd.h"

int main(int argc, char const *argv[]) {   
    if (!cur_term) {
        int result;
        setupterm( NULL, STDOUT_FILENO, &result );
        if (result <= 0) return -1;
    }
  
    putp(tigetstr((char *)"clear"));

    LcdScreen screen;
    screen._showDay = false;
    // screen.showMessage("Hello, it is a very-very-very-very long line", 1000);

    // screen.showMessage("ЁЁЁёёёЁЁЁёёё!", 1000);
    std::vector<unsigned char> content(32, 0xff);

    ScreenOffset offsetFrom;
    offsetFrom.x = 0;
    offsetFrom.y = 0;
    offsetFrom.atMs = micros64() / 1000ull;

    ScreenOffset offsetTo;
    offsetTo.x = 32;
    offsetTo.y = 8;
    offsetTo.atMs = offsetFrom.atMs + 1000;

    screen.showScreenContent(std::move(content), 32, 8, offsetFrom, offsetTo);

    for (int ii = 0; ii < 1600; ++ii) {
        // Move cursor to top left
        putp( tparm( tigetstr((char *)"cup" ), 0, 0, 0, 0, 0, 0, 0, 0, 0 ) );
        /*
        if (ii == 100) {
            screen.showMessage("Another string", 1000);
        }
        */
        
        screen.clear();

        const uint64_t dayInSecs = (24*60*60);
        screen.showTime(micros64() / dayInSecs / 1000000ull, (micros64() % (dayInSecs*1000000ull)/1000ull));

        // screen.clear();
        // screen.printStr(31, 8 - micros() / 1000 / 100 % 16, "четверг");
        // screen.printStr((micros() / 1000 / 50) % 300, 0, L"Четверг aaa bbb ююю {} || sdf 0123456789");

        /////////////////////////////////////////////////////////
        // OUT!
       
        for (int y = 0; y < 8; ++y) {
            for (int cnt = 0; cnt < 2; ++cnt) {
                printf("%01d |", y);
                for (int x = 0; x < screen.width(); ++x) {
                    printf("%s|", screen.get(screen.width() - 1 - x, y) ? "***" : "   ");
                }
                printf("\n");
            }
        }

        printf("  |");
        for (int x = 0; x < NUM_MAX*8; ++x) {
            printf("%02d |", screen.width() - 1 - x);
        }

        printf("\n");

        printf("Micros: %u\n", micros());

        usleep(10000); // will sleep for 10 ms
    }

    return 0;
}
