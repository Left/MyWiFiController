#include "pseudo_arduino.h"

#include "../lcd.h"

int main(int argc, char const *argv[]) {
    LcdScreen screen;
    screen.showMessage("Тест");
    return 0;
}