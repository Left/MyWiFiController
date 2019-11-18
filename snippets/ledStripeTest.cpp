#include "../src/ledStripe.h"
#include <stdio.h>

void print(LedStripe* ls) {
    for (uint32_t i = 0; i < ls->getPixelCount(); ++i) {
        uint32_t p = ls->getPixel(i);
        printf("%08x ", p);
    }

    printf("\t");
    for (std::vector<uint8_t>::const_iterator it = ls->curr().begin(); it != ls->curr().end(); ++it) {
        printf("%02x ", *it);
    }
    printf("\n");
/*
    for (std::vector<uint8_t>::const_iterator it = ls->target().begin(); it != ls->target().end(); ++it) {
        printf("%02x ", *it);
    }
    printf("\n");
*/
}

int main(int argc, char const *argv[]) {   
    LedStripe* ls = new LedStripe(3);
    std::vector<uint32_t> off;
    off.push_back(0xff000000); off.push_back(0x00ff0000); off.push_back(0x00008000); 
    ls->set(off, 0, 1);
    ls->update(1);

    std::vector<uint32_t> on;
    on.push_back(0x00ff0000); on.push_back(0x0000ffff); on.push_back(0x00000000); 

    int t = 0;

    for (int ii = 0; ii < 3; ++ii) {
        ls->set(on, 500, t);
        for (size_t i = 0; i <= 5; i++, t += 100) {
            ls->update(t);
            printf("%04d - ", t); print(ls);
        }
        printf("\n");

        ls->set(off, 500, t);
        for (size_t i = 0; i <= 5; i++, t += 100) {
            ls->update(t);
            printf("%04d - ", t); print(ls);
        }
        printf("\n");
    }
}