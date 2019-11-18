#include "common.h"

#include <vector>
#include <sys/types.h>
#include <stdint.h>

class LedStripe {
    const int NUMPIXELS;
    std::vector<uint8_t> initialLedStripe;
    std::vector<uint8_t> ledStripe;
    std::vector<uint8_t> currLedStripe;
    uint32_t ledStripeChangedStartedAt;
    uint32_t ledStripeChangedEndedAt;

public:
    LedStripe(uint32_t pixels) : NUMPIXELS(pixels), ledStripeChangedStartedAt(0), ledStripeChangedEndedAt(0) {
        ledStripe.resize(NUMPIXELS * 4, 0);
        currLedStripe.resize(NUMPIXELS * 4, 0);
        initialLedStripe.resize(NUMPIXELS * 4, 0);
    }


    void set(const std::vector<uint32_t>& colors, int periodMs, uint32_t millis) {
        std::vector<uint8_t>  newStripe;
        for (size_t p = 0; p < colors.size(); ++p) {
            newStripe.push_back((colors[p] >> 24) & 0xff);
            newStripe.push_back((colors[p] >> 16) & 0xff);
            newStripe.push_back((colors[p] >>  8) & 0xff);
            newStripe.push_back((colors[p] >>  0) & 0xff);
        }

        ledStripe.swap(newStripe);
        initialLedStripe = currLedStripe;
        
        ledStripeChangedStartedAt = millis;
        ledStripeChangedEndedAt = millis + periodMs;
    }

    uint32_t getPixelCount() const {
        return NUMPIXELS;
    }

    uint32_t getPixel(uint32_t i) const {
        return (currLedStripe[i * 4] << 24) |
            (currLedStripe[i * 4 + 1] << 16) |
            (currLedStripe[i * 4 + 2] << 8) |
            (currLedStripe[i * 4 + 3] << 0);
    }

    bool inProgress(uint32_t millis) const {
        return ledStripeChangedStartedAt <= millis && ledStripeChangedEndedAt >= millis;
    }

    bool update(uint32_t millis) {
        bool changed = false;
        int32_t fullMs = ledStripeChangedEndedAt - ledStripeChangedStartedAt;
        int32_t nowMs = millis - ledStripeChangedStartedAt;
        if (nowMs > fullMs)
            nowMs = fullMs;

        for (size_t i = 0; i < ledStripe.size(); ++i) {
            int32_t src = initialLedStripe[i];
            int32_t dst = ledStripe[i];
            int32_t full = dst - src;
            uint8_t val = nowMs == fullMs ? dst : ((int32_t)src + (int32_t)full * nowMs / fullMs);

            if (val != currLedStripe[i]) {
                currLedStripe[i] = val;
                changed = true;
            }
        }
        return changed;
    }

    const std::vector<uint8_t>& curr() {
        return currLedStripe;
    }

    const std::vector<uint8_t>& target() {
        return ledStripe;
    }
};
