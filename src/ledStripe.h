#include "common.h"

#include <cstdlib>
#include <vector>
#include <map>
#include <functional>
#include <sys/types.h>
#include <stdint.h>

class LedStripe {
    const int NUMPIXELS;
    std::vector<uint8_t> currLedStripe;
    const std::function<uint32_t(void)> millis;

    class LedEffect {
    public:
        virtual bool finished() = 0;
        virtual bool update(std::vector<uint8_t>& currLedStripe) = 0;
    };

    class Still : public LedEffect {
        virtual bool finished() { return false; }
        virtual bool update(std::vector<uint8_t>& currLedStripe) { return false; }
    };

    std::unique_ptr<LedEffect> effect;

    class MutateTo : public LedEffect {
    public:
        uint32_t ledStripeChangedStartedAt;
        uint32_t ledStripeChangedEndedAt;
        bool finish;
        const std::function<uint32_t(void)> millis;

        std::vector<uint8_t> initialLedStripe;
        std::vector<uint8_t> ledStripe;

        MutateTo(const std::vector<uint8_t>& currLedStripe, const std::vector<uint32_t>& colors, int periodMs, const std::function<uint32_t(void)> millis_) :
            finish(false), 
            millis(millis_) {

            ledStripe.resize(0);

            for (size_t p = 0; p < colors.size(); ++p) {
                ledStripe.push_back((colors[p] >> 24) & 0xff);
                ledStripe.push_back((colors[p] >> 16) & 0xff);
                ledStripe.push_back((colors[p] >>  8) & 0xff);
                ledStripe.push_back((colors[p] >>  0) & 0xff);
            }

            initialLedStripe = currLedStripe;
            ledStripeChangedStartedAt = millis();
            ledStripeChangedEndedAt = millis() + periodMs;
        }
    
        virtual bool finished() { 
            return finish; 
        }

        virtual bool update(std::vector<uint8_t>& currLedStripe) {
            bool changed = false;
            int32_t fullMs = ledStripeChangedEndedAt - ledStripeChangedStartedAt;
            int32_t nowMs = millis() - ledStripeChangedStartedAt;
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

            if (!finish) {
                finish = nowMs == fullMs;
            }
            return changed;
        }
    };

    class NewYear : public LedEffect {
    public:
        struct Blink {
            uint32_t color;
            uint32_t aliveFrom;
            uint32_t aliveTill;
        };

        std::map<size_t, Blink> _blinks;
        uint32_t mainClr;
        uint32_t periodMs;
        std::vector<uint8_t> prevLedStripe;
        const std::function<uint32_t(void)> millis;
        
        NewYear(const std::vector<uint8_t>& currLedStripe, uint32_t mainClr_, const std::vector<uint32_t>& blinks, uint32_t periodMs_, const std::function<uint32_t(void)> millis_) :
            mainClr(mainClr_),
            periodMs(periodMs_),
            millis(millis_) {
            const size_t NUMPIXELS = currLedStripe.size() / 4;

            for (size_t i = 0; i < blinks.size(); ++i) {
                makeNewBlink(NUMPIXELS, blinks[i]);
            }
        }

        void makeNewBlink(size_t NUMPIXELS, uint32_t clr) {
            for (;;) {
                size_t index = std::rand() % NUMPIXELS;
                if (_blinks.find(index) == _blinks.end()) {
                    _blinks[index] = { clr, millis(), millis() + periodMs / 3 + (std::rand() % (periodMs * 2 / 3)) };
                    break;
                }
            }
        }

        virtual bool finished() { 
            return false;
        }

        virtual bool update(std::vector<uint8_t>& currLedStripe) {
            uint32_t m = millis();
            const size_t NUMPIXELS = currLedStripe.size() / 4;
            prevLedStripe.resize(0);
            for (size_t i = 0; i < NUMPIXELS; ++i) {
                uint32_t clr = mainClr;
                if (_blinks.find(i) != _blinks.end()) {
                    const Blink bl = _blinks[i];
                    if (m >= bl.aliveTill) {
                        _blinks.erase(i);
                        makeNewBlink(NUMPIXELS, bl.color);
                    } else {
                        int32_t n = m - bl.aliveFrom;
                        int32_t fullMs = bl.aliveTill - bl.aliveFrom;
                        clr = 
                            (((((int32_t)((bl.color >> 24) & 0xff) + ((((int32_t)clr >> 24) & 0xff) - (((int32_t)bl.color >> 24) & 0xff)) * n / fullMs)) << 24) & 0xff000000) |
                            (((((int32_t)((bl.color >> 16) & 0xff) + ((((int32_t)clr >> 16) & 0xff) - (((int32_t)bl.color >> 16) & 0xff)) * n / fullMs)) << 16) & 0x00ff0000) |
                            (((((int32_t)((bl.color >>  8) & 0xff) + ((((int32_t)clr >>  8) & 0xff) - (((int32_t)bl.color >>  8) & 0xff)) * n / fullMs)) <<  8) & 0x0000ff00) |
                            (((((int32_t)((bl.color >>  0) & 0xff) + ((((int32_t)clr >>  0) & 0xff) - (((int32_t)bl.color >>  0) & 0xff)) * n / fullMs)) <<  0) & 0x000000ff)
                        ;
                    }
                }
                prevLedStripe.push_back((clr >> 24) & 0xff);
                prevLedStripe.push_back((clr >> 16) & 0xff);
                prevLedStripe.push_back((clr >>  8) & 0xff);
                prevLedStripe.push_back((clr >>  0) & 0xff);
            }
            if (prevLedStripe == currLedStripe) {
                return false;
            }
            prevLedStripe.swap(currLedStripe);
            return true;
        }
    };

public:
    LedStripe(uint32_t pixels, const std::function<uint32_t(void)>& millis_) : 
        NUMPIXELS(pixels), millis(millis_), effect(new Still()) { 
        currLedStripe.resize(NUMPIXELS * 4, 0);
    }

    void runLedStripeEffect(uint32_t mainClr, const std::vector<uint32_t>& blinks, uint32_t periodMs) {
        effect.reset(new NewYear(currLedStripe, mainClr, blinks, periodMs, millis));
    }

    void set(const std::vector<uint32_t>& colors, int periodMs) {
        effect.reset(new MutateTo(currLedStripe, colors, periodMs, millis));
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

    bool update() {
        bool up = effect->update(currLedStripe);
        if (effect->finished()) {
            effect.reset(new Still());
            return true;
        }
        return up;
    }

    const std::vector<uint8_t>& curr() {
        return currLedStripe;
    }
};
