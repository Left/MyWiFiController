#include "common.h"

#include <cstdlib>
#include <vector>
#include <array>
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
        virtual ~LedEffect() {}
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

    static std::array<uint8_t, 4> parseColor(uint32_t clr) {
        return { (uint8_t) (clr >> 24), (uint8_t) (clr >> 16), (uint8_t) (clr >> 8), (uint8_t) clr };
    }

    class NewYear : public LedEffect {
    public:
        struct Blink {
            std::array<uint8_t, 4> color;
            uint32_t aliveFrom;
            uint32_t aliveTill;
        };

        std::map<size_t, Blink> _blinks;
        std::array<uint8_t, 4> mainClr;
        uint32_t periodMs;
        std::vector<uint8_t> prevLedStripe;
        const std::function<uint32_t(void)> millis;
        
        NewYear(const std::vector<uint8_t>& currLedStripe, uint32_t mainClr_, const std::vector<uint32_t>& blinks, uint32_t periodMs_, const std::function<uint32_t(void)> millis_) :
            mainClr(parseColor(mainClr_)),
            periodMs(periodMs_),
            millis(millis_) {
            const size_t NUMPIXELS = currLedStripe.size() / 4;

            for (size_t i = 0; i < blinks.size(); ++i) {
                makeNewBlink(NUMPIXELS, parseColor(blinks[i]));
            }
        }

        void makeNewBlink(size_t NUMPIXELS, std::array<uint8_t, 4> clr) {
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
            for (size_t i = 0; i < NUMPIXELS * 4; ++i) {
                size_t pixel = i / 4;
                uint8_t clr = mainClr[i % 4];
                if (_blinks.find(pixel) != _blinks.end()) {
                    const Blink bl = _blinks[pixel];
                    if (m >= bl.aliveTill) {
                        _blinks.erase(pixel);
                        makeNewBlink(NUMPIXELS, bl.color);
                    } else {
                        int32_t n = m - bl.aliveFrom;
                        int32_t fullMs = bl.aliveTill - bl.aliveFrom;
                        int32_t stageLen = fullMs/3;
                        int stage = n/stageLen;
                        n -= stage*stageLen;
                        int32_t diffBetweenColors = ((int16_t)clr - ((int16_t)bl.color[i % 4]));
                        switch (stage) {
                            case 0:
                                // 
                                clr = 
                                    (((bl.color[i % 4] + diffBetweenColors * (stageLen - n) / (stageLen))));
                                break;
                            case 1:
                                clr = bl.color[i % 4];
                                break;
                            case 2:
                                clr = 
                                    (((bl.color[i % 4] + diffBetweenColors * n / (stageLen))));
                                break;
                        }
                    }
                }
                prevLedStripe.push_back(clr);
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

    uint32_t pixelCount() const {
        return NUMPIXELS;
    }

    std::array<uint8_t, 4> pixel(size_t i) const {
        auto ind = currLedStripe.begin() + i * 4;
        return { *(ind++), *(ind++), *(ind++), *(ind++) };
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
