#pragma once

#include <algorithm>

#include "common.h"

#include "fonts.h"
#include "dateutil.h"

#define NUM_MAX 4

typedef const wchar_t* WSTR;
typedef wchar_t* WSTR_MUTABLE;

const WSTR weekdays[] = {
    L"Пн", 
    L"Вт",
    L"Ср",
    L"Чт",
    L"Птн",
    L"Сб",
    L"Вс"
};

const WSTR monthes[] = {
    L"января",
    L"февраля",
    L"марта",
    L"апреля",
    L"мая",
    L"июня",
    L"июля",
    L"августа",
    L"сентября",
    L"октября",
    L"ноября",
    L"декабря"
};

const wchar_t BIG_NUM_SYM = 0xE010;
const wchar_t MIDDLE_NUM_SYM = 0xE020;
const wchar_t TINY_NUM_SYM = 0xE030;

const uint8_t* symbolPtrOrNull(wchar_t symbol) {
    uint32_t fontItem = fontUA[0];
    uint32_t unicodeSym = symbol & 0xffff;
    if (unicodeSym >= 0x410 && unicodeSym < 0x450) {
        unicodeSym = unicodeSym - 0x410 + 0xa0;
    } else if ((unicodeSym >= BIG_NUM_SYM) && (unicodeSym < (10 + BIG_NUM_SYM))) {
        unicodeSym = (unicodeSym - BIG_NUM_SYM) + 270 - 0x20; // Big numbers
        // printf("%d\n", unicodeSym);
    } else if ((unicodeSym >= MIDDLE_NUM_SYM) && (unicodeSym < (10 + MIDDLE_NUM_SYM))) {
        unicodeSym = (unicodeSym - MIDDLE_NUM_SYM) + 280 - 0x20; // Middle numbers
    } else if ((unicodeSym >= TINY_NUM_SYM) && (unicodeSym < (10 + TINY_NUM_SYM))) {
        unicodeSym = (unicodeSym - TINY_NUM_SYM) + 290 - 0x20; // tiny numbers
    } else if (unicodeSym == 0x401) {
        unicodeSym = 0x100 - 0x20; // Ё
    } else if (unicodeSym == 0x451) {
        unicodeSym = 0x101 - 0x20; // ё
    } else if (unicodeSym == 0xE000) {
        unicodeSym = 0x102 - 0x20; // Speaker
    } else {
        unicodeSym = unicodeSym - 0x20;
    }

    if (unicodeSym >= 0 && unicodeSym < (sizeof(fontUA) - 1)/fontItem) {
        return fontUA + 1 + unicodeSym*fontItem;
    }
    return nullptr;
}


class Figure {
public:
    virtual void pixels(std::function<void(int, int)> acceptor) const = 0;
};

class OnePixelAt : public Figure {
    const Figure& _f;
    int _num;
public:

    OnePixelAt(const Figure& f, int num) : _f(f), _num(num) {}

    virtual void pixels(std::function<void(int, int)> acceptor) const {
        int num = _num;
        _f.pixels([&](int x, int y) {
            if (num-- == 0) {
                acceptor(x, y);
            }
        });
    }
};

class Rectangle : public Figure {
public:
    const int x;
    const int y;
    const int w;
    const int h;

    Rectangle() : x(0), y(0), w(0), h(0) {}
    Rectangle(int _x, int _y, int _w, int _h) : x(_x), y(_y), w(_w), h(_h) {}

    virtual void pixels(std::function<void(int, int)> acceptor) const {
        for (int xx = x; xx < x + w; ++xx) {
            for (int yy = y; yy < y + h; ++yy) {
                acceptor(xx, yy);
            }
        }
    }
};

class CharacterBitmask : public Figure {
private:
    const uint8_t* symbol;
public:
    CharacterBitmask(const wchar_t ch) {
        symbol = symbolPtrOrNull(ch);
    }

    virtual void pixels(std::function<void(int, int)> acceptor) const {
        int symbolW = *symbol & 0xff;
                
        for (int x = 0; x < symbolW; ++x) {
            uint8_t data = symbol[1 + x];
            for (int y = 0; y < 8; ++y) {
                if ((data >> y) & 1)
                    acceptor(symbolW - x, y);
            }
        }
    }
};

class Bitmask : public Figure {
    typedef const uint8_t BitsContent[8];
public:
    const BitsContent& bits;
    Bitmask(const BitsContent& _bits) : bits(_bits) {
    }

    virtual void pixels(std::function<void(int, int)> acceptor) const {
        for (size_t yy = 0; yy < sizeof(bits); ++yy) {
            for (int i = 0; i < 8; ++i) {
                if ((bits[yy] >> i) & 1) {
                    acceptor(i, yy);
                }
            }
        }
    }
};

/**
 * Message to be shown on the screen
 */
class MsgToShow {
    WSTR_MUTABLE _msg = NULL;
public:
    int strStartAt = 0;
    int _totalMsToShow = 0;

    WSTR c_str() {
        return _msg;
    }

    bool isSet() const {
        return _msg != nullptr;
    }

    bool empty() const {
        return !isSet() || (_msg[0] == 0);
    }

    void clear() {
        if (_msg != NULL) {
            free(_msg);
            _msg = NULL;
        }
    }

    void set(const WSTR* ss, uint32_t count) {
        clear();
        if (ss != nullptr) { 
            uint32_t totalCnt = 0;
            for (uint32_t i = 0; i < count; ++i) {
                totalCnt += wcslen(ss[i]);
            }
            _msg = (WSTR_MUTABLE)malloc(2*totalCnt + 2);
            WSTR_MUTABLE s = _msg;
            for (uint32_t i = 0; i < count; ++i) {
                for (int t = 0; ss[i][t] != 0; ++t) {
                    *s = ss[i][t];
                    s++;
                    *s = 0;
                }
            }
            strStartAt = millis();
        }
    }

    void set(const char* utf8str, const int totalMsToShow) {
        clear();
        if (utf8str != nullptr) {
            _totalMsToShow = totalMsToShow;
            int maxSize = strlen(utf8str)*2;
            _msg = (WSTR_MUTABLE)malloc(maxSize + 2);
            memset(_msg, 0, maxSize + 2);
            int srcLen = strlen(utf8str);
            for (int i = 0, outIndex = 0; utf8str[i] != 0;) {
                uint16_t sym = utf8str[i];

                if (
                    (sym & 0b11110000) == 0b11100000 && 
                    ((i + 1) < srcLen) && (utf8str[i+1] & 0b11000000) == 0b10000000 &&
                    ((i + 2) < srcLen) && (utf8str[i+2] & 0b11000000) == 0b10000000
                ) {
                        // If an UCS fits 16 bits, it is coded as 1110xxxx 10xxxxxx 10xxxxxx
                    _msg[outIndex++] = 
                        (((sym & 0b1111) << 12) & 0b1111000000000000) |
                        ((((uint16_t)utf8str[i+1] & 0b111111) << 6) & 0b111111000000) |
                        ((uint16_t)utf8str[i+2] & 0b111111);

                    i+=3;
                } else if (
                    (sym & 0b11100000) == 0b11000000 && 
                    ((i + 1) < srcLen) && (utf8str[i+1] & 0b11000000) == 0b10000000) {
                    // If an UCS fits 11 bits, it is coded as 110xxxxx 10xxxxxx
                    _msg[outIndex++] = (((sym & 0b11111) << 6) & 0b11111000000) |
                        ((uint16_t)utf8str[i+1] & 0b111111);
                    i+=2;
                } else if ((sym & 0b10000000) == 0) {
                    // If an UCS fits 7 bits, its coded as 0xxxxxxx. This makes ASCII character represented by themselves
                    _msg[outIndex++] = sym;
                    i++;
                } else {
                    i++;
                }
            }

            strStartAt = millis();
        }
    }
};

class LcdScreen {

private:
    uint8_t screen[NUM_MAX * 8];

    uint32_t nextShowDateInMs = millis() + 5000;

    MsgToShow _rollingMsg;
    MsgToShow _tuningMsgNow;
    MsgToShow _additionalInfo;

    const uint32_t _scrollSpeed = 35;
    uint32_t _blinkStart = 0;
    const uint32_t _blinkTime = 30;

    uint32_t _showDayMin = 0;

public:
    bool _showDay = true;

    static const int secInUs = 1000000;

    LcdScreen() {
        memset(screen, 0, sizeof(screen));
    }

    bool fits(int x, int y) {
        return (x >= 0) && (y >= 0) && (x < width()) && (y < height());
    }

    int idx(int x, int y) {
        int res = x / 8 * 8 + y;
        return res;
    }

    bool get(int x, int y) {
        if (!fits(x, y)) {
            return false; // Outside world is black
        }
        return (screen[idx(x, y)] >> (x % 8)) & 1;
    }

    void set(int x, int y, bool clr) {
        if (fits(x, y)) {
            screen[idx(x, y)] &= ~(1 << (x % 8));
            screen[idx(x, y)] |= (clr ? 1 : 0) << (x % 8);
        }
    }

    void invert(int x, int y) {
        set(x, y, !get(x, y));
    }

    void invert(int _x, int _y, const Figure& fg) {
        fg.pixels([=](int x, int y) {
            this->invert(_x + x, _y + y);
        });
    }

    void set(int _x, int _y, const Figure& fg, bool clr) {
        fg.pixels([=](int x, int y) {
            this->set(_x + x, _y + y, clr);
        });
    }

    void clear() {
        memset(screen, 0, sizeof(screen));
    }

    uint8_t line8(int l) const {
        return screen[l];
    }

    int height() const {
        return 8;
    }

    int width() const {
        return NUM_MAX*8;
    }

    /**
     * This could be hour, min or sec
     */
    class TimeComponent {
        int val = 0;
        char asStr[4];
    public:
        TimeComponent(int _val) : val(_val) {
            memset(asStr, 0, sizeof(asStr));
            snprintf(asStr, sizeof(asStr), "%02d", val);
        }

        uint8_t charAt(int ind) {
            return asStr[ind];
        }

        const char* c_str() {
            return asStr;
        }
    };

    int getStrWidth(const WSTR* strs, int strsCnt) {
        int w = 0;
        for (;strsCnt > 0; --strsCnt, ++strs) {
            w += getStrWidth(*(strs)) + 1;
        }
        return w;
    }

    int getStrWidth(WSTR str) {
        int res = 0;
        for (int i=0; str[i] != 0; ++i) {
            const uint8_t* symbol = symbolPtrOrNull(str[i]);
            if (symbol != NULL) {
                if (res > 0) {
                    res += 1;
                }
                res += symbol[0];
            }
        }
        return res;
    }

    /**
     * Prints several strings at once
     */
    int printStr(int _x, int _y, const WSTR* strs, int strsCnt) {
        if (strs == nullptr) {
            return 0;
        }
        int ww = 0;
        for (;strsCnt > 0; --strsCnt, ++strs) {
            int w = printStr(_x, _y, *(strs));
            ww += w + 1;
            _x -= w + 1;
        }
        return ww;
    }

    /**
     * Prints string and returns printed string width
     */
    int printStr(int _x, int _y, WSTR str) {
        if (str == nullptr) {
            return 0;
        }
        int w = 0;
        for (int i=0; str[i] != 0; ++i) {
            const uint8_t* symbol = symbolPtrOrNull(str[i]);
            if (symbol > fontUA && symbol < (fontUA + sizeof(fontUA))) {
                int symbolW = *symbol & 0xff;
                
                for (int x = 0; x < symbolW; ++x) {
                    uint8_t data = symbol[1 + x];
                    for (int y = 0; y < 8; ++y) {
                        set(_x - w - x, y + _y, (data >> y) & 1);
                    }
                }
                w += symbolW + 1;
            }
        }
        return w;
    }

    void showTuningMsg(const char* utf8str) {
        _tuningMsgNow.set(utf8str, 3000);
    }

    void showMessage(const char* utf8str, int totalMsToShow) {
        _rollingMsg.set(utf8str, totalMsToShow);
    }

    void setAdditionalInfo(const char* utf8str) {
        _additionalInfo.set(utf8str, 0);
    }

    /**
     * micros is current time in microseconds
     */
    void showTime(uint32_t daysSince1970, uint32_t millisSince1200) {
        int millisNow = millis();
        if (_tuningMsgNow.isSet() && !_tuningMsgNow.empty()) {
            uint32_t showedTime = millisNow - _tuningMsgNow.strStartAt;
            printStr(width() - 1, 0, _tuningMsgNow.c_str());
            if (showedTime > 2000) {
                _tuningMsgNow.clear();
            }
            return; // Nothing more
        }

        if (_rollingMsg.isSet()) {
            int32_t waitBefore = 300;
            int32_t showedTime = std::max(millisNow - _rollingMsg.strStartAt, (int32_t)0);
            int32_t extraTime = _rollingMsg._totalMsToShow == 0 ? 300 : std::max(_rollingMsg._totalMsToShow - showedTime - waitBefore, (int32_t)0);
            int32_t strW = getStrWidth(_rollingMsg.c_str());
            int32_t timeToShow = ((strW - 32) * _scrollSpeed);
            int32_t x = 0;
            bool stop = false;
            if (showedTime < waitBefore) {
                x = 31;
            } else if (showedTime < (waitBefore + timeToShow)) {
                x = (showedTime - waitBefore) / _scrollSpeed + 32;
            } else if (showedTime < (waitBefore + timeToShow + extraTime)) {
                if (strW < 32) {
                    x = 31;
                } else {
                    x = strW;
                }
            } else {
                stop = true;
                _rollingMsg.clear();
            }

            if (!stop) {
                printStr(x, 0, _rollingMsg.c_str());
                return;
            }
        } 

        if (millisSince1200 / 1000 % 60 > 55 && _showDay && (_showDayMin != millisSince1200 / 1000 / 60)) {
            _showDayMin = millisSince1200 / 1000 / 60;
            date::RTC rtc = {0};
            date::epoc2rtc(daysSince1970, rtc);
            wchar_t yearStr[10] = { 0 };
            swprintf(yearStr, __countof(yearStr), L"%d", rtc.year);
            wchar_t dayStr[10] = { 0 };
            swprintf(dayStr, __countof(dayStr), L"%d", rtc.day+1);
            const WSTR ss[] = {
                weekdays[rtc.dow],
                L", ",
                dayStr,
                L" ",
                monthes[rtc.month],
                L" ",
                _additionalInfo.c_str()
            };
            _rollingMsg.set(ss, __countof(ss) - (_additionalInfo.empty() ? 2 : 0));
            // debugPrint("Month: " + String(rtc.month, 10) +
            //         " doy " + String(rtc.doy, 10) +
            //         " year " + String(rtc.year, 10) +
            //         " : " + String(daysSince1970, 10) );
            return;
        }
        
        int seconds = millisSince1200 / 1000;
        TimeComponent hours((seconds % 86400L) / 3600);
        TimeComponent mins((seconds % 3600) / 60);

        TimeComponent secs(seconds % 60);
        TimeComponent nextSecs((seconds + 1) % 60);
        TimeComponent nextNextSecs((seconds + 2) % 60);

        clear();

        // Mins
        // mins.charAt(n) - '0'
        set(7, 0, CharacterBitmask(BIG_NUM_SYM + (mins.charAt(1) - '0')), true);
        set(13, 0, CharacterBitmask(BIG_NUM_SYM + (mins.charAt(0) - '0')), true);

        // Hours
        /* hours.charAt(n) - '0'*/
        set(20, 0, CharacterBitmask(BIG_NUM_SYM + (hours.charAt(1) - '0')), true);
        set(26, 0, CharacterBitmask(BIG_NUM_SYM + (hours.charAt(0) - '0')), true);

        if (millisSince1200 % 1000 < 200) {
            set(19, 1, OnePixelAt(Rectangle(0, 0, 2, 2), millisSince1200 % 1000 / 50), true);
            set(19, 5, OnePixelAt(Rectangle(0, 0, 2, 2), millisSince1200 % 1000 / 50), true);
        }

        int movingTimeUs = 1000 / 4; // Period of time to do seconds moving transition
        int smallFontHeight = 6;
        for(int n = 1; n >= 0; n--) {
            int y;
            int ms  = (millisSince1200 % 1000);
            if (ms >= (1000 - movingTimeUs)) {
                y = (smallFontHeight - (1000 - ms) * smallFontHeight / movingTimeUs);
            } else {
                y = 0;
            }

            set(n*4 - 1, y, CharacterBitmask(TINY_NUM_SYM + (secs.charAt(1-n) - '0')), true);
            set(n*4 - 1, y-smallFontHeight, CharacterBitmask(TINY_NUM_SYM + (nextSecs.charAt(1-n) - '0')), true);
            set(n*4 - 1, y-smallFontHeight*2, CharacterBitmask(TINY_NUM_SYM + (nextNextSecs.charAt(1-n) - '0')), true);
        }
    }
};

//======================================================================================================

class MAX72xx {
    // Opcodes for the MAX7221 and MAX7219
    // All OP_DIGITn are offsets from OP_DIGIT0
    #define	OP_NOOP         0 ///< MAX72xx opcode for NO OP
    #define OP_DIGIT0       1 ///< MAX72xx opcode for DIGIT0
    #define OP_DIGIT1       2 ///< MAX72xx opcode for DIGIT1
    #define OP_DIGIT2       3 ///< MAX72xx opcode for DIGIT2
    #define OP_DIGIT3       4 ///< MAX72xx opcode for DIGIT3
    #define OP_DIGIT4       5 ///< MAX72xx opcode for DIGIT4
    #define OP_DIGIT5       6 ///< MAX72xx opcode for DIGIT5
    #define OP_DIGIT6       7 ///< MAX72xx opcode for DIGIT6
    #define OP_DIGIT7       8 ///< MAX72xx opcode for DIGIT7
    #define OP_DECODEMODE   9 ///< MAX72xx opcode for DECODE MODE
    #define OP_INTENSITY   10 ///< MAX72xx opcode for SET INTENSITY
    #define OP_SCANLIMIT   11 ///< MAX72xx opcode for SCAN LIMIT
    #define OP_SHUTDOWN    12 ///< MAX72xx opcode for SHUT DOWN
    #define OP_DISPLAYTEST 15 ///< MAX72xx opcode for DISPLAY TEST

public:
    MAX72xx(LcdScreen& _screen, 
            const int _CLK_PIN,
            const int _DATA_PIN,
            const int _CS_PIN,
            bool _rotated180) : 
            CLK_PIN(_CLK_PIN),
            DATA_PIN(_DATA_PIN),
            CS_PIN(_CS_PIN),
            rotated180(_rotated180),
            screen(_screen) {
    }

    void sendCmd(int addr, uint8_t cmd, uint8_t data) {
        digitalWrite(CS_PIN, LOW);
        for (int i = NUM_MAX - 1; i >= 0; i--) {
            shiftOut(DATA_PIN, CLK_PIN, MSBFIRST, i == addr ? cmd : 0);
            shiftOut(DATA_PIN, CLK_PIN, MSBFIRST, i == addr ? data : 0);
        }
        digitalWrite(CS_PIN, HIGH);
    }

    void sendCmdAll(uint8_t cmd, uint8_t data) {
        digitalWrite(CS_PIN, LOW);
        for (int i = NUM_MAX - 1; i >= 0; i--) {
            shiftOut(DATA_PIN, CLK_PIN, MSBFIRST, cmd);
            shiftOut(DATA_PIN, CLK_PIN, MSBFIRST, data);
        }
        digitalWrite(CS_PIN, HIGH);
    }

    void setBrightness(int percents) {
        sendCmdAll(OP_INTENSITY, percents * 15 / 100); // minimum brightness
        // debugPrint("Setting brightness to " + String(percents, DEC));
    }

    void setup() {
        pinMode(CS_PIN, OUTPUT);
        pinMode(DATA_PIN, OUTPUT);
        pinMode(CLK_PIN, OUTPUT);

        // sendCmdAll(OP_INTENSITY, 0); // minimum brightness
    }

    void refreshAll() {
        if (millis() % 500 == 0) {
            digitalWrite(CS_PIN, HIGH);
            sendCmdAll(OP_DISPLAYTEST, 0);
            sendCmdAll(OP_SCANLIMIT, 7);
            sendCmdAll(OP_DECODEMODE, 0);
            sendCmdAll(OP_SHUTDOWN, 1);
        }

        LcdScreen screen2;
        int w = screen.width();
        int h = screen.height();
        for (int x = 0; x < w; ++x) {
            for (int y = 0; y < h; ++y) {
                if (rotated180) {
                    screen2.set(x, y, screen.get(x, y));
                } else {
                    screen2.set(w - (x + 1), h - (y + 1), screen.get(x, y));
                }
            }
        }
        for (int line = 0; line < 8; line++) {
            digitalWrite(CS_PIN, LOW);
            for (int chip = NUM_MAX - 1; chip >= 0; chip--) {
                shiftOut(DATA_PIN, CLK_PIN, MSBFIRST, OP_DIGIT0 + line);
                shiftOut(DATA_PIN, CLK_PIN, MSBFIRST, screen2.line8(chip * 8 + line));
            }
            digitalWrite(CS_PIN, HIGH);
        }
        digitalWrite(CS_PIN, LOW);
    }

  private:
    const int CLK_PIN;
    const int DATA_PIN;
    const int CS_PIN;
    const bool rotated180;

    LcdScreen& screen;
};

//======================================================================================================
