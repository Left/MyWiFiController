#include "sceleton.h"

#include <memory>

#include <Adafruit_BME280.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_Sensor.h>
#include <WiFiUDP.h>

#include "worklogic.h"

#include <OneWire.h>
#include <Q2HX711.h>

#ifndef ESP01
#include "lcd.h"
#endif

#if defined(ESP32)
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#endif

#include "ledStripe.h"
#include <SoftwareSerial.h>

unsigned int localPort = 2390;  // local port to listen for UDP packets

const uint64_t dayInMs = 24 * 60 * 60 * 1000;

boolean isScreenEnabled = true;

#ifndef ESP01
LcdScreen screen;
MAX72xx* screenController = NULL;
#endif // ESP01

Adafruit_BME280* bme = NULL;  // I2C

// #define BEEPER_PIN D2 // Beeper

int testCntr = 0;

Q2HX711* hx711 = NULL;

struct Key {
    const char* bin;
    const char* value;

    Key(const char* bin_, const char* value_) : bin(bin_), value(value_) {}
};

struct Remote {
    const char* name;
    const std::vector<Key> keys;

    Remote(const char* _name, const std::vector<Key>& _keys)
        : name(_name), keys(_keys) {}
};

const Remote* remotes[] = {
};

int64_t lastIRChange = 0;
int32_t lastIRChangeMs = 0;
std::vector<uint32_t> ir;

static void IRAM_ATTR irIRQHandler() { 
    unsigned long m = micros();
    if (ir.size() < 248) {
        ir.push_back(m - lastIRChange);
    }
    lastIRChange = m;
    lastIRChangeMs = millis();
}

uint8_t bluePillPacketStart[] = {0x80, 0x1d, 0x7d, 0x2e, 0x00, 0x03, 0xb9, 0x13 };
uint8_t bluePillPacketEnd[] = {0xff, 0x5b, 0xa1, 0x35, 0x33, 0x6f, 0xf5, 0x37 };

std::vector<uint8_t> protobuf;

boolean invertRelayState = false;
boolean relayIsInitialized = false;
SoftwareSerial* relay = NULL;

OneWire* oneWire;

uint32_t lastMsp430Ping = millis();
SoftwareSerial* msp430 = NULL;  // RX, TX

SoftwareSerial* dfplayerSerial = NULL;  // RX, TX

const int NUMPIXELS = 64;
Adafruit_NeoPixel* stripe = NULL;

const long interval = 1000;  // Request each second
unsigned long nextRequest = millis();
unsigned long nextRead = ULONG_MAX;

typedef uint8_t DeviceAddress[8];
DeviceAddress deviceAddress = {0};

struct PWMState {
    uint8_t pin;
    uint32_t startMs;
    uint32_t endMs;
    uint32_t lastAnalogWriteMs;

    int32_t start;
    int32_t curr;
    int32_t target;

    PWMState(uint8_t _pin) :
        pin(_pin), startMs(0), endMs(0), lastAnalogWriteMs(0), start(0), curr(0), target(0) {
    }
};

PWMState pwmStates[] = { PWMState(D3), PWMState(D4), PWMState(D7), PWMState(D6), PWMState(D5) };

int interruptCounterD7 = 0;
int interruptCounterD5 = 0;
int interruptCounterD2 = 0;

uint32_t timeRetreivedInMs = 0;
uint32_t initialUnixTime = 0;
uint32_t restartAt = ULONG_MAX;
uint32_t nextPotentiometer = 0;

uint32_t lastHCSR = 0;
uint32_t lastHCSRSent = 0;
std::vector<uint32_t> destinies;

uint16_t analogInValues[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t analogReadIndex = 0;

int32_t reportedAnalogValue = -1;

int oldPowerState = -1;

uint32_t ssdPins[] = {D1, D7, D5, D6};

LedStripe* ledStripe = NULL;
uint32_t lastLedStripeUpdate = 0;

void IRAM_ATTR handleInterruptD7() { interruptCounterD7++; }
void IRAM_ATTR handleInterruptD2() { interruptCounterD2++; }
void IRAM_ATTR handleInterruptD5() { interruptCounterD5++; }

uint32_t d7start;
uint32_t d7changes;

std::vector<uint32_t> d7millis;
std::vector<bool> d7state;
std::vector<bool> d6state;

void writeHCRState(bool d6) {
    noInterrupts();
    auto d7 = digitalRead(D7);
    // auto d6 = digitalRead(D6);
    if (d7state.empty() || d6state.empty() || 
        d7state.back() != d7 || d6state.back() != d6) { 
        d7state.push_back(d7);
        d6state.push_back(d6);
        d7millis.push_back(micros());
    }
    interrupts();
}

uint32_t hcsrStart = micros();
uint32_t hcsrDist = 0;
uint32_t hcsrDistTime = 0;

void IRAM_ATTR handleInterruptHCSRfall() {
    if (hcsrDist == 0 && hcsrStart != 0) {
        hcsrDist = micros() - hcsrStart;
        hcsrDistTime = millis();
    }
    // writeHCRState(false);
}

boolean encoderPinChanged = false;

#define DFPLAYER_RECEIVED_LENGTH 10
#define DFPLAYER_SEND_LENGTH 10

#define Stack_Header 0
#define Stack_Version 1
#define Stack_Length 2
#define Stack_Command 3
#define Stack_ACK 4
#define Stack_Parameter 5
#define Stack_CheckSum 7
#define Stack_End 9

uint16_t calculateCheckSum(uint8_t* buffer) {
    uint16_t sum = 0;
    for (int i = Stack_Version; i < Stack_CheckSum; i++) {
        sum += buffer[i];
    }
    return -sum;
}

uint16_t arrayToUint16(uint8_t* array) {
    uint16_t value = *array;
    value <<= 8;
    value += *(array + 1);
    return value;
}

void uint16ToArray(uint16_t value, uint8_t* array) {
    *array = (uint8_t)(value >> 8);
    *(array + 1) = (uint8_t)(value);
}

void dfPlayerSend(uint8_t command, uint16_t argument) {
    debugSerial->print(String(F("dfPlayerSend:")));
    debugSerial->print(argument);
    debugSerial->println();

    uint8_t _sending[DFPLAYER_SEND_LENGTH] = {0x7E, 0xFF, 0x06, 0x00, 0x01,
                                              0x00, 0x0,  0x00, 0x00, 0xEF};

    _sending[Stack_Command] = command;
    // _sending[Stack_ACK] = 1;
    uint16ToArray(argument, _sending + Stack_Parameter);
    uint16ToArray(calculateCheckSum(_sending), _sending + Stack_CheckSum);

    debugSerial->print(String(F("SENDING to DFplayer:")));
    for (int i = 0; i < DFPLAYER_SEND_LENGTH; ++i) {
        debugSerial->print(String(_sending[i], HEX));
        debugSerial->print(" ");
    }
    debugSerial->println();

    dfplayerSerial->write(_sending, DFPLAYER_SEND_LENGTH);
    debugSerial->println(String(F("SENT")));
    debugPrint(String(F("SENT")));
}

class Encoder {
   public:
    Encoder(const char* name, int a, int b, int button)
        : encName(name), pinA(a), pinB(b), pinButton(button) {}

    void process() {
        if (encoderPinChanged) {
            int pA = digitalRead(pinA);
            int pB = digitalRead(pinB);
            int pBtn = digitalRead(pinButton);

            String s = F("encoder_");
            s += encName;
            if (pA != _pA || pB != _pB) {
                if (_pA == 0 && _pB == 1 && pA == 1 && pB == 1) {
                    String key = F("rotate_cw");
                    sceleton::udpSend([&](Msg& m) {
                        m.has_parsedRemote = true;
                        m.parsedRemote.remote = sceleton::str(s);
                        m.parsedRemote.key = sceleton::str(key);
                    });
                    /*
                    String toSend =
                        String(F("{ \"type\": \"ir_key\", ")) + F("\"remote\": \"") +
                        s + F("\", ") + F("\"key\": \"") + F("rotate_cw") + F("\", ") +
                        F("\"timeseq\": ") + String(millis(), DEC) + F(" ") + F("}");

                    sceleton::send(toSend);
                    */
                } else if (_pA == 1 && _pB == 0 && pA == 1 && pB == 1) {
                    String key = F("rotate_ccw");
                    sceleton::udpSend([&](Msg& m) {
                        m.has_parsedRemote = true;
                        m.parsedRemote.remote = sceleton::str(s);
                        m.parsedRemote.key = sceleton::str(key);
                    });
                    /*
                    String toSend =
                        String(F("{ \"type\": \"ir_key\", ")) + F("\"remote\": \"") +
                        s + F("\", ") + F("\"key\": \"") + F("rotate_ccw") + F("\", ") +
                        F("\"timeseq\": ") + String(millis(), DEC) + F(" ") + F("}");

                    sceleton::send(toSend);
                    */
                }
                _pA = pA;
                _pB = pB;
            }
            if (pBtn != _pBtn) {
                if (_pBtn == 0 && pBtn == 1) {
                    /*
                    String toSend =
                        String(F("{ \"type\": \"ir_key\", ")) + F("\"remote\": \"") +
                        s + F("\", ") + F("\"key\": \"") + F("click") + F("\", ") +
                        F("\"timeseq\": ") + String(millis(), DEC) + F(" ") + F("}");

                    sceleton::send(toSend);
                    */
                    String key = F("click");
                    sceleton::udpSend([&](Msg& m) {
                        m.has_parsedRemote = true;
                        m.parsedRemote.remote = sceleton::str(s);
                        m.parsedRemote.key = sceleton::str(key);
                    });
                }
                _pBtn = pBtn;
            }
        }
    }

    static void cont() { encoderPinChanged = false; }

    void init() {
        const int pins[] = {pinA, pinB, pinButton};
        for (size_t i = 0; i < __countof(pins); ++i) {
            pinMode(pins[i], INPUT_PULLUP);
            attachInterrupt(digitalPinToInterrupt(pins[i]), pinChange, CHANGE);
        }
        _pA = digitalRead(pinA);
        _pB = digitalRead(pinB);
        _pBtn = digitalRead(pinButton);
    }

   private:
    const char* encName;
    const int pinA;
    const int pinB;
    const int pinButton;
    int _pA, _pB, _pBtn;

    static void ICACHE_RAM_ATTR pinChange() { encoderPinChanged = true; }
};

Encoder encoders[] = {
    Encoder("left", D1, D2, D3),
    Encoder("right", D5, D6, D7),
};

std::unique_ptr<uart::Sender> bpSender;
std::unique_ptr<uart::Receiver> bpReceiver;

void setup() {
#if defined(ESP32)
    // Disable brown
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
#endif

    class SinkImpl : public sceleton::Sink {
       private:
        int currRelayState;  // All relays are off by default

       public:
        SinkImpl() : currRelayState(0) {}

        virtual boolean relayState(uint32_t id) {
            int bit = 1 << id;
            return (currRelayState & bit) != 0;
        }

        virtual void switchRelay(uint32_t id, bool val) {
            // debugSerial->println(String("switchRelaySink: ") + String(id, DEC) + " " + (val ? "true" : "false"));

            int bit = 1 << id;
            currRelayState = currRelayState & ~bit;
            if (val) {
                currRelayState = currRelayState | bit;
            }

            if (sceleton::hasSolidStateRelay.isSet()) {
                if (id >= 0 && id < 4) {
                    bool invert = sceleton::invertRelayControl.isSet();
                    digitalWrite(ssdPins[id], val ? (invert ? 0 : 1) : (invert ? 1 : 0));
                }
            } else {
                if (!relayIsInitialized) {
#ifdef ESP01
                    relay = new SoftwareSerial(0, 2);  // RX, TX
#else
                    relay = new SoftwareSerial(D1, D0);  // RX, TX
                    relay->enableRx(false);              // We don't want to receive from it
#endif
                    relay->begin(9600);
                    delay(100);
                    relay->write(0x50);
                    delay(100);
                    relay->write(0x51);
                    delay(100);
                    relayIsInitialized = true;
                }

                relay->write('0' |
                             (sceleton::invertRelayControl.isSet()
                                  ? ~currRelayState
                                  : currRelayState));
            }
        }

        virtual void showMessage(const char* dd, int totalMsToShow) {
            if (dd != nullptr) {
                debugSerial->println(dd);
#ifndef ESP01
                //
                screen.showMessage(dd, totalMsToShow);
#endif // ESP01
            }
        }

        virtual void showTuningMsg(const char* dd) {
#ifndef ESP01
            screen.showTuningMsg(dd);
#endif // ESP01
        }

        virtual void setAdditionalInfo(const char* dd) {
#ifndef ESP01
            //
            screen.setAdditionalInfo(dd);
#endif // ESP01
        }

        virtual void setBrightness(int percents) {
#ifndef ESP01
            if (screenController != NULL) {
                screenController->setBrightness(percents);
            }
#endif // ESP01
        }

        virtual void setTime(uint32_t unixTime) {
            initialUnixTime = unixTime;
            timeRetreivedInMs = millis();
        }

        virtual void setLedStripe(const std::vector<uint32_t>& colors,
                                  int periodMs) {
			ledStripe->set(colors, periodMs);
        }

        virtual void runLedStripeEffect(uint32_t mainClr, std::vector<uint32_t> blinks,
                                  int periodMs) {
			ledStripe->runLedStripeEffect(mainClr, blinks, periodMs);
        }

        virtual std::array<uint8_t, 4> getLedStripePixel(size_t i) {
			return ledStripe->pixel(i);
        }

        virtual uint32_t getLedStripeLen() { 
			return ledStripe->pixelCount(); 
		}

        uint32_t restartReportedAt = 0;

        virtual void reboot() {
            if (restartAt - millis() >= 200) {
                if (restartReportedAt < millis()) {
                    restartReportedAt = millis() + 300;
#ifndef ESP01
                    if (screenController != NULL) {
                        debugPrint("Rebooting");
                        screen.clear();
                        screen.showTuningMsg("Ребут");

                        screenController->refreshAll();
                    }
                    // sceleton::webSocketClient->disconnect();
#endif // ESP01
                }
                restartAt = millis() + 200;
            }
        }

        virtual void enableScreen(const boolean enabled) {
            isScreenEnabled = enabled;
            if (sceleton::hasGPIO1Relay.isSet()) {
                digitalWrite(D8, enabled ? 1 : 0);
            }
        }

        virtual boolean screenEnabled() { return isScreenEnabled; }

        virtual void switchATX(const boolean on) {
            digitalWrite(D8, on ? 0 : 1); // 1 means DOWN! to start ATX we connect green wire with black
        }

        virtual void setPWMOnPin(uint32_t val, uint8_t pin, uint32_t periodMs) {
            for (size_t i = 0; i < __countof(pwmStates); ++i) {
                PWMState& pwm = pwmStates[i];
                if (pwm.pin == pin) {
                    pwm.startMs = millis();
                    pwm.endMs = pwm.startMs + periodMs;
                    pwm.start = pwm.curr;
                    pwm.target = 1024 * val / 100;
                    break;
                }
            }
        }

        virtual void setVolume(uint32_t vol) {
            dfPlayerSend(0x06, (uint16_t)vol);  //
        }

        virtual void playMp3(uint32_t index) {
            dfPlayerSend(0x03, (uint16_t)index);  //
        }

        virtual void showScreenContent(std::vector<uint8_t>&& content, uint32_t width, uint32_t height, 
            const ScreenOffset& offsetFrom, const ScreenOffset& offsetTo) {
#ifndef ESP01
            screen.showScreenContent(std::move(content), width, height, offsetFrom, offsetTo);
#endif // ESP01
        }

        virtual void forwardBluePillMsg(const uint8_t* msg, size_t sz) {
            // 
            if (bpSender) {
                bpSender->send(msg, sz);
            }
        }
    };

    sceleton::setup(new SinkImpl());

    if (sceleton::hasLedStripe.isSet()) {
		ledStripe = new LedStripe(NUMPIXELS, [=]() { return millis(); });
        stripe = new Adafruit_NeoPixel(NUMPIXELS, 0, NEO_GRBW + NEO_KHZ800);
        stripe->begin();

        for (size_t i = 0; i < ledStripe->pixelCount(); i++) {
			auto v = ledStripe->pixel(i);
            stripe->setPixelColor(
                i, stripe->Color(v[0], v[1], v[2], v[3]));
        }
        stripe->show();
    }

    if (sceleton::hasIrReceiver.isSet()) {
        ir.reserve(250);
        attachInterrupt(digitalPinToInterrupt(D2), irIRQHandler, CHANGE);
    }

    if (sceleton::hasDS18B20.isSet()) {
        oneWire = new OneWire(D1);

        oneWire->reset_search();
        oneWire->search(deviceAddress);
    }

    if (sceleton::hasDFPlayer.isSet()) {
        debugSerial->println("Init DFPlayer Mini");
        dfplayerSerial = new SoftwareSerial(D1, D0);  // RX, TX
        dfplayerSerial->begin(9600);
        pinMode(D3, INPUT);  // BUSY
    }

    if (sceleton::hasBME280.isSet()) {
        Wire.begin(D4, D3);
        Wire.setClock(100000);

        bme = new Adafruit_BME280();
        bool res = bme->begin(0x76);
        if (!res) {
            delete bme;
            bme = NULL;
        }
    } else {
        // Turn LED off
        pinMode(D4, OUTPUT);
        digitalWrite(D4, 1);
    }

    if (sceleton::hasHX711.isSet()) {
        hx711 = new Q2HX711(D5, D6);
    }

#ifndef ESP01
    // Initialize comms hardware
    // pinMode(BEEPER_PIN, OUTPUT);
    if (sceleton::hasScreen.isSet()) {
        screenController = new MAX72xx(
            screen, D5, D7, D6, sceleton::hasScreen180Rotated.isSet());
        screenController->setup();
    }
#endif // ESP01

    if (sceleton::hasButton.isSet()) {
        pinMode(D7, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(D7), handleInterruptD7, CHANGE);
    }

    if (sceleton::hasButtonD2.isSet()) {
        pinMode(D2, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(D2), handleInterruptD2, CHANGE);
    }

    if (sceleton::hasButtonD5.isSet()) {
        pinMode(D5, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(D5), handleInterruptD5, CHANGE);
    }

    if (sceleton::hasEncoders.isSet()) {
        // if (false) {
        for (size_t i = 0; i < __countof(encoders); ++i) {
            encoders[i].init();
        }

        debugSerial->println("PINS initialized");
    }

    if (sceleton::hasPotenciometer.isSet()) {
        nextPotentiometer = millis() + 100;
    }

    if (sceleton::hasSolidStateRelay.isSet()) {
        bool invert = sceleton::invertRelayControl.isSet();
        for (size_t i = 0; i < __countof(ssdPins); ++i) {
            pinMode(ssdPins[i], OUTPUT);
            digitalWrite(ssdPins[i], invert ? 1 : 0);
        }
    }

    if (sceleton::hasMsp430.isSet()) {
        msp430 = new SoftwareSerial(D1, D0);  // RX, TX
        msp430->begin(9600);
        debugSerial->println("Initialized MSP430");
        pinMode(D2, OUTPUT);
        digitalWrite(D2, 1);
    }

    if (sceleton::hasATXPowerSupply.isSet()) {
        pinMode(D8, OUTPUT);
        digitalWrite(D8, 1); // 1 means DOWN
        pinMode(D2, INPUT);
    }

    if (sceleton::hasPWMOnD0.isSet()) {
#if defined(ESP8266)
        analogWriteFreq(21000);
        analogWriteRange(1024);
#endif
        pinMode(D3, OUTPUT);
        pinMode(D4, OUTPUT);
        pinMode(D7, OUTPUT);
        pinMode(D6, OUTPUT);
        pinMode(D5, OUTPUT);

        analogWrite(D3, 0);
        analogWrite(D4, 0);
        analogWrite(D7, 0);
        digitalWrite(D6, 1);
        digitalWrite(D5, 1);
    }

    if (sceleton::hasGPIO1Relay.isSet()) {
        pinMode(D8, OUTPUT);
    }

    if (sceleton::hasHC_SR.isSet()) {
        pinMode(D6, OUTPUT);
        pinMode(D7, INPUT);
        attachInterrupt(digitalPinToInterrupt(D7), handleInterruptHCSRfall, FALLING);
    }

    if (sceleton::hasBluePill.isSet()) {
        debugSerial = new sceleton::DummySerial();
        bpSender.reset(new uart::Sender(
            [&](const uint8_t* buffer, size_t size) {
                return Serial.write(buffer, size);
            }
        ));

        Serial.begin(460800);
    }
}

unsigned long oldMicros = micros();
uint32_t lastScreenRefresh = millis();

uint16_t hours = 0;
uint16_t mins = 0;
uint64_t nowMs = 0;


const int updateTimeEachSec = 600;  // By default, update time each 600 seconds

WiFiClient client;

uint32_t lastTemp = millis();

uint32_t lastWeighteningStarted = millis();
long lastWeight = 0;
uint32_t wrongTempValueReceivedCnt = 0;

uint32_t lastReadDFBusy = millis() - 200;
uint32_t dfBusyNow = -1;

long lastStripeFrame = millis();

long lastLoop = millis();
long lastLoopEnd = millis();


unsigned long Mypulsein(int pin, int level) {
  int i = 0;
  unsigned long start, startImp, finishImp;
  start =  millis();
  startImp =  micros();
  finishImp =  micros();
  do {
    if (digitalRead(pin)==level){
      i = 1;
      startImp =  micros();
    }
  } while((i==0)&&((millis()-start)<500));
  i = 0;
  do {
    if (digitalRead(pin)!=level){
      i = 1;
      finishImp =  micros();
    }
  } while((i==0)&&((millis()-start)<1000));
 
  return finishImp - startImp;
}

void loop() {
    if (millis() - lastLoop > 50) {
        debugSerial->println(String("Long main loop: ") +
                             String(millis() - lastLoop, DEC) + " " +
                             String(millis() - lastLoopEnd, DEC));
    }
    lastLoop = millis();


    uint32_t st = millis();

    if (restartAt < st) {
        // debugSerial->println("ESP.reset");
#if defined(ESP8266)
        ESP.reset();
#endif
        ESP.restart();
    }

    if (interruptCounterD7 > 0) {
        sceleton::udpSend([&](Msg& m) {
            m.has_buttonPressedD7 = true;
            m.buttonPressedD7 = digitalRead(D7) == LOW;
        });
        interruptCounterD7 = 0;
    }
    if (interruptCounterD5 > 0) {
        sceleton::udpSend([&](Msg& m) {
            m.has_buttonPressedD5 = true;
            m.buttonPressedD5 = digitalRead(D5) == LOW;
        });
        interruptCounterD5 = 0;
    }
    if (interruptCounterD2 > 0) {
        sceleton::udpSend([&](Msg& m) {
            m.has_buttonPressedD2 = true;
            m.buttonPressedD2 = digitalRead(D2) == LOW;
        });
        interruptCounterD2 = 0;
    }

    if (sceleton::hasHX711.isSet() &&
        (millis() - lastWeighteningStarted) > 100 && hx711->readyToSend()) {
        lastWeighteningStarted = millis();

        // debugSerial->println();
        long val = hx711->read();

        sceleton::udpSend([&](Msg& m) {
            m.has_weight = true;
            m.weight = val;
        });
/*
        String toSend = String("{ \"type\": \"weight\", ") +
                        "\"value\": " + String(val, DEC) + ", " +
                        "\"timeseq\": " + String((uint32_t)millis(), DEC) +
                        " " + "}";

        sceleton::send(toSend);
*/
        lastWeight = val;
    }

    if (bme != nullptr && ((millis() - lastTemp) > 1000)) {
        lastTemp = millis();
        float hum = bme->readHumidity();
        float temp = bme->readTemperature();
        float pressure = bme->readPressure();
        // 

        sceleton::udpSend([=](Msg& m) {
            if (!isnan(hum)) {
                m.has_humidity = true; 
                m.humidity = hum;
            }
            if (!isnan(temp)) {
                m.has_temp = true; 
                m.temp = temp;
            }
            if (!isnan(pressure)) {
                m.has_pressure = true; 
                m.pressure = pressure;
            }
        });
    }

    if (oneWire != NULL) {
        if (millis() > nextRequest) {
            oneWire->reset();
            oneWire->write(0xCC);  //Обращение ко всем датчикам
            oneWire->write(0x44);  //Команда на конвертацию
            nextRead = millis() + interval;
            nextRequest = millis() + interval * 2;
        } else if (millis() > nextRead) {
            // debugSerial->println("Temp reading");
            oneWire->reset();
            oneWire->select(deviceAddress);
            oneWire->write(0xBE);  //Считывание значения с датчика
            uint32_t byte1 = oneWire->read();
            uint32_t byte2 = oneWire->read();
            if (byte1 == 0xff && byte2 == 0xff) {
                wrongTempValueReceivedCnt++;
                if (wrongTempValueReceivedCnt % 10 == 0) {
                    debugPrint("Wrong temp: " +
                               String(wrongTempValueReceivedCnt, DEC));
                }
                if (wrongTempValueReceivedCnt == 40) {
                    // debugSerial->println("Rebooting because of bad temp");
                    sceleton::sink->reboot();
                }
            } else {
                wrongTempValueReceivedCnt = 0;
                uint32_t temp =
                    (byte1 << 3 | byte2
                                      << 11);  //Принимаем два байта температуры
                float val = (float)temp * 0.0078125;

                // debugPrint("Temp: " + String(byte1, HEX) + " " +
                // String(byte2, HEX) + " -> " + String(val));

                sceleton::udpSend([&](Msg& m) {
                    m.has_temp = true;
                    m.temp = val;
                });
            }

            nextRead = ULONG_MAX;
        }
    }

    oldMicros = micros();
    testCntr++;

#ifndef ESP01
    if (screenController != NULL) {
        if (millis() > (lastScreenRefresh + 25)) {
            lastScreenRefresh = millis();
            screen.clear();

            if (sceleton::initializedWiFi && timeRetreivedInMs != 0) {
                if (isScreenEnabled) {
                    // UTC is the time at Greenwich Meridian (GMT)
                    // print the hour (86400 equals secs per day)
                    nowMs = initialUnixTime * 1000ull +
                            ((uint64_t)millis() - (uint64_t)timeRetreivedInMs);
                    nowMs += 3 * 60 * 60 * 1000;  // Timezone (UTC+3)

                    uint32_t epoch = nowMs / 1000ull;
                    hours = (epoch % 86400L) / 3600;
                    mins = (epoch % 3600) / 60;

                    screen.showTime(nowMs / dayInMs, nowMs % dayInMs);
                }
                screenController->refreshAll();
            } else {
                screen.clear();
                screen.set(0, 0,
                           OnePixelAt(Rectangle(0, 0, 32, 8),
                                      (millis() / 25) % (32 * 8)),
                           true);
                screenController->refreshAll();
            }
        }
    }
#endif // ESP01

    if (sceleton::hasIrReceiver.isSet()) {
        auto irPause = (int64_t)millis() - lastIRChangeMs;

        if (irPause > 60) {
            if (ir.size() > 5) {
                sceleton::udpSend([&](Msg& msg) {
                    msg.irKeyPeriods = sceleton::repeated_int(ir);
                });
                /*
                String decoded = "";
                // We intentionally skip the very first period, because it is a pause between keys
                for (size_t i = 1; i < ir.size(); ++i) {
                    if (decoded.length() > 0) {
                        decoded += ",";
                    }
                    decoded += String(ir[i], DEC);
                }

                String toSend = String("{ \"type\": \"raw_ir_key\", ") +
                    "\"periods\": [" + decoded + "], " +
                    "\"timeseq\": " + String(millis(), DEC) + " " +
                    "}";

                if (sceleton::webSocketClient.get() != NULL) {
                    sceleton::send(toSend);
                }
                */
            }

            ir.clear();
        }
    }

    if (sceleton::hasMsp430.isSet() && msp430 != NULL) {
        for (; msp430->available() > 0;) {
            int ch = msp430->read();
            lastMsp430Ping = millis();
            const char encoders[] = {'A', 'G', 'O'};
            const char* encoderNames[] = {"left", "middle", "right"};
            if (ch == 'Z') {
                // restart
                debugPrint("MSP430 started");
                debugSerial->println("MSP430 started");
            } else if (ch == '0') {
                // ping
            } else {
                // Encoders
                for (size_t enc = 0; enc < __countof(encoders); ++enc) {
                    String ss;
                    if (encoders[enc] + 1 == ch) {
                        ss = "rotate_cw";
                    } else if (encoders[enc] + 2 == ch) {
                        ss = "rotate_ccw";
                    } else if (encoders[enc] + 3 == ch) {
                        ss = "click";
                    }
                    if (ss != NULL) {
                        String s = "encoder_";
                        s += encoderNames[enc];
                        
                        // debugPrint(String("") + s + " " + ss);

                        String key = F("rotate_ccw");
                        sceleton::udpSend([&](Msg& m) {
                            m.has_parsedRemote = true;
                            m.parsedRemote.remote = sceleton::str(s);
                            m.parsedRemote.key = sceleton::str(ss);
                        });
                    }
                }
            }
        }
        if (millis() - lastMsp430Ping > 3000) {
            // debugPrint("MSP430 didn't ping us for 3seconds, let's restart
            // it");
            digitalWrite(D2, 0);
            String msg(F("MSP430 didn't ping us for 3seconds, let's restart it"));
            debugSerial->println(msg);
            debugPrint(msg);
            delay(50);
            digitalWrite(D2, 1);
            lastMsp430Ping = millis();
        }
    }

    if (dfplayerSerial != NULL &&
        dfplayerSerial->available() >= DFPLAYER_RECEIVED_LENGTH) {
        uint8_t _serial[DFPLAYER_RECEIVED_LENGTH] = {0};
        dfplayerSerial->readBytes(_serial, DFPLAYER_RECEIVED_LENGTH);

        uint16_t parameter = arrayToUint16(_serial + Stack_Parameter);
        uint8_t cmd = _serial[Stack_Command];

        debugSerial->println("RECV CMD " + String(cmd, HEX));

        switch (cmd) {
            case 0x41: {
                debugSerial->println("ACK");
                debugPrint("ACK");
            } break;
            case 0x43: {
                debugSerial->println("Vol: " + String(parameter, DEC));
                debugPrint("Vol" + String(parameter, DEC));
            } break;
            default: {
                for (int i = 0; i < DFPLAYER_RECEIVED_LENGTH; ++i) {
                    debugSerial->print(String(_serial[i], HEX));
                    debugSerial->print(" ");
                }
                debugSerial->println();
            }
        }
    }

    sceleton::loop();

    // debugSerial->println(String(millis(), DEC));

    // Process encoders
    for (size_t i = 0; i < __countof(encoders); ++i) {
        encoders[i].process();
    }
    Encoder::cont();

    if (sceleton::hasPotenciometer.isSet() &&
        (nextPotentiometer < millis())) {
        nextPotentiometer = millis() + 50;
        int readingIn = analogRead(A0);
        analogInValues[analogReadIndex++ %
                            __countof(analogInValues)] = readingIn;

        String s = "";
        int32_t total = 0;
        for (uint8_t ind = 0; ind < __countof(analogInValues); ++ind) {
            total += analogInValues[ind];
            s += " ";
            s += String(analogInValues[ind], DEC);
        }

        readingIn = (int32_t)(total / __countof(analogInValues));

        if (reportedAnalogValue != readingIn) {
            sceleton::udpSend([&](Msg& m) {
                m.has_potentiometer = true;
                m.potentiometer = readingIn;
            });
            reportedAnalogValue = readingIn;
        }
    }

    if (sceleton::hasDFPlayer.isSet()) {
        if (millis() - lastReadDFBusy > 200) {
            lastReadDFBusy = millis();
            uint32_t dfBusy = (uint32_t)digitalRead(D3);
            if (dfBusy != dfBusyNow) {
                debugSerial->println("DFPlayerBusy: " + String(dfBusy, DEC));
                debugPrint("DFPlayerBusy: " + String(dfBusy, DEC));
                if (dfBusy == 1) {
                    // Stopped playing, let's set volume to 0
                    // dfPlayerSend(0x06, (uint16_t)0);
                }
                dfBusyNow = dfBusy;
            }
        }

        // for (;dfplayerSerial->available() > 0;) {
        //  int b = dfplayerSerial->read();
        //  debugSerial->println(b);
        //}
    }

    // Led stripe
    if (ledStripe != NULL && (millis() - lastLedStripeUpdate) > 50 ) {
        lastLedStripeUpdate = millis();
        if (ledStripe->update()) {
			for (size_t i = 0; i < ledStripe->pixelCount(); i++) {
				auto v = ledStripe->pixel(i);
				stripe->setPixelColor(
					i, stripe->Color(v[0], v[1], v[2], v[3]));
			}
			stripe->show();
		}
    }

    if (sceleton::hasPWMOnD0.isSet()) {
        const uint32_t n = millis();
        for (size_t i = 0; i < __countof(pwmStates); ++i) {
            PWMState& pwm = pwmStates[i];
            int32_t val = 0;
            if (n >= pwm.endMs) {
                val = pwm.target;
            } else if (n <= pwm.startMs) {
                val = pwm.curr;\
            } else {
                val = pwm.start + (int32_t)(n - pwm.startMs) * (int32_t)(pwm.target - pwm.start) / (int32_t)(pwm.endMs - pwm.startMs);
            }

            if (val < 0) {
                val = 0;
            } else if (val > 1023) {
                val = 1023;
            }

            if (val != pwm.curr) {
                if (pwm.lastAnalogWriteMs + 10 < millis()) {
                    pwm.lastAnalogWriteMs = millis();
                    pwm.curr = val;
                    if (val == 0) {
                        digitalWrite(pwm.pin, 0);
                    } else if (val >= 1023) {
                        digitalWrite(pwm.pin, 1);
                    } else {
                        analogWrite(pwm.pin, val);
                    }
                }
            }
        }
    }

    if (sceleton::hasBluePill.isSet()) {

    }

    if (sceleton::hasHC_SR.isSet()) {
        if (hcsrDist != 0 && hcsrDistTime != 0) {
            destinies.push_back(hcsrDistTime);
            destinies.push_back(hcsrDist);
            hcsrStart = 0;
            hcsrDist = 0;
            hcsrDistTime = 0;
        }

        if ((millis() - lastHCSR > 15)) {
            lastHCSR  = millis();

            digitalWrite(D6, LOW);
            delayMicroseconds(1);
            digitalWrite(D6, HIGH);
            delayMicroseconds(8);
            digitalWrite(D6, LOW);
            
            //writeHCRState(true);
            hcsrStart = micros();
            hcsrDist = 0;
            hcsrDistTime = 0;
        }

        if ((millis() - lastHCSRSent > 400)) {
            lastHCSRSent = millis();

            // debugPrint("Sent destinies !");

            sceleton::udpSend([&](Msg& msg) {
                    msg.destinies = sceleton::repeated_int(destinies);
                });
            destinies.resize(0);
        }

    }

    if (millis() % 500 == 12) {
        if (sceleton::hasATXPowerSupply.isSet()) {
            int currAtxState = digitalRead(D2);
            if (oldPowerState != currAtxState) {
                oldPowerState = currAtxState;
                
                sceleton::udpSend([&](Msg& m) {
                    m.has_atxState = true;
                    m.atxState = currAtxState != 0;
                });
            }
        }
    }

    lastLoopEnd = millis();
}
