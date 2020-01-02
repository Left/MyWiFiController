#include "sceleton.h"

#include <Adafruit_BME280.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_Sensor.h>
#include <WiFiUDP.h>

#include "worklogic.h"

#ifndef ESP01
#include <OneWire.h>
#include <Q2HX711.h>
#include "lcd.h"
#endif

#include "ledStripe.h"
#include <SoftwareSerial.h>


#include <pb_encode.h>
#include <pb_decode.h>
#include "protocol.pb.h"

unsigned int localPort = 2390;  // local port to listen for UDP packets

const uint64_t dayInMs = 24 * 60 * 60 * 1000;

boolean isScreenEnabled = true;

#ifndef ESP01
LcdScreen screen;
MAX72xx* screenController = NULL;
#endif

Adafruit_BME280* bme = NULL;  // I2C

WiFiUDP udpClient;
// #define BEEPER_PIN D2 // Beeper

int testCntr = 0;

#ifndef ESP01
Q2HX711* hx711 = NULL;
#endif

#ifndef ESP01
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

static void ICACHE_RAM_ATTR irIRQHandler() { 
    unsigned long m = micros();
    ir.push_back(m - lastIRChange);
    lastIRChange = m;
    lastIRChangeMs = millis();
}
#endif

uint8_t bluePillPacketStart[] PROGMEM = {0x80, 0x1d, 0x7d, 0x2e, 0x00, 0x03, 0xb9, 0x13 };
uint8_t bluePillPacketEnd[] PROGMEM = {0xff, 0x5b, 0xa1, 0x35, 0x33, 0x6f, 0xf5, 0x37 };

std::vector<uint8_t> protobuf;

boolean invertRelayState = false;
boolean relayIsInitialized = false;
SoftwareSerial* relay = NULL;

#ifndef ESP01
OneWire* oneWire;

uint32_t lastMsp430Ping = millis();
SoftwareSerial* msp430 = NULL;  // RX, TX

SoftwareSerial* dfplayerSerial = NULL;  // RX, TX
#endif

const int NUMPIXELS = 64;
Adafruit_NeoPixel* stripe = NULL;

const long interval = 1000;  // Request each second
unsigned long nextRequest = millis();
unsigned long nextRead = ULONG_MAX;

#ifndef ESP01
typedef uint8_t DeviceAddress[8];
DeviceAddress deviceAddress = {0};
#endif

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

int interruptCounter = 0;

uint32_t timeRetreivedInMs = 0;
uint32_t initialUnixTime = 0;
uint32_t restartAt = ULONG_MAX;
#ifndef ESP01
uint32_t nextPotentiometer = 0;

uint16_t analogInValues[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t analogReadIndex = 0;

int32_t reportedAnalogValue = -1;

int oldPowerState = -1;

uint32_t ssdPins[] PROGMEM = {D1, D2, D5, D6};
#endif

LedStripe* ledStripe = NULL;
uint32_t lastLedStripeUpdate = 0;

void ICACHE_RAM_ATTR handleInterrupt() { interruptCounter++; }

#ifndef ESP01
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
                    String toSend =
                        String(F("{ \"type\": \"ir_key\", ")) + F("\"remote\": \"") +
                        s + F("\", ") + F("\"key\": \"") + F("rotate_cw") + F("\", ") +
                        F("\"timeseq\": ") + String(millis(), DEC) + F(" ") + F("}");

                    sceleton::send(toSend);
                } else if (_pA == 1 && _pB == 0 && pA == 1 && pB == 1) {
                    String toSend =
                        String(F("{ \"type\": \"ir_key\", ")) + F("\"remote\": \"") +
                        s + F("\", ") + F("\"key\": \"") + F("rotate_ccw") + F("\", ") +
                        F("\"timeseq\": ") + String(millis(), DEC) + F(" ") + F("}");

                    sceleton::send(toSend);
                }
                _pA = pA;
                _pB = pB;
            }
            if (pBtn != _pBtn) {
                if (_pBtn == 0 && pBtn == 1) {
                    String toSend =
                        String(F("{ \"type\": \"ir_key\", ")) + F("\"remote\": \"") +
                        s + F("\", ") + F("\"key\": \"") + F("click") + F("\", ") +
                        F("\"timeseq\": ") + String(millis(), DEC) + F(" ") + F("}");

                    sceleton::send(toSend);
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
#endif  // ESP01

void setup() {
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
            // debugSerial->println(String("switchRelaySink: ") + (val ? "true"
            // : "false"));
            int bit = 1 << id;
            currRelayState = currRelayState & ~bit;
            if (val) {
                currRelayState = currRelayState | bit;
            }
#ifndef ESP01
            if (sceleton::hasSolidStateRelay.isSet()) {
                if (id >= 0 && id < 4) {
                    bool invert = sceleton::invertRelayControl.isSet();
                    digitalWrite(ssdPins[id], val ? (invert ? 0 : 1) : (invert ? 1 : 0));
                }
            } else {
#endif
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
#ifndef ESP01
            }
#endif
            if (sceleton::hasGPIO1Relay.isSet()) {
                digitalWrite(D4, val);
            }
        }

        virtual void showMessage(const char* dd, int totalMsToShow) {
#ifndef ESP01
            //
            screen.showMessage(dd, totalMsToShow);
#endif
        }

        virtual void showTuningMsg(const char* dd) {
#ifndef ESP01
            screen.showTuningMsg(dd);
#endif
        }

        virtual void setAdditionalInfo(const char* dd) {
#ifndef ESP01
            //
            screen.setAdditionalInfo(dd);
#endif
        }

        virtual void setBrightness(int percents) {
#ifndef ESP01
            if (screenController != NULL) {
                screenController->setBrightness(percents);
            }
#endif
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
                    // debugSerial->println("Rebooting");
                    if (screenController != NULL) {
                        debugPrint("Rebooting");
                        screen.clear();
                        screen.showTuningMsg("Ребут");

                        screenController->refreshAll();
                    }
                    sceleton::webSocketClient->disconnect();
#endif
                }
                restartAt = millis() + 200;
            }
        }

        virtual void enableScreen(const boolean enabled) {
            isScreenEnabled = enabled;
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
#ifndef ESP01
            dfPlayerSend(0x06, (uint16_t)vol);  //
#endif
        }

        virtual void playMp3(uint32_t index) {
#ifndef ESP01
            dfPlayerSend(0x03, (uint16_t)index);  //
#endif
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

#ifndef ESP01
    if (sceleton::hasIrReceiver.isSet()) {
       attachInterrupt(digitalPinToInterrupt(D2), irIRQHandler, CHANGE);
    }
#endif

#ifndef ESP01
    if (sceleton::hasDS18B20.isSet()) {
        oneWire = new OneWire(D1);

        oneWire->reset_search();
        oneWire->search(deviceAddress);
    }
#endif

#ifndef ESP01
    if (sceleton::hasDFPlayer.isSet()) {
        debugSerial->println("Init DFPlayer Mini");
        dfplayerSerial = new SoftwareSerial(D1, D0);  // RX, TX
        dfplayerSerial->begin(9600);
        pinMode(D3, INPUT);  // BUSY
    }
#endif

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

#ifndef ESP01
    if (sceleton::hasHX711.isSet()) {
        hx711 = new Q2HX711(D5, D6);
    }
#endif

    // Initialize comms hardware
    // pinMode(BEEPER_PIN, OUTPUT);
#ifndef ESP01
    if (sceleton::hasScreen.isSet()) {
        screenController = new MAX72xx(
            screen, D5, D7, D6, sceleton::hasScreen180Rotated.isSet());
        screenController->setup();
    }

    if (sceleton::hasButton.isSet()) {
        pinMode(D7, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(D7), handleInterrupt, CHANGE);
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
#endif
    if (sceleton::hasPWMOnD0.isSet()) {
        analogWriteFreq(21000);
        analogWriteRange(1024);

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
    }
    
    udpClient.begin(atoi(sceleton::websocketPort.value()) + 1);

    if (sceleton::hasBluePill.isSet()) {
        debugSerial = new sceleton::DummySerial();

        udpClient.begin(atoi(sceleton::websocketPort.value()) + 1);
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

#ifndef ESP01
uint32_t lastWeighteningStarted = millis();
long lastWeight = 0;
uint32_t wrongTempValueReceivedCnt = 0;

uint32_t lastReadDFBusy = millis() - 200;
uint32_t dfBusyNow = -1;
#endif

long lastStripeFrame = millis();

long lastLoop = millis();
long lastLoopEnd = millis();

std::vector<uint8_t> buffer;
Msg message = Msg_init_zero;

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
        ESP.reset();
        ESP.restart();
    }

    if (interruptCounter > 0) {
        debugSerial->println("1");
        String toSend =
            String("{ \"type\": \"button\", ") +
            "\"value\": " + (digitalRead(D7) == LOW ? "true" : "false") + ", " +
            "\"timeseq\": " + String((uint32_t)millis(), DEC) + " " + "}";

        sceleton::send(toSend);
        interruptCounter = 0;
    }

#ifndef ESP01
    if (sceleton::hasHX711.isSet() &&
        (millis() - lastWeighteningStarted) > 100 && hx711->readyToSend()) {
        debugSerial->println("3");
        lastWeighteningStarted = millis();

        // debugSerial->println();
        long val = hx711->read();

        String toSend = String("{ \"type\": \"weight\", ") +
                        "\"value\": " + String(val, DEC) + ", " +
                        "\"timeseq\": " + String((uint32_t)millis(), DEC) +
                        " " + "}";

        sceleton::send(toSend);

        lastWeight = val;
    }
#endif

    if (bme != NULL && ((millis() - lastTemp) > 4000)) {
        lastTemp = millis();
        float hum = bme->readHumidity();
        float temp = bme->readTemperature();
        float pressure = bme->readPressure();

        struct {
            const char* name;
            float value;
        } toSendArr[] = {
            {"temp", temp},
            {"humidity", hum},
            {"pressure", pressure},
        };
        for (size_t i = 0; i < __countof(toSendArr); ++i) {
            if (!isnan(toSendArr[i].value)) {
                String toSend =
                    String("{ \"type\": \"") + String(toSendArr[i].name) +
                    String("\", ") +
                    "\"value\": " + String(toSendArr[i].value) + ", " +
                    "\"timeseq\": " + String((uint32_t)millis(), DEC) + " " +
                    "}";
                sceleton::send(toSend);
            }
        }
    }

#ifndef ESP01
    if (oneWire != NULL) {
        debugSerial->println("5");
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

                String toSend =
                    String("{ \"type\": \"temp\", ") +
                    "\"value\": " + String(val) + ", " +
                    "\"timeseq\": " + String((uint32_t)millis(), DEC) + " " +
                    "}";
                sceleton::send(toSend);
            }

            nextRead = ULONG_MAX;
        }
    }
#endif

    oldMicros = micros();
    testCntr++;

#ifndef ESP01
    if (screenController != NULL) {
        if (millis() > (lastScreenRefresh + 20)) {
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
                                      (millis() / 30) % (32 * 8)),
                           true);
                screenController->refreshAll();
            }
        }
    }
#endif

#ifndef ESP01
    if (sceleton::hasIrReceiver.isSet()) {
        auto irPause = (int64_t)millis() - lastIRChangeMs;

        if (irPause > 60) {
            if (ir.size() > 5) {
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

                /*
                String decoded = "";
                
                message.id = 22;
                message.timeseq = millis();

                // We intentionally skip the very first period, because it is a pause between keys
                for (size_t i = 1; i < ir.size(); ++i) {
                    message.irKeyPeriods[i - 1] = ir[i];
                }
                message.irKeyPeriods_count = ir.size() - 1;

                buffer.resize(4000, 0);
                pb_ostream_t stream = pb_ostream_from_buffer(&buffer[0], buffer.size());
                pb_encode(&stream, Msg_fields, &message);

                debugSerial->println("!!!!!!!!!!!");
                debugSerial->println(String(stream.bytes_written, DEC));

                udpClient.beginPacket(
                    sceleton::websocketServer.value(), 
                    atoi(sceleton::websocketPort.value()) + 1);
                int b = stream.bytes_written;
                for (const uint8_t* p = &buffer[0];
                    b > 0; ++p, --b) {
                    udpClient.write(*p);
                } 
                udpClient.endPacket();
                */
            }

            ir.clear();
        }
    }

    if (sceleton::hasMsp430.isSet() && msp430 != NULL &&
        sceleton::webSocketClient.get() != NULL) {
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
                    const char* ss = NULL;
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
                        String toSend =
                            String("{ \"type\": \"ir_key\", ") +
                            "\"remote\": \"" + s + "\", " + "\"key\": \"" + ss +
                            "\", " + "\"timeseq\": " + String(millis(), DEC) +
                            " " + "}";
                        // debugSerial->printf("> %s %s\n", s.c_str(), ss);

                        sceleton::send(toSend);
                    }
                }
            }
        }
        if (millis() - lastMsp430Ping > 3000) {
            // debugPrint("MSP430 didn't ping us for 3seconds, let's restart
            // it");
            digitalWrite(D2, 0);
            debugSerial->println(
                "MSP430 didn't ping us for 3seconds, let's restart it");
            debugPrint("MSP430 didn't ping us for 3seconds, let's restart it");
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

        switch (cmd) {
            case 0x41: {
                debugSerial->println("ACK");
            } break;
            case 0x43: {
                debugSerial->println("Vol: " + String(parameter, DEC));
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
#endif

    sceleton::loop();

    // debugSerial->println(String(millis(), DEC));

#ifndef ESP01
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
            if (sceleton::webSocketClient.get() != NULL) {
                String toSend = String("{ \"type\": \"potentiometer\", ") +
                                "\"value\": \"" + readingIn + "\", " +
                                "\"timeseq\": " + String(millis(), DEC) + " " +
                                "}";
                sceleton::send(toSend);
                reportedAnalogValue = readingIn;
            }
        }
    }
#endif

#ifndef ESP01
    if (sceleton::hasDFPlayer.isSet()) {
        if (millis() - lastReadDFBusy > 200) {
            lastReadDFBusy = millis();
            uint32_t dfBusy = (uint32_t)digitalRead(D3);
            if (dfBusy != dfBusyNow) {
                debugSerial->println("DFPlayerBusy: " + String(dfBusy, DEC));
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
#endif

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
        size_t avail = Serial.available();
        if (avail > 0) {
            size_t olds = protobuf.size();
            protobuf.resize(olds + avail);            
            Serial.readBytes(&protobuf[olds], avail);

            uint8_t* bufferStart = &protobuf[0];
            uint8_t* bufferEnd = bufferStart + protobuf.size();

            uint8_t* startSeq = std::search(bufferStart, bufferEnd,
                &bluePillPacketStart[0], &bluePillPacketStart[0] + __countof(bluePillPacketStart));
            uint8_t* endSeq = std::search(bufferStart, bufferEnd,
                &bluePillPacketEnd[0], &bluePillPacketEnd[0] + __countof(bluePillPacketEnd));
            
            if ((startSeq != bufferEnd) && (endSeq != bufferEnd)) {               
                udpClient.beginPacket(
                    sceleton::websocketServer.value(), 
                    atoi(sceleton::websocketPort.value()) + 1);
                for (const uint8_t* p = startSeq + __countof(bluePillPacketStart);
                    p != endSeq; ++p) {
                    udpClient.write(*p);
                }
                udpClient.endPacket();
                
                uint8_t* out = bufferStart;
                for (const uint8_t* p = endSeq + __countof(bluePillPacketEnd);
                    p != bufferEnd; ++p, ++out) {
                    *out = *p;
                }

                protobuf.resize(out - bufferStart);
            }
        }
    }

#ifndef ESP01
    if (millis() % 500 == 12) {
        if (sceleton::hasATXPowerSupply.isSet()) {
            int currAtxState = digitalRead(D2);
            if (oldPowerState != currAtxState) {
                oldPowerState = currAtxState;
                if (sceleton::webSocketClient.get() != NULL) {
                    String toSend =
                        String("{ \"type\": \"atxState\", ") + "\"value\": " +  String(currAtxState, DEC) + ", " +
                        "\"timeseq\": " + String(millis(), DEC) + " " + "}";
                    sceleton::send(toSend);
                }
            }
        }
    }
#endif

    lastLoopEnd = millis();
}
