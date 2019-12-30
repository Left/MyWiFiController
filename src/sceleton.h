#pragma once

#include "common.h"

#include <array>
#include <Arduino.h>
#include <FS.h>
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <WebSockets.h>
#include <WebSocketsClient.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoOTA.h>

Stream* debugSerial;

void debugPrint(const String& str);

namespace sceleton {

class Sink {
    std::vector<uint32_t> empty;
public:
    virtual void showMessage(const char* s, int totalMsToShow) {}
    virtual void showTuningMsg(const char* s) {}
    virtual void setAdditionalInfo(const char* s) {}
    virtual void switchRelay(uint32_t id, bool val) {}
    virtual boolean relayState(uint32_t id) { return false; } 
    virtual void setBrightness(int percents) {}
    virtual void setTime(uint32_t unixTime) {}
    virtual void setLedStripe(const std::vector<uint32_t>& colors, int periodMs) {}
    virtual void runLedStripeEffect(uint32_t mainClr, std::vector<uint32_t> blinks, int periodMs) {}
    virtual std::array<uint8_t, 4> getLedStripePixel(size_t i) { return {0, 0, 0, 0}; }
    virtual uint32_t getLedStripeLen() { return 0; }
    virtual void setPWMOnPin(uint32_t val, uint8_t pin, uint32_t periodMs) {}
    virtual void playMp3(uint32_t index) {}
    virtual void setVolume(uint32_t vol) {}
    virtual void reboot() {}
    virtual void enableScreen(const boolean enabled) {}
    virtual boolean screenEnabled() { return false; }
    virtual void switchATX(const boolean on) {}
};

String fileToString(const String& fileName) {
    if (SPIFFS.exists(fileName.c_str())) {
        File f = SPIFFS.open(fileName.c_str(), "r");
        debugSerial->println(f.size());
        std::vector<uint8_t> buf(f.size() + 1, 0);
        if (f && f.size()) {
        f.read((uint8_t*)(&buf[0]), buf.size());
        }
        f.close();
        return String((const char*)(&buf[0]));
    }
    return String();
}

const char* typeKey PROGMEM = "type";
const char* firmwareVersion PROGMEM = "00.22";

std::auto_ptr<AsyncWebServer> setupServer;
std::auto_ptr<WebSocketsClient> webSocketClient;

const int dpin[] PROGMEM = { D0, D1, D2, D3, D4, D5, D6, D7, D8, D9 };

long vccVal = 0;
uint32_t rebootAt = 0x7FFFFFFF;

void send(const String& toSend) {
    webSocketClient->sendTXT(toSend.c_str(), toSend.length());
}

class DevParam {
protected:
    String _value;
public:
    String _name;
    String _jsonName;
    String _description;
    bool _password;

    DevParam(String name, String jsonName, String description, String value, bool pwd=false) :
        _value(value),
        _name(name),
        _jsonName(jsonName),
        _description(description),
        _password(pwd) {
    }

    const char* value() {
        return _value.c_str();
    }

    virtual void setValue(const String& v) {
        _value = v;
    }

    virtual void renderHTML(String& content) {
        content += F("<label class='lbl'>");
            content += _description;
            content += F(":</label>");
        content += F("<input name='");
            content += _name;
            content += F("' value='");
            content += _password ? String(F("")) : _value;
            content += F("' length=32/><br/>");
    }

    virtual bool parseHTML(const String& val) {
        // Param is set
        if (!_password || val.length() > 0) {
            if (_value != val) {
                _value = val;
                return true;
            }
        }
        return false;
    }

    virtual ~DevParam() {}
};

class BoolDevParam : public DevParam {
    bool _set = false;
public:
    BoolDevParam(String name, String jsonName, String description, bool boolValue) :
        DevParam(name, jsonName, description, (boolValue ? String(F("true")) : String(F("false"))), false),
        _set(boolValue) {
    }

    virtual void renderHTML(String& content) {
        content += F("<label class='lbl'>");
        content += F("<input type='checkbox' ");
        if (_set) {
            content += F(" checked");
        }
        content += F(" name='");
        content += _name;
        content += F("'");
        content += F("/>");
        content += _description;
        content += F("</label><br/>");
    }

    virtual bool parseHTML(const String& val) {
        return DevParam::parseHTML(val == F("on") ? F("true") : F("false"));
    }

    bool isSet() {
        return _set;
    }

    virtual void setValue(const String& v) {
        DevParam::setValue(v);
        _set = v == F("true");
    }
};

#ifndef SSID_NAME
#define SSID_NAME ''
#endif

#ifndef SSID_PASS
#define SSID_PASS ''
#endif

#define PASS_STRING TO_STR(SSID_PASS)
#define SSID_STRING TO_STR(SSID_NAME)

DevParam deviceName("device.name", "name", "Device Name", String("ESP_") + ESP.getChipId()) PROGMEM;
//DevParam deviceName("device.name", "name", "Device Name", String("RelayOnKitchen")) PROGMEM;
DevParam deviceNameRussian("device.name.russian", "rname", "Device Name (russian)", String("ESP_") + ESP.getChipId()) PROGMEM;
//DevParam deviceNameRussian("device.name.russian", "rname", "Device Name (russian)", String("Реле на кухне")) PROGMEM;
DevParam wifiName("wifi.name", "wifi", "WiFi SSID", SSID_STRING) PROGMEM;
DevParam wifiPwd("wifi.pwd", "wfpwd", "WiFi Password", PASS_STRING, true) PROGMEM;
BoolDevParam logToHardwareSerial("debug.to.serial", "debugserial", "Print debug to serial", true) PROGMEM;
DevParam websocketServer("websocket.server", "ws", "WebSocket server", "192.168.121.38") PROGMEM;
DevParam websocketPort("websocket.port", "wsport", "WebSocket port", "8080") PROGMEM;
BoolDevParam invertRelayControl("invertRelay", "invrelay", "Invert relays", false) PROGMEM;
#ifndef ESP01
BoolDevParam hasScreen("hasScreen", "screen", "Has screen", false) PROGMEM;
BoolDevParam hasScreen180Rotated("hasScreen180Rotated", "screen180", "Screen is rotated on 180", false) PROGMEM;
BoolDevParam hasHX711("hasHX711", "hx711", "Has HX711 (weight detector)", false) PROGMEM;
BoolDevParam hasIrReceiver("hasIrReceiver", "ir", "Has infrared receiver", false) PROGMEM;
BoolDevParam hasDS18B20("hasDS18B20", "ds18b20", "Has DS18B20 (temp sensor)", false) PROGMEM;
BoolDevParam hasDFPlayer("hasDFPlayer", "dfplayer", "Has DF player", false) PROGMEM;
#endif
BoolDevParam hasBME280("hasBME280", "bme280", "Has BME280 (temp & humidity sensor)", false) PROGMEM;
BoolDevParam hasLedStripe("hasLedStripe", "ledstrip", "Has RGBW Led stripe", false) PROGMEM;
BoolDevParam hasBluePill("hasBluePill", "bluepill", "Has bluepill", false) PROGMEM;
#ifndef ESP01
BoolDevParam hasButton("hasButton", "d7btn", "Has button on D7", false) PROGMEM;
DevParam brightness("brightness", "bright", "Brightness [0..100]", "0") PROGMEM;
BoolDevParam hasEncoders("hasEncoders", "enc", "Has encoders", false) PROGMEM;
BoolDevParam hasMsp430("hasMsp430WithEncoders", "msp430", "Has MSP430 with encoders", false) PROGMEM;
BoolDevParam hasPotenciometer("hasPotenciometer", "potent", "Has ADC connected", false) PROGMEM;
BoolDevParam hasSolidStateRelay("hasSSR", "ssr", "Has Solid State Relay (D1, D2, D5, D6)", false) PROGMEM;
BoolDevParam hasATXPowerSupply("hasATXPowerSupply", "atx", "Has ATX power supply", false) PROGMEM;
#endif
DevParam relayNames("relay.names", "relays", "Relay names, separated by ;", "") PROGMEM;
//DevParam relayNames("relay.names", "relays", "Relay names, separated by ;", "Потолок;Лента") PROGMEM;
BoolDevParam hasGPIO1Relay("hasGPIO1Relay", "gpio1relay", "Has GPIO1 Relay", false) PROGMEM;
BoolDevParam hasPWMOnD0("hasPWMOnD0", "pwmOnD0", "Has PWM on D0", false) PROGMEM;
DevParam secondsBeforeRestart("secondsBeforeRestart", "watchdog", "Seconds before restart", "60000") PROGMEM;

uint32_t msBeforeRestart = atoi(secondsBeforeRestart.value());

DevParam* devParams[] = { 
    &deviceName, 
    &deviceNameRussian,
    &wifiName, 
    &wifiPwd, 
    &logToHardwareSerial,
    &websocketServer, 
    &websocketPort, 
    &invertRelayControl, 
#ifndef ESP01
    &hasScreen, 
    &hasScreen180Rotated,
    &hasHX711,
    &hasIrReceiver,
    &hasDS18B20,
    &hasDFPlayer,
#endif
    &hasBME280,
    &hasLedStripe,
    &hasBluePill,
#ifndef ESP01
    &hasEncoders,
    &hasButton, 
    &brightness,
    &hasMsp430,
#endif
    &relayNames,
    &hasGPIO1Relay,
#ifndef ESP01
    &hasPotenciometer,
    &hasSolidStateRelay,
    &hasATXPowerSupply,
#endif
    &hasPWMOnD0,
    &secondsBeforeRestart
}; 
Sink* sink = new Sink();
boolean initializedWiFi = false;
uint32_t lastReceived = millis();
uint32_t reconnectWebsocketAt = 0x7FFFFFFF; // Never by def
uint32_t reportedGoingToReconnect = millis();

void reportRelayState(uint32_t id) {
    send(String(F("{ \"type\": \"relayState\", \"id\": ")) + String(id, DEC) + F(", \"value\":") + (sink->relayState(id) ? F("true") : F("false")) + F(" }"));
}

void onDisconnect(const WiFiEventStationModeDisconnected& event) {
    // debugSerial->println("WiFi On Disconnect.");
    // debugSerial->println(event.reason);
}

String encodeRGBWString(Sink* sink) {
    String res = "";
    for (uint32_t k = 0; k < sink->getLedStripeLen(); ++k) {
        auto toAdd = sink->getLedStripePixel(k);
        for (int i = 0; i<8; ++i) {
            char xx = (char)((toAdd[i/2] >> ((i % 2) * 4)) & 0xf);
            if (xx > 9) {
                res += (char)('A' + (xx - 10));
            } else {
                res += (char)('0' + xx);
            }
        }
    }
    return res;
}

void saveSettings() {
    uint32_t t = millis();
    debugSerial->println(F("Saving settings..."));

    DynamicJsonDocument jsonBuffer(1000);
    JsonObject root = jsonBuffer.to<JsonObject>();

    for (DevParam* d : devParams) {
        root[d->_jsonName] = d->value();
    }

    size_t sz = measureJsonPretty(root);
    char* buf = (char*)malloc(sz + 10);
    memset(buf, 0, sz+1);

    serializeJson(root, buf, sz);

    debugSerial->println(String(F("Settings:\n")) + String(buf) + F("\n\n"));

    File f = SPIFFS.open("settings.json", "w");
    f.write((uint8_t*)buf, sz);
    f.flush();
    f.close();
    
    free(buf);

    debugSerial->println(String(F("Saved settings in ")) + String((millis() - t), DEC));
}

std::vector<uint32_t> decodeRGBWString(const char* val) {
    std::vector<uint32_t> resArr;
    for (;;) {
        uint32_t toAdd = 0;
        for (int i = 0; i<8; ++val, ++i) {
            if (*val == 0) {
                return resArr;
            }

            const char c = *val;
            int x = (c >= '0' && c <= '9') ? (c - '0') : (c - 'A' + 10);
            toAdd |= (x << (28 - i*4));
        }
        resArr.push_back(toAdd);
    }
    return resArr;
}

bool wasConnected = false;
uint32_t saveBrightnessAt = 0x7FFFFFFF;

class DummySerial: public Stream {
    virtual size_t write(uint8_t) { return 0; }
    virtual int available() { return 0; }
    virtual int read() { return -1; }
    virtual int peek() { return -1; }
};

void setup(Sink* _sink) {
    sink = _sink;
    // Serial1.setDebugOutput(true);
    Serial1.begin(2000000);
    // Serial.begin(2000000);
    SPIFFS.begin();

    HardwareSerial* ds = &Serial;
    ds->begin(460800);

    debugSerial = ds;
    debugSerial->println(F("\n\n====================="));

    long was = millis();

    {   // Read initial settings
        String readVal = fileToString("settings.json");
        StaticJsonDocument<2000> jsonBuffer;
        // debugSerial->println(readVal);
        DeserializationError error = deserializeJson(jsonBuffer, readVal.c_str(), readVal.length());
        if (error == DeserializationError::Ok) {
            const JsonObject &root = jsonBuffer.as<JsonObject>();
            for (DevParam* d : devParams) {
                d->setValue((const char*) (root[d->_jsonName]));
            }
        } else {
            debugSerial->println(F("No settings read, use defaults"));
        }
    }

    if (!logToHardwareSerial.isSet()) {
        debugSerial = new DummySerial();
    }

    msBeforeRestart = atoi(secondsBeforeRestart.value());

    debugSerial->println(String(F("Initialized in ")) + String(millis() - was, DEC));
    debugSerial->println(wifiName.value());
    debugSerial->println(wifiPwd.value());

    WiFi.persistent(false);
    WiFi.setAutoConnect(false);
    WiFi.setAutoReconnect(false);
    WiFi.setPhyMode(WIFI_PHY_MODE_11G);
    WiFi.setSleepMode(WIFI_NONE_SLEEP);

    if (String(wifiName.value()).length() > 0 && String(wifiPwd.value()).length() > 0) {
        WiFi.onStationModeConnected([=](const WiFiEventStationModeConnected& e) {
            debugSerial->println(F("onStationModeConnected"));
            debugSerial->println(e.ssid);
            // debugSerial->println(e.bssid);
            debugSerial->println(e.channel);
        });
        WiFi.onStationModeDisconnected([=](const WiFiEventStationModeDisconnected& e) {
            debugSerial->println(F("onStationModeDisconnected"));
        });
        WiFi.onStationModeAuthModeChanged([=](const WiFiEventStationModeAuthModeChanged& e) {
            debugSerial->println(F("onStationModeAuthModeChanged"));
        });
        WiFi.onStationModeGotIP([=](const WiFiEventStationModeGotIP& e) {
            debugSerial->println(F("onStationModeGotIP"));
        });
        WiFi.onStationModeDHCPTimeout([=](void) {
            debugSerial->println(F("onStationModeDHCPTimeout"));
        });
        WiFi.onSoftAPModeStationConnected([=](const WiFiEventSoftAPModeStationConnected& e) {
            debugSerial->println(F("onSoftAPModeStationConnected"));
        });
        WiFi.onSoftAPModeStationDisconnected([=](const WiFiEventSoftAPModeStationDisconnected& e) {
            debugSerial->println(F("onSoftAPModeStationDisconnected"));
        });
        WiFi.onSoftAPModeProbeRequestReceived([=](const WiFiEventSoftAPModeProbeRequestReceived& e) {
            debugSerial->println(F("onSoftAPModeProbeRequestReceived"));
        });

        WiFi.mode(WIFI_STA);
        WiFi.hostname(String(F("ESP_")) + deviceName.value());
        WiFi.onStationModeDisconnected(onDisconnect);
        WiFi.begin(wifiName.value(), wifiPwd.value());
        // WiFi.waitForConnectResult();
    }
/*
    if (WiFi.status() == WL_CONNECTED) {
        IPAddress ip = WiFi.localIP();
        initializedWiFi = true;
        debugSerial->println("Connected to WiFi " + ip.toString());
    } else {
        WiFi.mode(WIFI_AP);

        String chidIp = String(ESP.getChipId(), HEX);
        String wifiAPName = ("ESP") + chidIp; // + String(millis() % 0xffff, HEX)
        String wifiPwd = String("pass") + chidIp;
        WiFi.softAP(wifiAPName.c_str(), wifiPwd.c_str(), 3); // , millis() % 5 + 1
        // WiFi.softAPConfig(IPAddress(192, 168, 4, 22), IPAddress(192, 168, 4, 9), IPAddress(255, 255, 255, 0));

        IPAddress accessIP = WiFi.softAPIP();
        debugSerial->println(String("ESP AccessPoint name       : ") + wifiAPName);
        debugSerial->println(String("ESP AccessPoint password   : ") + wifiPwd);
        debugSerial->println(String("ESP AccessPoint IP address : ") + accessIP.toString());

        // sink->showMessage((String("WiFi: ") + wifiAPName + ", password: " + wifiPwd + ", " + accessIP.toString()).c_str(), 0xffff);
    }
*/

    setupServer.reset(new AsyncWebServer(80));
    setupServer->on("/http_settup", [](AsyncWebServerRequest *request) {
        bool needReboot = false;
        for (DevParam* d : devParams) {
            auto p = request->getParam(d->_name);
            if (p != nullptr) {
                needReboot = needReboot || d->parseHTML(p->value());
            } else {
                needReboot = needReboot || d->parseHTML(String(F("")));
            }
        }
        
        if (needReboot) {
            saveSettings();
            request->send(200, String(F("text/html")), String(F("Settings changed, rebooting in 2 seconds...")));
            rebootAt = millis() + 2000;
        } else {
            request->send(200, String(F("text/html")), String(F("Nothing changed.")));  
        }
    });
    setupServer->on("/", [](AsyncWebServerRequest *request) {
        String content = F("<!DOCTYPE HTML>\r\n<html>");
        content += F("<head>");
        content += F("<meta http-equiv=\"content-type\" content=\"text/html; charset=UTF-8\">");
        content += F("</head>");
        content += F("<body>");
        content += F("<p>");
        content += F("<form method='get' action='http_settup'>");
        for (DevParam* d : devParams) {
            d->renderHTML(content);
        }
        content += F("<input type='submit'></form>");
        content += F("<form action='/reboot'><input type='submit' value='Reboot'/></form>");
        content += F("</html>");
        request->send(200, String(F("text/html")), content);  
    });
    setupServer->on("/reboot", [](AsyncWebServerRequest *request) {
        rebootAt = millis() + 100;
    });
    setupServer->onNotFound([](AsyncWebServerRequest *request) {
        request->send(404, String(F("text/plain")), String(F("Not found: ")) + request->url());
    });
    setupServer->begin();

    ArduinoOTA.setPort(8266);
    // set host name
    ArduinoOTA.setHostname(deviceName.value());

    ArduinoOTA.onStart([]() {
        // debugSerial->println("Start OTA");  //  "Начало OTA-апдейта"
        // debugSerial->println("Updating...");
    });
    ArduinoOTA.onEnd([]() {
        // debugSerial->println("End OTA");  //  "Завершение OTA-апдейта"
        // debugSerial->println("Done...");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        // debugSerial->printf("Progress: %u%%\r", (progress / (total / 100)));
        //  debugSerial->printf("Progress %d/%d\n", progress, total);
    });
    ArduinoOTA.onError([](ota_error_t error) {
        debugSerial->printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) {
            // debugSerial->println("Auth Failed");
            //  "Ошибка при аутентификации"
        } else if (error == OTA_BEGIN_ERROR) {
            // debugSerial->println("Begin Failed"); 
            //  "Ошибка при начале OTA-апдейта"
        } else if (error == OTA_CONNECT_ERROR) {
            // debugSerial->println("Connect Failed");
            //  "Ошибка при подключении"
        } else if (error == OTA_RECEIVE_ERROR) {
            // debugSerial->println("Receive Failed");
            //  "Ошибка при получении данных"
        } else if (error == OTA_END_ERROR) {
            // debugSerial->println("End Failed");
            //  "Ошибка при завершении OTA-апдейта"
        }
    });
}

int32_t lastEachSecond = millis() / 1000;
int32_t lastWiFiState = millis();
int32_t lastLoop = millis();

uint32_t oldStatus = WiFi.status();
uint32_t nextReconnect = millis();
uint32_t nextWiFiScan = millis();

void loop() {
    if (millis() - lastLoop > 50) {
         debugSerial->println(String(F("Long loop: ")) + String(millis() - lastLoop, DEC));
    }
    lastLoop = millis();

    if (WiFi.status() != WL_CONNECTED && (millis() > nextReconnect)) {
        debugSerial->println(String(F("WiFi.status() check: ")) + WiFi.status());
        bool ret = WiFi.reconnect();
        debugSerial->println(String(F("Reconnect returned ")) + String(ret, DEC));
        nextReconnect = millis() + (ret ? 4000 : 300);
    }

    if (millis() > nextWiFiScan) {
        debugSerial->println(String(F("Start scanning")));
        WiFi.scanNetworks(true);

        nextWiFiScan = millis() + 60000; // Scan every 10 seconds
    }

    int n = WiFi.scanComplete();
    if(n >= 0) {
        debugSerial->println(String(F("------------")));
        debugSerial->printf("%d network(s) found\n", n);
        for (int i = 0; i < n; i++) {
            debugSerial->printf("%d: %s, Ch:%d (%ddBm) %s\n", i+1, WiFi.SSID(i).c_str(), WiFi.channel(i), WiFi.RSSI(i), WiFi.encryptionType(i) == ENC_TYPE_NONE ? "open" : "");
        }
        WiFi.scanDelete();
        debugSerial->println(String(F("------------")));
    }

    if (oldStatus != WiFi.status()) {
        oldStatus = WiFi.status();
        debugSerial->println(String(F("WiFi.status(): ")) + oldStatus);

        if (WiFi.status() == WL_IDLE_STATUS || WiFi.status() == WL_DISCONNECTED) {
            debugSerial->println(String(F("Reconnecting")));
            reconnectWebsocketAt = 0x7FFFFFFF;
            WiFi.reconnect();
        }

        if (WiFi.status() == WL_CONNECTED) {
            debugSerial->println(String(F("Connected to WiFi, IP:")) + WiFi.localIP().toString());
            reconnectWebsocketAt = millis(); // Wait 1000 ms and connect to websocket
            ArduinoOTA.begin(false); // Begin OTA immediately
            initializedWiFi = true;
        }
    }

    if (millis() >= reconnectWebsocketAt) {
        webSocketClient.reset(new WebSocketsClient());
        auto wsHandler = [&](WStype_t type, uint8_t *payload, size_t length) {
            // debugSerial->println(String("WS Event") + type);
            switch (type) {
                case WStype_ERROR: {
                    debugSerial->println(String(F("WStype_ERROR")));
                    break;
                }
                case WStype_CONNECTED: {
                    if (wasConnected) { 
                        break;
                    }
                    reconnectWebsocketAt = 0x7FFFFFFF;  // No need to reconnect anymore
                    debugSerial->println(String(F("Connected to server")));
                    lastReceived = millis();
                    wasConnected = true;

                    String devParamsStr = F("{ ");
                    bool first = true;
                    for (DevParam* d : devParams) {
                        if (!d->_password && String(d->value()) != F("false")) {
                            if (!first) { 
                                devParamsStr += F(",");
                            }
                            first = false;
                            devParamsStr += F("\""),
                            devParamsStr += d->_name;
                            devParamsStr += F("\": \""),
                            devParamsStr += d->value();
                            devParamsStr += F("\" ");
                        }
                    }
                    devParamsStr += "}";

                    // Let's say hello and show all we can
                    send(String(F("{ ")) +
                        F("\"type\":\"hello\", ") +
                        F("\"firmware\":\"") + firmwareVersion + F("\", ") +
                        F("\"devParams\": ") + devParamsStr + F(", ") + 
                        F("\"screenEnabled\": ") + sink->screenEnabled() + F(", ") + 
                        F("\"deviceName\":\"") + sceleton::deviceName.value() + F("\"") + 
                        F(" }"));

                    // send message to client
                    // debugPrint("Hello server " + " (" + sceleton::deviceName._value +  "), firmware ver = " + firmwareVersion);
                    debugSerial->println(String(F("Hello sent")));

                    int cnt = 0;
                    for (const char* p = sceleton::relayNames.value(); *p != 0; ++p) {
                        if (*p == ';') {
                            if (cnt == 0) {
                                cnt = 1;
                            }

                            cnt++;
                        }
                    }

                    if (cnt > 0) {
                        for (int id = 0; id < cnt; ++id) {
                            send(String(F("{ \"type\": \"relayState\", \"id\": ")) + String(id, DEC) + F(", \"value\":") + (sink->relayState(id) ? F("true") : F("false")) + F(" }"));
                        }
                        debugSerial->println("Relays state sent");
                    }

                    if (sceleton::hasLedStripe.isSet()) {
                        send(String(F("{ \"type\": \"ledstripeState\", \"value\":\"")) + encodeRGBWString(sink) + F("\" }"));
                        debugSerial->println(String(F("LED stripe state sent")));
                    }

                    break;
                }
                case WStype_TEXT: {
                    // debugSerial->printf("[%u] get Text: %s\n", payload);
                    DynamicJsonDocument jsonBuffer(1000);

                    DeserializationError error = deserializeJson(jsonBuffer, payload);

                    if (error) {
                        // debugSerial->println("parseObject() failed");
                        send(String(F("{ \"errorMsg\":\"Failed to parse JSON\" }")));
                        return;
                    }
                    lastReceived = millis();

                    const JsonObject &root = jsonBuffer.as<JsonObject>();

                    String type = root[typeKey];
                    if (type == String(F("ping"))) {
                        // debugSerial->print(String(millis(), DEC) + ":");debugSerial->print("Ping "); debugSerial->println((const char*)(root["pingid"]));
                        String res = F("{ \"type\": \"pingresult\", \"pid\":\"");
                        res += (const char*)(root["pingid"]);
                        res += F("\" }");
                        send(res);
                    } else if (type == String(F("switch"))) {
                        // debugSerial->println("switch!");
                        bool sw = root["on"] == String(F("true"));
                        uint32_t id = atoi(root["id"]);
                        sink->switchRelay(id, sw);
                        reportRelayState(id);
                    } else if (type == String(F("setProp"))) {
                        saveSettings();
                    #ifndef ESP01
                    } else if (type == String(F("show"))) {
                        sink->showMessage(root["text"], root["totalMsToShow"].as<int>());
                    } else if (type == String(F("tune"))) {
                        sink->showTuningMsg(root["text"]);
                    } else if (type == String(F("unixtime"))) {
                        sink->setTime(root["value"].as<int>());
                    #endif
                    } else if (type == String(F("ledstripe"))) {
                        if (root["newyear"]) {
                            const char* basecolor = (const char*)(root["basecolor"]);
                            const char* blinkcolors = (const char*)(root["blinkcolors"]);
                            int periodMs = root["period"].as<int>();
                            sink->runLedStripeEffect(decodeRGBWString(basecolor)[0], decodeRGBWString(blinkcolors), periodMs);
                        } else {
                            const char* val = (const char*)(root["value"]);
                            int periodMs = root["period"].as<int>();
                            sink->setLedStripe(decodeRGBWString(val), periodMs);
                        }
                    
                        
                    } else if (type == String(F("playmp3"))) {
                        uint32_t index = (uint32_t)(root["index"].as<int>());
                        sink->playMp3(index);
                    } else if (type == String(F("setvolume"))) {
                        uint32_t index = (uint32_t)(root["value"].as<int>());
                        sink->setVolume(index);
                    } else if (type == String(F("pwm"))) {
                        int val = root["value"].as<int>();
                        const char* pin = root["pin"];
                        uint32_t periodMs = root["period"].as<int>();
                        if (pin[0] == 'D') {
                            sink->setPWMOnPin(val, dpin[pin[1] - '0'], periodMs);
                        }
                    #ifndef ESP01
                    } else if (type == String(F("atxEnable"))) {
                        int val = root[String(F("value"))].as<boolean>();
                        sink->switchATX(val);
                    } else if (type == String(F("screenEnable"))) {
                        int val = root[String(F("value"))].as<boolean>();
                        sink->enableScreen(val);
                        saveBrightnessAt = millis() + 1000; // In 1 second, save brightness
                    } else if (type == String(F("brightness"))) {
                        int val = root[String(F("value"))].as<int>();
                        val = std::max(std::min(val, 100), 0);
                        sink->setBrightness(val);
                        brightness.setValue(String(val, DEC));
                        saveBrightnessAt = millis() + 1000; // In 1 second, save brightness
                    #endif
                    } else if (type == String(F("additional-info"))) {
                        // 
                        sink->setAdditionalInfo(root[String(F("text"))]);
                    } else if (type == String(F("reboot"))) {
                        debugPrint(String(F("Let's reboot self")));
                        sink->reboot();
                    }
                    break;
                }
                case WStype_BIN: {
                    // debugSerial->printf("[%u] get binary length: %u\n", length);
                    // hexdump(payload, length);
                    debugPrint(String(F("Received binary packet of size ")) + String(length, DEC));

                    // send message to client
                    // webSocketClient.sendBIN(payload, length);
                    break;
                }
                case WStype_DISCONNECTED: {
                    // debugSerial->print(String(millis(), DEC) + ":"); debugSerial->printf("Disconnected [%u]!\n", WiFi.status());
                    if (WiFi.status() == WL_CONNECTED && wasConnected) {
                        wasConnected = false;
                        debugSerial->println(String(F("Disconnected from server ")) + String(length, DEC));
                        reconnectWebsocketAt = millis() + 4000; // In 4 second, let's try to reconnect
                    }
                    break;
                }
                case WStype_PING:
                case WStype_PONG:
                    break;

                default:
                    debugSerial->println(String(F("Unknown type: ")) + String(type, DEC));
            }
        };

        webSocketClient->onEvent(wsHandler);

        debugSerial->println(String(F("webSocketClient connecting to ")) + websocketServer.value());
        uint32_t ms = millis();
        webSocketClient->disableHeartbeat();
        webSocketClient->begin(websocketServer.value(), atoi(websocketPort.value()), "/esp");
        debugSerial->println(String(F("webSocketClient.begin() took ")) + String(millis() - ms, DEC));
        reconnectWebsocketAt = millis() + 8000; // 8 seconds should be enough to cennect WS
    }

    if (initializedWiFi) {
        if (webSocketClient.get() != NULL) {
            uint32_t ms = millis();
            // debugSerial->println("before loop");
            webSocketClient->loop();
            // debugSerial->println("after loop");
            if ((millis() - ms) > 50) {
                debugSerial->println(String(F("webSocketClient.loop() took ")) + String(millis() - ms, DEC));
            }
        }
    }

    if (initializedWiFi) {
        if (millis() - lastReceived > msBeforeRestart) {
            //debugSerial->println("Rebooting...");
            if (reportedGoingToReconnect <= lastReceived) {
                sink->showMessage((String(msBeforeRestart / 1000, DEC) + F(" секунд без связи с сервером, перезагружаемся")).c_str(), 3000);
                debugSerial->println(String(msBeforeRestart / 1000, DEC) + F(" seconds w/o connect to server"));
                reportedGoingToReconnect = millis();
            }

            rebootAt = millis();
        }

        if (rebootAt <= millis()) {
            sink->reboot();
        }
    }

    if (saveBrightnessAt < millis()) {
        saveBrightnessAt = 0x7FFFFFFF;
    }

    ArduinoOTA.handle();
/*
    if (millis() % 1000 == 0) {
        debugSerial->println("Heap size: " + String(ESP.getFreeHeap(), DEC) + " bytes");
    }
*/

}

} // namespace

void debugPrint(const String& str) {
    if (sceleton::webSocketClient.get() != NULL) {
        String toSend;
        toSend = String(F("{ \"type\": \"log\", \"val\": \"")) + str + F("\" }");

        sceleton::send(toSend);
    }
}
