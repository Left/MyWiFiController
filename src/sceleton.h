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
#include <WiFiUDP.h>
#include <ArduinoOTA.h>

#include <pb_common.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include "protocol.pb.h"

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

const char* typeKey = "type";
const uint32_t firmwareVersion[] = { 0, 22, 0 };

std::auto_ptr<AsyncWebServer> setupServer;
// std::auto_ptr<WebSocketsClient> webSocketClient;

const int dpin[] = { D0, D1, D2, D3, D4, D5, D6, D7, D8, D9 };

long vccVal = 0;
uint32_t rebootAt = 0x7FFFFFFF;

/*
void send(const String& toSend) {
    webSocketClient->sendTXT(toSend.c_str(), toSend.length());
}
*/

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

    const String& val() const {
        return _value;
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

DevParam deviceName("device.name", "name", "Device Name", String("ESP_") + ESP.getChipId());
//DevParam deviceName("device.name", "name", "Device Name", String("RelayOnKitchen"));
DevParam deviceNameRussian("device.name.russian", "rname", "Device Name (russian)", String("ESP_") + ESP.getChipId());
//DevParam deviceNameRussian("device.name.russian", "rname", "Device Name (russian)", String("Реле на кухне"));
DevParam wifiName("wifi.name", "wifi", "WiFi SSID", SSID_STRING);
DevParam wifiPwd("wifi.pwd", "wfpwd", "WiFi Password", PASS_STRING, true);
BoolDevParam logToHardwareSerial("debug.to.serial", "debugserial", "Print debug to serial", true);
DevParam websocketServer("websocket.server", "ws", "WebSocket server", "192.168.121.38");
DevParam websocketPort("websocket.port", "wsport", "WebSocket port", "8080");
BoolDevParam invertRelayControl("invertRelay", "invrelay", "Invert relays", false);
BoolDevParam hasScreen("hasScreen", "screen", "Has screen", false);
BoolDevParam hasScreen180Rotated("hasScreen180Rotated", "screen180", "Screen is rotated on 180", false);
BoolDevParam hasHX711("hasHX711", "hx711", "Has HX711 (weight detector)", false);
BoolDevParam hasIrReceiver("hasIrReceiver", "ir", "Has infrared receiver", false);
BoolDevParam hasDS18B20("hasDS18B20", "ds18b20", "Has DS18B20 (temp sensor)", false);
BoolDevParam hasDFPlayer("hasDFPlayer", "dfplayer", "Has DF player", false);
BoolDevParam hasBME280("hasBME280", "bme280", "Has BME280 (temp & humidity sensor)", false);
BoolDevParam hasLedStripe("hasLedStripe", "ledstrip", "Has RGBW Led stripe", false);
BoolDevParam hasBluePill("hasBluePill", "bluepill", "Has bluepill", false);
BoolDevParam hasButton("hasButton", "d7btn", "Has button on D7", false);
DevParam brightness("brightness", "bright", "Brightness [0..100]", "0");
BoolDevParam hasEncoders("hasEncoders", "enc", "Has encoders", false);
BoolDevParam hasMsp430("hasMsp430WithEncoders", "msp430", "Has MSP430 with encoders", false);
BoolDevParam hasPotenciometer("hasPotenciometer", "potent", "Has ADC connected", false);
BoolDevParam hasSolidStateRelay("hasSSR", "ssr", "Has Solid State Relay (D1, D2, D5, D6)", false);
BoolDevParam hasATXPowerSupply("hasATXPowerSupply", "atx", "Has ATX power supply", false);
DevParam relayNames("relay.names", "relays", "Relay names, separated by ;", "");
//DevParam relayNames("relay.names", "relays", "Relay names, separated by ;", "Потолок;Лента");
BoolDevParam hasGPIO1Relay("hasGPIO1Relay", "gpio1relay", "Has GPIO1 Relay", false);
BoolDevParam hasPWMOnD0("hasPWMOnD0", "pwmOnD0", "Has PWM on D0", false);

// uint32_t msBeforeRestart = atoi(secondsBeforeRestart.value());

DevParam* devParams[] = { 
    &deviceName, 
    &deviceNameRussian,
    &wifiName, 
    &wifiPwd, 
    &logToHardwareSerial,
    &websocketServer, 
    &websocketPort, 
    &invertRelayControl, 
    &hasScreen, 
    &hasScreen180Rotated,
    &hasHX711,
    &hasIrReceiver,
    &hasDS18B20,
    &hasDFPlayer,
    &hasBME280,
    &hasLedStripe,
    &hasBluePill,
    &hasEncoders,
    &hasButton, 
    &brightness,
    &hasMsp430,
    &relayNames,
    &hasGPIO1Relay,
    &hasPotenciometer,
    &hasSolidStateRelay,
    &hasATXPowerSupply,
    &hasPWMOnD0,
    // &secondsBeforeRestart
}; 

Sink* sink = new Sink();
boolean initializedWiFi = false;
uint32_t lastReceived = millis();
uint32_t reconnectWebsocketAt = 0x7FFFFFFF; // Never by def
uint32_t reportedGoingToReconnect = millis();

uint32_t lastPing = millis();

WiFiUDP udpClient;

std::vector<uint8_t> buffer;
Msg message_empty = Msg_init_zero;
Msg message = Msg_init_zero;

MsgBack messageBack_empty = MsgBack_init_zero;
MsgBack messageBack = MsgBack_init_zero;

int32_t msgId = 1;

bool write_string(pb_ostream_t* stream, const pb_field_t* field, void * const * arg) {
    const char* str = reinterpret_cast<const char*>(*arg);

    return pb_encode_tag_for_field(stream, field) &&
        pb_encode_string(stream, (uint8_t*)str, strlen(str));
}

pb_callback_t str(const String& str) {
    pb_callback_t res = {};
    res.arg = reinterpret_cast<void*>(const_cast<char*>(str.c_str()));
    res.funcs.encode = reinterpret_cast<decltype(res.funcs.encode)>(write_string);

    return res;
}

bool read_string(pb_istream_t *stream, const pb_field_t *field, void **arg) {
    size_t byteLen = (stream == nullptr) ? 0 : stream->bytes_left;
    if (byteLen > 0) {
        std::vector<uint8_t> buffer(byteLen + 2, 0);

        if (!pb_read(stream, &buffer[0], byteLen)) {
            return false;
        }

        String* strRes = reinterpret_cast<String*>(*arg);
        
        *strRes = (const char*)(&buffer[0]);
    }

    return true;
}


pb_callback_t strDecode(String& str) {
    pb_callback_t res = { 0 };
    auto strRes = (String*)(&str);
    res.arg = strRes;
    res.funcs.decode = reinterpret_cast<decltype(res.funcs.decode)>(read_string);

    return res;
}

bool write_ints(pb_ostream_t* stream, const pb_field_t* field, void * const * arg) {
    std::vector<uint32_t>* vec = (std::vector<uint32_t>*)(*arg);

    if (!pb_encode_tag(stream, PB_WT_STRING, field->tag)) {
        return false;
    }

    pb_ostream_t sizestream = PB_OSTREAM_SIZING;
    for (size_t i = 0; i < vec->size(); i++) {
        if (!pb_encode_varint(&sizestream, vec->at(i))) {
            return false;
        }
    }

    if (!pb_encode_varint(stream, sizestream.bytes_written)) {
        return false;
    }

    for (uint32_t v : *vec) {
        if (!pb_encode_varint(stream, v)) {
             return false;
        }
    }
    return true;
}

pb_callback_t repeated_int(const std::vector<uint32_t>& arg) {
    pb_callback_t res = {};
    res.arg = (void*)(&arg);
    res.funcs.decode = reinterpret_cast<decltype(res.funcs.decode)>(write_ints);
    return res;
}

void udpSend(std::function<void(Msg&)>&& updateData) {   
    message = message_empty;
    message.id = (msgId++);
    message.timeseq = millis();

    updateData(message);
/*
    // We intentionally skip the very first period, because it is a pause between keys
    for (size_t i = 1; i < ir.size(); ++i) {
        message.irKeyPeriods[i - 1] = ir[i];
    }
    message.irKeyPeriods_count = ir.size() - 1;
*/

    buffer.resize(1000, 0);
    pb_ostream_t stream = pb_ostream_from_buffer(&buffer[0], buffer.size());
    pb_encode(&stream, Msg_fields, &message);

    udpClient.beginPacket(
        sceleton::websocketServer.value(), 
        atoi(sceleton::websocketPort.value()) + 1);
    int b = stream.bytes_written;
    for (const uint8_t* p = &buffer[0];
        b > 0; ++p, --b) {
        udpClient.write(*p);
    } 
    udpClient.endPacket();
}

void sendHelloMsg() {
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

/*
    send(String(F("{ ")) +
        F("\"type\":\"hello\", ") +
        F("\"devParams\": ") + devParamsStr + F(", ") + 
        F("\"screenEnabled\": ") + sink->screenEnabled() + F(", ") + 
        F("\"deviceName\":\"") + sceleton::deviceName.value() + F("\"") + 
        F(" }"));
*/

    udpSend([=](Msg& msg) {
        msg.has_hello = true;
        msg.hello.settings = str(devParamsStr);
        msg.hello.versionLast = firmwareVersion[2];
        msg.hello.versionMinor = firmwareVersion[1];
        msg.hello.versionMajor = firmwareVersion[0];
        msg.hello.screenEnabled = sink->screenEnabled();
    }); 

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
        sceleton::udpSend([=](Msg& msg) {
            msg.relayStates_count = cnt;
            for (int id = 0; id < cnt; ++id) {
                msg.relayStates[id].id = id;
                msg.relayStates[id].state = sink->relayState(id);
            }
        });
    }
}

void reportRelayState(uint32_t id) {
    sceleton::udpSend([=](Msg& msg) {
        msg.relayStates_count = 1;
        msg.relayStates[0].id = id;
        msg.relayStates[0].state = sink->relayState(id);
    });
    // send(String(F("{ \"type\": \"relayState\", \"id\": ")) + String(id, DEC) + F(", \"value\":") + (sink->relayState(id) ? F("true") : F("false")) + F(" }"));
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

    // msBeforeRestart = atoi(secondsBeforeRestart.value());

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
            auto n = false;
            if (p != nullptr) {
                n = d->parseHTML(p->value());
            } else {
                n = d->parseHTML(String(F("")));
            }
            needReboot = needReboot || n;
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

            udpClient.begin(atoi(sceleton::websocketPort.value()) + 1);

            reconnectWebsocketAt = millis() + 3000; // Wait 1000 ms and connect to websocket

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

            ArduinoOTA.begin(false); // Begin OTA immediately
            initializedWiFi = true;

            // Let's say hello and show all we can
            sendHelloMsg();

            debugSerial->println(String(F("Hello sent")));
        }
    }


    if (initializedWiFi) {
        if (rebootAt <= millis()) {
            sink->reboot();
        }
    }

    if (saveBrightnessAt < millis()) {
        saveBrightnessAt = 0x7FFFFFFF;
    }

    if (initializedWiFi && (millis() - lastPing) > 2000) {
        lastPing = millis();

        udpSend([=](Msg& msg) {}); // Simple UDP packet send, no data!
        // debugPrint("Sent ping");
    }

    auto bytesInUdbBuf = udpClient.parsePacket();
    if (bytesInUdbBuf > 0) {
        // debugSerial->println("RECEIVED UDP " + String(bytesInUdbBuf, DEC));
        buffer.resize(bytesInUdbBuf + 1, 0);
        auto wasRead = udpClient.readBytes(&buffer[0], bytesInUdbBuf);

        // debugSerial->println("Decoding " + String(wasRead, DEC));
        pb_istream_t stream = pb_istream_from_buffer(&buffer[0], wasRead);
        messageBack = messageBack_empty;

        String textToShow;
        messageBack.textToShow = strDecode(textToShow);
        String ledBaseColor;
        messageBack.ledBaseColor = strDecode(ledBaseColor);
        String ledBlinkColors;
        messageBack.ledBlinkColors = strDecode(ledBlinkColors);
        String ledValue;
        messageBack.ledValue = strDecode(ledValue);

        bool status = pb_decode(&stream, MsgBack_fields, &messageBack);
        if (!status) {
            debugSerial->println(F("Decoding failed"));
        } else {
            if (messageBack.has_introduceYourself && messageBack.introduceYourself) {
                sendHelloMsg();
            }
            if (messageBack.has_unixtime) {
                sink->setTime(messageBack.unixtime);
            }
            if (textToShow.length() > 0) {
                if (messageBack.showType == MsgBack_ShowType_SHOW) {
                    sink->showMessage(textToShow.c_str(), messageBack.timeMsToShow);
                } else if (messageBack.showType == MsgBack_ShowType_TUNE) {
                    sink->showTuningMsg(textToShow.c_str());
                } else if (messageBack.showType == MsgBack_ShowType_ADDITIONAL) {
                    sink->setAdditionalInfo(textToShow.c_str());
                }
            }
            if (messageBack.has_reboot && messageBack.reboot) {
                sink->reboot();
            }
            if (messageBack.has_relaysToSwitch && messageBack.has_relaysToSwitchState) {
                sink->switchRelay(messageBack.relaysToSwitch, messageBack.relaysToSwitchState);
                reportRelayState(messageBack.relaysToSwitch);
            }
            if (messageBack.has_atxEnable) {
                sink->switchATX(messageBack.atxEnable);
            }
            if (messageBack.has_playMp3) {
                sink->playMp3(messageBack.playMp3);
            }
            if (messageBack.has_volume) {
                sink->setVolume(messageBack.volume);
            }
            if (messageBack.has_screenEnable) {
                sink->enableScreen(messageBack.screenEnable);
            }
            if (messageBack.has_brightness) {
                uint32_t val = std::max(std::min(messageBack.brightness, 100u), 0u);
                sink->setBrightness(val);
                brightness.setValue(String(val, DEC));
            }
            if (messageBack.has_pwmPeriod && messageBack.has_pwmPin && messageBack.has_pwmValue) {
                uint8_t pinIndex = dpin[messageBack.pwmPin];

                sink->setPWMOnPin(messageBack.pwmValue, pinIndex, messageBack.pwmPeriod);
            }
            if (messageBack.has_ledPeriod) {
                if (ledValue.length() > 0) {
                    sink->setLedStripe(decodeRGBWString(ledValue.c_str()), messageBack.ledPeriod);
                } else {
                    // new year
                    sink->runLedStripeEffect(decodeRGBWString(ledBaseColor.c_str())[0], decodeRGBWString(ledBlinkColors.c_str()), messageBack.ledPeriod);
                }
            }
        }
    }

    ArduinoOTA.handle();
/*
    if (millis() % 1000 == 0) {
        debugSerial->println("Heap size: " + String(ESP.getFreeHeap(), DEC) + " bytes");
    }
*/

}

} // namespace

void debugPrint(const String& stringg) {
    sceleton::udpSend([=](Msg& msg) {
        msg.debugLogMessage = sceleton::str(stringg);
    });
}
