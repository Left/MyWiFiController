#pragma once

#include "common.h"

#include <Arduino.h>
#include <FS.h>
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <WebSockets.h>
#include <WebSocketsClient.h>
#include <ESPAsyncWebServer.h>
// #include <ArduinoOTA.h>

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
    virtual uint32_t getLedStripePixel(size_t i) { return 0; }
    virtual uint32_t getLedStripeLen() { return 0; }
    virtual void setPWMOnPin(uint32_t val, uint8_t pin) {}
    virtual void playMp3(uint32_t index) {}
    virtual void setVolume(uint32_t vol) {}
    virtual void reboot() {}
    virtual void enableScreen(const boolean enabled) {}
    virtual boolean screenEnabled() { return false; }
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

const String typeKey("type");

const char* firmwareVersion = "00.22";

std::auto_ptr<AsyncWebServer> setupServer;
std::auto_ptr<WebSocketsClient> webSocketClient;

long vccVal = 0;
uint32_t rebootAt = 0x7FFFFFFF;

void send(const String& toSend) {
    webSocketClient->sendTXT(toSend.c_str(), toSend.length());
}

class DevParam {
public:
    const char* _name;
    const char* _jsonName;
    const char* _description;
    String _value;
    boolean _password;

    DevParam(const char* name, const char* jsonName, const char* description, String value, boolean pwd=false) :
        _name(name),
        _jsonName(jsonName),
        _description(description),
        _value(value),
        _password(pwd) {
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
DevParam logToHardwareSerial("debug.to.serial", "debugserial", "Print debug to serial", "true");
DevParam websocketServer("websocket.server", "ws", "WebSocket server", "192.168.121.38");
DevParam websocketPort("websocket.port", "wsport", "WebSocket port", "8080");
DevParam invertRelayControl("invertRelay", "invrelay", "Invert relays", "false");
#ifndef ESP01
DevParam hasScreen("hasScreen", "screen", "Has screen", "false");
DevParam hasScreen180Rotated("hasScreen180Rotated", "screen180", "Screen is rotated on 180", "false");
DevParam hasHX711("hasHX711", "hx711", "Has HX711 (weight detector)", "false");
DevParam hasIrReceiver("hasIrReceiver", "ir", "Has infrared receiver", "false");
DevParam hasDS18B20("hasDS18B20", "ds18b20", "Has DS18B20 (temp sensor)", "false");
DevParam hasDFPlayer("hasDFPlayer", "dfplayer", "Has DF player", "false");
#endif
DevParam hasBME280("hasBME280", "bme280", "Has BME280 (temp & humidity sensor)", "false");
DevParam hasLedStripe("hasLedStripe", "ledstrip", "Has RGBW Led stripe", "false");
#ifndef ESP01
DevParam hasButton("hasButton", "d7btn", "Has button on D7", "false");
DevParam brightness("brightness", "bright", "Brightness [0..100]", "0");
DevParam hasEncoders("hasEncoders", "enc", "Has encoders", "false");
DevParam hasMsp430("hasMsp430WithEncoders", "msp430", "Has MSP430 with encoders", "false");
DevParam hasPotenciometer("hasPotenciometer", "potent", "Has potenciometer", "false");
DevParam hasSolidStateRelay("hasSSR", "ssr", "Has Solid State Relay (D1, D2, D5, D6)", "false");
#endif
DevParam relayNames("relay.names", "relays", "Relay names, separated by ;", "");
//DevParam relayNames("relay.names", "relays", "Relay names, separated by ;", "Потолок;Лента");
DevParam hasGPIO1Relay("hasGPIO1Relay", "gpio1relay", "Has GPIO1 Relay", "false");
DevParam hasPWMOnD0("hasPWMOnD0", "pwmOnD0", "Has PWM on D0", "false");
DevParam secondsBeforeRestart("secondsBeforeRestart", "watchdog", "Seconds before restart", "60000");

uint32_t msBeforeRestart = atoi(secondsBeforeRestart._value.c_str());

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
    send("{ \"type\": \"relayState\", \"id\": " + String(id, DEC) + ", \"value\":" + (sink->relayState(id) ? "true" : "false") + " }");
}

void onDisconnect(const WiFiEventStationModeDisconnected& event) {
    // debugSerial->println("WiFi On Disconnect.");
    // debugSerial->println(event.reason);
}

String encodeRGBWString(Sink* sink) {
    String res = "";
    for (uint32_t k = 0; k < sink->getLedStripeLen(); ++k) {
        uint32_t toAdd = sink->getLedStripePixel(k);
        for (int i = 0; i<8; ++i) {
            char xx = (char)((toAdd >> (28 - i*4)) & 0xf);
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
    debugSerial->println("Saving settings...");

    DynamicJsonDocument jsonBuffer(1000);
    JsonObject root = jsonBuffer.to<JsonObject>();

    for (DevParam* d : devParams) {
        root[d->_jsonName] = d->_value;
    }

    size_t sz = measureJsonPretty(root);
    char* buf = (char*)malloc(sz + 10);
    memset(buf, 0, sz+1);

    serializeJson(root, buf, sz);

    debugSerial->println("Settings:\n" + String(buf) + "\n\n");

    File f = SPIFFS.open("settings.json", "w");
    f.write((uint8_t*)buf, sz);
    f.flush();
    f.close();
    
    free(buf);

    debugSerial->println("Saved settings in " + String((millis() - t), DEC));
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
    debugSerial->println("\n\n=====================");

    long was = millis();

    {   // Read initial settings
        String readVal = fileToString("settings.json");
        StaticJsonDocument<2000> jsonBuffer;
        // debugSerial->println(readVal);
        DeserializationError error = deserializeJson(jsonBuffer, readVal.c_str(), readVal.length());
        if (error == DeserializationError::Ok) {
            const JsonObject &root = jsonBuffer.as<JsonObject>();
            for (DevParam* d : devParams) {
                d->_value = (const char*) (root[d->_jsonName]);
            }
        } else {
            debugSerial->println("No settings read, use defaults");    
        }
    }

    if (logToHardwareSerial._value == "false") {
        debugSerial = new DummySerial();
    }

    msBeforeRestart = atoi(secondsBeforeRestart._value.c_str());

    debugSerial->println("Initialized in " + String(millis() - was, DEC));
    debugSerial->println(wifiName._value.c_str());
    debugSerial->println(wifiPwd._value.c_str());

    WiFi.persistent(false);
    WiFi.setAutoConnect(false);
    WiFi.setAutoReconnect(false);
    WiFi.setPhyMode(WIFI_PHY_MODE_11G);
    WiFi.setSleepMode(WIFI_NONE_SLEEP);

    if (wifiName._value.length() > 0 && wifiPwd._value.length() > 0) {
        WiFi.onStationModeConnected([=](const WiFiEventStationModeConnected& e) {
            debugSerial->println("onStationModeConnected");
            debugSerial->println(e.ssid);
            // debugSerial->println(e.bssid);
            debugSerial->println(e.channel);
        });
        WiFi.onStationModeDisconnected([=](const WiFiEventStationModeDisconnected& e) {
            debugSerial->println("onStationModeDisconnected");
        });
        WiFi.onStationModeAuthModeChanged([=](const WiFiEventStationModeAuthModeChanged& e) {
            debugSerial->println("onStationModeAuthModeChanged");
        });
        WiFi.onStationModeGotIP([=](const WiFiEventStationModeGotIP& e) {
            debugSerial->println("onStationModeGotIP");
        });
        WiFi.onStationModeDHCPTimeout([=](void) {
            debugSerial->println("onStationModeDHCPTimeout");
        });
        WiFi.onSoftAPModeStationConnected([=](const WiFiEventSoftAPModeStationConnected& e) {
            debugSerial->println("onSoftAPModeStationConnected");
        });
        WiFi.onSoftAPModeStationDisconnected([=](const WiFiEventSoftAPModeStationDisconnected& e) {
            debugSerial->println("onSoftAPModeStationDisconnected");
        });
        WiFi.onSoftAPModeProbeRequestReceived([=](const WiFiEventSoftAPModeProbeRequestReceived& e) {
            debugSerial->println("onSoftAPModeProbeRequestReceived");
        });

        WiFi.mode(WIFI_STA);
        WiFi.hostname("ESP_" + deviceName._value);
        WiFi.onStationModeDisconnected(onDisconnect);
        WiFi.begin(wifiName._value.c_str(), wifiPwd._value.c_str());
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
            if (request->hasParam(d->_name)) {
                // Param is set
                String val = request->getParam(d->_name)->value();
                if (!d->_password || val.length() > 0) {
                    if (d->_value != val) {
                        d->_value = val;
                        needReboot = true;
                    }
                }
            }
        }
        
        if (needReboot) {
            saveSettings();
            request->send(200, "text/html", "Settings changed, rebooting in 2 seconds...");  
            rebootAt = millis() + 2000;
        } else {
            request->send(200, "text/html", "Nothing changed.");  
        }
    });
    setupServer->on("/", [](AsyncWebServerRequest *request) {
        String content = "<!DOCTYPE HTML>\r\n<html>";
        content += "<head>";
        content += "<meta http-equiv=\"content-type\" content=\"text/html; charset=UTF-8\">";
        content += "</head>";
        content += "<body>";
        content += "<p>";
        content += "<form method='get' action='http_settup'>";
        for (DevParam* d : devParams) {
            content += "<label class='lbl'>";
                content += d->_description;
                content += ":</label>";
            content +="<input name='";
                content += d->_name;
                content += "' value='";
                content += d->_password ? String("") : d->_value;
                content += "' length=32/><br/>";
        }
        content += "<input type='submit'></form>";
        content += "<form action='/reboot'><input type='submit' value='Reboot'/></form>";
        content += "</html>";
        request->send(200, "text/html", content);  
    });
    setupServer->on("/reboot", [](AsyncWebServerRequest *request) {
        rebootAt = millis() + 100;
    });
    setupServer->onNotFound([](AsyncWebServerRequest *request) {
        request->send(404, "text/plain", "Not found: " + request->url());
    });
    setupServer->begin();
/*
    ArduinoOTA.setPort(8266);
    // set host name
    ArduinoOTA.setHostname(deviceName._value.c_str());

    ArduinoOTA.onStart([]() {
        // debugSerial->println("Start OTA");  //  "Начало OTA-апдейта"
        sink->showMessage("Updating...", 10000);
    });
    ArduinoOTA.onEnd([]() {
        // debugSerial->println("End OTA");  //  "Завершение OTA-апдейта"
        sink->showMessage("Done...", 10000);
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        // debugSerial->printf("Progress: %u%%\r", (progress / (total / 100)));
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
*/
}

int32_t lastEachSecond = millis() / 1000;
int32_t lastWiFiState = millis();
int32_t lastLoop = millis();

uint32_t oldStatus = WiFi.status();
uint32_t nextReconnect = millis();
uint32_t nextWiFiScan = millis();

void loop() {
    if (millis() - lastLoop > 50) {
         debugSerial->println(String("Long loop: ") + String(millis() - lastLoop, DEC));
    }
    lastLoop = millis();

    if (WiFi.status() != WL_CONNECTED && (millis() > nextReconnect)) {
        debugSerial->println(String("WiFi.status() check: ") + WiFi.status());
        bool ret = WiFi.reconnect();
        debugSerial->println(String("Reconnect returned ") + String(ret, DEC));
        nextReconnect = millis() + (ret ? 4000 : 300);
    }

    if (millis() > nextWiFiScan) {
        debugSerial->println("Start scanning");
        WiFi.scanNetworks(true);

        nextWiFiScan = millis() + 10000; // Scan every 10 seconds
    }

    int n = WiFi.scanComplete();
    if(n >= 0) {
        debugSerial->println("------------");
        debugSerial->printf("%d network(s) found\n", n);
        for (int i = 0; i < n; i++) {
            debugSerial->printf("%d: %s, Ch:%d (%ddBm) %s\n", i+1, WiFi.SSID(i).c_str(), WiFi.channel(i), WiFi.RSSI(i), WiFi.encryptionType(i) == ENC_TYPE_NONE ? "open" : "");
        }
        WiFi.scanDelete();
        debugSerial->println("------------");
    }

    if (oldStatus != WiFi.status()) {
        oldStatus = WiFi.status();
        debugSerial->println(String("WiFi.status(): ") + oldStatus);

        if (WiFi.status() == WL_IDLE_STATUS || WiFi.status() == WL_DISCONNECTED) {
            debugSerial->println("Reconnecting");
            reconnectWebsocketAt = 0x7FFFFFFF;
            WiFi.reconnect();
        }

        if (WiFi.status() == WL_CONNECTED) {
            debugSerial->println(String("Connected to WiFi, IP:") + WiFi.localIP().toString());
            reconnectWebsocketAt = millis(); // Wait 1000 ms and connect to websocket
            // ArduinoOTA.begin(); // Begin OTA immediately
            initializedWiFi = true;
        }
    }

    if (millis() >= reconnectWebsocketAt) {
        webSocketClient.reset(new WebSocketsClient());
        auto wsHandler = [&](WStype_t type, uint8_t *payload, size_t length) {
            // debugSerial->println(String("WS Event") + type);
            switch (type) {
                case WStype_ERROR: {
                    debugSerial->println("WStype_ERROR");
                    break;
                }
                case WStype_CONNECTED: {
                    if (wasConnected) { 
                        break;
                    }
                    reconnectWebsocketAt = 0x7FFFFFFF;  // No need to reconnect anymore
                    debugSerial->println("Connected to server");
                    lastReceived = millis();
                    wasConnected = true;

                    String devParamsStr = "{ ";
                    bool first = true;
                    for (DevParam* d : devParams) {
                        if (!d->_password && d->_value != "false") {
                            if (!first) { 
                                devParamsStr += ",";
                            }
                            first = false;
                            devParamsStr += "\"",
                            devParamsStr += d->_name;
                            devParamsStr += "\": \"",
                            devParamsStr += d->_value;
                            devParamsStr += "\" ";
                        }
                    }
                    devParamsStr += "}";

                    // Let's say hello and show all we can
                    send(String("{ ") +
                        "\"type\":\"hello\", " +
                        "\"firmware\":\"" + firmwareVersion + "\", " +
                        "\"devParams\": " + devParamsStr + ", " + 
                        "\"screenEnabled\": " + sink->screenEnabled() + ", " + 
                        "\"deviceName\":\"" + sceleton::deviceName._value + "\"" + 
                        " }");

                    // send message to client
                    // debugPrint("Hello server " + " (" + sceleton::deviceName._value +  "), firmware ver = " + firmwareVersion);
                    debugSerial->println("Hello sent");

                    int cnt = 0;
                    for (const char* p = sceleton::relayNames._value.c_str(); *p != 0; ++p) {
                        if (*p == ';') {
                            if (cnt == 0) {
                                cnt = 1;
                            }

                            cnt++;
                        }
                    }

                    if (cnt > 0) {
                        for (int id = 0; id < cnt; ++id) {
                            send("{ \"type\": \"relayState\", \"id\": " + String(id, DEC) + ", \"value\":" + (sink->relayState(id) ? "true" : "false") + " }");
                        }
                        debugSerial->println("Relays state sent");
                    }

                    if (sceleton::hasLedStripe._value == "true") {
                        send("{ \"type\": \"ledstripeState\", \"value\":\"" + encodeRGBWString(sink) + "\" }");
                        debugSerial->println("LED stripe state sent");
                    }

                    break;
                }
                case WStype_TEXT: {
                    // debugSerial->printf("[%u] get Text: %s\n", payload);
                    DynamicJsonDocument jsonBuffer(1000);

                    DeserializationError error = deserializeJson(jsonBuffer, payload);

                    if (error) {
                        // debugSerial->println("parseObject() failed");
                        send("{ \"errorMsg\":\"Failed to parse JSON\" }");
                        return;
                    }
                    lastReceived = millis();

                    const JsonObject &root = jsonBuffer.as<JsonObject>();

                    String type = root[typeKey];
                    if (type == "ping") {
                        debugSerial->print(String(millis(), DEC) + ":");debugSerial->print("Ping "); debugSerial->println((const char*)(root["pingid"]));
                        String res = "{ \"type\": \"pingresult\", \"pid\":\"";
                        res += (const char*)(root["pingid"]);
                        res += "\" }";
                        send(res);
                    } else if (type == "switch") {
                        // debugSerial->println("switch!");
                        bool sw = root["on"] == "true";
                        uint32_t id = atoi(root["id"]);
                        sink->switchRelay(id, sw);
                        reportRelayState(id);
                    } else if (type == "setProp") {
                        saveSettings();
                    #ifndef ESP01
                    } else if (type == "show") {
                        sink->showMessage(root["text"], root["totalMsToShow"].as<int>());
                    } else if (type == "tune") {
                        sink->showTuningMsg(root["text"]);
                    } else if (type == "unixtime") {
                        sink->setTime(root["value"].as<int>());
                    #endif
                    } else if (type == "ledstripe") {
                        const char* val = (const char*)(root["value"]);
                        int periodMs = root["period"].as<int>();
                        sink->setLedStripe(decodeRGBWString(val), periodMs);
                    } else if (type == "playmp3") {
                        uint32_t index = (uint32_t)(root["index"].as<int>());
                        sink->playMp3(index);
                    } else if (type == "setvolume") {
                        uint32_t index = (uint32_t)(root["value"].as<int>());
                        sink->setVolume(index);
                    } else if (type == "pwm") {
                        int val = root["value"].as<int>();
                        const char* pin = root["pin"];
                        if (strcmp(pin, "D3") == 0) {
                            sink->setPWMOnPin(val, D3);
                        } else if (strcmp(pin, "D4") == 0) {
                            sink->setPWMOnPin(val, D4);
                        }
                    #ifndef ESP01
                    } else if (type == "screenEnable") {
                        int val = root["value"].as<boolean>();
                        sink->enableScreen(val);
                        saveBrightnessAt = millis() + 1000; // In 1 second, save brightness
                    } else if (type == "brightness") {
                        int val = root["value"].as<int>();
                        val = std::max(std::min(val, 100), 0);
                        sink->setBrightness(val);
                        brightness._value = String(val, DEC);
                        saveBrightnessAt = millis() + 1000; // In 1 second, save brightness
                    #endif
                    } else if (type == "additional-info") {
                        // 
                        sink->setAdditionalInfo(root["text"]);
                    } else if (type == "reboot") {
                        debugPrint("Let's reboot self");
                        sink->reboot();
                    }
                    break;
                }
                case WStype_BIN: {
                    // debugSerial->printf("[%u] get binary length: %u\n", length);
                    // hexdump(payload, length);
                    debugPrint("Received binary packet of size " + String(length, DEC));

                    // send message to client
                    // webSocketClient.sendBIN(payload, length);
                    break;
                }
                case WStype_DISCONNECTED: {
                    // debugSerial->print(String(millis(), DEC) + ":"); debugSerial->printf("Disconnected [%u]!\n", WiFi.status());
                    if (WiFi.status() == WL_CONNECTED && wasConnected) {
                        wasConnected = false;
                        debugSerial->println("Disconnected from server " + String(length, DEC));
                        reconnectWebsocketAt = millis() + 4000; // In 4 second, let's try to reconnect
                    }
                    break;
                }
                case WStype_PING:
                case WStype_PONG:
                    break;

                default:
                    debugSerial->println("Unknown type: " + String(type, DEC));
            }
        };

        webSocketClient->onEvent(wsHandler);

        debugSerial->println(String("webSocketClient connecting to ") + websocketServer._value.c_str());
        uint32_t ms = millis();
        webSocketClient->disableHeartbeat();
        webSocketClient->begin(websocketServer._value.c_str(), websocketPort._value.toInt(), "/esp");
        debugSerial->println(String("webSocketClient.begin() took " + String(millis() - ms, DEC)));
        reconnectWebsocketAt = millis() + 8000; // 8 seconds should be enough to cennect WS
    }

    if (initializedWiFi) {
        if (webSocketClient.get() != NULL) {
            uint32_t ms = millis();
            // debugSerial->println("before loop");
            webSocketClient->loop();
            // debugSerial->println("after loop");
            if ((millis() - ms) > 50) {
                debugSerial->println(String("webSocketClient.loop() took " + String(millis() - ms, DEC)));
            }
        }
    }

    if (initializedWiFi) {
        if (millis() - lastReceived > msBeforeRestart) {
            //debugSerial->println("Rebooting...");
            if (reportedGoingToReconnect <= lastReceived) {
                sink->showMessage((String(msBeforeRestart / 1000, DEC) + " секунд без связи с сервером, перезагружаемся").c_str(), 3000);
                debugSerial->println(String(msBeforeRestart / 1000, DEC) + " seconds w/o connect to server");
                reportedGoingToReconnect = millis();
            }

            rebootAt = millis();
        }

        if (rebootAt <= millis()) {
            sink->reboot();
        }
    }

    if (saveBrightnessAt < millis()) {
        saveSettings();
        saveBrightnessAt = 0x7FFFFFFF;
    }
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
        toSend = "{ \"type\": \"log\", \"val\": \"" + str + "\" }";

        sceleton::send(toSend);
    }
}
