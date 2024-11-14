#ifndef Html_config_h
#define Html_config_h

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>  // 设置AP热点模式时需要用到
#include <HTTPClient.h>  // 使用esp32发送http请求
#include <ArduinoJson.h> // 使用ArduinoJson库解析JSON数据
#include <ESPAsyncWebServer.h>

#define ssid "Dog2"
#define password "12345678"

void Wifi_init(AsyncWebServer server);
#endif