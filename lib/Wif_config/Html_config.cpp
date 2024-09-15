#include "Html_config.h"
void Wifi_init(AsyncWebServer server)
{
    digitalWrite(2, HIGH);
    delay(3000);
    WiFi.softAP(ssid, password);
    server.on("/hi", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(200, "text/plain", "Hello, world!"); });
    server.begin();
    digitalWrite(2, LOW);
    delay(200);
    digitalWrite(2, HIGH);
    delay(200);
    digitalWrite(2, LOW);
    delay(200);
    digitalWrite(2, HIGH);
    delay(200);
    digitalWrite(2, LOW);
}
