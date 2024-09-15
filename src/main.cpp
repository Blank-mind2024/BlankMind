#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "DogC.h"
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>  // 设置AP热点模式时需要用到
#include <HTTPClient.h>  // 使用esp32发送http请求
#include <ArduinoJson.h> // 使用ArduinoJson库解析JSON数据
#include "Html_config.h"
#include <ESPAsyncWebServer.h>

#define SDA 4
#define SCL 2
#define LED_board 2

// 舵机编号
/*
RH : 11--10--
LH : 8--9--
RF : 1--0--
LF : 4--2--
                         8--9--|-------------------------|--10--11
                               |-------------------------|
                               |-------------------------|
                               |-------------------------|
                               |-------------------------|
                               |-------------------------|
                         4--2--|-------------------------|--0--1
*/

int servos[] = {10, 11, 9, 8, 0, 1, 2, 4};
float t = 0;
float dt = 0.05;
double x_pos[4];
double y_pos[4];
double ys = 0;
double yf = 40;
double h = 30;
double r1 = 1;
double r2 = 1;
double r3 = 1;
double r4 = 1;

// AsyncWebServer server(80);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

void setup()
{
    pinMode(LED_board, OUTPUT);
    for (int j = 0; j < 3; j++)
    {
        digitalWrite(LED_board, HIGH);
        delay(500);
        digitalWrite(LED_board, LOW);
        delay(500);
    }
    // Wifi_init(server);
    Wire.begin(SDA, SCL, 100000); // 100KHz I2C frequency
    pwm.begin();
    pwm.setPWMFreq(SERVO_FREQ); // Analog servos run at ~50 Hz updates
    delay(10);
    angle_init(pwm, servos, 8);
}

void loop()
{

    if (t >= Ts)
    {
        t = 0;
    }
    else
    {
        t += dt;
    }
    // trot_plan(t, dt, 30, 30, 1, 1, 1, 1, y_pos, x_pos);
    trot_plan(t, ys, yf, h, r1, r2, r3, r4, y_pos, x_pos);
    dog_move_legs(pwm, servos, x_pos, y_pos);
    // float alpha = -50;
    // float beta = 0;
    // hip_move_angle(pwm, servos, alpha);

    // knee_move_angle(pwm, servos, beta);
    // delay(20);
}
