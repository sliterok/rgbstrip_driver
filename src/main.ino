#include <ESP8266TimerInterrupt.h>
#include <ESP8266_ISR_Timer.h>
#include <ESP8266_ISR_Timer.hpp>
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include "FastLED.h"
#include <RingBufCPP.h>
#include <Esp.h>
extern "C"
{
#include "user_interface.h"
}

#define USING_TIM_DIV1 true
#define FASTLED_INTERRUPT_RETRY_COUNT 1
#define XSTR(x) #x
#define STR(x) XSTR(x)

#define NUM_LEDS 288
#define LED_PIN 0
#define UDP_PORT 4210
#define DESTINATION_PORT 8008
#define PING_INTERVAL 2000
#define TIMEOUT 3000
#define QUEUE_SIZE 50
#define TARGET_QUEUE_SIZE 30
#define FRAME_INTERVAL 10

const int frameSize = NUM_LEDS * 3;

WiFiClient client;
WiFiUDP udp;
uint8_t incomingPacket[frameSize];
uint8_t queueOut[frameSize];
RingBufCPP<uint8_t[frameSize], QUEUE_SIZE> queue;
IPAddress destinationIp;

void setup()
{
    Serial.begin(115200);
    WiFi.setSleepMode(WIFI_NONE_SLEEP);
    WiFi.mode(WIFI_STA);
    WiFi.begin(STR(WIFI_SSID), STR(WIFI_PASS));

    Serial.println("Connecting");
    waitForWifi();

    WiFi.setAutoReconnect(true);
    WiFi.persistent(true);

    updateDestination();

    udp.begin(UDP_PORT);
    FastLED.addLeds<NEOPIXEL, LED_PIN>((CRGB *)queueOut, NUM_LEDS);
}

long lastPingTime = 0;
long lastReceivedPacket = 0;
long nextFrame = 0;
long timeNow = millis();

void loop()
{
    timeNow = millis();
    int queueSizeDiff = TARGET_QUEUE_SIZE - static_cast<int>(queue.numElements());
    int frameDelay = (queueSizeDiff > 0) - (queueSizeDiff < 0);
    bool shouldPull = timeNow > nextFrame + frameDelay;

    if (shouldPull)
    {
        showFrame();
    }

    if (int(timeNow - lastPingTime) > PING_INTERVAL)
    {
        ping();
    }

    int packetSize = udp.parsePacket();
    if (packetSize > 0)
    {
        readPacket();
    }
    else
    {
        checkReconnect();
    }
}

void showFrame()
{
    bool popped = queue.pull(&queueOut);
    if (popped)
    {
        FastLED.show();
        while (Serial.available() > 0)
        {
            Serial.read();
        }
        nextFrame = timeNow + FRAME_INTERVAL;
    }
}

void ping()
{
    udp.beginPacket(destinationIp, DESTINATION_PORT);
    udp.endPacket();
    udp.flush();
    lastPingTime = timeNow;
}

void readPacket()
{
    int len = udp.read(incomingPacket, frameSize);
    if (len == frameSize)
    {
        queue.add(&incomingPacket, true);
        lastReceivedPacket = timeNow;
    }
}

void waitForWifi()
{
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
    }

    Serial.print("Connected, IP address: ");
    Serial.println(WiFi.localIP());
}

void checkReconnect()
{
    if (int(lastReceivedPacket) > 0 and int(timeNow - lastReceivedPacket) > TIMEOUT)
    {
        WiFi.reconnect();
        Serial.print("Reconnecting...");

        waitForWifi();
        lastReceivedPacket = millis();
    }
}

void updateDestination()
{
    int err = WiFi.hostByName(STR(DESTINATION), destinationIp);
    if (err == 1)
    {
        Serial.print("Destination IP address: ");
        Serial.println(destinationIp);
    }
    else
    {
        Serial.print("Error getting destination: ");
        Serial.println(err);
    }
}