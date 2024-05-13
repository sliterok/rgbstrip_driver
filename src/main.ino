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
#define QUEUE_SIZE 35
#define TARGET_QUEUE_SIZE 30
#define FRAME_INTERVAL 16

const int normalPacketSize = NUM_LEDS * 3;

WiFiClient client;
WiFiUDP udp;
uint8_t incomingPacket[normalPacketSize];
uint8_t queueOut[normalPacketSize];
RingBufCPP<uint8_t[normalPacketSize], QUEUE_SIZE> queue;
IPAddress destinationIp;

void setup()
{
    WiFi.setSleepMode(WIFI_NONE_SLEEP);
    WiFi.mode(WIFI_STA);
    pinMode(LED_BUILTIN, OUTPUT);

    WiFi.begin(STR(WIFI_SSID), STR(WIFI_PASS));

    Serial.begin(115200);
    Serial.println("Connecting");
    Serial.setDebugOutput(true);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
    }
    Serial.begin(115200);

    Serial.print("Connected, IP address: ");
    Serial.println(WiFi.localIP());
    WiFi.setAutoReconnect(true);
    WiFi.persistent(true);

    updateDestination();

    udp.begin(UDP_PORT);
    FastLED.addLeds<NEOPIXEL, LED_PIN>((CRGB *)queueOut, NUM_LEDS);
    Serial.setDebugOutput(true);
}

long lastPingTime = 0;
long lastReceivedPacket = 0;
long lastFrame = 0;

void loop()
{
    long timeNow = millis();
    if (lastFrame == 0)
    {
        lastFrame = timeNow - FRAME_INTERVAL;
    }

    int diff = int(timeNow - lastFrame);
    int frameDelay = TARGET_QUEUE_SIZE - static_cast<int>(queue.numElements());
    bool shouldPull = diff > FRAME_INTERVAL + frameDelay;

    if (shouldPull)
    {
        bool popped = queue.pull(&queueOut);
        if (popped)
        {
            FastLED.show();
            while (Serial.available() > 0)
            {
                Serial.read();
            }
            lastFrame = timeNow;
        }
    }
    if (int(timeNow - lastPingTime) > 2000)
    {
        udp.beginPacket(destinationIp, 8008);
        udp.endPacket();
        udp.flush();
        lastPingTime = timeNow;
    }

    int packetSize = udp.parsePacket();
    if (packetSize == 0)
    {
        checkReconnect(timeNow);
    }
    else
    {
        int len = udp.read(incomingPacket, normalPacketSize);
        if (len == normalPacketSize)
        {
            lastReceivedPacket = timeNow;
            queue.add(&incomingPacket, true);
        }
    }
}

void checkReconnect(int timeNow)
{
    if (int(lastReceivedPacket) > 0 and int(timeNow - lastReceivedPacket) > 3000)
    {
        WiFi.reconnect();
        Serial.print("Reconnecting...");

        while (WiFi.status() != WL_CONNECTED)
        {
            delay(500);
        }
        Serial.print("Reconnected");
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