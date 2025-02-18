#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include "FastLED.h"
#include <RingBufCPP.h>
#include <Esp.h>

#define FASTLED_INTERRUPT_RETRY_COUNT 3
#define XSTR(x) #x
#define STR(x) XSTR(x)

#define NUM_LEDS 288
#define LED_PIN 0
#define UDP_PORT 8008
#define DESTINATION_PORT 8009
#define PING_INTERVAL 2000
#define TIMEOUT 3000
#define QUEUE_SIZE 52
#define TARGET_QUEUE_SIZE 42
#define FRAME_INTERVAL 12

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

    // updateDestination();

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
    int queueSize = queue.numElements();
    int queueSizeDiff = TARGET_QUEUE_SIZE - queueSize;
    int frameDelay = abs(queueSizeDiff) > 2 ? (queueSizeDiff > 0) - (queueSizeDiff < 0) : 0;
    int timeDiff = timeNow - nextFrame - frameDelay;
    bool shouldPull = timeDiff >= 0;

    if (shouldPull)
    {
        showFrame();
    }

    if (int(timeNow - lastPingTime) > PING_INTERVAL)
    {
        ping();
    }

    int parsedPackets = 0;
    while (parsedPackets++ < 10 && udp.parsePacket())
    {
        readPacket();
    }

    checkReconnect();
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
    udp.beginPacket(STR(DESTINATION), DESTINATION_PORT);
    udp.endPacket();
    udp.flush();
    lastPingTime = timeNow;
}

void readPacket()
{
    int len = udp.read(incomingPacket, frameSize);
    if (len == 0)
        return;

    if (len == 3)
    {
        for (int i = 3; i < frameSize; i++)
        {
            incomingPacket[i] = incomingPacket[i % 3];
        }
    }
    queue.add(&incomingPacket, true);
    lastReceivedPacket = timeNow;
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

// void updateDestination()
// {
//     int err = WiFi.hostByName(STR(DESTINATION), destinationIp);
//     if (err == 1)
//     {
//         Serial.print("Destination IP address: ");
//         Serial.println(destinationIp);
//     }
//     else
//     {
//         Serial.print("Error getting destination: ");
//         Serial.println(err);
//     }
// }