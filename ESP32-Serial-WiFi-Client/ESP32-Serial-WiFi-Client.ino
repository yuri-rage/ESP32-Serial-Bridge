/*********************************************************************************
 * ESP32-Serial-WiFi-Client
 *
 * Simple ESP32 TCP or UDP client for use with ESP-Serial-Bridge
 *
 *   -- Yuri - Apr 2023
 *
 * Disclaimer: Don't use for life support systems or any other situation
 * where system failure may affect user or environmental safety.
 *********************************************************************************/

#include <Arduino.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <esp_wifi.h>

#include "client_config.h"

#ifdef PROTOCOL_TCP
#include <WiFiClient.h>
#elif defined(PROTOCOL_UDP)
#include <AsyncUDP.h>
#endif

#ifdef OTA_HANDLER
#include <ArduinoOTA.h>
#endif

uint8_t buf[BUFFERSIZE];
uint16_t num = 0;

#ifdef PROTOCOL_TCP
WiFiClient client;
void connect_to_host() {
    debug.printf("Connecting to %s:%d...", HOST_IP.toString(), HOST_PORT);
    while (!client.connect(HOST_IP, HOST_PORT)) {
        delay(500);
        debug.print('.');
    }
    debug.println("connected\n");
    delay(1000);
}
#endif

#ifdef PROTOCOL_UDP
AsyncUDP udp;
#endif

void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info) {
    debug.print("WiFi disconnected: ");
    debug.println(info.wifi_sta_disconnected.reason);
    debug.println("Trying to reconnect..");
    WiFi.begin(SSID, PASSWD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        debug.print(".");
    }
    debug.println("connected");
    debug.print("IP address: ");
    debug.println(WiFi.localIP());
}

void setup() {
    delay(500);

    Serial.begin(CLIENT_BAUD, CLIENT_PARAM, CLIENT_RXPIN, CLIENT_TXPIN);

    debug.print("\n\nWiFi serial bridge client ");
    debug.println(VERSION);

    debug.println("Open ESP Station Mode");
    WiFi.mode(WIFI_STA);
    WiFi.onEvent(WiFiStationDisconnected, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
    WiFi.begin(SSID, PASSWD);
    debug.print("Connecting to: ");
    debug.print(SSID);
    debug.print("..");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        debug.print(".");
    }
    debug.println("connected");
    debug.print("IP address: ");
    debug.println(WiFi.localIP());

#ifdef OTA_HANDLER
    ArduinoOTA.onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
            type = "sketch";
        else  // U_SPIFFS
            type = "filesystem";

        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS
        // using SPIFFS.end()
        Serial.println("Start updating " + type);
    });
    ArduinoOTA.onEnd([]() { Serial.println("\nEnd"); });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR)
            Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR)
            Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR)
            Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR)
            Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR)
            Serial.println("End Failed");
    });

    ArduinoOTA.begin();
#endif

#ifdef PROTOCOL_UDP
    if (udp.listen(HOST_PORT)) {
        debug.printf("Listening on UDP port %d\n", HOST_PORT);
        udp.onPacket([](AsyncUDPPacket packet) {
            if (packet.localPort() == HOST_PORT)
                Serial.write(packet.data(), packet.length());
        });
    }
#endif
}

void loop() {
#ifdef OTA_HANDLER
    ArduinoOTA.handle();
#endif

#ifdef PROTOCOL_TCP
    if (!client.connected()) connect_to_host();

    while (client.available()) {
        buf[num] = client.read();
        num++;
        if (num == BUFFERSIZE - 1) break;
    }
    if (num > 0) Serial.write(buf, num);
    num = 0;
#endif

    while (Serial.available()) {
        buf[num] = Serial.read();
        num++;
        if (num == BUFFERSIZE - 1) break;
    }
#ifdef PROTOCOL_TCP
    if (num > 0) client.write(buf, num);
#elif defined(PROTOCOL_UDP)
    udp.broadcastTo(buf, num, HOST_PORT + 1);
#endif
    num = 0;
}
