/*********************************************************************************
 * ESP-Serial-Bridge
 * 
 * Simple WiFi Serial Bridge for Espressif microcontrollers
 * 
 * Forked from https://github.com/AlphaLima/ESP32-Serial-Bridge
 * 
 * Added compatibility for ESP8266, WiFi reconnect on failure, and mDNS discovery.
 * 
 * Note: ESP8266 is limited to 115200 baud and may be somewhat unreliable in
 *       this application.
 * 
 *   -- Yuri - Aug 2021
 * 
 * Disclaimer: Don't use for life support systems or any other situation
 * where system failure may affect user or environmental safety.
*********************************************************************************/

#ifdef ESP32
#include <esp_wifi.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <SoftwareSerial.h>  // use SoftwareSerial for ESP8266
#endif

#include "config.h"

#ifdef OTA_HANDLER  
#include <ArduinoOTA.h> 
#endif

#ifdef ESP32
#ifdef BLUETOOTH  // inside the ESP32 condition, since ESP8266 doesn't have BT
#include <BluetoothSerial.h>
BluetoothSerial SerialBT; 
#endif
#endif

#ifdef ESP32
HardwareSerial Serial_one(1);
HardwareSerial Serial_two(2);
HardwareSerial* COM[NUM_COM] = {&Serial, &Serial_one , &Serial_two};
#elif defined(ESP8266)
SoftwareSerial Serial_zero;
SoftwareSerial Serial_one;
SoftwareSerial* COM[NUM_COM] = {&Serial_zero, &Serial_one};
#endif

#define MAX_NMEA_CLIENTS 4
#ifdef PROTOCOL_TCP
#include <WiFiClient.h>
WiFiServer server_0(SERIAL0_TCP_PORT);
WiFiServer server_1(SERIAL1_TCP_PORT);
#ifdef ESP32
WiFiServer server_2(SERIAL2_TCP_PORT);
WiFiServer *server[NUM_COM]={&server_0, &server_1, &server_2};
#elif defined(ESP8266)
WiFiServer *server[NUM_COM] = {&server_0, &server_1};
#endif
WiFiClient TCPClient[NUM_COM][MAX_NMEA_CLIENTS];
#endif

uint8_t buf1[NUM_COM][bufferSize];
uint8_t buf2[NUM_COM][bufferSize];

#ifdef ESP32
uint16_t i1[NUM_COM]={0, 0, 0};
uint16_t i2[NUM_COM]={0, 0, 0};
#elif defined(ESP8266)
uint16_t i1[NUM_COM]={0, 0};
uint16_t i2[NUM_COM]={0, 0};
#endif

uint8_t BTbuf[bufferSize];
uint16_t iBT = 0;

#ifdef MODE_STA
#ifdef ESP32
void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info){
#elif defined(ESP8266)
WiFiEventHandler stationDisconnectedHandler;
void WiFiStationDisconnected(const WiFiEventSoftAPModeStationDisconnected& evt) {
#endif
  if (debug) Serial.print("WiFi disconnected: ");
#ifdef ESP32
  if (debug) Serial.println(info.disconnected.reason);
#endif
  if (debug) Serial.println("Trying to reconnect..");
  WiFi.begin(ssid, pw);
  while (WiFi.status() != WL_CONNECTED) {   
    delay(500);
    if (debug) Serial.print(".");
  }
  if (debug) Serial.println("connected");
  if (debug) Serial.print("IP address: ");
  if (debug) Serial.println(WiFi.localIP());
}
#endif

void setup() {

  delay(500);

#ifdef ESP32
  COM[0]->begin(UART_BAUD0, SERIAL_PARAM0, SERIAL0_RXPIN, SERIAL0_TXPIN);
  COM[1]->begin(UART_BAUD1, SERIAL_PARAM1, SERIAL1_RXPIN, SERIAL1_TXPIN);
  COM[2]->begin(UART_BAUD2, SERIAL_PARAM2, SERIAL2_RXPIN, SERIAL2_TXPIN);
#elif defined(ESP8266)
  COM[0]->begin(UART_BAUD0, SERIAL_PARAM0, SERIAL0_RXPIN, SERIAL0_TXPIN);
  COM[1]->begin(UART_BAUD1, SERIAL_PARAM1, SERIAL1_RXPIN, SERIAL1_TXPIN);
  if (debug) Serial.begin(UART_BAUD0);
#endif
  
  if (debug) Serial.print("\n\nWiFi serial bridge ");
  if (debug) Serial.println(VERSION);
  
  #ifdef MODE_AP 
  if (debug) Serial.println("Open ESP Access Point mode");
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, pw); // configure ssid and password for softAP
  delay(2000); // VERY IMPORTANT
  WiFi.softAPConfig(ip, ip, netmask); // configure ip address for softAP
  #endif

  #ifdef MODE_STA
  if (debug) Serial.println("Open ESP Station Mode");
  WiFi.mode(WIFI_STA);
  #ifdef ESP32
  WiFi.onEvent(WiFiStationDisconnected, SYSTEM_EVENT_STA_DISCONNECTED);
  #elif defined(ESP8266)
  stationDisconnectedHandler = WiFi.onSoftAPModeStationDisconnected(&WiFiStationDisconnected);
  #endif
  WiFi.begin(ssid, pw);
  if (debug) Serial.print("Connecting to: ");
  if (debug) Serial.print(ssid);
  if (debug) Serial.print("..");
  while (WiFi.status() != WL_CONNECTED) {   
    delay(500);
    if (debug) Serial.print(".");
  }
  if (debug) Serial.println("connected");
  if (debug) Serial.print("IP address: ");
  if (debug) Serial.println(WiFi.localIP());

  if(!MDNS.begin(mdns_name)) {
     if (debug) Serial.println("Error starting mDNS");
  } else {
    if (debug) Serial.print("Started mDNS, discoverable as: ");
    if (debug) Serial.println(mdns_name);
    MDNS.addService("telnet", "tcp", SERIAL0_TCP_PORT);
    MDNS.addService("telnet", "tcp", SERIAL1_TCP_PORT);
    #ifdef ESP32
    MDNS.addService("telnet", "tcp", SERIAL2_TCP_PORT);
    #endif
  }
  #endif

#ifdef ESP32
#ifdef BLUETOOTH
  if (debug) Serial.println("Open Bluetooth Server");  
  SerialBT.begin(ssid); //Bluetooth device name
#endif
#endif

#ifdef OTA_HANDLER  
    ArduinoOTA.onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    });
    ArduinoOTA.onEnd([]() {
      Serial.println("\nEnd");
    });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });
    ArduinoOTA.onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
  // if DNSServer is started with "*" for domain name, it will reply with
  // provided IP to all DNS request

  ArduinoOTA.begin();
#endif   

#ifdef PROTOCOL_TCP
  for(int num=0; num < NUM_COM ; num++) {
    if (debug) {
      Serial.print("Starting TCP Server ");
      Serial.println(num + 1);
    }
    server[num]->begin(); // start TCP server 
    server[num]->setNoDelay(true);
  }
#endif

#ifdef BATTERY_SAVER
#ifdef ESP32
  esp_err_t esp_wifi_set_max_tx_power(50);  //lower WiFi Power
#elif defined(ESP8266)
  WiFi.setOutputPower(15);
#endif
#endif

}

void loop() 
{  
#ifdef OTA_HANDLER  
  ArduinoOTA.handle();
#endif

#ifdef ESP32
#ifdef BLUETOOTH
  if(SerialBT.hasClient()) 
  {
    while(SerialBT.available())
    {
      BTbuf[iBT] = SerialBT.read();
      if(iBT <bufferSize-1) iBT++;
    }
    // this is unchanged from the AlphaLima implementation
    // all serial communication (up to 3 ports) is echoed on BlueTooth
    // may not be ideal...
    for(int num= 0; num < NUM_COM ; num++)
      COM[num]->write(BTbuf, iBT);
    iBT = 0;
  }  
#endif
#endif

#ifdef PROTOCOL_TCP
  for(int num=0; num < NUM_COM ; num++)
  {
    if (server[num]->hasClient())
    {
      for(byte i = 0; i < MAX_NMEA_CLIENTS; i++){
        //find free/disconnected spot
        if (!TCPClient[num][i] || !TCPClient[num][i].connected()){
          if(TCPClient[num][i]) TCPClient[num][i].stop();
          TCPClient[num][i] = server[num]->available();
          if (debug) Serial.print("New client for COM"); 
          if (debug) Serial.print(num);
          if (debug) Serial.print(" #");
          if (debug) Serial.println(i);
          continue;
        }
      }
      //no free/disconnected spot so reject
      WiFiClient TmpserverClient = server[num]->available();
      TmpserverClient.stop();
    }
  }
#endif
 
  for(int num= 0; num < NUM_COM ; num++)
  {
    if(COM[num] != NULL)          
    {
      for(byte cln = 0; cln < MAX_NMEA_CLIENTS; cln++)
      {               
        if(TCPClient[num][cln]) 
        {
          while(TCPClient[num][cln].available())
          {
            buf1[num][i1[num]] = TCPClient[num][cln].read(); // read char from client
            if(i1[num]<bufferSize-1) i1[num]++;
          } 

          COM[num]->write(buf1[num], i1[num]); // now send to UART(num):
          i1[num] = 0;
        }
      }
  
      if(COM[num]->available())
      {
        while(COM[num]->available())
        {     
          buf2[num][i2[num]] = COM[num]->read(); // read char from UART(num)
          if(i2[num]<bufferSize-1) i2[num]++;
        }
        // now send to WiFi:
        for(byte cln = 0; cln < MAX_NMEA_CLIENTS; cln++)
        {   
          if(TCPClient[num][cln])                     
            TCPClient[num][cln].write(buf2[num], i2[num]);
        }

#ifdef ESP32
#ifdef BLUETOOTH        
        // now send to Bluetooth:
        if(SerialBT.hasClient())      
          SerialBT.write(buf2[num], i2[num]);               
#endif
#endif
        i2[num] = 0;
      }
    }    
  }
}
