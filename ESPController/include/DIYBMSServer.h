
#ifndef DIYBMSServer_H_
#define DIYBMSServer_H_

#include <Arduino.h>
#if defined(ESP8266)
//https://github.com/esp8266/Arduino
#include <ESP8266WiFi.h>          
#include <ESPAsyncTCP.h>
#else
#include <WiFi.h>
#include <AsyncTCP.h>
#endif

#include <ESPAsyncWebServer.h>

#include <EEPROM.h>

#include "settings.h"
#include "ArduinoJson.h"
#include "PacketRequestGenerator.h"
#include "PacketReceiveProcessor.h"

#if defined(ESP8266)
#include "ESP8266TrueRandom.h"
#endif

class DIYBMSServer {
   public:
      static void StartServer(AsyncWebServer  *webserver);
      static void generateUUID();

  private:
      static AsyncWebServer * _myserver;
      static String UUIDString;

      static void handleNotFound(AsyncWebServerRequest *request);
      static void monitor(AsyncWebServerRequest *request);
      static void modules(AsyncWebServerRequest *request);
      static void integration(AsyncWebServerRequest *request);
      static void identifyModule(AsyncWebServerRequest *request);
      static void rules(AsyncWebServerRequest *request);
      static String TemplateProcessor(const String& var);
      static bool validateXSS(AsyncWebServerRequest *request);
      static void SendSuccess(AsyncWebServerRequest *request);
      static void settings(AsyncWebServerRequest *request);
      static void clearModuleValues(uint8_t bank, uint8_t module);
      static void resetCounters(AsyncWebServerRequest *request);
      static void handleRestartController(AsyncWebServerRequest *request);

      static void saveSetting(AsyncWebServerRequest *request);
      static void saveInfluxDBSetting(AsyncWebServerRequest *request);
      static void saveMQTTSetting(AsyncWebServerRequest *request);
      static void saveGlobalSetting(AsyncWebServerRequest *request);
      static void saveBankConfiguration(AsyncWebServerRequest *request);
      static void saveRuleConfiguration(AsyncWebServerRequest *request);
      static void saveNTP(AsyncWebServerRequest *request);

      static String uuidToString(uint8_t* uuidLocation);
};

//TODO: Mixing of classes, static and extern is not great
extern PacketRequestGenerator prg;
extern PacketReceiveProcessor receiveProc;
extern diybms_eeprom_settings mysettings;
extern uint16_t ConfigHasChanged;
extern bool rule_outcome[RELAY_RULES];
extern bool PCF8574Enabled;
#endif
