
#ifndef DIYBMSServer_H_
#define DIYBMSServer_H_

#include <Arduino.h>

#if defined(ESP8266)
//https://github.com/esp8266/Arduino
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include "ESP8266TrueRandom.h"
#else
#include <WiFi.h>
#include <AsyncTCP.h>
#endif

#include <ESPAsyncWebServer.h>

#include <EEPROM.h>

#include "defines.h"
#include "Rules.h"
#include "settings.h"
#include "ArduinoJson.h"
#include "PacketRequestGenerator.h"
#include "PacketReceiveProcessor.h"

class DIYBMSServer
{
public:
    static void StartServer(AsyncWebServer *webserver,
                            diybms_eeprom_settings *mysettings,
                            sdcard_info (*sdcardcallback)(),
                            PacketRequestGenerator *prg,
                            PacketReceiveProcessor *pktreceiveproc,
                            ControllerState *controlState,
                            Rules *rules,
                            void (*sdcardaction_callback)(uint8_t action)
                            );

    static void generateUUID();
    static void clearModuleValues(uint8_t module);

private:
    static AsyncWebServer *_myserver;
    static String UUIDString;

    //Pointers to other classes (not always a good idea in static classes)
    static sdcard_info (*_sdcardcallback)();
    static void (*_sdcardaction_callback)(uint8_t action);
    static PacketRequestGenerator *_prg;
    static PacketReceiveProcessor *_receiveProc;
    static diybms_eeprom_settings *_mysettings;
    static Rules *_rules;
    static ControllerState *_controlState;


    static void saveConfiguration()
    {
        Settings::WriteConfigToEEPROM((char *)_mysettings, sizeof(diybms_eeprom_settings), EEPROM_SETTINGS_START_ADDRESS);
    }
    static void PrintStreamComma(AsyncResponseStream *response,const __FlashStringHelper *ifsh, uint32_t value);

    static void handleNotFound(AsyncWebServerRequest *request);
    static void monitor2(AsyncWebServerRequest *request);
    static void monitor3(AsyncWebServerRequest *request);
    //static void monitor(AsyncWebServerRequest *request);
    static void modules(AsyncWebServerRequest *request);
    static void integration(AsyncWebServerRequest *request);
    static void identifyModule(AsyncWebServerRequest *request);
    static void GetRules(AsyncWebServerRequest *request);
    static String TemplateProcessor(const String &var);
    static bool validateXSS(AsyncWebServerRequest *request);
    static void SendSuccess(AsyncWebServerRequest *request);
    static void SendFailure(AsyncWebServerRequest *request);
    static void settings(AsyncWebServerRequest *request);
    static void resetCounters(AsyncWebServerRequest *request);
    static void handleRestartController(AsyncWebServerRequest *request);
    static void storage(AsyncWebServerRequest *request);

    static void saveSetting(AsyncWebServerRequest *request);
    static void saveInfluxDBSetting(AsyncWebServerRequest *request);
    static void saveMQTTSetting(AsyncWebServerRequest *request);
    static void saveGlobalSetting(AsyncWebServerRequest *request);
    static void saveBankConfiguration(AsyncWebServerRequest *request);
    static void saveRuleConfiguration(AsyncWebServerRequest *request);
    static void saveNTP(AsyncWebServerRequest *request);
    static void saveStorage(AsyncWebServerRequest *request);

    static void saveDisplaySetting(AsyncWebServerRequest *request);

    static void sdMount(AsyncWebServerRequest *request);
    static void sdUnmount(AsyncWebServerRequest *request);

    static String uuidToString(uint8_t *uuidLocation);
    static void SetCacheAndETagGzip(AsyncWebServerResponse *response, String ETag);
    static void SetCacheAndETag(AsyncWebServerResponse *response, String ETag);
};



extern bool OutputsEnabled;
extern bool InputsEnabled;

#endif
