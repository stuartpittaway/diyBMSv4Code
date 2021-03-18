/*

 ____  ____  _  _  ____  __  __  ___    _  _  __
(  _ \(_  _)( \/ )(  _ \(  \/  )/ __)  ( \/ )/. |
 )(_) )_)(_  \  /  ) _ < )    ( \__ \   \  /(_  _)
(____/(____) (__) (____/(_/\/\_)(___/    \/   (_)

  (c) 2017/18/19/20 Stuart Pittaway

  This is the code for the controller - it talks to the V4.X cell modules over isolated serial bus

  This code runs on ESP-8266 WEMOS D1 (Mini or Pro) and compiles with VS CODE and PLATFORM IO environment
*/

/*
   ESP8266 PINS
   D0 = GREEN_LED
   D1 = i2c SDA
   D2 = i2c SCL
   D3 = switch to ground (reset WIFI configuration on power up)
   D4 = GPIO2 = TXD1 = TRANSMIT DEBUG SERIAL (and blue led on esp8266)
   D5 = GPIO14 = Interrupt in from PCF8574
   D7 = GPIO13 = RECEIVE SERIAL
   D8 = GPIO15 = TRANSMIT SERIAL

*/

#include <Arduino.h>

//#define PACKET_LOGGING_RECEIVE
//#define PACKET_LOGGING_SEND
//#define RULES_LOGGING
//#define MQTT_LOGGING

#include "FS.h"

//Libraries just for ESP8266

#include <TimeLib.h>
#include <ESP8266WiFi.h>
#include <NtpClientLib.h>
#include <LittleFS.h>
#include <ESP8266mDNS.h>

#include <Ticker.h>
#include <ESPAsyncWebServer.h>
#include <AsyncMqttClient.h>
#include <SerialEncoder.h>
#include <cppQueue.h>

#include "defines.h"

#include <ArduinoOTA.h>

#include "HAL_ESP8266.h"
HAL_ESP8266 hal;

#include "Rules.h"

volatile bool emergencyStop = false;

Rules rules;

bool _sd_card_installed = false;
diybms_eeprom_settings mysettings;
uint16_t ConfigHasChanged = 0;

uint16_t TotalNumberOfCells() { return mysettings.totalNumberOfBanks * mysettings.totalNumberOfSeriesModules; }

bool server_running = false;
RelayState previousRelayState[RELAY_TOTAL];
bool previousRelayPulse[RELAY_TOTAL];

volatile enumInputState InputState[INPUTS_TOTAL];

bool NTPsyncEventTriggered = false; // True if a time even has been triggered
NTPSyncEvent_t ntpEvent;            // Last triggered event

AsyncWebServer server(80);

void IRAM_ATTR ExternalInputInterrupt()
{
  if ((hal.ReadInputRegisters() & B00010000) == 0)
  {
    //Emergency Stop (J1) has triggered
    emergencyStop = true;
  }
}

//This large array holds all the information about the modules
//up to 4x16
CellModuleInfo cmi[maximum_controller_cell_modules];

#include "crc16.h"

#include "settings.h"
#include "SoftAP.h"
#include "DIYBMSServer.h"
#include "PacketRequestGenerator.h"
#include "PacketReceiveProcessor.h"

// Instantiate queue to hold packets ready for transmission
cppQueue requestQueue(sizeof(PacketStruct), 24, FIFO);

cppQueue replyQueue(sizeof(PacketStruct), 8, FIFO);

PacketRequestGenerator prg = PacketRequestGenerator(&requestQueue);

PacketReceiveProcessor receiveProc = PacketReceiveProcessor();

// Memory to hold in and out serial buffer
uint8_t SerialPacketReceiveBuffer[2 * sizeof(PacketStruct)];

SerialEncoder myPacketSerial;

WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;

Ticker myTimerRelay;
Ticker myTimer;
Ticker myTransmitTimer;
Ticker myReplyTimer;
Ticker myLazyTimer;
Ticker wifiReconnectTimer;
Ticker mqttReconnectTimer;
Ticker myTimerSendMqttPacket;
Ticker myTimerSendMqttStatus;
Ticker myTimerSendInfluxdbPacket;
Ticker myTimerSwitchPulsedRelay;

uint16_t sequence = 0;

ControllerState ControlState = ControllerState::Unknown;

bool OutputsEnabled;
bool InputsEnabled;

AsyncMqttClient mqttClient;

void dumpByte(uint8_t data)
{
  if (data <= 0x0F)
    SERIAL_DEBUG.print('0');
  SERIAL_DEBUG.print(data, HEX);
}
void dumpPacketToDebug(char indicator, PacketStruct *buffer)
{
  //Filter on selected commands
  //if ((buffer->command & 0x0F) != COMMAND::Timing)    return;

  SERIAL_DEBUG.print(millis());
  SERIAL_DEBUG.print(':');

  SERIAL_DEBUG.print(indicator);

  SERIAL_DEBUG.print(':');
  dumpByte(buffer->start_address);
  SERIAL_DEBUG.print('-');
  dumpByte(buffer->end_address);
  SERIAL_DEBUG.print('/');
  dumpByte(buffer->hops);
  SERIAL_DEBUG.print('/');
  dumpByte(buffer->command);
  SERIAL_DEBUG.print(' ');

  //TODO: Could store these in PROGMEM char array
  switch (buffer->command & 0x0F)
  {
  case COMMAND::ResetBadPacketCounter:
    SERIAL_DEBUG.print(F("ResetC   "));
    break;
  case COMMAND::ReadVoltageAndStatus:
    SERIAL_DEBUG.print(F("RdVolt   "));
    break;
  case COMMAND::Identify:
    SERIAL_DEBUG.print(F("Ident    "));
    break;
  case COMMAND::ReadTemperature:
    SERIAL_DEBUG.print(F("RdTemp   "));
    break;
  case COMMAND::ReadBadPacketCounter:
    SERIAL_DEBUG.print(F("RdBadPkC "));
    break;
  case COMMAND::ReadSettings:
    SERIAL_DEBUG.print(F("RdSettin "));
    break;
  case COMMAND::WriteSettings:
    SERIAL_DEBUG.print(F("WriteSet "));
    break;
  case COMMAND::ReadBalancePowerPWM:
    SERIAL_DEBUG.print(F("RdBalanc "));
    break;
  case COMMAND::Timing:
    SERIAL_DEBUG.print(F("Timing   "));
    break;
  case COMMAND::ReadBalanceCurrentCounter:
    SERIAL_DEBUG.print(F("Bal mAh  "));
    break;
  case COMMAND::ReadPacketReceivedCounter:
    SERIAL_DEBUG.print(F("PktRvd   "));
    break;
  default:
    SERIAL_DEBUG.print(F("??????   "));
    break;
  }

  SERIAL_DEBUG.printf("%.4X", buffer->sequence);
  //SERIAL_DEBUG.print(buffer->sequence, HEX);
  SERIAL_DEBUG.print('=');
  for (size_t i = 0; i < maximum_cell_modules_per_packet; i++)
  {
    //SERIAL_DEBUG.print(buffer->moduledata[i], HEX);
    SERIAL_DEBUG.printf("%.4X", buffer->moduledata[i]);
    SERIAL_DEBUG.print(" ");
  }
  SERIAL_DEBUG.print("=");
  //SERIAL_DEBUG.print(buffer->crc, HEX);
  SERIAL_DEBUG.printf("%.4X", buffer->crc);

  SERIAL_DEBUG.println();
}

const char *ControllerStateString(ControllerState value)
{
  switch (value)
  {
  case ControllerState::PowerUp:
    return "PowerUp";
  case ControllerState::ConfigurationSoftAP:
    return "ConfigurationSoftAP";
  case ControllerState::Stabilizing:
    return "Stabilizing";
  case ControllerState::Running:
    return "Running";
  case ControllerState::Unknown:
    return "Unknown";
  }

  return "?";
}

void SetControllerState(ControllerState newState)
{
  if (ControlState != newState)
  {
    ControlState = newState;

    SERIAL_DEBUG.println("");
    SERIAL_DEBUG.print(F("** Controller changed to state = "));
    SERIAL_DEBUG.println(ControllerStateString(newState));
  }
}

uint16_t minutesSinceMidnight()
{

  return (hour() * 60) + minute();
}

void processSyncEvent(NTPSyncEvent_t ntpEvent)
{
  if (ntpEvent < 0)
  {
    SERIAL_DEBUG.printf("Time Sync error: %d\n", ntpEvent);
    /*
    if (ntpEvent == noResponse)
      SERIAL_DEBUG.println(F("NTP svr not reachable"));
    else if (ntpEvent == invalidAddress)
      SERIAL_DEBUG.println(F("Invalid NTP svr address"));
    else if (ntpEvent == errorSending)
      SERIAL_DEBUG.println(F("Error sending request"));
    else if (ntpEvent == responseError)
      SERIAL_DEBUG.println(F("NTP response error"));
      */
  }
  else
  {
    if (ntpEvent == timeSyncd)
    {
      SERIAL_DEBUG.print(F("NTP time "));
      time_t lastTime = NTP.getLastNTPSync();
      SERIAL_DEBUG.println(NTP.getTimeDateString(lastTime));
      setTime(lastTime);
    }
  }
}

void serviceReplyQueue()
{
  //if (replyQueue.isEmpty()) return;

  while (!replyQueue.isEmpty())
  {
    PacketStruct ps;
    replyQueue.pop(&ps);

#if defined(PACKET_LOGGING_RECEIVE)
    // Process decoded incoming packet
    dumpPacketToDebug('R', &ps);
#endif

    if (receiveProc.ProcessReply(&ps))
    {
      //Success, do nothing
    }
    else
    {
      SERIAL_DEBUG.print(F("*FAIL*"));
      dumpPacketToDebug('F', &ps);
    }
  }
}

void onPacketReceived()
{

  hal.GreenLedOn();

  PacketStruct ps;
  memcpy(&ps, SerialPacketReceiveBuffer, sizeof(PacketStruct));

  if ((ps.command & 0x0F) == COMMAND::Timing)
  {
    //Timestamp at the earliest possible moment
    uint32_t t = millis();
    ps.moduledata[2] = (t & 0xFFFF0000) >> 16;
    ps.moduledata[3] = t & 0x0000FFFF;
    //Ensure CRC is correct
    ps.crc = CRC16::CalculateArray((uint8_t *)&ps, sizeof(PacketStruct) - 2);
  }

  if (!replyQueue.push(&ps))
  {
    SERIAL_DEBUG.println(F("*Failed to queue reply*"));
  }

  //#if defined(PACKET_LOGGING_RECEIVE)
  // Process decoded incoming packet
  //dumpPacketToDebug('Q', &ps);
  //#endif

  hal.GreenLedOff();
}

void timerTransmitCallback()
{
  if (requestQueue.isEmpty())
    return;

  // Called to transmit the next packet in the queue need to ensure this procedure
  // is called more frequently than items are added into the queue

  PacketStruct transmitBuffer;

  requestQueue.pop(&transmitBuffer);
  sequence++;
  transmitBuffer.sequence = sequence;

  if (transmitBuffer.command == COMMAND::Timing)
  {
    //Timestamp at the last possible moment
    uint32_t t = millis();
    transmitBuffer.moduledata[0] = (t & 0xFFFF0000) >> 16;
    transmitBuffer.moduledata[1] = t & 0x0000FFFF;
  }

  transmitBuffer.crc = CRC16::CalculateArray((uint8_t *)&transmitBuffer, sizeof(PacketStruct) - 2);
  myPacketSerial.sendBuffer((byte *)&transmitBuffer);

// Output the packet we just transmitted to debug console
#if defined(PACKET_LOGGING_SEND)
  dumpPacketToDebug('S', &transmitBuffer);
#endif
}

//Runs the rules and populates rule_outcome array with true/false for each rule
//Rules based on module parameters/readings like voltage and temperature
//are only processed once every module has returned at least 1 reading/communication
void ProcessRules()
{
  rules.ClearValues();
  rules.ClearWarnings();
  rules.ClearErrors();

  rules.rule_outcome[Rule::BMSError] = false;

  uint16_t totalConfiguredModules = TotalNumberOfCells();
  if (totalConfiguredModules > maximum_controller_cell_modules)
  {
    //System is configured with more than maximum modules - abort!
    rules.SetError(InternalErrorCode::TooManyModules);
  }

  if (receiveProc.totalModulesFound > 0 && receiveProc.totalModulesFound != totalConfiguredModules)
  {
    //Found more or less modules than configured for
    rules.SetError(InternalErrorCode::ModuleCountMismatch);
  }

  //Communications error...
  if (receiveProc.HasCommsTimedOut())
  {
    rules.SetError(InternalErrorCode::CommunicationsError);
    rules.rule_outcome[Rule::BMSError] = true;
  }

  uint8_t cellid = 0;
  for (int8_t bank = 0; bank < mysettings.totalNumberOfBanks; bank++)
  {
    for (int8_t i = 0; i < mysettings.totalNumberOfSeriesModules; i++)
    {
      rules.ProcessCell(bank, &cmi[cellid]);

      if (cmi[cellid].valid && cmi[cellid].settingsCached)
      {
        if (cmi[cellid].BypassThresholdmV != mysettings.BypassThresholdmV)
        {
          rules.SetWarning(InternalWarningCode::ModuleInconsistantBypassVoltage);
        }

        if (cmi[cellid].BypassOverTempShutdown != mysettings.BypassOverTempShutdown)
        {
          rules.SetWarning(InternalWarningCode::ModuleInconsistantBypassTemperature);
        }

        if (cmi[0].settingsCached && cmi[cellid].CodeVersionNumber != cmi[0].CodeVersionNumber)
        {
          //Do all the modules have the same version of code as module zero?
          rules.SetWarning(InternalWarningCode::ModuleInconsistantCodeVersion);
        }

        if (cmi[0].settingsCached && cmi[cellid].BoardVersionNumber != cmi[0].BoardVersionNumber)
        {
          //Do all the modules have the same hardware revision?
          rules.SetWarning(InternalWarningCode::ModuleInconsistantBoardRevision);
        }
      }

      cellid++;
    }
    rules.ProcessBank(bank);
  }

  if (mysettings.loggingEnabled && !_sd_card_installed)
  {
    rules.SetWarning(InternalWarningCode::LoggingEnabledNoSDCard);
  }

  if (rules.invalidModuleCount > 0)
  {
    //Some modules are not yet valid
    rules.SetError(InternalErrorCode::WaitingForModulesToReply);
  }

  if (ControlState == ControllerState::Running && rules.zeroVoltageModuleCount > 0)
  {
    rules.SetError(InternalErrorCode::ZeroVoltModule);
    rules.rule_outcome[Rule::BMSError] = true;
  }

  rules.RunRules(
      mysettings.rulevalue,
      mysettings.rulehysteresis,
      emergencyStop,
      minutesSinceMidnight());

  if (ControlState == ControllerState::Stabilizing)
  {
    //Check for zero volt modules - not a problem whilst we are in stabilizing start up mode
    if (rules.zeroVoltageModuleCount == 0 && rules.invalidModuleCount == 0)
    {
      //Every module has been read and they all returned a voltage move to running state
      SetControllerState(ControllerState::Running);
    }
  }
}

void timerSwitchPulsedRelay()
{
  //Set defaults based on configuration
  for (int8_t y = 0; y < RELAY_TOTAL; y++)
  {
    if (previousRelayPulse[y])
    {
      //We now need to rapidly turn off the relay after a fixed period of time (pulse mode)
      //However we leave the relay and previousRelayState looking like the relay has triggered (it has!)
      //to prevent multiple pulses being sent on each rule refresh

      hal.SetOutputState(y, previousRelayState[y] == RelayState::RELAY_ON ? RelayState::RELAY_OFF : RelayState::RELAY_ON);

      previousRelayPulse[y] = false;
    }
  }

  //This only fires once
  myTimerSwitchPulsedRelay.detach();
}

void timerProcessRules()
{

  //Run the rules
  ProcessRules();

#if defined(RULES_LOGGING)
  SERIAL_DEBUG.print(F("Rules:"));
  for (int8_t r = 0; r < RELAY_RULES; r++)
  {
    SERIAL_DEBUG.print(rules.rule_outcome[r]);
  }
  SERIAL_DEBUG.print("=");
#endif

  RelayState relay[RELAY_TOTAL];

  //Set defaults based on configuration
  for (int8_t y = 0; y < RELAY_TOTAL; y++)
  {
    relay[y] = mysettings.rulerelaydefault[y] == RELAY_ON ? RELAY_ON : RELAY_OFF;
  }

  //Test the rules (in reverse order)
  for (int8_t n = RELAY_RULES - 1; n >= 0; n--)
  {
    if (rules.rule_outcome[n] == true)
    {

      for (int8_t y = 0; y < RELAY_TOTAL; y++)
      {
        //Dont change relay if its set to ignore/X
        if (mysettings.rulerelaystate[n][y] != RELAY_X)
        {
          if (mysettings.rulerelaystate[n][y] == RELAY_ON)
          {
            relay[y] = RELAY_ON;
          }
          else
          {
            relay[y] = RELAY_OFF;
          }
        }
      }
    }
  }

  for (int8_t n = 0; n < RELAY_TOTAL; n++)
  {
    if (previousRelayState[n] != relay[n])
    {
//Would be better here to use the WRITE8 to lower i2c traffic
#if defined(RULES_LOGGING)
      SERIAL_DEBUG.print(F("Relay:"));
      SERIAL_DEBUG.print(n);
      SERIAL_DEBUG.print("=");
      SERIAL_DEBUG.print(relay[n]);
#endif
      //hal.SetOutputState(n, relay[n]);

      //This would be better if we worked out the bit pattern first and then just
      //submitted that as a single i2c read/write transaction

      hal.SetOutputState(n, relay[n]);

      previousRelayState[n] = relay[n];

      if (mysettings.relaytype[n] == RELAY_PULSE)
      {
        //If its a pulsed relay, invert the output quickly via a single shot timer
        previousRelayPulse[n] = true;
        myTimerSwitchPulsedRelay.attach(0.1, timerSwitchPulsedRelay);
#if defined(RULES_LOGGING)
        SERIAL_DEBUG.print("P");
#endif
      }
    }
  }
#if defined(RULES_LOGGING)
  SERIAL_DEBUG.println("");
#endif
}

void timerEnqueueCallback()
{
  //this is called regularly on a timer, it determines what request to make to the modules (via the request queue)
  uint16_t i = 0;
  uint16_t max = TotalNumberOfCells();

  uint8_t startmodule = 0;

  while (i < max)
  {
    uint16_t endmodule = (startmodule + maximum_cell_modules_per_packet) - 1;

    //Limit to number of modules we have configured
    if (endmodule > max)
    {
      endmodule = max - 1;
    }

    //Need to watch overflow of the uint8 here...
    prg.sendCellVoltageRequest(startmodule, endmodule);
    prg.sendCellTemperatureRequest(startmodule, endmodule);

    //If any module is in bypass then request PWM reading for whole bank
    for (uint8_t m = startmodule; m <= endmodule; m++)
    {
      if (cmi[m].inBypass)
      {
        prg.sendReadBalancePowerRequest(startmodule, endmodule);
        //We only need 1 reading for whole bank
        break;
      }
    }

    //Move to the next bank
    startmodule = endmodule + 1;
    i += maximum_cell_modules_per_packet;
  }
}

void connectToWifi()
{
  wl_status_t status = WiFi.status();
  if (status == WL_CONNECTED)
  {
    return;
  }

  /*
WiFi.status() only returns:

    switch(status) {
        case STATION_GOT_IP:
            return WL_CONNECTED;
        case STATION_NO_AP_FOUND:
            return WL_NO_SSID_AVAIL;
        case STATION_CONNECT_FAIL:
        case STATION_WRONG_PASSWORD:
            return WL_CONNECT_FAILED;
        case STATION_IDLE:
            return WL_IDLE_STATUS;
        default:
            return WL_DISCONNECTED;
    }
*/

  WiFi.mode(WIFI_STA);

  char hostname[40];

  sprintf(hostname, "DIYBMS-%08X", ESP.getChipId());
  wifi_station_set_hostname(hostname);
  WiFi.hostname(hostname);

  SERIAL_DEBUG.print(F("Hostname: "));
  SERIAL_DEBUG.print(hostname);
  SERIAL_DEBUG.print(F(" Current state: "));
  SERIAL_DEBUG.print((uint8_t)status);

  SERIAL_DEBUG.println(F(",Connect to Wi-Fi..."));
  WiFi.begin(DIYBMSSoftAP::WifiSSID(), DIYBMSSoftAP::WifiPassword());
}

void connectToMqtt()
{
  SERIAL_DEBUG.println(F("Connecting to MQTT..."));
  mqttClient.connect();
}

static AsyncClient *aClient = NULL;

void setupInfluxClient()
{

  if (aClient) //client already exists
    return;

  aClient = new AsyncClient();
  if (!aClient) //could not allocate client
    return;

  aClient->onError([](void *arg, AsyncClient *client, err_t error) {
    SERIAL_DEBUG.println(F("Connect Error"));
    aClient = NULL;
    delete client;
  },
                   NULL);

  aClient->onConnect([](void *arg, AsyncClient *client) {
    SERIAL_DEBUG.println(F("Connected"));

    //Send the packet here

    aClient->onError(NULL, NULL);

    client->onDisconnect([](void *arg, AsyncClient *c) {
      SERIAL_DEBUG.println(F("Disconnected"));
      aClient = NULL;
      delete c;
    },
                         NULL);

    client->onData([](void *arg, AsyncClient *c, void *data, size_t len) {
      //Data received
      SERIAL_DEBUG.print(F("\r\nData: "));
      SERIAL_DEBUG.println(len);
      //uint8_t* d = (uint8_t*)data;
      //for (size_t i = 0; i < len; i++) {SERIAL_DEBUG.write(d[i]);}
    },
                   NULL);

    //send the request

    //Construct URL for the influxdb
    //See API at https://docs.influxdata.com/influxdb/v1.7/tools/api/#write-http-endpoint

    String poststring;

    for (uint8_t bank = 0; bank < mysettings.totalNumberOfBanks; bank++)
    {
      //TODO: We should send a request per bank not just a single POST as we are likely to exceed capabilities of ESP
      for (uint8_t i = 0; i < mysettings.totalNumberOfSeriesModules; i++)
      {
        //Data in LINE PROTOCOL format https://docs.influxdata.com/influxdb/v1.7/write_protocols/line_protocol_tutorial/
        poststring = poststring + "cells," + "cell=" + String(bank) + "_" + String(i) + " v=" + String((float)cmi[i].voltagemV / 1000.0, 3) + ",i=" + String(cmi[i].internalTemp) + "i" + ",e=" + String(cmi[i].externalTemp) + "i" + ",b=" + (cmi[i].inBypass ? String("true") : String("false")) + "\n";
      }
    }

    //TODO: Need to URLEncode these values
    String url = "/write?db=" + String(mysettings.influxdb_database) + "&u=" + String(mysettings.influxdb_user) + "&p=" + String(mysettings.influxdb_password);
    String header = "POST " + url + " HTTP/1.1\r\n" + "Host: " + String(mysettings.influxdb_host) + "\r\n" + "Connection: close\r\n" + "Content-Length: " + poststring.length() + "\r\n" + "Content-Type: text/plain\r\n" + "\r\n";

    //SERIAL_DEBUG.println(header.c_str());
    //SERIAL_DEBUG.println(poststring.c_str());

    client->write(header.c_str());
    client->write(poststring.c_str());
  },
                     NULL);
}

void SendInfluxdbPacket()
{
  if (!mysettings.influxdb_enabled)
    return;

  SERIAL_DEBUG.println(F("SendInfluxdbPacket"));

  setupInfluxClient();

  if (!aClient->connect(mysettings.influxdb_host, mysettings.influxdb_httpPort))
  {
    SERIAL_DEBUG.println(F("Influxdb connect fail"));
    AsyncClient *client = aClient;
    aClient = NULL;
    delete client;
  }
}

void startTimerToInfluxdb()
{
  myTimerSendInfluxdbPacket.attach(30, SendInfluxdbPacket);
}

void SetupOTA()
{

  ArduinoOTA.setPort(3232);
  ArduinoOTA.setPassword("1jiOOx12AQgEco4e");

  ArduinoOTA
      .onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
          type = "sketch";
        else // U_SPIFFS
          type = "filesystem";

        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
        SERIAL_DEBUG.println("Start updating " + type);
      });
  ArduinoOTA.onEnd([]() {
    SERIAL_DEBUG.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    SERIAL_DEBUG.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    SERIAL_DEBUG.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR)
      SERIAL_DEBUG.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR)
      SERIAL_DEBUG.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR)
      SERIAL_DEBUG.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR)
      SERIAL_DEBUG.println("Receive Failed");
    else if (error == OTA_END_ERROR)
      SERIAL_DEBUG.println("End Failed");
  });

  ArduinoOTA.begin();
}

sdcard_info sdcard_callback()
{
  //Fake
  sdcard_info ret;
  ret.available = _sd_card_installed;
  ret.totalkilobytes = 0;
  ret.usedkilobytes = 0;
  ret.flash_totalkilobytes = 0;
  ret.flash_usedkilobytes = 0;
  return ret;
}
void sdcardaction_callback(uint8_t action)
{
  //Fake
  _sd_card_installed = false;
}

void onWifiConnect(const WiFiEventStationModeGotIP &event)
{

  SERIAL_DEBUG.print(F("onWifiConnect status="));
  SERIAL_DEBUG.println(WiFi.status());
  SERIAL_DEBUG.print(F("Connected IP:"));
  SERIAL_DEBUG.println(WiFi.localIP());
  SERIAL_DEBUG.print(F("Hostname:"));
  SERIAL_DEBUG.println(WiFi.hostname().c_str());

  //SERIAL_DEBUG.print(F("Request NTP from "));
  //SERIAL_DEBUG.println(mysettings.ntpServer);

  //Update time every 20 minutes
  NTP.setInterval(1200);
  NTP.setNTPTimeout(NTP_TIMEOUT);
  // String ntpServerName, int8_t timeZone, bool daylight, int8_t minutes, AsyncUDP* udp_conn
  NTP.begin(mysettings.ntpServer, mysettings.timeZone, mysettings.daylight, mysettings.minutesTimeZone);

  /*
  TODO: CHECK ERROR CODES BETTER!
  0 : WL_IDLE_STATUS when Wi-Fi is in process of changing between statuses
  1 : WL_NO_SSID_AVAIL in case configured SSID cannot be reached
  3 : WL_CONNECTED after successful connection is established
  4 : WL_CONNECT_FAILED if password is incorrect
  6 : WL_DISCONNECTED if module is not configured in station mode
  */
  if (!server_running)
  {
    DIYBMSServer::StartServer(&server, &mysettings, &sdcard_callback, &prg, &receiveProc, &ControlState, &rules, &sdcardaction_callback);
    server_running = true;
  }

  if (mysettings.mqtt_enabled)
  {
    connectToMqtt();
  }

  if (mysettings.influxdb_enabled)
  {
    startTimerToInfluxdb();
  }

  SetupOTA();

  // Set up mDNS responder:
  // - first argument is the domain name, in this example
  //   the fully-qualified domain name is "esp8266.local"
  // - second argument is the IP address to advertise
  //   we send our IP address on the WiFi network

  if (MDNS.begin(WiFi.hostname().c_str(), WiFi.localIP()))
  {
    // Add service to MDNS-SD
    MDNS.addService("http", "tcp", 80);
  }
  else
  {
    SERIAL_DEBUG.println("Error setting up MDNS responder!");
  }
}

void onWifiDisconnect(const WiFiEventStationModeDisconnected &event)
{
  SERIAL_DEBUG.println(F("Disconnected from Wi-Fi."));

  // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
  mqttReconnectTimer.detach();
  myTimerSendMqttPacket.detach();
  myTimerSendMqttStatus.detach();
  myTimerSendInfluxdbPacket.detach();

  NTP.stop();
  MDNS.end();
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
  SERIAL_DEBUG.println(F("Disconnected from MQTT."));

  myTimerSendMqttPacket.detach();
  myTimerSendMqttStatus.detach();

  if (WiFi.isConnected())
  {
    mqttReconnectTimer.once(2, connectToMqtt);
  }
}

void sendMqttStatus()
{
  if (!mysettings.mqtt_enabled && !mqttClient.connected())
    return;

  char topic[80];
  char jsonbuffer[220];
  DynamicJsonDocument doc(220);
  JsonObject root = doc.to<JsonObject>();

  root["banks"] = mysettings.totalNumberOfBanks;
  root["cells"] = mysettings.totalNumberOfSeriesModules;
  root["uptime"] = millis() / 1000; // I want to know the uptime of the device.

  // Set error flag if we have attempted to send 2*number of banks without a reply
  root["commserr"] = receiveProc.HasCommsTimedOut() ? 1 : 0;
  root["sent"] = prg.packetsGenerated;
  root["received"] = receiveProc.packetsReceived;
  root["badcrc"] = receiveProc.totalCRCErrors;
  root["ignored"] = receiveProc.totalNotProcessedErrors;
  root["oos"] = receiveProc.totalOutofSequenceErrors;
  root["roundtrip"] = receiveProc.packetTimerMillisecond;

  serializeJson(doc, jsonbuffer, sizeof(jsonbuffer));
  sprintf(topic, "%s/status", mysettings.mqtt_topic);
  mqttClient.publish(topic, 0, false, jsonbuffer);
#if defined(MQTT_LOGGING)
  SERIAL_DEBUG.print("MQTT - ");
  SERIAL_DEBUG.print(topic);
  SERIAL_DEBUG.print('=');
  SERIAL_DEBUG.println(jsonbuffer);
#endif
  //Output bank level information (just voltage for now)
  for (int8_t bank = 0; bank < mysettings.totalNumberOfBanks; bank++)
  {
    doc.clear();
    doc["voltage"] = (float)rules.packvoltage[bank] / (float)1000.0;

    serializeJson(doc, jsonbuffer, sizeof(jsonbuffer));
    sprintf(topic, "%s/bank/%d", mysettings.mqtt_topic, bank);
    mqttClient.publish(topic, 0, false, jsonbuffer);
#if defined(MQTT_LOGGING)
    SERIAL_DEBUG.print("MQTT - ");
    SERIAL_DEBUG.print(topic);
    SERIAL_DEBUG.print('=');
    SERIAL_DEBUG.println(jsonbuffer);
#endif
  }

  //Using Json for below reduced MQTT messages from 14 to 2. Could be combined into same json object too. But even better is status + event driven.
  doc.clear(); // Need to clear the json object for next message
  sprintf(topic, "%s/rule", mysettings.mqtt_topic);
  for (uint8_t i = 0; i < RELAY_RULES; i++)
  {
    doc[(String)i] = rules.rule_outcome[i] ? 1 : 0; // String conversion should be removed but just quick to get json format nice
  }
  serializeJson(doc, jsonbuffer, sizeof(jsonbuffer));
#if defined(MQTT_LOGGING)
  SERIAL_DEBUG.print("MQTT - ");
  SERIAL_DEBUG.print(topic);
  SERIAL_DEBUG.print('=');
  SERIAL_DEBUG.println(jsonbuffer);
#endif
  mqttClient.publish(topic, 0, false, jsonbuffer);

  doc.clear(); // Need to clear the json object for next message
  sprintf(topic, "%s/output", mysettings.mqtt_topic);
  for (uint8_t i = 0; i < RELAY_TOTAL; i++)
  {
    doc[(String)i] = (previousRelayState[i] == RelayState::RELAY_ON) ? 1 : 0;
  }

  serializeJson(doc, jsonbuffer, sizeof(jsonbuffer));
#if defined(MQTT_LOGGING)
  SERIAL_DEBUG.print("MQTT - ");
  SERIAL_DEBUG.print(topic);
  SERIAL_DEBUG.print('=');
  SERIAL_DEBUG.println(jsonbuffer);
#endif
  mqttClient.publish(topic, 0, false, jsonbuffer);
}

//Send a few MQTT packets and keep track so we send the next batch on following calls
uint8_t mqttStartModule = 0;

void sendMqttPacket()
{
#if defined(MQTT_LOGGING)
  SERIAL_DEBUG.println("sendMqttPacket");
#endif

  if (!mysettings.mqtt_enabled && !mqttClient.connected())
    return;

  char topic[80];
  char jsonbuffer[200];
  StaticJsonDocument<200> doc;

  //If the BMS is in error, stop sending MQTT packets for the data
  if (!rules.rule_outcome[Rule::BMSError])
  {
    uint8_t counter = 0;
    for (uint8_t i = mqttStartModule; i < TotalNumberOfCells(); i++)
    {
      //Only send valid module data
      if (cmi[i].valid)
      {
        uint8_t bank = i / mysettings.totalNumberOfSeriesModules;
        uint8_t module = i - (bank * mysettings.totalNumberOfSeriesModules);

        doc.clear();
        doc["voltage"] = (float)cmi[i].voltagemV / (float)1000.0;
        doc["vMax"] = (float)cmi[i].voltagemVMax / (float)1000.0;
        doc["vMin"] = (float)cmi[i].voltagemVMin / (float)1000.0;
        doc["inttemp"] = cmi[i].internalTemp;
        doc["exttemp"] = cmi[i].externalTemp;
        doc["bypass"] = cmi[i].inBypass ? 1 : 0;
        doc["PWM"] = (int)((float)cmi[i].PWMValue / (float)255.0 * 100);
        doc["bypassT"] = cmi[i].bypassOverTemp ? 1 : 0;
        doc["bpc"] = cmi[i].badPacketCount;
        doc["mAh"] = cmi[i].BalanceCurrentCount;
        serializeJson(doc, jsonbuffer, sizeof(jsonbuffer));

        sprintf(topic, "%s/%d/%d", mysettings.mqtt_topic, bank, module);

        mqttClient.publish(topic, 0, false, jsonbuffer);

#if defined(MQTT_LOGGING)
        SERIAL_DEBUG.print("MQTT - ");
        SERIAL_DEBUG.print(topic);
        SERIAL_DEBUG.print('=');
        SERIAL_DEBUG.println(jsonbuffer);
#endif
      }

      counter++;

      //After transmitting this many packets over MQTT, store our current state and exit the function.
      //this prevents flooding the ESP controllers wifi stack and potentially causing reboots/fatal exceptions
      if (counter == 6)
      {
        mqttStartModule = i + 1;

        if (mqttStartModule > TotalNumberOfCells())
        {
          mqttStartModule = 0;
        }

        return;
      }
    }

    //Completed the loop, start at zero
    mqttStartModule = 0;
  }
}

void onMqttConnect(bool sessionPresent)
{
  SERIAL_DEBUG.println(F("Connected to MQTT."));
  myTimerSendMqttPacket.attach(5, sendMqttPacket);
  myTimerSendMqttStatus.attach(25, sendMqttStatus);
}

void LoadConfiguration()
{
  if (Settings::ReadConfigFromEEPROM((char *)&mysettings, sizeof(mysettings), EEPROM_SETTINGS_START_ADDRESS))
    return;

  SERIAL_DEBUG.println(F("Apply default config"));

  //Zero all the bytes
  memset(&mysettings, 0, sizeof(mysettings));

  //Default to a single module
  mysettings.totalNumberOfBanks = 1;
  mysettings.totalNumberOfSeriesModules = 1;
  mysettings.BypassOverTempShutdown = 65;
  //4.10V bypass
  mysettings.BypassThresholdmV = 4100;
  mysettings.graph_voltagehigh = 4.5;
  mysettings.graph_voltagelow = 2.75;

  //EEPROM settings are invalid so default configuration
  mysettings.mqtt_enabled = false;
  mysettings.mqtt_port = 1883;

  mysettings.loggingEnabled = false;
  mysettings.loggingFrequencySeconds = 15;

  //Default to EMONPI default MQTT settings
  strcpy(mysettings.mqtt_topic, "diybms");
  strcpy(mysettings.mqtt_server, "192.168.0.26");
  strcpy(mysettings.mqtt_username, "emonpi");
  strcpy(mysettings.mqtt_password, "emonpimqtt2016");

  mysettings.influxdb_enabled = false;
  strcpy(mysettings.influxdb_host, "myinfluxserver");
  strcpy(mysettings.influxdb_database, "database");
  strcpy(mysettings.influxdb_user, "user");
  strcpy(mysettings.influxdb_password, "");

  mysettings.timeZone = 0;
  mysettings.minutesTimeZone = 0;
  mysettings.daylight = false;
  strcpy(mysettings.ntpServer, "time.google.com");

  for (size_t x = 0; x < RELAY_TOTAL; x++)
  {
    mysettings.rulerelaydefault[x] = RELAY_OFF;
  }

  //Emergency stop
  mysettings.rulevalue[Rule::EmergencyStop] = 0;
  //Internal BMS error (communication issues, fault readings from modules etc)
  mysettings.rulevalue[Rule::BMSError] = 0;
  //Individual cell over voltage
  mysettings.rulevalue[Rule::Individualcellovervoltage] = 4150;
  //Individual cell under voltage
  mysettings.rulevalue[Rule::Individualcellundervoltage] = 3000;
  //Individual cell over temperature (external probe)
  mysettings.rulevalue[Rule::IndividualcellovertemperatureExternal] = 55;
  //Pack over voltage (mV)
  mysettings.rulevalue[Rule::IndividualcellundertemperatureExternal] = 5;
  //Pack under voltage (mV)
  mysettings.rulevalue[Rule::PackOverVoltage] = 4200 * 8;
  //RULE_PackUnderVoltage
  mysettings.rulevalue[Rule::PackUnderVoltage] = 3000 * 8;
  mysettings.rulevalue[Rule::Timer1] = 60 * 8;  //8am
  mysettings.rulevalue[Rule::Timer2] = 60 * 17; //5pm

  mysettings.rulevalue[Rule::ModuleOverTemperatureInternal] = 60;
  mysettings.rulevalue[Rule::ModuleUnderTemperatureInternal] = 50;

  for (size_t i = 0; i < RELAY_RULES; i++)
  {
    mysettings.rulehysteresis[i] = mysettings.rulevalue[i];

    //Set all relays to don't care
    for (size_t x = 0; x < RELAY_TOTAL; x++)
    {
      mysettings.rulerelaystate[i][x] = RELAY_X;
    }
  }

  for (size_t x = 0; x < RELAY_TOTAL; x++)
  {
    mysettings.relaytype[x] = RELAY_STANDARD;
  }
}

uint8_t lazyTimerMode = 0;
//Do activities which are not critical to the system like background loading of config, or updating timing results etc.
void timerLazyCallback()
{
  if (requestQueue.getRemainingCount() < 6)
  {
    //Exit here to avoid overflowing the queue
    SERIAL_DEBUG.print("ERR: Lazy overflow Q=");
    SERIAL_DEBUG.println(requestQueue.getRemainingCount());
    return;
  }

  lazyTimerMode++;

  if (lazyTimerMode == 1)
  {
    //Send a "ping" message through the cells to get a round trip time
    prg.sendTimingRequest();
    return;
  }

  if (lazyTimerMode == 2)
  {
    uint8_t counter = 0;
    //Find modules that don't have settings cached and request them
    for (uint8_t module = 0; module < TotalNumberOfCells(); module++)
    {
      if (cmi[module].valid && !cmi[module].settingsCached)
      {
        if (requestQueue.getRemainingCount() < 6)
        {
          //Exit here to avoid flooding the queue
          return;
        }

        prg.sendGetSettingsRequest(module);
        counter++;
      }
    }

    return;
  }

  //Send these requests to all banks of modules
  uint16_t i = 0;
  uint16_t max = TotalNumberOfCells();

  uint8_t startmodule = 0;

  while (i < max)
  {
    uint16_t endmodule = (startmodule + maximum_cell_modules_per_packet) - 1;

    //Limit to number of modules we have configured
    if (endmodule > max)
    {
      endmodule = max - 1;
    }

    switch (lazyTimerMode)
    {
    case 3:
      prg.sendReadBalanceCurrentCountRequest(startmodule, endmodule);
      break;

    case 4:
      prg.sendReadPacketsReceivedRequest(startmodule, endmodule);
      break;

    case 5:
      prg.sendReadBadPacketCounter(startmodule, endmodule);
      break;
    }

    //Move to the next bank
    startmodule = endmodule + 1;
    i += maximum_cell_modules_per_packet;
  }

  if (lazyTimerMode >= 5)
  {
    //This must go on the last action for the lazyTimerMode to reset to zero
    lazyTimerMode = 0;
  }
}

void resetAllRules()
{
  //Clear all rules
  for (int8_t r = 0; r < RELAY_RULES; r++)
  {
    rules.rule_outcome[r] = false;
  }
}

bool CaptureSerialInput(HardwareSerial stream, char *buffer, int buffersize, bool OnlyDigits, bool ShowPasswordChar)
{
  int length = 0;
  unsigned long timer = millis() + 30000;

  while (true)
  {

    //Abort after 30 seconds of inactivity
    if (millis() > timer)
      return false;

    //We should add a timeout in here, and return FALSE when we abort....
    while (stream.available())
    {
      //Reset timer on serial input
      timer = millis() + 30000;

      int data = stream.read();
      if (data == '\b' || data == '\177')
      { // BS and DEL
        if (length)
        {
          length--;
          stream.write("\b \b");
        }
      }
      else if (data == '\n')
      {
        //Ignore
      }
      else if (data == '\r')
      {
        if (length > 0)
        {
          stream.write("\r\n"); // output CRLF
          buffer[length] = '\0';

          //Soak up any other characters on the buffer and throw away
          while (stream.available())
          {
            stream.read();
          }

          //Return to caller
          return true;
        }

        length = 0;
      }
      else if (length < buffersize - 1)
      {
        if (OnlyDigits && (data < '0' || data > '9'))
        {
          //We need to filter out non-digit characters
        }
        else
        {
          buffer[length++] = data;
          if (ShowPasswordChar)
          {
            //Hide real character
            stream.write('*');
          }
          else
          {
            stream.write(data);
          }
        }
      }
    }
  }
}

void TerminalBasedWifiSetup(HardwareSerial stream)
{
  stream.println(F("\r\n\r\nDIYBMS CONTROLLER - Scanning Wifi"));

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  int n = WiFi.scanNetworks();

  if (n == 0)
    stream.println(F("no networks found"));
  else
  {
    for (int i = 0; i < n; ++i)
    {
      if (i < 10)
      {
        stream.print(' ');
      }
      stream.print(i);
      stream.print(':');
      stream.print(WiFi.SSID(i));

      //Pad out the wifi names into 2 columns
      for (size_t spaces = WiFi.SSID(i).length(); spaces < 36; spaces++)
      {
        stream.print(' ');
      }

      if ((i + 1) % 2 == 0)
      {
        stream.println();
      }
      delay(5);
    }
    stream.println();
  }

  WiFi.mode(WIFI_OFF);

  stream.print(F("Enter the NUMBER of the Wifi network to connect to:"));

  bool result;
  char buffer[10];
  result = CaptureSerialInput(stream, buffer, 10, true, false);
  if (result)
  {
    int index = String(buffer).toInt();
    stream.print(F("Enter the password to use when connecting to '"));
    stream.print(WiFi.SSID(index));
    stream.print("':");

    char passwordbuffer[80];
    result = CaptureSerialInput(stream, passwordbuffer, 80, false, true);

    if (result)
    {
      wifi_eeprom_settings config;
      memset(&config, 0, sizeof(config));
      WiFi.SSID(index).toCharArray(config.wifi_ssid, sizeof(config.wifi_ssid));
      strcpy(config.wifi_passphrase, passwordbuffer);
      Settings::WriteConfigToEEPROM((char *)&config, sizeof(config), EEPROM_WIFI_START_ADDRESS);
    }
  }

  stream.println(F("REBOOTING IN 5..."));
  delay(5000);
  ESP.restart();
}
void setup()
{
  WiFi.mode(WIFI_OFF);

  //Debug serial output

  //ESP8266 uses dedicated 2nd serial port, but transmit only
  SERIAL_DEBUG.begin(115200, SERIAL_8N1, SERIAL_TX_ONLY);
  SERIAL_DEBUG.setDebugOutput(true);

  //We generate a unique number which is used in all following JSON requests
  //we use this as a simple method to avoid cross site scripting attacks
  DIYBMSServer::generateUUID();

  SetControllerState(ControllerState::PowerUp);

  hal.ConfigurePins();

  hal.ConfigureI2C(ExternalInputInterrupt);

  //Pre configure the array
  memset(&cmi, 0, sizeof(cmi));
  for (size_t i = 0; i < maximum_controller_cell_modules; i++)
  {
    DIYBMSServer::clearModuleValues(i);
  }

  resetAllRules();

  // initialize LittleFS
  if (!LittleFS.begin())
  {
    SERIAL_DEBUG.println(F("An Error has occurred while mounting LittleFS"));
  }

  LoadConfiguration();

  //Force logging off for ESP8266
  mysettings.loggingEnabled = false;

  InputsEnabled = hal.InputsEnabled;
  OutputsEnabled = hal.OutputsEnabled;

  //Set relay defaults
  for (int8_t y = 0; y < RELAY_TOTAL; y++)
  {
    previousRelayState[y] = mysettings.rulerelaydefault[y];
    //Set relay defaults
    hal.SetOutputState(y, mysettings.rulerelaydefault[y]);
  }

  //Pretend the button is not pressed
  uint8_t clearAPSettings = 0xFF;
  //Fix for issue 5, delay for 3 seconds on power up with green LED lit so
  //people get chance to jump WIFI reset pin (d3)
  hal.GreenLedOn();

  SERIAL_DATA.begin(115200, SERIAL_8N1); // Serial for comms to modules

  //Allow user to press SPACE BAR key on serial terminal
  //to enter text based WIFI setup
  SERIAL_DATA.print(F("\r\n\r\n\r\nPress SPACE BAR to enter terminal based configuration...."));
  for (size_t i = 0; i < (3000 / 250); i++)
  {
    SERIAL_DATA.print('.');
    while (SERIAL_DATA.available())
    {
      int x = SERIAL_DATA.read();
      //SPACE BAR
      if (x == 32)
      {
        TerminalBasedWifiSetup(SERIAL_DATA);
      }
    }
    delay(250);
  }
  SERIAL_DATA.println(F("skipped"));
  SERIAL_DATA.flush();
  SERIAL_DATA.end();

  //This is normally pulled high, D3 is used to reset WIFI details
  clearAPSettings = digitalRead(RESET_WIFI_PIN);
  hal.GreenLedOff();

  SERIAL_DATA.begin(COMMS_BAUD_RATE, SERIAL_8N1); // Serial for comms to modules
  //Use alternative GPIO pins of D7/D8
  //D7 = GPIO13 = RECEIVE SERIAL
  //D8 = GPIO15 = TRANSMIT SERIAL
  SERIAL_DATA.swap();

  myPacketSerial.begin(&SERIAL_DATA, &onPacketReceived, sizeof(PacketStruct), SerialPacketReceiveBuffer, sizeof(SerialPacketReceiveBuffer));

  //Temporarly force WIFI settings
  //wifi_eeprom_settings xxxx;
  //strcpy(xxxx.wifi_ssid,"XXXXXX");
  //strcpy(xxxx.wifi_passphrase,"XXXXXX");
  //Settings::WriteConfigToEEPROM((char*)&xxxx, sizeof(xxxx), EEPROM_WIFI_START_ADDRESS);
  //clearAPSettings = 0;

  if (!DIYBMSSoftAP::LoadConfigFromEEPROM() || clearAPSettings == 0)
  {
    //We have just started...
    SetControllerState(ControllerState::ConfigurationSoftAP);

    SERIAL_DEBUG.print(F("Clear AP settings"));
    SERIAL_DEBUG.println(clearAPSettings);
    SERIAL_DEBUG.println(F("Setup Access Point"));
    //We are in initial power on mode (factory reset)
    DIYBMSSoftAP::SetupAccessPoint(&server);
  }
  else
  {

    //Config NTP
    NTP.onNTPSyncEvent([](NTPSyncEvent_t event) {
      ntpEvent = event;
      NTPsyncEventTriggered = true;
    });

    SERIAL_DEBUG.println(F("Connecting to WIFI"));

    /* Explicitly set the ESP8266 to be a WiFi-client, otherwise by default,
      would try to act as both a client and an access-point */

    wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
    wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);

    mqttClient.onConnect(onMqttConnect);
    mqttClient.onDisconnect(onMqttDisconnect);

    if (mysettings.mqtt_enabled)
    {
      SERIAL_DEBUG.println("MQTT Enabled");
      mqttClient.setServer(mysettings.mqtt_server, mysettings.mqtt_port);
      mqttClient.setCredentials(mysettings.mqtt_username, mysettings.mqtt_password);
    }

    //Ensure we service the cell modules every 5 or 10 seconds, depending on number of cells being serviced
    //slower stops the queues from overflowing when a lot of cells are being monitored
    myTimer.attach((TotalNumberOfCells() <= maximum_cell_modules_per_packet) ? 5 : 10, timerEnqueueCallback);

    //Process rules every 5 seconds
    myTimerRelay.attach(5, timerProcessRules);

    //We process the transmit queue every 1 second (this needs to be lower delay than the queue fills)
    //and slower than it takes a single module to process a command (about 200ms @ 2400baud)
    myTransmitTimer.attach(1, timerTransmitCallback);

    //Service reply queue
    myReplyTimer.attach(1, serviceReplyQueue);

    //This is a lazy timer for low priority tasks
    myLazyTimer.attach(8, timerLazyCallback);

    //We have just started...
    SetControllerState(ControllerState::Stabilizing);

    //Attempt connection in setup(), loop() will also try every 30 seconds
    connectToWifi();
  }
}

unsigned long wifitimer = 0;

void loop()
{
  //ESP_LOGW("LOOP","LOOP");
  unsigned long currentMillis = millis();

  if (ControlState != ControllerState::ConfigurationSoftAP)
  {
    //on first pass wifitimer is zero
    if (currentMillis - wifitimer > 30000)
    {
      //Attempt to connect to WiFi every 30 seconds, this caters for when WiFi drops
      //such as AP reboot, its written to return without action if we are already connected
      connectToWifi();
      wifitimer = currentMillis;
    }
  }

  ArduinoOTA.handle();

  // Call update to receive, decode and process incoming packets.
  myPacketSerial.checkInputStream();

  if (NTPsyncEventTriggered)
  {
    processSyncEvent(ntpEvent);
    NTPsyncEventTriggered = false;
  }

  MDNS.update();
}
