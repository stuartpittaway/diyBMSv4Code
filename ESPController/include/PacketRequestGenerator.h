#ifndef PacketRequestGenerator_H_
#define PacketRequestGenerator_H_

#include <Arduino.h>
#include <cppQueue.h>
#include <defines.h>


//command byte
// WRRR CCCC
// W    = 1 bit indicator packet was processed (controller send (0) module processed (1))
// R    = 3 bits reserved not used
// C    = 4 bits command (16 possible commands)

//commands
// 1000 0000  = set bank identity
// 0000 0001  = read voltage and status
// 0000 0010  = identify module (flash leds)
// 0000 0011  = Read temperature
// 0000 0100  = Report number of bad packets
// 0000 0101  = Report settings/configuration


class PacketRequestGenerator {
   public:
     PacketRequestGenerator(Queue* requestQ) {_requestq=requestQ;}
     ~PacketRequestGenerator() {}
     void sendGetSettingsRequest(uint8_t b,uint8_t m);
     void sendIdentifyModuleRequest(uint8_t b,uint8_t m);
     void sendSaveSetting(uint8_t b,uint8_t m,uint16_t BypassThresholdmV,uint8_t BypassOverTempShutdown,float LoadResistance,float Calibration,float mVPerADC,uint16_t Internal_BCoefficient,uint16_t External_BCoefficient);

     void sendSaveGlobalSetting(uint8_t numberOfBanks,uint16_t BypassThresholdmV,uint8_t BypassOverTempShutdown);
     void sendReadBadPacketCounter(uint8_t b);

     void sendCellVoltageRequest(uint8_t b);
     void sendCellTemperatureRequest(uint8_t b);
     void sendMoveToBank(uint8_t b,uint8_t m,uint8_t movetobank);
     void sendReadBalancePowerRequest(uint8_t b);

     uint32_t packetsGenerated = 0;

  private:
    Queue* _requestq;
    packet _packetbuffer;
    void pushPacketToQueue();
    void setPacketAddress(bool broadcast,uint8_t bank,uint8_t module);
    void clearmoduledata();
    void clearSettingsForAllModules();
};



#endif
