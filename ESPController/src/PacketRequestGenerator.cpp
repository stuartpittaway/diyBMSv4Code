#include "PacketRequestGenerator.h"

void PacketRequestGenerator::clearSettingsForAllModules()
{
  // Force refresh of settings
  for (size_t i = 0; i < maximum_cell_modules; i++)
  {
    cmi[0][i].settingsCached = false;
    cmi[1][i].settingsCached = false;
    cmi[2][i].settingsCached = false;
    cmi[3][i].settingsCached = false;
  }
}

void PacketRequestGenerator::sendMoveToBank(uint8_t b, uint8_t m, uint8_t movetobank)
{
  setPacketAddress(false, b, m);

  //Command - SetBankIdentity
  _packetbuffer.command = COMMAND::SetBankIdentity;

  clearmoduledata();

  _packetbuffer.moduledata[m] = movetobank & 0x03;

  pushPacketToQueue();

  clearSettingsForAllModules();
}

void PacketRequestGenerator::sendSaveGlobalSetting(uint8_t totalNumberOfBanks, uint16_t BypassThresholdmV, uint8_t BypassOverTempShutdown)
{
  for (uint8_t bank = 0; bank < totalNumberOfBanks; bank++)
  {
    //Ask all modules to set bypass and temperature value
    setPacketAddress(true, bank, 0);
    //Command - WriteSettings
    _packetbuffer.command = COMMAND::WriteSettings;

    //Fill packet with 0xFFFF values - module ignores settings with this value
    for (uint8_t a = 0; a < maximum_cell_modules; a++)
    {
      _packetbuffer.moduledata[a] = 0xFFFF;
    }
    _packetbuffer.moduledata[6] = BypassOverTempShutdown;
    _packetbuffer.moduledata[7] = BypassThresholdmV;
    pushPacketToQueue();
  }

  clearSettingsForAllModules();
}
void PacketRequestGenerator::sendSaveSetting(uint8_t b, uint8_t m, uint16_t BypassThresholdmV, uint8_t BypassOverTempShutdown, float LoadResistance, float Calibration, float mVPerADC, uint16_t Internal_BCoefficient, uint16_t External_BCoefficient)
{
  setPacketAddress(false, b, m);
  //Command - WriteSettings
  _packetbuffer.command = COMMAND::WriteSettings;

  //Fill packet with 0xFFFF values - module ignores settings
  //with this value
  for (int a = 0; a < maximum_cell_modules; a++)
  {
    _packetbuffer.moduledata[a] = 0xFFFF;
  }

  // Force refresh of settings
  cmi[b][m].settingsCached = false;

  FLOATUNION_t myFloat;

  myFloat.number = LoadResistance;
  _packetbuffer.moduledata[0] = myFloat.word[0];
  _packetbuffer.moduledata[1] = myFloat.word[1];

  // Arduino float(4 byte)
  myFloat.number = Calibration;
  _packetbuffer.moduledata[2] = myFloat.word[0];
  _packetbuffer.moduledata[3] = myFloat.word[1];

  // Arduino float(4 byte)
  myFloat.number = mVPerADC;
  _packetbuffer.moduledata[4] = myFloat.word[0];
  _packetbuffer.moduledata[5] = myFloat.word[1];

  _packetbuffer.moduledata[6] = BypassOverTempShutdown;
  _packetbuffer.moduledata[7] = BypassThresholdmV;
  _packetbuffer.moduledata[8] = Internal_BCoefficient;
  _packetbuffer.moduledata[9] = External_BCoefficient;

  pushPacketToQueue();
}

void PacketRequestGenerator::sendReadBadPacketCounter(uint8_t b)
{
  //Read bad packet count (broadcast) to bank
  setPacketAddress(true, b, 0);

  _packetbuffer.command = COMMAND::ReadBadPacketCounter;

  //AVR MCUs are little endian (least significant byte first in memory)
  clearmoduledata();

  pushPacketToQueue();
}

void PacketRequestGenerator::sendCellVoltageRequest(uint8_t b)
{
  //Read voltage (broadcast) to bank
  setPacketAddress(true, b, 0);

  //Command 1 - read voltage
  _packetbuffer.command = COMMAND::ReadVoltageAndStatus;

  //AVR MCUs are little endian (least significant byte first in memory)
  clearmoduledata();

  pushPacketToQueue();
}

void PacketRequestGenerator::sendIdentifyModuleRequest(uint8_t b, uint8_t m)
{
  //Read settings from single module
  setPacketAddress(false, b, m);

  //Command 3 - identify
  _packetbuffer.command = COMMAND::Identify;

  clearmoduledata();

  pushPacketToQueue();
}

void PacketRequestGenerator::sendGetSettingsRequest(uint8_t b, uint8_t m)
{
  //SERIAL_DEBUG.println("sendGetSettingsRequest");

  //Read settings from single module
  setPacketAddress(false, b, m);
  //Command 5 - read settings
  _packetbuffer.command = COMMAND::ReadSettings;

  clearmoduledata();

  pushPacketToQueue();
}

void PacketRequestGenerator::sendCellTemperatureRequest(uint8_t b)
{
  //Read voltage (broadcast) to bank
  setPacketAddress(true, b, 0);
  //Command 3 - read temperatures
  _packetbuffer.command = COMMAND::ReadTemperature;
  clearmoduledata();
  pushPacketToQueue();
}

void PacketRequestGenerator::sendReadBalancePowerRequest(uint8_t b)
{
  //Read PWM value
  setPacketAddress(true, b, 0);
  //Command 7 - read PWM
  _packetbuffer.command = COMMAND::ReadBalancePowerPWM;
  clearmoduledata();
  pushPacketToQueue();
}

void PacketRequestGenerator::pushPacketToQueue()
{
  _requestq->push(&_packetbuffer);
  packetsGenerated++;
}

void PacketRequestGenerator::setPacketAddress(bool broadcast, uint8_t bank, uint8_t module)
{
  if (broadcast)
  {
    //12 is a debug value to pretend we have more modules than really exist
    _packetbuffer.address = (B10000000 | ((bank & B00000011) << 4)); // + (12 & B00001111) ;
  }
  else
  {
    _packetbuffer.address = ((bank & B00000011) << 4) + (module & B00001111);
  }
}

void PacketRequestGenerator::clearmoduledata()
{
  //todo replace with memset/memclr
  for (int a = 0; a < maximum_cell_modules; a++)
  {
    _packetbuffer.moduledata[a] = 0;
  }
}
