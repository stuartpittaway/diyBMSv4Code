#include <SDM.h>

#define MODBUS_NUM   10
#define SERIAL_MODBUS Serial1                                                   //number of sdm registers to read

#define MB_READ_REGISTER 0x04

struct ModbusInfo
{
  const uint8_t addr;
  const uint16_t op;
  volatile float val;
  const uint16_t reg;
  const char* name;
  const char* unit;
  const char* desc;
};

extern ModbusInfo ModBus[MODBUS_NUM];

void InitModbus();
void ReadModbus();
