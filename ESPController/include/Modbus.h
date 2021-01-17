
#ifndef DIYBMS_MODBUS_H_
#define DIYBMS_MODBUS_H_

#define MODBUS_NUM 10
#define MODBUS_NAME_LEN 20
#define MODBUS_UNIT_LEN 10
#define MODBUS_DESC_LEN 40

#define MODBUS_FILE_NAME "/modbus.json"

#define MB_READ_REGISTER 0x04

#define TYPE_BIT 0
#define TYPE_BYTE 1
#define TYPE_WORD 2
#define TYPE_LONG 3
#define TYPE_FLOAT 4

struct ModbusInfo
{
  uint8_t type;
  uint8_t addr;
  uint16_t op;
  uint32_t readInt;
  uint32_t sendInt;
  uint16_t reg;
  bool rule;
  bool mqtt;
  bool influx;
  char name[MODBUS_NAME_LEN];
  char unit[MODBUS_UNIT_LEN];
  char desc[MODBUS_DESC_LEN];
};

struct ModbusVal
{
  volatile float val;
  volatile uint32_t lastRead;
};

// I needed bring it in again - tbd if modbus modules should be separated or go to main
extern ModbusInfo ModBus[MODBUS_NUM];
extern ModbusVal ModBusVal[MODBUS_NUM];

extern uint8_t ModbusNum;

void setupModbus(uint16_t baud, int8_t rx, int8_t tx, int8_t dere);
void initModbus();
int readModbusVal(uint8_t ind);
uint16_t calculateCRC(uint8_t *buf, uint8_t len);


#endif
