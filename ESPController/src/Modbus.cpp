#include <Arduino.h>
#include "defines.h"
#include "Modbus.h"
#include <ArduinoJson.h>

#define REQ_LEN 8
#define MAX_RES_LEN 40
#define MIN_RES_LEN 9
#define MODBUS_TIMEOUT 100

byte res[MAX_RES_LEN];

struct modbusReqT {
  uint8_t addr;
  uint8_t func;
  uint16_t reg;
  uint16_t len;
  uint16_t crc;
};

int8_t RS485dere = NOT_A_PIN;

void setupModbus(uint16_t baud, int8_t rx, int8_t tx, int8_t dere) {

  RS485dere = dere;

  SERIAL_RS485.begin(baud, SERIAL_8N1, rx, tx); // pins for DIYBMS => RX pin 16, TX pin 17

  if(RS485dere != NOT_A_PIN) {
    pinMode(RS485dere, OUTPUT);
    digitalWrite(RS485dere, LOW);
  }
}

int receiveModbus(byte *buf) {

uint8_t len = 0;
uint32_t now = millis();
uint32_t ts = now;

  while(now - ts < 5) {

    if(SERIAL_RS485.available()) {
      ts = millis();
      byte c = SERIAL_RS485.read();
      //ESP_LOGD(TAG, " %d", c);
      if(len < MAX_RES_LEN) buf[len++] = c;
    }

    now = millis();
  }

//  if(len) ESP_LOGD(TAG, "Len %d", len);
  return len;
}


//int sendModbus(byte* buf, uint8_t len) {
int sendModbus(uint8_t addr, uint8_t func, uint16_t reg, uint16_t datalen) {

  uint8_t req[REQ_LEN];
  uint16_t len = datalen+6;
  uint16_t crc;

  while(SERIAL_RS485.available()) SERIAL_RS485.read();    // clear buffer

  if(RS485dere != NOT_A_PIN)
    digitalWrite(RS485dere, HIGH);

  req[0] = addr;
  req[1] = func;
  req[2] = reg >> 8;
  req[3] = reg & 0xff;
  req[4] = datalen >> 8;
  req[5] = datalen & 0xff;
  crc = calculateCRC(req, len - 2);
  req[7] = crc >> 8;
  req[6] = crc & 0xff;

//  ESP_LOGD(TAG, "SendBuf: %d %d %d %d %d %d %d %d ", req[0], req[1], req[2], req[3], req[4], req[5], req[6], req[7]);
//  ESP_LOGD(TAG, "SendModbus: Addr %d  Func  %d  Reg %d  Len %d  CRC %d ", addr, func, reg, len, crc);

  byte* bp = (byte*) &req;
  for(int i = 0; i<len; i++) {
    SERIAL_RS485.write(bp[i]);
  }

    SERIAL_RS485.flush();

    delay(1);

    if(RS485dere != NOT_A_PIN)
      digitalWrite(RS485dere, LOW);

    return 0;
}


uint16_t calculateCRC(uint8_t *buf, uint8_t len) {
  uint16_t _crc, _flag;
  _crc = 0xFFFF;
  for (uint8_t i = 0; i < len; i++) {
    _crc = _crc ^ buf[i];
    for (uint8_t j = 8; j; j--) {
      _flag = _crc & 0x0001;
      _crc >>= 1;
      if (_flag)
        _crc ^= 0xA001;
    }
  }
  return _crc;
}


int readModbusVal(uint8_t ind) {

  uint32_t now;
  uint32_t ts;
  uint8_t len;
  float valf;
  uint32_t val;

  sendModbus(ModBus[ind].addr, MB_READ_REGISTER, ModBus[ind].reg, 2);

    ts = now = millis();
    while(((now - ts) < MODBUS_TIMEOUT) && ((len = receiveModbus(res)) < MIN_RES_LEN))
      now = millis();

    //ESP_LOGD(TAG, "readVal: Len %d  Addr %d  Func %d  Len %d", len, res[0], res[1], res[2]);

    if(len >= MIN_RES_LEN) {
      //ESP_LOGD(TAG, "readVal: Ind %d  Addr %d  Func %d  Len %d", ind, res[0], res[1], res[2]);

      switch(ModBus[ind].type) {

        case TYPE_BYTE:                   // not tested
          val = (uint8_t) *(res+3);
          ModBusVal[ind].val = val;
          ModBusVal[ind].lastRead = now;
//          ESP_LOGD(TAG, "readVal: Val %d ", val);
          break;

        case TYPE_WORD:                   // not tested
          val = (uint16_t) *(res+3);
          ModBusVal[ind].val = val;
          ModBusVal[ind].lastRead = now;
//          ESP_LOGD(TAG, "readVal: Val %d ", val);
          break;

          case TYPE_LONG:                   // not tested
            val = (uint32_t) *(res+3);
            ModBusVal[ind].val = val;
            ModBusVal[ind].lastRead = now;
//            ESP_LOGD(TAG, "readVal: Val %ld ", val);
            break;

        case TYPE_FLOAT:
          ((uint8_t*)&valf)[3]= res[3];
          ((uint8_t*)&valf)[2]= res[4];
          ((uint8_t*)&valf)[1]= res[5];
          ((uint8_t*)&valf)[0]= res[6];
          ModBusVal[ind].val = valf;
          ModBusVal[ind].lastRead = now;
//          ESP_LOGD(TAG, "readVal: Float Val %f ", valf);
          break;
      }
//      SerialUSB.print(now); SerialUSB.print(" . "); SerialUSB.print(items[readChan].name); SerialUSB.print(":"); SerialUSB.println(items[readChan].value);
    }
    return 0;
}
