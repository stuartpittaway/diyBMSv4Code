#include <Arduino.h>

#include "defines.h"

#include <SDM.h>
#include "ModBus.h"

ModbusInfo ModBus[MODBUS_NUM] = {
  {71, MB_READ_REGISTER, 0.00, SDM_TOTAL_ACTIVE_ENERGY, "BAT_IN_E",  "kWh",  "Powersupply Energy"},
  {71, MB_READ_REGISTER, 0.00, SDM_PHASE_1_POWER,       "BAT_IN_P",  "W",    "Powersupply Power"},
  {71, MB_READ_REGISTER, 0.00, SDM_PHASE_1_VOLTAGE,     "BAT_IN_U",  "V",    "Powersupply Voltage"},
  {71, MB_READ_REGISTER, 0.00, SDM_PHASE_1_CURRENT,     "BAT_IN_I",  "A",    "Powersupply Current"},
  {71, MB_READ_REGISTER, 0.00, SDM_FREQUENCY,           "BAT_IN_F",  "Hz",   "Powersupply Frequency"},
  {72, MB_READ_REGISTER, 0.00, SDM_TOTAL_ACTIVE_ENERGY, "BAT_OUT_E", "kWh",  "Powerwall AC Energy"},
  {72, MB_READ_REGISTER, 0.00, SDM_PHASE_1_POWER,       "BAT_OUT_P", "W",    "Powerwall AC Power"},
  {72, MB_READ_REGISTER, 0.00, SDM_PHASE_1_VOLTAGE,     "BAT_OUT_U", "V",    "Powerwall AC Voltage"},
  {72, MB_READ_REGISTER, 0.00, SDM_PHASE_1_CURRENT,     "BAT_OUT_I", "A",    "Powerwall AC Current"},
  {72, MB_READ_REGISTER, 0.00, SDM_FREQUENCY,           "BAT_OUT_F", "Hz",   "Powerwall AC Freqency"}
};


static uint8_t modBusInd = 0;

SDM sdm(SERIAL_MODBUS, 9600, NOT_A_PIN, SERIAL_8N1);


void InitModbus() {
  sdm.begin();                                                                  //initialize SDM communication
  delay(1000);                                                                  //wait a while before next loop

}

void ReadModbus () {

  if(MODBUS_NUM){
    ModBus[modBusInd].val = sdm.readVal(ModBus[modBusInd].reg, ModBus[modBusInd].addr);

    SERIAL_DEBUG.printf("Read Modbus: %f\n", ModBus[modBusInd].val);

    if(++modBusInd >= MODBUS_NUM)
      modBusInd = 0;


  }

}
