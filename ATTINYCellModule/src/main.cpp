/*
 ____  ____  _  _  ____  __  __  ___    _  _  __
(  _ \(_  _)( \/ )(  _ \(  \/  )/ __)  ( \/ )/. |
 )(_) )_)(_  \  /  ) _ < )    ( \__ \   \  /(_  _)
(____/(____) (__) (____/(_/\/\_)(___/    \/   (_)

DIYBMS V4.0
CELL MODULE FOR ATTINY841

(c)2019 Stuart Pittaway

COMPILE THIS CODE USING PLATFORM.IO

LICENSE
Attribution-NonCommercial-ShareAlike 2.0 UK: England & Wales (CC BY-NC-SA 2.0 UK)
https://creativecommons.org/licenses/by-nc-sa/2.0/uk/

* Non-Commercial — You may not use the material for commercial purposes.
* Attribution — You must give appropriate credit, provide a link to the license, and indicate if changes were made.
  You may do so in any reasonable manner, but not in any way that suggests the licensor endorses you or your use.
* ShareAlike — If you remix, transform, or build upon the material, you must distribute your
  contributions under the same license as the original.
* No additional restrictions — You may not apply legal terms or technological measures
  that legally restrict others from doing anything the license permits.
*/
/*
IMPORTANT

You need to configure the correct DIYBMSMODULEVERSION in defines.h file to build for your module

*/
#include <Arduino.h>

#if !(F_CPU == 8000000)
#error Processor speed should be 8 Mhz internal
#endif

//An Arduino Library that facilitates packet-based serial communication using COBS or SLIP encoding.
//https://github.com/bakercp/PacketSerial
#include <PacketSerial.h>

#define framingmarker (uint8_t)0x00

//Consistant byte stuffing mode
//Use ZERO for packet header/terminator
//64 byte buffer
PacketSerial_<COBS, framingmarker, 64> myPacketSerial;

//Our project code includes
#include "defines.h"
#include "settings.h"
#include <FastPID.h>

#include "diybms_attiny841.h"
#include "packet_processor.h"

//Default values which get overwritten by EEPROM on power up
CellModuleConfig myConfig;

DiyBMSATTiny841 hardware;

PacketProcessor PP(&hardware, &myConfig);

volatile bool wdt_triggered = false;
uint16_t bypassCountDown = 0;
uint8_t bypassHasJustFinished = 0;
bool pwmrunning = false;

void DefaultConfig()
{


bool packetValidForMe = false;
bool packetRecieved = false;
#define WAIT_PACKET 200  //miliseconds after sleep to wait packetSerial.update();
uint32_t waitUpdateStart;

void DefaultConfig() {

  // Value of the resistor for load shedding

#if (defined(DIYBMSMODULEVERSION) && DIYBMSMODULEVERSION == 410)
  myConfig.LoadResistance = 4.00;
#elif defined(DIYBMSMODULEVERSION) && (DIYBMSMODULEVERSION >= 421)
  myConfig.LoadResistance = 4.96;
#else
  //For v4.00
  myConfig.LoadResistance = 4.40;
#endif

  //Default bank zero
  myConfig.mybank = 0;

  //About 2.2100 seems about right
  myConfig.Calibration = 2.21000;

  //2mV per ADC resolution
  myConfig.mVPerADC = 2.0; //2048.0/1024.0;

#if defined(DIYBMSMODULEVERSION) && (DIYBMSMODULEVERSION == 420 && !defined(SWAPR19R20))
  //Keep temperature low for modules with R19 and R20 not swapped
  myConfig.BypassOverTempShutdown = 45;
#else
  //Stop running bypass if temperature over 70 degrees C
  myConfig.BypassOverTempShutdown = 70;
#endif

  //Start bypass at 4.1V
  myConfig.BypassThresholdmV = 4100;

#if defined(DIYBMSMODULEVERSION) && (DIYBMSMODULEVERSION == 430 || DIYBMSMODULEVERSION == 420 || DIYBMSMODULEVERSION == 421)
  //Murata Electronics NCP18WB473J03RB = 47K ±5% 4050K ±2% 100mW 0603 NTC Thermistors RoHS
  myConfig.Internal_BCoefficient = 4050;
#else
  //4150 = B constant (25-50℃)
  myConfig.Internal_BCoefficient = 4150;
#endif

  //4150 = B constant (25-50℃)
  myConfig.External_BCoefficient = 4150;

  // Resistance @ 25℃ = 47k, B Constant 4150, 0.20mA max current
  //Using https://www.thinksrs.com/downloads/programs/therm%20calc/ntccalibrator/ntccalculator.html
}

ISR(WDT_vect)
{
  //This is the watchdog timer - something went wrong and no activity recieved in a while
  wdt_triggered = true;
  PP.IncrementWatchdogCounter();
}

ISR(ADC_vect)
{
  // when ADC completed, take an interrupt and process result
  PP.ADCReading(hardware.ReadADC());
}


void onPacketReceived(const uint8_t* receivebuffer, size_t len) {
  if (len > 0) {
    packetRecieved = true;
    //A data packet has just arrived, check it. Save if it is valid to switch on green led.
    packetValidForMe = PP.isValidPacketForMe(receivebuffer, len);

  }
}

ISR(USART0_START_vect) {
  //Needs to be here!
  asm("NOP");
}

//Kp: Determines how aggressively the PID reacts to the current amount of error (Proportional)
//Ki: Determines how aggressively the PID reacts to error over time (Integral)
//Kd: Determines how aggressively the PID reacts to the change in error (Derivative)

//6Hz rate - number of times we call this code in Loop
//Kp, Ki, Kd, Hz, output_bits, output_signed);
//Settings for V4.00 boards with 2R2 resistors = (4.0, 0.5, 0.2, 6, 8, false);
FastPID myPID(4.0, 0.5, 0.2, 6, 8, false);

void setup()
{
  //Must be first line of code
  wdt_disable();
  wdt_reset();

  //8 second between watchdogs
  hardware.SetWatchdog8sec();

  //Setup IO ports
  hardware.ConfigurePorts();

  //More power saving changes
  hardware.EnableSerial0();

  hardware.DisableSerial1();

  //Check if setup routine needs to be run
  if (!Settings::ReadConfigFromEEPROM((uint8_t *)&myConfig, sizeof(myConfig), EEPROM_CONFIG_ADDRESS))
  {
    DefaultConfig();
    //Save settings
    Settings::WriteConfigToEEPROM((uint8_t *)&myConfig, sizeof(myConfig), EEPROM_CONFIG_ADDRESS);
  }

#if defined(DIYBMSMODULEVERSION) && (DIYBMSMODULEVERSION == 420 && !defined(SWAPR19R20))
  //Keep temperature low for modules with R19 and R20 not swapped
  if (myConfig.BypassOverTempShutdown > 45)
  {
    myConfig.BypassOverTempShutdown = 45;
  }
#endif

  hardware.double_tap_green_led();

#if defined(DIYBMSMODULEVERSION) && DIYBMSMODULEVERSION < 430
  hardware.double_tap_blue_led();
#endif

  //The PID can vary between 0 and 100
  myPID.setOutputRange(0, 100);

  //Set up data handler
  Serial.begin(COMMS_BAUD_RATE, SERIAL_8N1);

  myPacketSerial.setStream(&Serial);
  myPacketSerial.setPacketHandler(&onPacketReceived);
}

void loop()
{
  //This loop runs around 3 times per second when the module is in bypass

  wdt_reset();

  //if (hztiming) {  hardware.SparePinOn();} else {  hardware.SparePinOff();}hztiming=!hztiming;

  if (PP.identifyModule > 0)
  {
    hardware.GreenLedOn();
    PP.identifyModule--;

    if (PP.identifyModule == 0)
    {
      hardware.GreenLedOff();
    }
  }

  //#ifndef DIYBMS_DEBUG
  if (!PP.WeAreInBypass && bypassHasJustFinished == 0)
  {
    //We don't sleep if we are in bypass mode or just after completing bypass
    hardware.EnableStartFrameDetection();

    //Program stops here until woken by watchdog or pin change interrupt
    hardware.Sleep();
  }
  //#endif

  //We are awake....

  if (wdt_triggered)
  {
#if defined(DIYBMSMODULEVERSION) && DIYBMSMODULEVERSION < 430
    //Flash blue LED twice after a watchdog wake up
    hardware.double_tap_blue_led();
#else
    //Flash green LED twice after a watchdog wake up
    hardware.double_tap_green_led();
#endif
    //We got here because the watchdog (after 8 seconds) went off - we didnt receive a packet of data
    wdt_triggered = false;    
  } else {
    //We have wake up because of UART
    //Loop to process packet until WAIT_PACKET time or packetRecieved. I will recieve ONLY one packet as Wemos sends them at 0.5 seconds rate  
    waitUpdateStart = millis();
    while ((millis() - waitUpdateStart < WAIT_PACKET) && !packetRecieved) {
        myPacketSerial.update();
    }
  }

  //We always take a voltage and temperature reading on every loop cycle to check if we need to go into bypass
  //this is also triggered by the watchdog should comms fail or the module is running standalone
  //Probably over kill to do it this frequently
  hardware.ReferenceVoltageOn();

  //allow reference voltage to stabalize
  delay(4);

  PP.TakeAnAnalogueReading(ADC_CELL_VOLTAGE);
  //Internal temperature
  PP.TakeAnAnalogueReading(ADC_INTERNAL_TEMP);
  //External temperature
  PP.TakeAnAnalogueReading(ADC_EXTERNAL_TEMP);

  hardware.ReferenceVoltageOff();

  if (PP.BypassCheck())
  {
    //Our cell voltage is OVER the setpoint limit, start draining cell using load bypass resistor

    if (!PP.WeAreInBypass)
    {
      //We have just entered the bypass code
      PP.WeAreInBypass = true;

      //This controls how many cycles of loop() we make before re-checking the situation
      //about every 30 seconds
      bypassCountDown = 200;
    }
  }

  if (bypassCountDown > 0)
  {

    uint8_t exttemp = PP.InternalTemperature() & 0xFF;

    if (exttemp < (myConfig.BypassOverTempShutdown - 10))
    {
      //Full power if we are nowhere near the setpoint (more than 10 degrees C away)
      hardware.DumpLoadOn();
      hardware.StopTimer2();
      PP.PWMValue = 100;
      pwmrunning = false;
    }
    else
    {
      if (!pwmrunning)
      {
        //We have approached the set point, enable PWM
        hardware.DumpLoadOff();
        //Start timer2 with zero value
        hardware.StartTimer2();
        pwmrunning = true;
        //myPID.clear();
      }

      //Compare the real temperature against max setpoint, we want the PID to keep at this temperature
      PP.PWMValue = myPID.step(myConfig.BypassOverTempShutdown, exttemp);

      //Scale PWM up to 0-10000
      hardware.SetTimer2Value(PP.PWMValue * 100);
    }

    bypassCountDown--;

    if (bypassCountDown == 0)
    {
      //Switch everything off for this cycle
      pwmrunning = false;

      PP.WeAreInBypass = false;

      //myPID.clear();
      hardware.StopTimer2();

      //switch off
      hardware.DumpLoadOff();

      //On the next iteration of loop, don't sleep so we are forced to take another
      //cell voltage reading without the bypass being enabled, and we can then
      //evaludate if we need to stay in bypass mode, we do this a few times
      //as the cell has a tendancy to float back up in voltage once load resistor is removed
      bypassHasJustFinished = 200;
    }
  }


  if (packetRecieved) {
    if (packetValidForMe) {
      //Before send the packet, we have to process it first.
      if (PP.preparePacketToSend()) {
        //Only light green if packet is good
        hardware.GreenLedOn();          
      }
      packetValidForMe = false;
    }   
    hardware.EnableSerial0TX();

    //Wake up the connected cell module from sleep, send a framingmarker
    //byte which the receiver will ignore
    Serial.write(framingmarker);
    //Let connected module wake up
    hardware.FlushSerial0();
    delay(3); //As we do in wemos code.


    //Send the packet (even if it was invalid so controller can count crc errors)
    myPacketSerial.send(PP.GetBufferPointer(), PP.GetBufferSize());

    //DEBUG: Are there any known issues with Serial Flush causing a CPU to hang?
    hardware.FlushSerial0();

    //Replace flush with a simple delay - we have 35+ bytes to transmit at 2400 baud + COBS encoding
    //delay(10);

    //At 2400bits per second, = 300 bytes per second = 1000ms/300bytes/sec= 3ms per byte

    hardware.DisableSerial0TX();
    packetRecieved = false;
  }


  hardware.GreenLedOff();

  if (bypassHasJustFinished > 0) {

    bypassHasJustFinished--;
  }
}
