/**************************************************************************/
/* 
This is an Arduino library for SHT21 & HTU21D Digital Humidity & Temperature Sensor

Written by enjoyneering79

These sensor uses I2C to communicate, 2 pins are required to interface

Connect HTU21D to pins :  SDA  SCL
Uno, Redboard, Pro:       A4   A5
Mega2560, Due:            20   21
Leonardo:                 2    3

BSD license, all text above must be included in any redistribution
*/
/**************************************************************************/

#include <Wire.h>
#include "HTU21D.h"

/*
HTU21D(resolution)

resolution:
HTU21D_RES_RH12_TEMP14 - RH: 12Bit, measuring time 16ms. Temperature: 14Bit, measuring time 50ms
HTU21D_RES_RH8_TEMP12  - RH: 8Bit,  measuring time 8ms.  Temperature: 12Bit, measuring time 25ms
HTU21D_RES_RH10_TEMP13 - RH: 10Bit, measuring time 5ms.  Temperature: 13Bit, measuring time 13ms.
HTU21D_RES_RH11_TEMP11 - RH: 11Bit, measuring time 3ms.  Temperature: 11Bit, measuring time 7ms.


DEFAULT
HTU21D(HTU21D_RES_RH12_TEMP14)
*/

HTU21D myHTU21D;

void setup()
{
  Serial.begin(115200);
  Serial.println(F(""));
  
  while (myHTU21D.begin() != true)
  {
    Serial.println(F("HTU21D sensor is not present"));
    delay(5000);
  }
  
  Serial.println(F("HTU21D sensor is present"));
}

void loop()
{
  Serial.println(F(""));
  Serial.println(F("<<DEMO: Default settings, %RH: 12Bit, Temperature - 14Bit>>"));
  
  Serial.println(F(""));
  Serial.print(F("Humidity: "));
  Serial.print(myHTU21D.readHumidity());
  Serial.println(F(" +-2%RH     in range 20%RH - 80%RH at 25deg.C only"));
  
  Serial.println(F(""));
  Serial.print(F("Compensated Humidity: "));
  Serial.print(myHTU21D.readCompensatedHumidity());
  Serial.println(F(" +-2%RH     in range 0%RH - 100%RH at tmp. range 0deg.C - 80deg.C"));

  Serial.println(F(""));
  Serial.print(F("Temperature: "));
  Serial.print(myHTU21D.readTemperature());
  Serial.println(F(" +-0.5 deg.C"));
  
  Serial.println(F(""));
  Serial.println(F(""));
  Serial.println(F("<<DEMO: %RH: 11Bit, Temperature - 11Bit>>"));
  myHTU21D.setResolution(HTU21D_RES_RH11_TEMP11);
  
  Serial.println(F(""));
  Serial.print(F("Humidity: "));
  Serial.print(myHTU21D.readHumidity());
  Serial.println(F(" +-2%RH     in range 20%RH - 80%RH at 25deg.C only"));
  
  Serial.println(F(""));
  Serial.print(F("Compensated Humidity: "));
  Serial.print(myHTU21D.readCompensatedHumidity());
  Serial.println(F(" +-2%RH     in range 0%RH - 100%RH at tmp. range 0deg.C - 80deg.C"));
  
  Serial.println(F(""));
  Serial.print(F("Temperature: "));
  Serial.print(myHTU21D.readTemperature());
  Serial.println(F(" +-0.5 deg.C"));
    
  
  Serial.println(F(""));
  Serial.println(F(""));
  Serial.println(F("<<DEMO: Battery Status>>"));
  
  if (myHTU21D.batteryStatus() == true)
  {
    Serial.println("Battery OK. Level > 2.25v");
  }
  else
  {
    Serial.println("Battery LOW. Level < 2.25v");
  }

/* UNCOMENT FOR SENSOR'S FUNCTIONALITY DIAGNOSTIC ONLY. TEMPERATURE SHOULD INCREASE BY 0.5-1.5 deg.C and HUMIDITY DROP
  Serial.println(F(""));
  Serial.println(F(""));
  Serial.println(F("<<DEMO: built-in Heater test>>"));
  
  Serial.println(F(""));
  Serial.println(F("<<built-in Heater OFF>>"));
  myHTU21D.setHeater(OFF);
  
  Serial.println(F(""));
  Serial.print(F("Compensated Humidity: "));
  Serial.print(myHTU21D.readCompensatedHumidity());
  Serial.println(F(" %RH"));
  
  Serial.println(F(""));
  Serial.print(F("Temperature: "));
  Serial.print(myHTU21D.readTemperature());
  Serial.println(F(" deg.C"));
  
  Serial.println(F(""));
  Serial.println(F("<<built-in Heater ON. Wait for 5 sec.>>"));
  myHTU21D.setHeater(ON);
  delay(5000);
  
  Serial.println(F(""));
  Serial.println(F(""));
  Serial.print(F("Compensated Humidity: "));
  Serial.print(myHTU21D.readCompensatedHumidity());
  Serial.println(F(" %RH"));
  
  Serial.println(F(""));
  Serial.print(F("Temperature: "));
  Serial.print(myHTU21D.readTemperature());
  Serial.println(F(" deg.C"));
  
  Serial.println(F(""));
  Serial.println(F("<<built-in Heater OFF. Wait for 5 sec. to cool it down>>"));
  myHTU21D.setHeater(OFF);
  delay(5000);
  
  Serial.println(F(""));
  Serial.print(F("Compensated Humidity: "));
  Serial.print(myHTU21D.readCompensatedHumidity());
  Serial.println(F(" %RH"));
  
  Serial.println(F(""));
  Serial.print(F("Temperature: "));
  Serial.print(myHTU21D.readTemperature());
  Serial.println(F(" deg.C"));
*/

  Serial.println(F(""));
  Serial.println(F(""));
  Serial.print(F("DEMO: Start over again in 5 sec."));
  delay(5000);
}
