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

/* UNCOMENT FOR DIAGNOSTIC OF SENSOR'S FUNCTIONALITY. TEMPERATURE SHOULD INCREASE OF 0.5-1.5 deg.C and HUMIDITY DROP
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
