/***************************************************************************************************/
/* 
Board:                                    SDA                    SCL

Linkikt 9697                               P9                     P8

*/
/***************************************************************************************************/

#include <Wire.h>
#include <HTU21D.h>

HTU21D myHTU21D(HTU21D_RES_RH12_TEMP14);

void setup()
{
  Serial.begin(9600);
  Serial.println();
  
  while (myHTU21D.begin() != true)
  {
    Serial.println(F("HTU21D, SHT21 sensor is faild or not connected")); //(F()) saves string to flash & keeps dynamic memory free
    delay(5000);
  }
  Serial.println(F("HTU21D, SHT21 sensor is active"));
}


void loop()
{
  /* DEMO - 1 */
  Serial.print(F("Humidity............: ")); Serial.print(myHTU21D.readHumidity());    Serial.println("");//   Serial.println(F(" +-2%"));
  Serial.print(F("Compensated Humidity: ")); Serial.print(myHTU21D.readCompensatedHumidity()); Serial.println(F(" +-2%"));

  Serial.print(F("Temperature.........: ")); Serial.print(myHTU21D.readTemperature()); Serial.println(""); // Serial.println(F(" +-0.3C"));

  /* DEMO - END */
//  Serial.println(F("DEMO starts over again in 2 sec."));
  delay(2000);
}
