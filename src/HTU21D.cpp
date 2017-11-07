/***************************************************************************************************/
/*
  This is an Arduino library for SHT21, HTU21D & Si70xx Digital Humidity and Temperature Sensor

  written by : enjoyneering79
  sourse code: https://github.com/enjoyneering/

  This sensor uses I2C bus to communicate, specials pins are required to interface

  Connect chip to pins:    SDA        SCL
  Uno, Mini, Pro:          A4         A5
  Mega2560, Due:           20         21
  Leonardo:                2          3
  ATtiny85:                0(5)       2/A1(7)   (ATTinyCore  - https://github.com/SpenceKonde/ATTinyCore
                                                 & TinyWireM - https://github.com/SpenceKonde/TinyWireM)
  ESP8266 ESP-01:          GPIO0/D5   GPIO2/D3  (ESP8266Core - https://github.com/esp8266/Arduino)
  NodeMCU 1.0:             GPIO4/D2   GPIO5/D1
  WeMos D1 Mini:           GPIO4/D2   GPIO5/D1

  BSD license, all text above must be included in any redistribution
*/
/***************************************************************************************************/

#include "HTU21D.h"


/**************************************************************************/
/*
    Constructor
*/
/**************************************************************************/
HTU21D::HTU21D(HTU21D_RESOLUTION sensorResolution)
{
  _resolution = sensorResolution;
}

/**************************************************************************/
/*
    Initializes I2C and configures the sensor (call this function before
    doing anything else)

    Wire.endTransmission():
    0 - success
    1 - data too long to fit in transmit data16
    2 - received NACK on transmit of address
    3 - received NACK on transmit of data
    4 - other error
*/
/**************************************************************************/
#if defined(ESP8266)
bool HTU21D::begin(uint8_t sda, uint8_t scl)
{
  Wire.begin(sda, scl);
  Wire.setClock(100000UL);                //experimental! ESP8266 i2c bus speed: 100kHz..400kHz/100000UL..400000UL, default 100000UL
  Wire.setClockStretchLimit(230);         //experimental! default 230
#else
bool HTU21D::begin(void) 
{
  Wire.begin();
  Wire.setClock(100000UL);                //experimental! AVR i2c bus speed: 31kHz..400kHz/31000UL..400000UL, default 100000UL
#endif

  Wire.beginTransmission(HTU21D_ADDRESS);
  if (Wire.endTransmission(true) != 0)    //safety check, make sure the sensor is connected
  {
    #ifdef HTU21D_DEBUG_INFO
    Serial.println("HTU21D: can't find the sensor on the bus");
    #endif
    return false;
  }

  setResolution(_resolution);
  setHeater(HTU21D_OFF);
  return true;
}

/**************************************************************************/
/*
    Sets sensor's resolution
*/
/**************************************************************************/
void HTU21D::setResolution(HTU21D_RESOLUTION sensorResolution)
{
  uint8_t userRegisterData = 0;

  userRegisterData  = read8(HTU21D_USER_REGISTER_READ); //reads current register state
  userRegisterData &= 0x7E;                             //clears current resolution bits with "0"
  userRegisterData |= sensorResolution;                 //adds new resolution bits to userRegisterData

  write8(HTU21D_USER_REGISTER_WRITE, userRegisterData); //writes updeted register values to sensor

  _resolution = sensorResolution;                       //updates private variable
}

/**************************************************************************/
/*
    Soft reset.
 
    Switch sensor OFF & ON again. Takes about 15ms

    NOTE: All registers set to default except heater bit.
*/
/**************************************************************************/
void HTU21D::softReset(void)
{
  int8_t pollCounter = HTU21D_POLL_LIMIT;

  do
  {
    pollCounter--;
    if (pollCounter == 0)                                      //error handler
    {
      #ifdef HTU21D_DEBUG_INFO
      Serial.println("HTU21D: can't send soft reset command");
      #endif
      return;
    }
    Wire.beginTransmission(HTU21D_ADDRESS);
    #if ARDUINO >= 100
    Wire.write(HTU21D_SOFT_RESET);
    #else
    Wire.send(HTU21D_SOFT_RESET);
    #endif
  }
  while (Wire.endTransmission(true) != 0);

  delay(HTU21D_SOFT_RESET_DELAY);
}

/**************************************************************************/
/*
    Checks Battery Status

    for SHT21, HTU21D
    if VDD > 2.25v -+0.1v return TRUE
    if VDD < 2.25v -+0.1v return FALSE

    for Si70xx
    if VDD > 1.9v -+0.1v return TRUE
    if VDD < 1.9v -+0.1v return FALSE
*/
/**************************************************************************/
bool HTU21D::batteryStatus(void)
{
  uint8_t userRegisterData = 0;
  
  userRegisterData  = read8(HTU21D_USER_REGISTER_READ);
  userRegisterData &= 0x40;

  if (userRegisterData == 0x00)
  {
    return true;
  }
  return false;
}

/**************************************************************************/
/*
    Turn ON/OFF build-in Heater

    The heater consumtion is 3.09mA-94.20mA @ 3.3v.
    
    NOTE: Used to raise the temperature of the sensor. This element can be
          used to test the sensor, to drive off condensation, or to implement
          dew-point measurement when the Si7021 is used in conjunction with a
          separate temperature sensor (the heater will raise the temperature of
          the internal temperature sensor).
*/
/**************************************************************************/
void HTU21D::setHeater(HTU21D_HEATER_SWITCH heaterSwitch)
{
  uint8_t userRegisterData = 0;

  userRegisterData = read8(HTU21D_USER_REGISTER_READ);

  switch(heaterSwitch)
  {
    case HTU21D_ON:
      userRegisterData |= heaterSwitch;
      break;
    case HTU21D_OFF:
      userRegisterData &= heaterSwitch;
      break;
  }
  write8(HTU21D_USER_REGISTER_WRITE, userRegisterData);
}

/**************************************************************************/
/*
    Reads Humidity, %

    The "operationMode" could be set up as:
    - "HTU21D_TRIGGER_TEMP_MEASURE_NOHOLD" mode, allows communication with another
       slave devices on I2C bus while sensor is measuring.
       WARNING!!! Could create collision if more than one slave devices are
       connected to the same bus.
    - "HTU21D_TRIGGER_HUMD_MEASURE_HOLD" mode, blocks communication on I2C bus while
       sensor is measuring.

    NOTE: - Accuracy +-2%RH in range 20%..80% at 25C
          - Max. possible measurement time ~32ms
          - Suggested min time between measurements is 17 sec. - 18 sec.
            (sensor could faster but it's pointless)
          - endTransmission()
            0 - success
            1 - data too long to fit in transmit data16
            2 - received NACK on transmit of address
            3 - received NACK on transmit of data
            4 - can't start, line busy
*/
/**************************************************************************/
float HTU21D::readHumidity(HTU21D_HUMD_OPERATION_MODE sensorOperationMode)
{
  int8_t   pollCounter = HTU21D_POLL_LIMIT;
  uint8_t  checksum    = 0;
  uint16_t rawHumidity = 0;
  float    humidity    = 0;

  /* request humidity measurement */
  do
  {
    pollCounter--;
    if (pollCounter == 0)                                //error handler
    {
      #ifdef HTU21D_DEBUG_INFO
      Serial.println("HTU21D: can't send humidity measurement command");
      #endif
      return HTU21D_ERROR;
    }
    Wire.beginTransmission(HTU21D_ADDRESS);
    #if ARDUINO >= 100
    Wire.write(sensorOperationMode);
    #else
    Wire.send(sensorOperationMode);
    #endif
  }
  while (Wire.endTransmission(true) != 0);               //true = stop message after transmission & releas the I2C bus

  /* humidity measurement delay */
  switch(_resolution)
  {
    case HTU21D_RES_RH12_TEMP14:
      delay(16);                                         //Si7021 - 17..23msec, SHT21 - 22..29msec
      break;
    case HTU21D_RES_RH8_TEMP12:
      delay(3);                                          //Si7021 - 5..7msec,   SHT21 - 3..4msec
      break;
    case HTU21D_RES_RH10_TEMP13:
      delay(5);                                          //Si7021 - 8..11msec,  SHT21 - 7..9msec
      break;
    case HTU21D_RES_RH11_TEMP11:
      delay(8);                                          //HTU21 - 8..10msec,   SHT21 - 12..15msec
      break;
  }

  pollCounter = HTU21D_POLL_LIMIT;

  do
  {
    pollCounter--;
    if (pollCounter == 0)                                //error handler
    {
      #ifdef HTU21D_DEBUG_INFO
      Serial.println("HTU21D: can't read humidity measurement result");
      #endif
      return HTU21D_ERROR;
    }
    if (pollCounter < (HTU21D_POLL_LIMIT - 1)) delay(8); //bacause Si7021 & SHT21 is slower than HTU21D
    Wire.requestFrom(HTU21D_ADDRESS, 3, true);           //true = stop message after transmission & releas the I2C bus
  }
  while (Wire.available() != 3);                         //check rxBuffer

  /* reads MSB byte, LSB byte & Checksum from "wire.h" buffer */
  #if ARDUINO >= 100
  rawHumidity  = Wire.read() << 8;                       //reads MSB byte & shift it to the right
  rawHumidity |= Wire.read();                            //reads LSB byte & sum. with MSB byte
  checksum     = Wire.read();                            //checksum
  #else
  rawHumidity  = Wire.receive() << 8;
  rawHumidity |= Wire.receive();
  checksum     = Wire.receive();
  #endif

  if (checkCRC8(rawHumidity) != checksum)                //error handler
  {
    #ifdef HTU21D_DEBUG_INFO
    Serial.println("HTU21D: humidity CRC8 doesn't match");
    #endif
    return HTU21D_ERROR;
  }

  rawHumidity ^= 0x02;                                   //clear status bits, humidity measurement always returns xxxxxx10 in the LSB field
  humidity     = 0.001907 * (float)rawHumidity - 6;
  
  if (humidity < 0)
  {
    humidity = 0;
  }
  else if (humidity > 100)
  {
    humidity = 100;
  }
  return humidity;
}

/**************************************************************************/
/*
    Reads Temperature, C

    The "operationMode" could be set up as:
    - "HTU21D_TRIGGER_TEMP_MEASURE_NOHOLD" mode, allows communication with another
      slave devices on I2C bus while sensor is measuring.
      WARNING!!! Could create collision if more than one slave devices are
      connected to the same bus.
    - "HTU21D_TRIGGER_TEMP_MEASURE_HOLD" mode, blocks communication on I2C bus while
      sensor is measuring.
    - "SI7021_TEMP_READ_AFTER_RH_MEASURMENT" mode, allows to retrive temperature
      measurement, which was made at previouse RH measurement. For HTU21D & SHT21
      you have to manualy call "readCompensatedHumidity()"

    NOTE: - Accuracy +-0.3C in range 0C..60C
          - Max. possible measurement time ~91ms
          - Suggested min time between measurements is 17 sec. - 18 sec.
            (sensor could faster but it's pointless)
          - endTransmission()
            0 - success
            1 - data too long to fit in transmit data16
            2 - received NACK on transmit of address
            3 - received NACK on transmit of data
            4 - can't start, line busy
*/
/**************************************************************************/
float HTU21D::readTemperature(HTU21D_TEMP_OPERATION_MODE sensorOperationMode)
{
  int8_t   pollCounter    = HTU21D_POLL_LIMIT;
  uint8_t  checksum       = 0;
  uint16_t rawTemperature = 0;

  /* request temperature measurement */
  do
  {
    pollCounter--;
    if (pollCounter == 0)                                 //error handler
    {
      #ifdef HTU21D_DEBUG_INFO
      Serial.println("HTU21D: can't send temperature measurement command");
      #endif
      return HTU21D_ERROR;
    }
    Wire.beginTransmission(HTU21D_ADDRESS);
    #if ARDUINO >= 100
    Wire.write(sensorOperationMode); 
    #else
    Wire.send(sensorOperationMode);
    #endif
  }
  while (Wire.endTransmission(true) != 0);                //true = stop message after transmission & releas the I2C bus

  /* temperature measurement delay */
  if (sensorOperationMode != SI70xx_TEMP_READ_AFTER_RH_MEASURMENT)
  {
    switch(_resolution)
    {
      case HTU21D_RES_RH12_TEMP14:
        delay(11);                                        //HTU21D - 44..50msec, SHT21 - 66..85msec
        break;
      case HTU21D_RES_RH8_TEMP12:
        delay(4);                                         //HTU21D - 11..13msec, SHT21 - 17..22msec
        break;
      case HTU21D_RES_RH10_TEMP13:
        delay(7);                                         //Si7021 - 22..25msec, SHT21 - 33..43msec
        break;
      case HTU21D_RES_RH11_TEMP11:
        delay(3);                                         //Si7021 - 6..7msec, SHT21 - 9..11msec
        break;
    }
  }

  pollCounter = HTU21D_POLL_LIMIT;

  do
  {
    pollCounter--;
    if (pollCounter == 0)                                 //error handler
    {
      #ifdef HTU21D_DEBUG_INFO
      Serial.println("HTU21D: can't read temperature measurement result");
      #endif
      return HTU21D_ERROR;
    }
    if (pollCounter < (HTU21D_POLL_LIMIT - 1)) delay(16); //bacause HTU21D & SHT21 is slower than Si7021
    Wire.requestFrom(HTU21D_ADDRESS, 3, true);            //true = stop message after transmission & releas the I2C bus
  }
  while (Wire.available() != 3);                          //check rxBuffer

  /* reads MSB byte, LSB byte & Checksum from "wire.h" buffer */
  #if ARDUINO >= 100
  rawTemperature  = Wire.read() << 8;                     //reads MSB byte & shift it to the right
  rawTemperature |= Wire.read();                          //reads LSB byte and sum. with MSB byte
  checksum        = Wire.read();
  #else
  rawTemperature  = Wire.receive() << 8;
  rawTemperature |= Wire.receive();
  checksum        = Wire.receive();
  #endif

  if (checkCRC8(rawTemperature) != checksum)              //error handler
  {
    #ifdef HTU21D_DEBUG_INFO
    Serial.println("HTU21D: temperature CRC8 doesn't match");
    #endif
    return HTU21D_ERROR;
  }

  return 0.002681 * (float)rawTemperature - 46.85;        //temperature measurement always returns xxxxxx00 in the LSB field
}

/**************************************************************************/
/*
    Calculates Compensated Humidity, %RH

    Only for HTU21D & SHT21. Compensate the temperature affect on RH measurement.
    Si7021 automatically compensates temperature influence on RH every humidity
    measurement.

    NOTE: - Accuracy +-2%RH in range 0%..100% at 0C..80C
          - Max. possible measurement time ~115ms
          - Suggested min time between measurements is 17 sec. - 18 sec.
            (sensor could faster but it's pointless)
*/
/**************************************************************************/
float HTU21D::readCompensatedHumidity(void)
{
  float humidity    = 0;
  float temperature = 0;

  humidity    = readHumidity();
  temperature = readTemperature();

  if (humidity == HTU21D_ERROR || temperature == HTU21D_ERROR)          //error handler
  {
    return HTU21D_ERROR;
  }
  
  if (temperature > 0 && temperature < 80)
  {
    humidity = humidity + (25 - temperature) * HTU21D_TEMP_COEFFICIENT;
  }

  return humidity;
}

/***************************************************************************/
/*
    Reads Device ID

    SerialNumber = {SNA3, SNA2, SNA1, SNA0, SNB3**, SNB2, SNB1, SNB0}
    
    **chip ID: 0x0D - Si7013
               0x14 - Si7020
               0x15 - Si7021 
               0x32 - HTU21D & SHT21  

    NOTE: see p.23 of Si7021 datasheet for details
*/
/**************************************************************************/
uint16_t HTU21D::readDeviceID(void)
{
  uint16_t deviceID   = 0;
  uint8_t  checksum   = 0;

  /* Serial_2 requests SNB3**, SNB2, SNB1, SNB0 */
  Wire.beginTransmission(HTU21D_ADDRESS);
  #if ARDUINO >= 100
  Wire.write(HTU21D_SERIAL2_READ1);
  Wire.write(HTU21D_SERIAL2_READ2);
  #else
  Wire.send(HTU21D_SERIAL2_READ1);
  Wire.send(HTU21D_SERIAL2_READ2);
  #endif
  Wire.endTransmission(true);

  /* Serial_2 reads SNB3**, SNB2 & CRC */
  Wire.requestFrom(HTU21D_ADDRESS, 3, true); //true = stop message after transmission & releas the I2C bus
  #if ARDUINO >= 100
  deviceID  = Wire.read() << 8;
  deviceID |= Wire.read();
  checksum  = Wire.read();
  #else
  deviceID  = Wire.receive() << 8;
  deviceID |= Wire.receive();
  checksum  = Wire.receive();
  #endif
  if (checkCRC8(deviceID) != checksum)
  {
    #ifdef HTU21D_DEBUG_INFO
    Serial.println("HTU21D: device ID CRC8 doesn't match");
    #endif
    return HTU21D_ERROR;
  }

  deviceID = deviceID >> 8;

  switch(deviceID)
  {
    case HTU21D_CHIPID:
      deviceID = 21;
      break;
    case SI7013_CHIPID:
      deviceID = 7013;
      break;
    case SI7020_CHIPID:
      deviceID = 7020;
      break;
    case SI7021_CHIPID:
      deviceID = 7021;
      break;
    default:
      deviceID = HTU21D_ERROR;
      break;
  }
  return deviceID;
}

/***************************************************************************/
/*
    Reads Firware Version

    NOTE: see p.24 of Si7021 datasheet for details
*/
/**************************************************************************/
uint8_t HTU21D::readFirmwareVersion(void)
{
  uint8_t firmwareVersion = 0;

  Wire.beginTransmission(HTU21D_ADDRESS);
  #if ARDUINO >= 100
  Wire.write(HTU21D_FIRMWARE_READ1);
  Wire.write(HTU21D_FIRMWARE_READ2);
  #else
  Wire.send(HTU21D_FIRMWARE_READ1);
  Wire.send(HTU21D_FIRMWARE_READ2);
  #endif
  Wire.endTransmission(true);

  Wire.requestFrom(HTU21D_ADDRESS, 1, true); //true = stop message after transmission & releas the I2C bus
  #if ARDUINO >= 100
  firmwareVersion = Wire.read();
  #else
  firmwareVersion = Wire.read();
  #endif

  switch(firmwareVersion)
  {
    case HTU21D_FIRMWARE_V1:
      firmwareVersion = 1;
      break;
    case HTU21D_FIRMWARE_V2:
      firmwareVersion = 2;
      break;
    default:
      firmwareVersion = HTU21D_ERROR;
      break;
  }
  return firmwareVersion;
}

/**************************************************************************/
/*
    Writes 8 bit value to the sensor register over I2C
*/
/**************************************************************************/
void HTU21D::write8(uint8_t reg, uint8_t value)
{
  int8_t pollCounter = HTU21D_POLL_LIMIT;

  do
  {
    pollCounter--;
    if (pollCounter == 0)                           //error handler
    {
      #ifdef HTU21D_DEBUG_INFO
      Serial.println("HTU21D: can't write a byte");
      #endif
      return;
    }
    Wire.beginTransmission(HTU21D_ADDRESS);
    #if ARDUINO >= 100
    Wire.write(reg);
    Wire.write(value);
    #else
    Wire.send(reg);
    Wire.send(value);
    #endif
  }
  while (Wire.endTransmission(true) != 0);
}

/**************************************************************************/
/*
    Reads 8 bit value from the sensor, over I2C
*/
/**************************************************************************/
uint8_t HTU21D::read8(uint8_t reg)
{
  int8_t pollCounter = HTU21D_POLL_LIMIT;

  do
  {
    pollCounter--;
    if (pollCounter == 0)                             //error handler
    {
      #ifdef HTU21D_DEBUG_INFO
      Serial.println("HTU21D: can't request a byte");
      #endif
      return HTU21D_ERROR;
    }
    Wire.beginTransmission(HTU21D_ADDRESS);
    #if ARDUINO >= 100
    Wire.write(reg);
    #else
    Wire.send(reg);
    #endif
  }
  while (Wire.endTransmission(true) != 0);

  pollCounter = HTU21D_POLL_LIMIT;

  do
  {
    pollCounter--;
    if (pollCounter == 0)                             //error handler
    {
      #ifdef HTU21D_DEBUG_INFO
      Serial.println("HTU21D: can't read a byte");
      #endif
      return HTU21D_ERROR;
    }
    Wire.requestFrom(HTU21D_ADDRESS, 1, true);        //true = stop message after transmission & releas the I2C bus
  }
  while (Wire.available() != 1);                      //check rxBuffer

  /* read byte from "wire.h" buffer */
  #if ARDUINO >= 100
  return Wire.read();
  #else
  return Wire.receive();
  #endif
}

/**************************************************************************/
/*
    Calculates CRC8 for 16 bit received Data

    NOTE: For more info about Cyclic Redundancy Check (CRC) see
          http://en.wikipedia.org/wiki/Computation_of_cyclic_redundancy_checks
*/
/**************************************************************************/
uint8_t HTU21D::checkCRC8(uint16_t data)
{
  for (uint8_t bit = 0; bit < 16; bit++)
  {
    if (data & 0x8000)
    {
      data =  (data << 1) ^ HTU21D_CRC8_POLYNOMINAL;
    } 
    else
    {
      data <<= 1;
    }
  }

  return data >>= 8;
}
