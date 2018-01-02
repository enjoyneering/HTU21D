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

#ifndef HTU21D_h
#define HTU21D_h

#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#if defined(__AVR_ATtinyX4__) || defined(__AVR_ATtinyX5__) || defined(__AVR_ATtinyX313__)
#include <TinyWireM.h>
#define  Wire TinyWireM
#else
#include <Wire.h>
#endif

#if defined(__AVR__)
#include <avr/pgmspace.h>
#elif defined(ESP8266)
#include <pgmspace.h>
#endif

//#define HTU21D_DEBUG_INFO                  //enable/disable serial debug info

#define HTU21D_ADDRESS               0x40    //Chip I2C address

#define HTU21D_USER_REGISTER_WRITE   0xE6    //Write user register
#define HTU21D_USER_REGISTER_READ    0xE7    //Read  user register

#define HTU21D_HEATER_REGISTER_WRITE 0x51    //Write Heater Control Register
#define HTU21D_HEATER_REGISTER_READ  0x11    //Read  Heater Control Register

#define HTU21D_SOFT_RESET            0xFE    //Soft Reset (takes 15ms). Switchs sensor OFF & ON. All registers will set to default exept Heater bit!

#define HTU21D_SERIAL1_READ1         0xFA    //Read 1st two Serial bytes
#define HTU21D_SERIAL1_READ2         0x0F    //Read 2nd two Serial bytes
#define HTU21D_SERIAL2_READ1         0xFC    //Read 3rd two Serial bytes
#define HTU21D_SERIAL2_READ2         0xC9    //Read 4th two Serial bytes

#define SI7013_CHIPID                0x0D    //Si7013 device ID 
#define SI7020_CHIPID                0x14    //Si7020 device ID
#define SI7021_CHIPID                0x15    //Si7021 device ID
#define HTU21D_CHIPID                0x32    //HTU21D device ID

#define HTU21D_FIRMWARE_READ1        0x84    //Read Firmware revision, first  command
#define HTU21D_FIRMWARE_READ2        0xB8    //Read Firmware revision, second command

#define HTU21D_FIRMWARE_V1           0xFF    //Firmware Version 1.0
#define HTU21D_FIRMWARE_V2           0x20    //Firmware version 2.0

#define HTU21D_TEMP_COEFFICIENT      -0.15   //Temperature coefficient for RH measurement, for HTU21D & SHT21 only at range 0C..80C
#define HTU21D_CRC8_POLYNOMINAL      0x13100 //CRC8 polynomial for 16bit CRC8 x^8 + x^5 + x^4 + 1


#define HTU21D_SOFT_RESET_DELAY      15      //milliseconds
#define HTU21D_POLL_LIMIT            8       //i2c retry limit
#define HTU21D_ERROR                 0xFF    //Returns 255, if CRC8 or communication error is occurred
#define HTU21D_READ_TEMP             0xFE    //Indicates to read the temperature from the sensor before humidity compensation

typedef enum
{
  HTU21D_RES_RH12_TEMP14 = 0x00,             //RH: 12Bit, Temperature: 14Bit (by default)
  HTU21D_RES_RH8_TEMP12  = 0x01,             //RH: 8Bit,  Temperature: 12Bit
  HTU21D_RES_RH10_TEMP13 = 0x80,             //RH: 10Bit, Temperature: 13Bit
  HTU21D_RES_RH11_TEMP11 = 0x81              //RH: 11Bit, Temperature: 11Bit
}
HTU21D_RESOLUTION;

typedef enum
{
  HTU21D_TRIGGER_HUMD_MEASURE_HOLD   = 0xE5, //Humidity measurement. Hold master, SCK line is blocked. (by default)
  HTU21D_TRIGGER_HUMD_MEASURE_NOHOLD = 0xF5  //Humidity measurement. No Hold master. Could create collision if more than one slave devices are connected to I2C bus
}
HTU21D_HUMD_OPERATION_MODE;

typedef enum
{
  HTU21D_TRIGGER_TEMP_MEASURE_HOLD     = 0xE3, //Temperature measurement. Hold master, SCK line is blocked. (by default)
  HTU21D_TRIGGER_TEMP_MEASURE_NOHOLD   = 0xF3, //Temperature measurement. No Hold master. Could create collision if more than one devices are connected to I2C bus
  SI70xx_TEMP_READ_AFTER_RH_MEASURMENT = 0xE0  //Read Temperature value from previous RH measurement, for Si7021 only.
}
HTU21D_TEMP_OPERATION_MODE;

typedef enum
{
  HTU21D_ON  = 0x04,                           //Heater ON
  HTU21D_OFF = 0xFB                            //Heater OFF
}
HTU21D_HEATER_SWITCH;

typedef enum
{
  HTU21D_3_090 = 0x00,                         //3.09 mA @ 3.3v
  HTU21D_9_180 = 0x01,                         //9.18 mA @ 3.3v
  HTU21D_94_20 = 0x0F                          //94.2 mA @ 3.3v
}
HTU21D_HEATER_CONTROL_SETTINGS;


class HTU21D
{
  public:
   HTU21D(HTU21D_RESOLUTION = HTU21D_RES_RH12_TEMP14);

   #if defined(ESP8266)
   bool     begin(uint8_t sda = SDA, uint8_t scl = SCL);
   #else
   bool     begin(void);
   #endif
   float    readHumidity(HTU21D_HUMD_OPERATION_MODE = HTU21D_TRIGGER_HUMD_MEASURE_HOLD);    //Accuracy +-2%RH  in range 20%..80% at 25C
   float    readCompensatedHumidity(float = HTU21D_READ_TEMP);                              //Accuracy +-2%RH  in range 0%..100% at 0C..80C
   float    readTemperature(HTU21D_TEMP_OPERATION_MODE = HTU21D_TRIGGER_TEMP_MEASURE_HOLD); //Accuracy +-0.3C  in range 0C..60C
   void     setResolution(HTU21D_RESOLUTION sensorResolution);
   void     softReset(void);
   bool     batteryStatus(void);
   void     setHeater(HTU21D_HEATER_SWITCH heaterSwitch);
   uint16_t readDeviceID(void);
   uint8_t  readFirmwareVersion(void);

  private:
   HTU21D_RESOLUTION _resolution;

   void    write8(uint8_t reg, uint8_t value);
   uint8_t read8(uint8_t reg);
   uint8_t checkCRC8(uint16_t data);
};

#endif
