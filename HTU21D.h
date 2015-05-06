/**************************************************************************/
/*
This is an Arduino library for SHT21 & HTU21D Digital Humidity & 
Temperature Sensor

  written by enjoyneering79

  These sensor uses I2C to communicate, 2 pins are required to  
  interface

  Connect HTU21D to pins :  SDA  SCL
  Uno, Redboard, Pro:       A4   A5
  Mega2560, Due:            20   21
  Leonardo:                 2    3

  BSD license, all text above must be included in any redistribution
*/
 /**************************************************************************/

#ifndef HTU21D_h
#define HTU21D_h

#if ARDUINO >= 100
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif
 #include <Wire.h>

#define HTDU21D_ADDRESS     0x40    /* I2C address */

#define USER_REGISTER_WRITE 0xE6    /* Write user register*/
#define USER_REGISTER_READ  0xE7    /* Read  user register*/

#define SOFT_RESET          0xFE    /* Soft Reset (takes 15ms). Switch sensor OFF & ON again. All registers set to default exept heater bit. */

#define TEMP_COEFFICIENT    -0.15   /* Temperature coefficient (from 0deg.C to 80deg.C) */
#define CRC8_POLYNOMINAL    0x13100 /* CRC8 polynomial for 16bit CRC8 x^8 + x^5 + x^4 + 1 */


typedef enum
{
HTU21D_RES_RH12_TEMP14  = 0x00,  /* RH: 12Bit, measuring time 16ms. Temperature: 14Bit, measuring time 50ms (dafault on Power ON) */
HTU21D_RES_RH8_TEMP12   = 0x01,  /* RH: 8Bit,  measuring time 8ms.  Temperature: 12Bit, measuring time 25ms   */
HTU21D_RES_RH10_TEMP13  = 0x80,  /* RH: 10Bit, measuring time 5ms.  Temperature: 13Bit, measuring time 13ms.  */
HTU21D_RES_RH11_TEMP11  = 0x81   /* RH: 11Bit, measuring time 3ms.  Temperature: 11Bit, measuring time 7ms.   */
}
HTU21D_Resolution;

typedef enum
{
TRIGGER_HUMD_MEASURE_HOLD   = 0xE5,  /* Trigger Humidity Measurement. Hold master (SCK line is blocked) */
TRIGGER_HUMD_MEASURE_NOHOLD = 0xF5   /* Trigger Humidity Measurement. No Hold master (allows other I2C communication on a bus while sensor is measuring) */
}
humdOperationMode;

typedef enum
{
TRIGGER_TEMP_MEASURE_HOLD   = 0xE3,  /* Trigger Temperature Measurement. Hold master (SCK line is blocked) */
TRIGGER_TEMP_MEASURE_NOHOLD = 0xF3   /* Trigger Temperature Measurement. No Hold master (allows other I2C communication on a bus while sensor is measuring) */
}
tempOperationMode;

typedef enum
{
ON  = 0x04,  /* Heater ON */
OFF = 0xFB   /* Heater OFF */
}
toggleHeaterSwitch;

class HTU21D
{
  public:
  HTU21D(HTU21D_Resolution = HTU21D_RES_RH12_TEMP14);

  bool    begin(void);
  float   readHumidity(humdOperationMode = TRIGGER_HUMD_MEASURE_HOLD);    //Accuracy +-2%RH     in range 20%RH - 80%RH at 25deg.C only
  float   readCompensatedHumidity(void);                                  //Accuracy +-2%RH     in range 0%RH - 100%RH at range 0deg.C - 80deg.C
  float   readTemperature(tempOperationMode = TRIGGER_TEMP_MEASURE_HOLD); //Accuracy +-0.3deg.C in range 0deg.C - 60deg.C
  void    setResolution(HTU21D_Resolution it);
  void    softReset(void);
  bool    batteryStatus(void);
  void    setHeater(toggleHeaterSwitch it);

  private:
  void     write8 (uint8_t reg, uint32_t value);
  uint8_t  read8 (uint8_t reg);
  uint8_t  checkCRC8(uint16_t data);

  bool _HTDU21Dinitialisation;

  HTU21D_Resolution  _HTU21D_Resolution;
};

#endif
