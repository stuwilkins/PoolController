/*
 * =====================================================================================
 *
 *       Filename:  eeprom_i2c.cpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  07/14/2018 13:05:17
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Stuart B. Wilkins (sbw), stuwilkins@mac.com
 *   Organization:  
 *
 * =====================================================================================
 */

#include <Wire.h>
#include "eeprom_i2c.h"

EEProm_I2C::EEProm_I2C(uint8_t addr)
{
  _addr = addr;
}

bool EEProm_I2C::begin(void)
{
  Wire.begin();
  return true;
}

bool EEProm_I2C::write(uint16_t offset, uint8_t* data, int size)
{
  Wire.beginTransmission(_addr);

  Wire.write(offset >> 8);
  Wire.write(offset & 0xFF);

  for(int i = 0;i<size;i++){
    Wire.write(data[i]); 
  }

  Wire.endTransmission(_addr);

  return true;
}

bool EEProm_I2C::read(uint16_t offset, uint8_t* data, int size)
{

  for(int i=0;i<size;i++){
    Wire.beginTransmission(_addr);

    Wire.write(offset >> 8);
    Wire.write(offset & 0xFF);

    Wire.endTransmission();

    Wire.requestFrom(_addr, size);
    for(int i=0;i<size;i++)
    {
      if(Wire.available())
      {
        data[i] = Wire.read();
      }
    }
  }
  
  return true;
}
