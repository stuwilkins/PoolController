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
#include <uCRC16Lib.h>
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

bool EEProm_I2C::store(uint16_t offset, uint8_t *data, int size)
{

 Serial.print(F("Write offset = "));
 Serial.print(offset);
 Serial.print(F(" size = "));
 Serial.println(size);

 // Calculate CRC
 uint16_t crc = uCRC16Lib::calculate((char*)data, size); 
 Serial.print(F("crc = "));
 Serial.println(crc, HEX);

 Serial.print(F("Data = "));
 for(int i=size-1;i>=0;i--)
 {
   Serial.print(data[i], HEX);
 }

 Serial.println("");
 write(offset, data, size);

 // Write the CRC
 write(offset + size, (uint8_t*)(&crc), sizeof(crc));

 return true;
}

bool EEProm_I2C::retrieve(uint16_t offset, uint8_t *data, int size)
{

 Serial.print(F("Retrieve offset = "));
 Serial.print(offset);
 Serial.print(F(" size = "));
 Serial.println(size);

 read(offset, data, size);

 Serial.print(F("Data = "));
 for(int i=size-1;i>=0;i--)
 {
   Serial.print(data[i], HEX);
 }
 Serial.println("");

 // Now check CRC
 
 uint16_t c_crc = uCRC16Lib::calculate((char*)data, size);
 uint16_t s_crc;
 read(offset+size, (uint8_t*)(&s_crc), sizeof(s_crc));

 Serial.print("s_crc = ");
 Serial.println(s_crc, HEX);
 Serial.print("c_crc = ");
 Serial.println(c_crc, HEX);

 if(s_crc != c_crc){
   Serial.println("CRC Mismach");
   return false;
 }

 return true;
}

bool EEProm_I2C::write(uint16_t offset, uint8_t *data, int size)
{
  Serial.print("Write offset = ");
  Serial.println(offset);
  for(int i = 0;i<size;i++){
    Wire.beginTransmission(_addr);
    uint16_t _offset = offset + i;
    Wire.write((int)(_offset >> 8));
    Wire.write((int)(_offset & 0xFF));
    Wire.write((int)data[i]); 
    Wire.endTransmission();

    Serial.print(_offset);
    Serial.print(F(" "));
    Serial.println(data[i], HEX);

  }

  return true;
}

bool EEProm_I2C::read(uint16_t offset, uint8_t* data, int size)
{
  Serial.print("Read offset = ");
  Serial.println(offset);

  for(int i = 0;i<size;i++)
  {
    Wire.beginTransmission(_addr);

    uint16_t _offset = offset + i;
    Wire.write(_offset >> 8);
    Wire.write(_offset & 0xFF);

    Serial.print(_offset);
    Serial.print(F(" "));

    Wire.endTransmission();

    Wire.requestFrom(_addr, 1);
    if(Wire.available())
    {
      data[i] = Wire.read();
      Serial.println(data[i], HEX);
    } else {
      Serial.println(F("ERROR"));
    }
  }
  
  return true;
}
