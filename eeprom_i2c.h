/*
 * =====================================================================================
 *
 *       Filename:  eeprom_i2c.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  07/14/2018 13:05:38
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Stuart B. Wilkins (sbw), stuwilkins@mac.com
 *   Organization:  
 *
 * =====================================================================================
 */

#ifndef __EEPROM_I2C__
#define __EEPROM_I2C__

class EEProm_I2C
{
  public:
    EEProm_I2C(uint8_t addr);
    bool begin(void);
    bool store(uint16_t offset, uint8_t *data, int size);
    bool retrieve(uint16_t offset, uint8_t* data, int size);

  private:
    uint8_t _addr;
    bool write(uint16_t offset, uint8_t* data, int size);
    bool read(uint16_t offset, uint8_t* data, int size);
    uint16_t chunk = 1;
};

#endif

