/*
 * =====================================================================================
 *
 *       Filename:  PoolController.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  06/17/2018 14:42:19
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Stuart B. Wilkins (sbw), stuwilkins@mac.com
 *   Organization:  
 *
 * =====================================================================================
 */


#define BUFSIZE                        128
#define VERBOSE_MODE                   true
#define BLUEFRUIT_SPI_CS               8
#define BLUEFRUIT_SPI_IRQ              7
#define BLUEFRUIT_SPI_RST              4


// 
// Service UUID Definitions
//

// UUID 036451ed-6956-41ae-9840-5aca6c150ec7
uint8_t environmentServiceUUID[]   = { 0x03, 0x64, 0x51, 0xed, 0x69, 0x56, 0x41, 0xae,
                                       0x98, 0x40, 0x5a, 0xca, 0x6c, 0x15, 0x0e, 0xc7};
// 
// BLE Character UUIDs
//
uint16_t waterTempCharUUID =        0x1001;                                       
uint16_t airTempCharUUID =          0x1002;
uint16_t humidityCharUUID =         0x1003;
uint16_t flowSwitchCharUUID =       0x1004;
uint16_t waterLevelCharUUID =       0x1005;
uint16_t uvIndexCharUUID =          0x1006;
uint16_t visCharUUID =              0x1007;
uint16_t irCharUUID =               0x1008;  
uint16_t pumpSpeedUUID =            0x1010;
uint16_t pumpSpeedCmdUUID =         0x1011;
uint16_t errorUUID =                0x1012;

// 5d67af3f-b46e-4836-abfa-f7bffab6bceb
uint8_t timeServiceUUID[]   =       { 0x5d, 0x67, 0xaf, 0x3f, 0xb4, 0x6e, 0x48, 0x36,
                                      0xab, 0xfa, 0xf7, 0xbf, 0xfa, 0xb6, 0xbc, 0xeb};
uint16_t timeCharUUID =             0x2000;
uint16_t timeSetCharUUID =          0x2001;
