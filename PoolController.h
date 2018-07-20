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


// Analog input for level meter
#define WATER_LEVEL_PIN			                A0
#define WATER_LEVEL_X     		              79.69
#define WATER_LEVEL_C     		              -36455

#define PUMP_PRESSURE_PIN                   A1
#define PUMP_PRESSURE_CONV                  13200

#define PUMP_FLOW_PIN                       11
#define PUMP_FLOW_BOUNCE                    10
// Calculate flow rate
// F = 0.2Q(l.min^-1)
// Q(l.min^-1) = 5F 
// 1 l.min^-1 = 0.26417287472922 GPM
#define PUMP_FLOW_F_TO_Q                    5
#define LPM_TO_GPM                          0.26417287472922

// Other digital pins
#define OUTPUT_LED_L			                  13
#define FLOW_SWITCH       		              0
#define PUMP_STOP_BIT     		              10
#define PUMP_BIT_0        		              9
#define PUMP_BIT_1        		              6
#define PUMP_BIT_2        		              5
#define ONE_WIRE_BUS                        1

#define UTC_TIME_OFFSET	                    4 // Hours

#define EPS_PUMP_FLOW_TIMEOUT               30000L

#define WATCHDOG_TIME                       8000
#define OTA_CONNECTION_NAME                 "pool_controller_ota"

#define EEPROM_PUMP_SPEED                   0
#define EEPROM_PUMP_STOP                    1

// Programs to run
enum programs {
  PROGRAM_HALT         = 0,
  PROGRAM_RUN          = 1,
  PROGRAM_TIMER        = 2,
  PROGRAM_DRAIN        = 3,
  PROGRAM_FILL         = 4
};

// Days of the week

const char daysOfTheWeek_0[12] PROGMEM = "Sunday";
const char daysOfTheWeek_1[12] PROGMEM = "Monday";
const char daysOfTheWeek_2[12] PROGMEM = "Tuesday";
const char daysOfTheWeek_3[12] PROGMEM = "Wednesday";
const char daysOfTheWeek_4[12] PROGMEM = "Thursday";
const char daysOfTheWeek_5[12] PROGMEM = "Friday";
const char daysOfTheWeek_6[12] PROGMEM = "Saturday";
const char* const daysOfTheWeek[] PROGMEM = {daysOfTheWeek_0,
                                           daysOfTheWeek_1,
                                           daysOfTheWeek_2,
                                           daysOfTheWeek_3,
                                           daysOfTheWeek_4,
                                           daysOfTheWeek_5,
                                           daysOfTheWeek_6};

// Pump Speeds
uint16_t pumpSpeeds[] = {0, 600, 1075, 1550, 2025, 2500, 2975, 3450};

struct DataReadings {
  float water_temp;
  float air_temp;
  float air_humidity;
  int32_t water_level;
  int32_t pump_pressure;
  int32_t uv_index;
  int32_t vis;
  int32_t ir;
  int32_t flow_switch;
  int32_t pump_speed;
  int32_t error_count;
  int32_t pump_flow;
};

struct PumpFlowRate {
		unsigned long last;
		unsigned long last_read;
		unsigned long clicks;
};

struct ProgramData {
  int current;
  uint32_t level_target;
};
