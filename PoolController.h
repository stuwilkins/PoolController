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


// Buffer lengths
#define JSON_BUFFER_LEN                     500

// Analog input for level meter

#define MAX31865_CS                         19
#define MAX31865_RREF                       430.0
#define MAX31865_RNOMINAL                   100.0

#define WATER_LEVEL_PIN		                A0
#define WATER_LEVEL_X     	                79.69
#define WATER_LEVEL_C     	                -36455

#define PUMP_PRESSURE_PIN                   A1
#define PUMP_PRESSURE_CONV                  13200

#define PUMP_FLOW_PIN                       11
// Calculate flow rate
// F = 0.2Q(l.min^-1)
// Q(l.min^-1) = 5F 
// 1 l.min^-1 = 0.26417287472922 GPM
#define PUMP_FLOW_F_TO_Q                    5
#define FILL_FLOW_PIN                       12
// Calculate flow rate
// F = 4.8Q(l.min^-1)
// Q(l.min^-1) = 0.2083F 
#define FILL_FLOW_F_TO_Q                    0.2083
#define LPM_TO_GPM                          0.2641

// Other digital pins
#define OUTPUT_LED_L			                  13
#define FLOW_SWITCH       		              0
#define PUMP_STOP_BIT     		              10
#define PUMP_BIT_0        		              9
#define PUMP_BIT_1        		              6
#define PUMP_BIT_2        		              5

#define EPS_PUMP_FLOW_TIMEOUT               30000L

#define WATCHDOG_TIME                       8000

#define EEPROM_PROGRAM_DATA                 16
#define EEPROM_MAGIC_DATA                   0
#define EEPROM_MAGIC                        0xDEADF00D

#define MQTT_CLIENT_NAME                    "pool_controller"

#define NTP_ADDRESS                         "north-america.pool.ntp.org"
#define NTP_OFFSET                          -4 * 60 * 60 // In seconds
#define NTP_INTERVAL                        60 * 1000 // In miliseconds

#define PROGRAM_LEVEL_TARGET                25500
#define PROGRAM_PUMP_RUN_SPEED              3
#define PROGRAM_PUMP_DRAIN_SPEED            7

#define UPLOAD_WINDOW                       60000

// Programs to run
enum programs {
  PROGRAM_HALT         = 0,
  PROGRAM_RUN          = 1,
  PROGRAM_TIMER        = 2,
  PROGRAM_DRAIN        = 3,
  PROGRAM_FILL         = 4,
  PROGRAM_BOOST        = 5
};

// Days of the week

static const char daysOfTheWeek_0[12]  = "Sunday";
static const char daysOfTheWeek_1[12]  = "Monday";
static const char daysOfTheWeek_2[12]  = "Tuesday";
static const char daysOfTheWeek_3[12]  = "Wednesday";
static const char daysOfTheWeek_4[12]  = "Thursday";
static const char daysOfTheWeek_5[12]  = "Friday";
static const char daysOfTheWeek_6[12]  = "Saturday";
static const char* const daysOfTheWeek[]  = {daysOfTheWeek_0,
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
  int32_t fill_flow;
};

struct PumpFlowRate {
		unsigned long last;
		unsigned long last_read;
		unsigned long clicks;
};

struct ProgramData {
  int current;
  int32_t level_target;
  uint8_t run_pump_speed;
  uint8_t drain_pump_speed;
  uint8_t prime;
  DateTime boost_time;
  int32_t boost_duration;
  int boost_program;
  uint8_t boost_pump_speed;
  int32_t boost_counter;
};


// Function definitions

void pump_flow_ISR();
void fill_flow_ISR();
bool eeprom_read(void);
bool eeprom_write(void);
void process_eps_pump();
void process_telemetry(uint32_t now);
void read_sensors(DataReadings *readings);
bool get_flow_switch(void);
int32_t get_fill_flow(void);
int32_t get_pump_flow(void);
void publish_readings(DataReadings *readings, uint32_t now);
void print_readings(DataReadings *readings);
void setup_sensors(void);
int32_t get_water_sensor_level(void);
int32_t get_pump_pressure(void);
void setup_clock(void);
void set_clock(void);
void set_pump_speed(uint8_t speed);
int get_pump_speed(void);
void set_pump_stop(bool stop);
bool get_pump_stop();
void setup_wifi(void);
void wifi_connect();
void mqtt_connect(PubSubClient *client, const char* name, const char* uname, 
        const char* pass, const char* subscribe[]);
void mqtt_publish_data(const char *pub, uint32_t timestamp, int32_t val, int persist);
void mqtt_callback(char* topic, byte* payload, unsigned int length);
void tb_mqtt_callback(char* topic, byte* payload, unsigned int length);
void write_state(void);
void wifi_list_networks();
void tb_publish_readings(PubSubClient *client, DataReadings *readings, uint32_t now);


