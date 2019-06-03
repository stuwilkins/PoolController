// 
// Copyright (c) 2018 Stuart B. WIlkins
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include <filter.h>

// VERSION INFO
#ifndef VERSION
#define VERSION                             "UNKNOWN"
#endif

// Buffer lengths
#define JSON_BUFFER_LEN                     500

// Analog input for level meter

#define MAX31865_CS                         19
#define MAX31865_RREF                       430.0
#define MAX31865_RNOMINAL                   100.0

#define WATER_LEVEL_PIN		                A0
//#define WATER_LEVEL_X     	                79.69
//#define WATER_LEVEL_C     	                -36455
#define WATER_LEVEL_X     	                0.0124515
#define WATER_LEVEL_C     	                -364.55

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
#define EPS_PUMP_START_TIMEOUT              30000L
#define EPS_PUMP_FLOW_RATE                  7.5

#define WATCHDOG_TIME                       8000

#define EEPROM_PROGRAM_DATA                 16
#define EEPROM_MAGIC_DATA                   0
#define EEPROM_MAGIC                        0xDEADF00D
#define EEPROM_VERSION                      17

#define MQTT_CLIENT_NAME                    "pool_controller"

#define NTP_ADDRESS                         "north-america.pool.ntp.org"
#define NTP_OFFSET                          -4 * 60 * 60 // In seconds
#define NTP_INTERVAL                        60 * 1000 // In miliseconds

#define PROGRAM_LEVEL_TARGET                25500
#define PUMP_RUN_SPEED                      5
#define PUMP_DRAIN_SPEED                    7
#define PUMP_BOOST_SPEED                    7

#define UPLOAD_WINDOW                       15000

#define ALPHA                               0.025

#define TB_TELEMETRY_TOPIC                  "v1/devices/me/telemetry"
#define TELEMETRY_TOPIC                     "home/poolcontroller/telemetry"
#define TB_ATTRIBUTES_TOPIC                 "v1/devices/me/attributes"
#define ATTRIBUTES_TOPIC                    "home/poolcontroller/attributes"

#define SYSLOG_PORT                         514

#define AC_OUTPUT_0                         17
#define AC_OUTPUT_1                         18

#define PUMP_PRIME_STOP_DURATION            60
#define PUMP_PRIME_INTERVAL                 12
#define PUMP_PRIME_MEASURE_DURATION         270

// Cl Dosing
// 33.6 ml per minute


// Programs to run
enum programs {
    PROGRAM_HALT         = 0,
    PROGRAM_RUN          = 1,
    PROGRAM_TIMER        = 2,
    PROGRAM_DRAIN        = 3,
};

enum switch_programs {
    SWITCH_PROGRAM_OFF   = 0,
    SWITCH_PROGRAM_ON    = 1,
    SWITCH_PROGRAM_RUN   = 2,
    SWITCH_PROGRAM_ABORT = 3
};

enum prime {
    PUMP_PRIME_IDLE      = 0,
    PUMP_PRIME_OFF       = 1,
    PUMP_PRIME_RESTART   = 2,
    PUMP_PRIME_MEASURE   = 2,
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

extern template class ExpFilter<float>;

struct DataReadings {
    float water_temp;
    float water_level;
    float pump_pressure;
    ExpFilter<float> water_temp_s;
    ExpFilter<float> water_level_s;
    ExpFilter<float> pump_pressure_s;
    bool flow_switch;
    uint8_t pump_speed;
    bool pump_stop;
    float pump_flow;
    float fill_flow;
	unsigned long loop_time;
	unsigned long loop_time_telemetry;
    unsigned long millis;
    unsigned long unix_time;
    bool valid;
    int cl_output;
    int robot_output;
};

struct PumpFlowRate {
    unsigned long last;
    unsigned long last_read;
    unsigned long clicks;
};

struct Switch {
    DateTime time;
    int program;
    int32_t duration;
    int32_t counter;
    bool output;
};

struct BoostSwitch {
    DateTime time;
    int program;
    int32_t duration;
    int32_t counter;
    bool output;
    uint8_t pump_speed;
};

struct EPS {
    bool error;
    bool fault;
    DateTime time;
    DateTime start_time;
};

struct Pump {
    uint8_t run_speed;
    uint8_t drain_speed;
    uint8_t boost_speed;
    uint8_t speed;
    bool stop;
    DateTime prime_start;
    uint8_t prime_state;
};

struct ProgramData {
    int current;
    float level_target;
    Pump pump;
    Switch boost;
    Switch cl_pump;
    Switch robot;
    EPS eps;
    DateTime last_run;
    DateTime last_telemetry;
    int update_interval;
    float alpha;
    bool force_update;
};


// Function definitions

void pump_flow_ISR();
void fill_flow_ISR();
bool eeprom_read(void);
bool eeprom_write(void);
void process_eps(DateTime now);
void process_telemetry(uint32_t now);
void read_sensors();
void read_slow_sensors(DataReadings *readings);
bool get_flow_switch(void);
float get_fill_flow(void);
float get_pump_flow(void);
void print_readings(DataReadings *readings, uint16_t log_level);
void setup_sensors(void);
float get_water_sensor_level(void);
float get_pump_pressure(void);
void setup_clock(void);
void set_clock(void);
bool set_pump(bool stop, uint8_t speed);
void get_pump(bool &stop, uint8_t &speed);
void setup_wifi(void);
void tb_rpc_callback(char* topic, byte* payload, unsigned int length);
void write_state(void);
void publish_readings(PubSubClient *client, const char *topic, const char *buffer);
void upload_telemetry(uint32_t now);
void upload_telemetry_prime(uint32_t now);
void upload_attributes(DateTime *now);
void upload_attributes_start(DateTime *now);
float read_pt100_sensor(Adafruit_MAX31865 *sensor);
void make_datetime(char* buffer, size_t len, DateTime *now);
void check_switch(Switch &prog, DateTime &now, const char* progname);
void handle_error(const char *err);

