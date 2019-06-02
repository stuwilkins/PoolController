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

#include <Wire.h>
#include <WiFi101.h>
#include <WiFi101OTA.h>
#include <WiFiUdp.h>
#include <Adafruit_SleepyDog.h>
#include <Adafruit_MAX31865.h>
#include <pt100rtd.h>
#include <RTClib.h>
#include <PubSubClient.h>
#include <NTPClient.h>
#include <eeprom_i2c.h>
#include <Syslog.h>
#include <ArduinoJson.h>
#include <MCP23008.h>
#include <pushsafer.h>
#include <filter.h>
#include <network.h>

#include "PoolController.h"
#include "auth.h"
#include "timer.h"
#include "program.h"

#define USE_EEPROM 1

// Setup Sensors
Adafruit_MAX31865 water_pt100 = Adafruit_MAX31865(MAX31865_CS);

// PT100 Conversion Routines
pt100rtd PT100 = pt100rtd();

// Realtime Clock (RTClib)
#ifdef SOFT_RTC
RTC_Millis rtc;
#else
RTC_DS3231 rtc;
#endif

// EEPROM
EEPROM_I2C eeprom = EEPROM_I2C(0x50);

// WIFI and MQTT setup
WiFiUDP ntp_udp;
NTPClient ntp_client(ntp_udp, NTP_ADDRESS, NTP_OFFSET, NTP_INTERVAL);

WiFiClient wifi_client;
PubSubClient mqtt_client(wifi_client);
WiFiClient tb_wifi_client;
PubSubClient tb_mqtt_client(tb_wifi_client);

WiFiUDP syslog_udp;
Syslog syslog(syslog_udp, SYSLOG_SERVER, SYSLOG_PORT, DEVICE_HOSTNAME, APP_NAME, LOG_KERN);

WiFiClient pushsafer_client;
Pushsafer pushsafer(PUSHSAFER_KEY, pushsafer_client);

static const char* tb_mqtt_subscribe[]  = {"v1/devices/me/rpc/request/+", 0};
static const char* mqtt_subscribe[]  = {"home/poolcontroller/request/+", 0};

// Data Readings
DataReadings sensor_readings;
DateTime last;
DateTime last_telemetry;
DateTime restart_pump;
int prime_stop = 0;

// Running program
ProgramData program_data;
unsigned long loop_time_millis;

// ISR Data
volatile PumpFlowRate pump_flow;
volatile PumpFlowRate fill_flow;
volatile bool flow_switch;
volatile unsigned long flow_switch_millis;

// Error handler which pauses and flashes the L led. 
void error(const __FlashStringHelper *err) {
	Serial.println(err);
	while(1)
	{
		digitalWrite(OUTPUT_LED_L, !digitalRead(OUTPUT_LED_L));
		delay(100);
	}
}

// ISR Routines for flow rate meters

void flow_switch_ISR()
{
    if(!digitalRead(FLOW_SWITCH))
    {
       flow_switch = true;
       flow_switch_millis = millis();
    } else {
       flow_switch = false;
       flow_switch_millis = millis();
    }
}

void pump_flow_ISR()
{
	pump_flow.last = millis();
	pump_flow.clicks++;
}

void fill_flow_ISR()
{
	fill_flow.last = millis();
	fill_flow.clicks++;
}

void setup() {
	// Set digital pins 

	pinMode(OUTPUT_LED_L, OUTPUT);   
	pinMode(PUMP_STOP_BIT, OUTPUT);  
	pinMode(PUMP_BIT_0, OUTPUT);  
	pinMode(PUMP_BIT_1, OUTPUT);
	pinMode(PUMP_BIT_2, OUTPUT);
	pinMode(FLOW_SWITCH, INPUT_PULLUP);
	pinMode(PUMP_FLOW_PIN, INPUT_PULLUP);
	pinMode(FILL_FLOW_PIN, INPUT_PULLUP);
	pinMode(AC_OUTPUT_0, OUTPUT);
	pinMode(AC_OUTPUT_1, OUTPUT);

    // Set to 16bit ADC resolution
    analogReadResolution(16);

    set_pump_stop(1);
    digitalWrite(AC_OUTPUT_0, 0);
    digitalWrite(AC_OUTPUT_1, 0);

	Serial.begin(115200, 0);
        
	// Set the L led hight to show we are configuring
	digitalWrite(OUTPUT_LED_L, HIGH);

    int countdownMS = Watchdog.enable(WATCHDOG_TIME);

	// Setup WIFI

	setup_wifi();
	wifi_connect(WIFI_SSID, WIFI_PASSWORD);

    // Now WiFi is up, connect to the syslog console
	syslog_udp.begin(SYSLOG_PORT);
    syslog.serial(&Serial);
    syslog.serialMask(0x7F);
    syslog.logMask(0x7F);
    syslog.log(LOG_CRIT, "setup()");
    syslog.logf(LOG_CRIT, "VERSION = %s", VERSION);
    syslog.logf(LOG_CRIT, "Watchdog time %d miliseconds", countdownMS);

	// start the WiFi OTA library
	syslog.log("Setup OTA Updates");
	WiFiOTA.begin(OTA_NAME, OTA_PASSWORD, InternalStorage);

	Watchdog.reset();  // Pet the dog!
    
    // Enter a loop to wait for programming if fault
    syslog.logf(LOG_CRIT, "Entering upload window (%d ms)", UPLOAD_WINDOW);
    unsigned long _t = millis();
    while((millis() - _t) < UPLOAD_WINDOW)
    {
        WiFiOTA.poll();
        Watchdog.reset();  // Pet the dog!
        delay(250);
    }
    syslog.log(LOG_CRIT, "Upload window ended");

	Watchdog.reset();  // Pet the dog!

	mqtt_client.setServer(MQTT_SERVER, MQTT_PORT);
	mqtt_client.setCallback(tb_rpc_callback);
	tb_mqtt_client.setServer(TB_MQTT_SERVER, TB_MQTT_PORT);
	tb_mqtt_client.setCallback(tb_rpc_callback);
    syslog.log(LOG_INFO, "MQTT setup finished");
	Watchdog.reset();  // Pet the dog!

	ntp_client.begin();
    syslog.log(LOG_INFO, "NTP time setup finished");
	Watchdog.reset();

	// Setup Sensors
	setup_sensors();
    syslog.log(LOG_INFO, "Sensor setup finished");
	Watchdog.reset();  // Pet the dog!

#ifdef USE_EEPROM
	// Setup eeprom
	eeprom.begin();
    syslog.log(LOG_INFO, "EEPROM setup finished");
#endif

	// Setup RTC
	setup_clock();
    syslog.log(LOG_INFO, "RTC setup finished");
	Watchdog.reset();  // Pet the dog!

	// Set defaults
	last = rtc.now();
	last_telemetry = rtc.now();

	sensor_readings.error_count = 0;
	sensor_readings.valid = false;
    sensor_readings.water_temp_s.setAlpha(ALPHA);
    sensor_readings.water_level_s.setAlpha(ALPHA);
    sensor_readings.pump_pressure_s.setAlpha(ALPHA);
    sensor_readings.cl_output = 0;
    sensor_readings.robot_output = 0;

	pump_flow.last = millis();
	pump_flow.clicks = 0;

	fill_flow.last = millis();
	fill_flow.clicks = 0;

    flow_switch_millis = millis();
    flow_switch = !digitalRead(FLOW_SWITCH);

	program_data.current = PROGRAM_RUN;
	program_data.level_target = PROGRAM_LEVEL_TARGET;
	program_data.run_pump_speed = PROGRAM_PUMP_RUN_SPEED;
	program_data.drain_pump_speed = PROGRAM_PUMP_DRAIN_SPEED;
	program_data.prime = 0;
    program_data.update_interval = 10;
    program_data.alpha = ALPHA;
    program_data.force_update = false;

	program_data.boost_time = last;
	program_data.boost_pump_speed = PROGRAM_PUMP_BOOST_SPEED;
	program_data.boost_duration = 60 * 60 * 8; // 30m
	program_data.boost_counter = 0;

    program_data.cl_pump.program = SWITCH_PROGRAM_OFF;
    program_data.cl_pump.time = last;
    program_data.cl_pump.duration = 0.5 * 60 * 60;
    program_data.cl_pump.counter = 0;
    program_data.cl_pump.output = false;

    program_data.robot.program = SWITCH_PROGRAM_OFF;
    program_data.robot.time = last;
    program_data.robot.duration = 2 * 60 * 60;
    program_data.robot.counter = 0;
    program_data.robot.output = false;

    syslog.log(LOG_INFO, "Default setup finished");

#ifdef USE_EEPROM
	// Now restore from memory
	eeprom_read();
#endif

	// Setup ISR
    syslog.log(LOG_INFO, "Setup interrupts");

	attachInterrupt(digitalPinToInterrupt(PUMP_FLOW_PIN), 
					pump_flow_ISR, FALLING);	
	attachInterrupt(digitalPinToInterrupt(FILL_FLOW_PIN), 
					fill_flow_ISR, FALLING);	
	attachInterrupt(digitalPinToInterrupt(FLOW_SWITCH), 
					flow_switch_ISR, CHANGE);	

	// Do WIFI and MQTT Connection
	wifi_connect(WIFI_SSID, WIFI_PASSWORD);
	Watchdog.reset(); // Pet the dog!
	mqtt_connect(&mqtt_client, MQTT_NAME, NULL, NULL, mqtt_subscribe);
	Watchdog.reset(); // Pet the dog!
	mqtt_connect(&tb_mqtt_client, TB_MQTT_NAME, TB_MQTT_USERNAME, "", tb_mqtt_subscribe);
	Watchdog.reset(); // Pet the dog!

    // Upload attributes on load
    DateTime now = rtc.now();
    upload_attributes_start(&now);

    send_push_event("Controller started... setup() finished", "1");
	Watchdog.reset(); // Pet the dog!
    syslog.log(LOG_INFO, "End of setup()");

	// Set L low to show we are initialized.
	digitalWrite(OUTPUT_LED_L, LOW);

}

bool eeprom_write(void)
{
	uint64_t magic = EEPROM_MAGIC;

    // Add the version
    uint64_t version = EEPROM_VERSION; 
    magic |= (version << 32);

    int rtn;
	rtn = eeprom.writeIfDiff(EEPROM_MAGIC_DATA, (uint8_t *)(&magic), sizeof(magic));
    if(rtn != EEPROM_I2C::NO_WRITE)
    {
        syslog.logf(LOG_INFO, "EEPROM magic does not match, written to EEPROM");
    }
	rtn = eeprom.writeIfDiff(EEPROM_PROGRAM_DATA, (uint8_t *)(&program_data), sizeof(program_data), true, true);
    if(rtn != EEPROM_I2C::NO_WRITE)
    {
        syslog.logf(LOG_DEBUG, "EEPROM data written, data does not match");
    }

	return true;
}

bool eeprom_read(void)
{
	uint64_t magic;

	if(eeprom.read(EEPROM_MAGIC_DATA, (uint8_t*)(&magic), sizeof(magic)))
	{
		syslog.log(LOG_ERR, "Unable to read MAGIC from EEPROM");
		return false;
	}

	uint64_t _magic = EEPROM_MAGIC;
    uint64_t _version = EEPROM_VERSION;
    _magic |= (_version << 32);
	if(magic != _magic)
	{
		syslog.log(LOG_ERR, "EPROM Magic does not match");
		return false;
	}

	int rc;
	if((rc = eeprom.read(EEPROM_PROGRAM_DATA, (uint8_t *)(&program_data), sizeof(program_data), true)))
	{
		syslog.logf(LOG_ERR, "Failed to read EEPROM Data rc = %d", rc);
		return false;
	}

	return true;
}

void loop() {
	DateTime now;
    bool telemetry = false;

	// Pet the dog!
	Watchdog.reset();

	// Do WIFI and MQTT Connection
    syslog.log(LOG_DEBUG, "wifi_connect()");
	wifi_connect(WIFI_SSID, WIFI_PASSWORD);
	Watchdog.reset(); // Pet the dog!
    syslog.log(LOG_DEBUG, "mqtt_connect()");
	mqtt_connect(&mqtt_client, MQTT_NAME, NULL, NULL, mqtt_subscribe);
	Watchdog.reset(); // Pet the dog!
	mqtt_connect(&tb_mqtt_client, TB_MQTT_NAME, TB_MQTT_USERNAME, "", tb_mqtt_subscribe);
	Watchdog.reset(); // Pet the dog!

	// Poll for over the air updates
    syslog.log(LOG_DEBUG, "WiFiOTA.poll()");
	WiFiOTA.poll();
	Watchdog.reset(); // Pet the dog!

	// Poll for MQTT updates
    syslog.log(LOG_DEBUG, "mqtt_client.loop()");
	tb_mqtt_client.loop();
	mqtt_client.loop();
	Watchdog.reset(); // Pet the dog!
    
    // Sync NTP client
    //syslog.log(LOG_DEBUG, "ntp_client.update()");
    //if(!ntp_client.update())
    //{
    //    syslog.log(LOG_CRIT, "Unable to update from NTP server");
    //}
	//Watchdog.reset(); // Pet the dog!

	// Read the current time
    syslog.log(LOG_DEBUG, "Read RTC");
	now = rtc.now();
	sensor_readings.unix_time = now.unixtime();
	Watchdog.reset(); // Pet the dog!
    
    // Each loop read sensors
    read_sensors();
    Watchdog.reset();  // Pet the dog!

	if(now.hour() != last.hour())
	{
        print_readings(&sensor_readings, LOG_INFO);

		// We have a new hour
		if(((now.hour() % 6) == 0) && 
			((program_data.current == PROGRAM_RUN) ||
			 (program_data.current == PROGRAM_TIMER)))
		{
			program_data.prime = 1;
		}
	}

	if(program_data.prime)
	{
		restart_pump = now;
		prime_stop = 1;
		program_data.prime = 0;
		set_pump_stop(1);
	}

	// Every second call poll
	if(now.second() != last.second()) 
	{
        // Each second read slow sensors
        read_slow_sensors(&sensor_readings);
        Watchdog.reset();  // Pet the dog!

        // Now check for EPS for Cl Pump
        if(program_data.cl_pump.program != SWITCH_PROGRAM_OFF)
        {
            process_eps_cl_pump();
        }

		// Now check for running programs
        
        syslog.logf(LOG_DEBUG, "Program = %d", program_data.current);

		if(program_data.current == PROGRAM_TIMER)
		{
			// Running TIMER Program
			process_eps_pump();
			program_data.current = PROGRAM_HALT;
		}

		if(program_data.current == PROGRAM_BOOST)
		{
			// Check if time is up
			int32_t delta = (program_data.boost_time - now).totalseconds();
			if(delta <= 0)
			{
				// We are done
				program_data.current = program_data.boost_program;
				program_data.boost_counter = 0;
			} else {
				if(get_pump_speed() != program_data.boost_pump_speed)
				{
					set_pump_speed(program_data.boost_pump_speed);
				}
				program_data.boost_counter = delta;
			}
		}
		
		if(program_data.current == PROGRAM_DRAIN) {
			// Running DRAIN Program
			float _wl = get_water_sensor_level();
			if(_wl < program_data.level_target)
			{
				program_data.current = PROGRAM_HALT;
			} else {
				if(get_pump_speed() != program_data.drain_pump_speed)
				{
					set_pump_speed(program_data.drain_pump_speed);
				}
			}
		}
		
		if(program_data.current == PROGRAM_FILL) {
			// Running FILL Program
			program_data.current = PROGRAM_HALT;
		}

		if(program_data.current == PROGRAM_RUN) {
			// Running RUN program
			process_eps_pump();
			if(get_pump_speed() != program_data.run_pump_speed)
			{
				set_pump_speed(program_data.run_pump_speed);
                syslog.logf(LOG_INFO, "Set pump speed to : %d", program_data.run_pump_speed);
			}
		}
		
		if(program_data.current == PROGRAM_HALT) {
			// Program Halted
			// All stop! 
			if(get_pump_speed() != 0)
			{
				set_pump_speed(0);
			}
			if(!get_pump_stop())
			{
				set_pump_stop(true);
			}
		}

		if((prime_stop == 1) && ((now - restart_pump).totalseconds() > 30))
		{
			prime_stop = 2;
			set_pump_stop(false);
		}

		if((prime_stop == 2) && ((now - restart_pump).totalseconds() > 270))
		{
			prime_stop = 0;
			// store flow and pressure during prime. 
            syslog.logf(LOG_INFO, "Storing prime values");
            upload_telemetry_prime(now.unixtime() - NTP_OFFSET);
		}

        Watchdog.reset(); // Pet the dog!

#ifdef USE_EEPROM
        syslog.log(LOG_DEBUG, "eeprom_write()");
		eeprom_write();
        Watchdog.reset(); // Pet the dog!
#endif
        last = now;
    }

    if(((now - last_telemetry).totalseconds() >= program_data.update_interval) ||
            program_data.force_update)
    {
        // Flash the L led
        digitalWrite(OUTPUT_LED_L, !digitalRead(OUTPUT_LED_L));
        
        // Publish with MQTT
        upload_telemetry(now.unixtime() - NTP_OFFSET);
        upload_attributes(&now);
        Watchdog.reset();  // Pet the dog!
        
        telemetry = true;
        last_telemetry = now;
        program_data.force_update = false;
	} 

    // Check switch programs
    if(program_data.robot.program == SWITCH_PROGRAM_ON)
    {
        // We turned on
        program_data.robot.time = now + program_data.robot.duration;
        program_data.robot.program = SWITCH_PROGRAM_RUN;
        program_data.robot.output = true;
        syslog.log(LOG_INFO, "Robot Program ON");
        send_push_event("Robot Program ON", "1");
    }

    if(program_data.robot.program == SWITCH_PROGRAM_ABORT)
    {
        program_data.robot.output = false;
        program_data.robot.program = SWITCH_PROGRAM_OFF;
        program_data.robot.counter = 0;
        syslog.log(LOG_INFO, "Robot Program Aborted");
        send_push_event("Robot Program Aborted", "1");
    }

    if(program_data.robot.program == SWITCH_PROGRAM_RUN)
    {
        // We are off and running
        program_data.robot.counter = (program_data.robot.time - now).totalseconds();
        if(program_data.robot.counter <= 0)
        {
            // we finished
            program_data.robot.output = false;
            program_data.robot.program = SWITCH_PROGRAM_OFF;
            program_data.robot.counter = 0;
            syslog.log(LOG_INFO, "Robot Program OFF");
            send_push_event("Robot Program OFF", "1");
        }
    }
    
    // Check switch programs
    if(program_data.cl_pump.program == SWITCH_PROGRAM_ON)
    {
        // We turned on
        program_data.cl_pump.time = now + program_data.cl_pump.duration;
        program_data.cl_pump.program = SWITCH_PROGRAM_RUN;
        program_data.cl_pump.output = true;
        syslog.log(LOG_INFO, "Chlorine Program ON");
        send_push_event("Chlorine Program ON", "1");
    }

    if(program_data.cl_pump.program == SWITCH_PROGRAM_ABORT)
    {
        program_data.cl_pump.output = false;
        program_data.cl_pump.program = SWITCH_PROGRAM_OFF;
        program_data.cl_pump.counter = 0;
        syslog.log(LOG_INFO, "Cl Pump Program Aborted");
        send_push_event("Cl Pump Program Aborted", "1");
    }

    if(program_data.cl_pump.program == SWITCH_PROGRAM_RUN)
    {
        // We are off and running
        program_data.cl_pump.counter = (program_data.cl_pump.time - now).totalseconds();
        if(program_data.cl_pump.counter <= 0)
        {
            // we finished
            program_data.cl_pump.output = false;
            program_data.cl_pump.program = SWITCH_PROGRAM_OFF;
            program_data.cl_pump.counter = 0;
            syslog.log(LOG_INFO, "Chlorine Program OFF");
            send_push_event("Chlorine Program OFF", "1");
        }
    }

    // Now set switches
    digitalWrite(AC_OUTPUT_0, program_data.cl_pump.output);
    digitalWrite(AC_OUTPUT_1, program_data.robot.output);

    if(telemetry)
    {
        sensor_readings.loop_time_telemetry = millis() - loop_time_millis;
    } else {
        sensor_readings.loop_time = millis() - loop_time_millis;
    }

    loop_time_millis = millis();
}

void process_eps_cl_pump()
{
    syslog.log(LOG_DEBUG, "process_eps_cl_pump()");

	bool error = !flow_switch;
    unsigned long switch_delta = millis() - flow_switch_millis;

    if(error)
    {
        syslog.logf(LOG_ERR, "EPS : Pump flow error (Cl Pump), delta = %ld ms, flow = %f gpm", 
                switch_delta, sensor_readings.pump_flow);
        if(switch_delta > EPS_PUMP_FLOW_TIMEOUT_CL)
        {
            // We need to turn off CL
            program_data.cl_pump.program = SWITCH_PROGRAM_OFF;
            syslog.log(LOG_ERR, "EPS : Flow error - turning off Cl pump");
            send_push_event("EPS detected flow error. Cl pump turned off", "2");
        }
    }
}

void process_eps_pump()
{
    syslog.log(LOG_DEBUG, "process_eps_pump()");

    // Check if we recently started pump
    unsigned long delta = millis() - program_data.pump_start_counter;
    if(delta < EPS_PUMP_START_TIMEOUT)
    {
        syslog.logf(LOG_DEBUG, "EPS : Pump started %ld ms ago, ignoring EPS timeout is %ld", 
                delta, EPS_PUMP_START_TIMEOUT);
        return;
    }

	// Now check for fault conditions (pump running, no flow)
	uint8_t _pump = get_pump_speed();
	bool _pump_stop = get_pump_stop();

	bool error = (!flow_switch && _pump && !_pump_stop);
    unsigned long switch_delta = millis() - flow_switch_millis;

    if(error)
    {
        syslog.logf(LOG_ERR, "EPS : Pump flow error (Main Program), delta = %ld ms, flow = %f gpm", 
                switch_delta, sensor_readings.pump_flow);
        if(switch_delta > EPS_PUMP_FLOW_TIMEOUT)
        {
            // We have a flow error
            set_pump_stop(1);
            set_pump_speed(0);
            program_data.current = PROGRAM_HALT;

            syslog.logf(LOG_ERR, "EPS : Flow error - turning off pump (%ld ms)", switch_delta); 
            send_push_event("EPS detected flow error. Pump turned off", "2");

            if(program_data.cl_pump.program)
            {
                // We need to turn off CL
                program_data.cl_pump.program = SWITCH_PROGRAM_OFF;
                syslog.log(LOG_ERR, "EPS : Flow error - turning off Cl pump");
                send_push_event("EPS detected flow error. Cl pump turned off", "2");
            }

        }
    }
}

void read_slow_sensors(DataReadings *readings)
{
	readings->pump_flow = get_pump_flow();
	readings->fill_flow = get_fill_flow();
}

float read_pt100_sensor(Adafruit_MAX31865 *sensor)
{
	uint8_t fault = sensor->readFault();
	if(fault)
	{
        syslog.logf(LOG_CRIT, "RTD Fault = %d", fault);
		sensor->clearFault();
		return 0.0;
	} else {
		uint16_t rtd, ohmsx100;
		uint32_t dummy;

		rtd = sensor->readRTD();
		dummy = ((uint32_t)(rtd << 1)) * 100 * ((uint32_t) floor(MAX31865_RREF)) ;
		dummy >>= 16 ;
		ohmsx100 = (uint16_t)(dummy & 0xFFFF);
        // ohms = (float)(ohmsx100 / 100) + ((float)(ohmsx100 % 100) / 100.0) ;

		return PT100.celsius(ohmsx100);
	}
} 

void read_sensors()
{

    sensor_readings.water_temp = read_pt100_sensor(&water_pt100);
	sensor_readings.water_level = get_water_sensor_level();
	sensor_readings.pump_pressure = get_pump_pressure();
	sensor_readings.flow_switch = get_flow_switch();
	sensor_readings.pump_speed = get_pump_speed();

    sensor_readings.millis = millis();

    sensor_readings.cl_output = digitalRead(AC_OUTPUT_0);
    sensor_readings.robot_output = digitalRead(AC_OUTPUT_1);

    // Now filter readings
  
    sensor_readings.water_level_s.newValue(sensor_readings.water_level);
    sensor_readings.water_temp_s.newValue(sensor_readings.water_temp);
    sensor_readings.pump_pressure_s.newValue(sensor_readings.pump_pressure);
}

bool get_flow_switch(void)
{
	return flow_switch;
}

float get_fill_flow(void)
{
	// Get current values
	long int delta = millis() - fill_flow.last_read;
	long int clicks = fill_flow.clicks;

	// Zero values
	fill_flow.clicks = 0;
	fill_flow.last_read = millis();

	float _flow = (float)(1000 * clicks) / (float)delta;

	_flow = _flow * FILL_FLOW_F_TO_Q;
	_flow = _flow * LPM_TO_GPM;

	return _flow;
}	

float get_pump_flow(void)
{
	// Get current values
	long int delta = millis() - pump_flow.last_read;
	long int clicks = pump_flow.clicks;

	// Zero values
	pump_flow.clicks = 0;
	pump_flow.last_read = millis();

	float _flow = (float)(1000 * clicks) / (float)delta;

	_flow = _flow * PUMP_FLOW_F_TO_Q;
	_flow = _flow * LPM_TO_GPM;

	return _flow;
}	

void upload_attributes(DateTime *now)
{
    StaticJsonDocument<JSON_BUFFER_LEN> attributes;
    char _date[128];

    make_datetime(_date, 128, now);

    attributes["loop_time"] = sensor_readings.loop_time;
    attributes["loop_time_telemetry"] = sensor_readings.loop_time_telemetry;
    attributes["boost_counter"] = program_data.boost_counter;
    attributes["cl_counter"] = program_data.cl_pump.counter;
    attributes["robot_counter"] = program_data.robot.counter;
    attributes["unix_time"] = sensor_readings.unix_time;
    attributes["pump_speed_val"] = sensor_readings.pump_speed;
    attributes["datetime"] = _date;

    char buffer[JSON_BUFFER_LEN];

    serializeJson(attributes, buffer, JSON_BUFFER_LEN);

    publish_readings(&tb_mqtt_client, TB_ATTRIBUTES_TOPIC, buffer);
    publish_readings(&mqtt_client, ATTRIBUTES_TOPIC, buffer);
}

void upload_attributes_start(DateTime *now)
{
    StaticJsonDocument<JSON_BUFFER_LEN> attributes;
    char _date[128];

    make_datetime(_date, 128, now);

    attributes["start_unix_time"] = now->unixtime();
    attributes["start_datetime"] = _date;
    attributes["version"] = VERSION;

    char buffer[JSON_BUFFER_LEN];

    serializeJson(attributes, buffer, JSON_BUFFER_LEN);

    publish_readings(&tb_mqtt_client, TB_ATTRIBUTES_TOPIC, buffer);
    publish_readings(&mqtt_client, ATTRIBUTES_TOPIC, buffer);
}

void upload_telemetry(uint32_t now)
{
    StaticJsonDocument<JSON_BUFFER_LEN> telemetry_root;
    JsonObject telemetry_values = telemetry_root.createNestedObject("values");

    telemetry_values["water_temp_s"] = sensor_readings.water_temp_s.getValue();
    telemetry_values["water_level_s"] = sensor_readings.water_level_s.getValue();
    telemetry_values["pump_pressure_s"] = sensor_readings.pump_pressure_s.getValue();
    telemetry_values["flow_switch"] = sensor_readings.flow_switch;
    telemetry_values["pump_flow"] = sensor_readings.pump_flow;
    telemetry_values["pump_speed"] = pumpSpeeds[sensor_readings.pump_speed];
    telemetry_values["program"] = program_data.current;
    telemetry_values["cl_output"] = sensor_readings.cl_output;
    telemetry_values["robot_output"] = sensor_readings.robot_output;

    telemetry_root["ts"] = String(now) + "000"; // This gets round the lack of uint64_t conversion

	Watchdog.reset();  // Pet the dog!

    char buffer[JSON_BUFFER_LEN];

    serializeJson(telemetry_root, buffer, JSON_BUFFER_LEN);

    publish_readings(&tb_mqtt_client, TB_TELEMETRY_TOPIC, buffer);
    publish_readings(&mqtt_client, TELEMETRY_TOPIC, buffer);
}

void upload_telemetry_prime(uint32_t now)
{
    StaticJsonDocument<JSON_BUFFER_LEN> telemetry_root;
    JsonObject telemetry_values = telemetry_root.createNestedObject("values");

    telemetry_values["prime_pump_pressure"] = sensor_readings.pump_pressure;
    telemetry_values["prime_pump_flow"] = sensor_readings.pump_flow;

    telemetry_root["ts"] = String(now) + "000"; // This gets round the lack of uint64_t conversion

    char buffer[JSON_BUFFER_LEN];
    serializeJson(telemetry_root, buffer, JSON_BUFFER_LEN);

    publish_readings(&tb_mqtt_client, TB_TELEMETRY_TOPIC, buffer);
    publish_readings(&mqtt_client, TELEMETRY_TOPIC, buffer);
}


void publish_readings(PubSubClient *client, const char *topic, const char *buffer)
{
    if(client->connected())
    {
        if(!client->publish(topic, buffer, true))
        {
            syslog.logf(LOG_ERR, "Error publishing to %s", client->getServer());
        }
    } else {
        syslog.log(LOG_ERR, "Error publishing - not connected");
    }
}

void print_readings(DataReadings *readings, uint16_t log_level)
{

    syslog.logf(log_level, "Water Temp             : %f degC", readings->water_temp);
    syslog.logf(log_level, "Water Level            : %f mm", readings->water_level);
    syslog.logf(log_level, "Pump Pressure          : %f PSI", readings->pump_pressure);
    syslog.logf(log_level, "Water Temp Smoothed    : %f degC", readings->water_temp_s.getValue());
    syslog.logf(log_level, "Water Level Smoothed   : %f mm", readings->water_level_s.getValue());
    syslog.logf(log_level, "Pump Pressure Smoothed : %f PSI", readings->pump_pressure_s.getValue());
    if(readings->flow_switch)
    {
        syslog.log(log_level, "Flow Switch            : ON");
    } else {
        syslog.log(log_level, "Flow Switch            : OFF");
    }

    syslog.logf(log_level, "Speed                  : %d (%d)", readings->pump_speed,
            pumpSpeeds[readings->pump_speed]);
    syslog.logf(log_level, "Flow rate              : %f GPM", readings->pump_flow);
    syslog.logf(log_level, "Loop time              : %ld", readings->loop_time);
    syslog.logf(log_level, "Loop time (telemetry)  : %ld", readings->loop_time_telemetry);

}

void setup_sensors(void)
{
	// Setup MAX31865	
	if(!water_pt100.begin(MAX31865_3WIRE))
	{
        syslog.log(LOG_CRIT, "Unable to initialize MAX31865 sensor");
	}
}

float get_water_sensor_level(void)
{
	float val = (float)analogRead(WATER_LEVEL_PIN);

	val = (val * WATER_LEVEL_X) + WATER_LEVEL_C;

	return val;
}

float get_pump_pressure(void)
{
	float val = (float)analogRead(PUMP_PRESSURE_PIN);

	// Now do conversion
	val = 3300 * val;
	val = 2 * val / 65535;

	val = val - 335; // Zero point offset
	val = val * 75;
    val = val / 10000;

	return val;
}

void setup_clock(void)
{ 
#ifdef SOFT_RTC
	rtc.begin(DateTime(F(__DATE__), F(__TIME__)));
    set_clock();
#else
	if(!rtc.begin()) {
		syslog.log(LOG_CRIT, "Unable to start RTC");
	}

	if(rtc.lostPower())
	{
		syslog.log(LOG_ALERT, "RTC Lost Power");
		set_clock();
	}
#endif
}

void set_clock(void)
{
	if(!ntp_client.forceUpdate())
    {
        syslog.logf(LOG_CRIT, "Unable to update time from NTP Server");
        return;
    }
	DateTime now = rtc.now();
	unsigned long epoch_ntp = ntp_client.getEpochTime();
	unsigned long epoch_rtc = now.unixtime();

	long diff = epoch_ntp - epoch_rtc;

    syslog.logf(LOG_INFO, "RTC reports time as %ld", epoch_rtc);
    syslog.logf(LOG_INFO, "NTP reports time as %ld", epoch_ntp);
    syslog.logf(LOG_INFO, "RTC vs NTP diff is %ld", diff);

	rtc.adjust(epoch_ntp);
}

void set_pump_speed(uint8_t speed)
{
	// Set the pump bits
	digitalWrite(PUMP_BIT_0, speed & 0x1);
	digitalWrite(PUMP_BIT_1, speed & 0x2);
	digitalWrite(PUMP_BIT_2, speed & 0x4);

	// If the pump is stopped, start it.
	if(get_pump_stop())
	{
		set_pump_stop(0);
	}
}

uint8_t get_pump_speed(void)
{
	int bits = 0;
	bits |= digitalRead(PUMP_BIT_0);
	bits |= (digitalRead(PUMP_BIT_1) << 1);
	bits |= (digitalRead(PUMP_BIT_2) << 2);
	return bits;
}

void set_pump_stop(bool stop)
{
	digitalWrite(PUMP_STOP_BIT, stop);
    if(!stop)
    {
        // Start pump counter
        program_data.pump_start_counter = millis();
    }
}

bool get_pump_stop()
{
	return digitalRead(PUMP_STOP_BIT);
}

void setup_wifi(void)
{

	WiFi.setPins(8,7,4,2);
	if (WiFi.status() == WL_NO_SHIELD) {
		error(F("WiFi module not present"));
	}

	Watchdog.reset();
	Serial.println("Waiting 5s for descovery of networks");
	delay(5000);
	
	Watchdog.reset();
	wifi_list_networks();

	Watchdog.reset();
}

void tb_rpc_callback(char* topic, byte* payload, unsigned int length)
{

    Watchdog.reset(); // Reset here as this could stop the loop running?

    if((!length) || (length > 254))
    {
        syslog.log(LOG_ERR, "Invalid tb_mqtt callback");
    }

    payload[length] = 0;

    char *_req = strrchr(topic, '/');
    if(!_req)
    {
        syslog.log(LOG_ERR, "Unable to parse request ID");
        return;
    }

    int requestId = String(_req + 1).toInt();
    syslog.logf(LOG_DEBUG, "MQTT Message arrived [%s] %s (requestId = %d)", topic, payload, requestId);

    // Copy payload for later work
    char _payload[256];
    strncpy(_payload, (char*)payload, 256);

    StaticJsonDocument<JSON_BUFFER_LEN> json;
    DeserializationError error = deserializeJson(json, (char*)payload);

    if(error)
    {
        syslog.logf(LOG_ERR, "deserializeJson() failed: %s", error.c_str());
        return;
    } 

    const char* method = json["method"];
    syslog.logf(LOG_INFO, "Recieved RPC Request, Method = %s", method);

    String _response_topic = String("v1/devices/me/rpc/response/") + requestId;
    String _response_payload;

    if(!strcmp(method, "setPumpSpeed"))
    {
        int speed = json["params"];
        syslog.logf(LOG_INFO, "Setting pump speed to %d", speed);
        program_data.run_pump_speed = speed;
    } else if(!strcmp(method, "setRunProgram")) {
        int val = json["params"];
        if(val)
        {
            program_data.current = PROGRAM_RUN;
            syslog.logf(LOG_INFO, "Setting program to %d", program_data.current);
        }
    } else if(!strcmp(method, "setHaltProgram")) {
        int val = json["params"];
        if(val)
        {
            program_data.current = PROGRAM_HALT;
            syslog.logf(LOG_INFO, "Setting program to %d", program_data.current);
        }
    } else if(!strcmp(method, "setDrainProgram")) {
        int val = json["params"];
        if(val)
        {
            program_data.current = PROGRAM_DRAIN;
            syslog.logf(LOG_INFO, "Setting program to %d", program_data.current);
        }
    } else if(!strcmp(method, "setClProgram")) {
        int val = json["params"];
        if(val)
        {
            program_data.cl_pump.program = SWITCH_PROGRAM_ON;
            syslog.log(LOG_INFO, "Setting Cl program on");
        } else {
            program_data.cl_pump.program = SWITCH_PROGRAM_ABORT;
            syslog.log(LOG_INFO, "Setting Cl program off");
        }
    } else if(!strcmp(method, "setClTime")) {
        float val = json["params"];
        if(val)
        {
            program_data.cl_pump.duration = val * 60 * 60; // Need seconds from hrs
            syslog.logf(LOG_INFO, "Setting Cl pump duration to %ld", program_data.cl_pump.duration);
        }
    } else if(!strcmp(method, "setRobotProgram")) {
        int val = json["params"];
        if(val)
        {
            program_data.robot.program = SWITCH_PROGRAM_ON;
            syslog.log(LOG_INFO, "Setting robot program on");
        } else {
            program_data.robot.program = SWITCH_PROGRAM_ABORT;
            syslog.log(LOG_INFO, "Setting robot program off");
        }
    } else if(!strcmp(method, "setRobotTime")) {
        float val = json["params"];
        if(val)
        {
            program_data.robot.duration = val * 60 * 60; // Need seconds from hrs
            syslog.logf(LOG_INFO, "Setting robot duration to %ld", program_data.robot.duration);
        }
    } else if(!strcmp(method, "setBoostProgram")) {
        int val = json["params"];
        if(val)
        {
            program_data.boost_program = program_data.current;
            program_data.current = PROGRAM_BOOST;
            program_data.boost_time = rtc.now() + program_data.boost_duration;
            syslog.logf(LOG_INFO, "Setting program to %d", program_data.current);
        }
    } else if(!strcmp(method, "setBoostTime")) {
        float val = json["params"];
        if(val)
        {
            program_data.boost_duration = val * 60 * 60; // Need seconds from hrs
            syslog.logf(LOG_INFO, "Setting boost duration to %ld", program_data.boost_duration);
        }
    } else if(!strcmp(method, "getBoostTime")) {
        // Return the boost time for widget
        _response_payload = String(program_data.boost_duration / (60 * 60));
    } else if(!strcmp(method, "getClTime")) {
        // Return the boost time for widget
        _response_payload = String(program_data.cl_pump.duration / (60 * 60));
    } else if(!strcmp(method, "getRobotTime")) {
        // Return the boost time for widget
        _response_payload = String(program_data.robot.duration / (60 * 60));
    } else if(!strcmp(method, "getPumpSpeed")) {
        // Return the boost time for widget
        _response_payload = String(program_data.run_pump_speed);
    } else if(!strcmp(method, "getTermInfo")) {
        // Return the boost time for widget
        StaticJsonDocument<JSON_BUFFER_LEN> response;
        response["ok"] = true;
        response["platform"] = "arduino M0";
        response["type"] = "";
        response["release"] = VERSION;
        serializeJson(response, _response_payload);
    } else if(!strcmp(method, "sendCommand")) {
        char *command = json["params"]["command"];
        syslog.logf(LOG_INFO, "Command = %s", command);
        _response_payload = String("Hello World");
    } else {
        return;
    }
    
    // Echo back by publishing

    //serializeJson(json_response, buffer, JSON_BUFFER_LEN);
    tb_mqtt_client.publish(_response_topic.c_str(), _response_payload.c_str());
    program_data.force_update = true;
}

void make_datetime(char* buffer, size_t len, DateTime *now)
{
    snprintf(buffer, len, "Date is %02d/%02d/%02d (%s) %02d:%02d:%02d", 
            now->year(), now->month(), now->day(), 
            daysOfTheWeek[now->dayOfTheWeek()],
            now->hour(), now->minute(), now->second());
}

bool send_push_event(const char* message, const char* priority)
{
    struct PushSaferInput input;
    input.message = message;
    input.title = PUSHSAFER_TITLE;
    input.sound = "8";
    input.vibration = "3";
    input.icon = "2";
    input.iconcolor = "#FF0000";
    input.priority = priority;
    input.device = "a";
    input.url = "https://thingsboard.stuwilkins.org";
    input.urlTitle = "Open ThingsBoard";
    input.picture = "";
    input.picture2 = "";
    input.picture3 = "";
    input.time2live = "";
    input.retry = "";
    input.expire = "";
    input.answer = "";

    //pushsafer.sendEvent(input);
    syslog.logf(LOG_INFO, "Pushed message \"%s\"", message);

    return true;
}
