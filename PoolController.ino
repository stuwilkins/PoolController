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
#include <Adafruit_HTU21DF.h>
#include <Adafruit_SI1145.h>
#include <Adafruit_MAX31865.h>
#include <pt100rtd.h>
#include <RTClib.h>
#include <PubSubClient.h>
#include <NTPClient.h>
#include <eeprom_i2c.h>
#include <Syslog.h>
#include <ArduinoJson.h>

#include "auth.h"
#include "PoolController.h"
#include "timer.h"

#define USE_EEPROM

// Setup other sensors
Adafruit_HTU21DF htu = Adafruit_HTU21DF();
Adafruit_SI1145 uv = Adafruit_SI1145();
Adafruit_MAX31865 max_ts = Adafruit_MAX31865(MAX31865_CS);
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

WiFiClient mqtt_wifi_client;
PubSubClient mqtt_client(mqtt_wifi_client);
WiFiClient tb_wifi_client;
PubSubClient tb_mqtt_client(tb_wifi_client);

WiFiUDP syslog_udp;
Syslog syslog(syslog_udp, SYSLOG_SERVER, SYSLOG_PORT, DEVICE_HOSTNAME, APP_NAME, LOG_KERN);

static const char* mqtt_water_temp     = "homeauto/pool/water_temp";
static const char* mqtt_air_temp       = "homeauto/pool/air_temp";
static const char* mqtt_air_humidity   = "homeauto/pool/air_humidity";
static const char* mqtt_water_level    = "homeauto/pool/water_level";
static const char* mqtt_flow_switch    = "homeauto/pool/flow_switch";
static const char* mqtt_uv_index       = "homeauto/pool/uv_index";
static const char* mqtt_vis            = "homeauto/pool/vis_light";
static const char* mqtt_ir             = "homeauto/pool/ir_light";
static const char* mqtt_pump_speed     = "homeauto/pool/pump_speed";
static const char* mqtt_error          = "homeauto/pool/error";
static const char* mqtt_pump_pressure  = "homeauto/pool/pump_pressure";
static const char* mqtt_pump_flow      = "homeauto/pool/pump_flow";
static const char* mqtt_fill_flow      = "homeauto/pool/fill_flow";
static const char* mqtt_program        = "homeauto/pool/program";
static const char* mqtt_prime_flow     = "homeauto/pool/prime_flow";
static const char* mqtt_prime_pressure = "homeauto/pool/prime_pressure";

static const char* mqtt_pump_speed_sp  = "homeauto/pool/pump_speed_sp";
static const char* mqtt_program_sp     = "homeauto/pool/program_sp";
static const char* mqtt_drain_sp       = "homeauto/pool/set_drain_sp";
static const char* mqtt_fill_sp        = "homeauto/pool/set_fill_sp";
static const char* mqtt_reset          = "homeauto/pool/reset";
static const char* mqtt_prime_pump     = "homeauto/pool/prime_pump_sp";
static const char* mqtt_boost_sp       = "homeauto/pool/boost_sp";
static const char* mqtt_boost_time_sp  = "homeauto/pool/boost_time_sp";
static const char* mqtt_boost_speed_sp = "homeauto/pool/boost_speed_sp";

static const char* mqtt_subscribe[]    = {mqtt_pump_speed_sp, mqtt_program_sp, mqtt_drain_sp,
                                          mqtt_fill_sp, mqtt_reset, mqtt_prime_pump, 
										  mqtt_boost_sp, mqtt_boost_time_sp, 
										  mqtt_boost_speed_sp, 0};

static const char* tb_mqtt_subscribe[]  = {"v1/devices/me/rpc/request/+", 0};

// Data Readings
DataReadings sensor_readings;
DateTime last;
DateTime last_telemetry;
DateTime restart_pump;
int prime_stop = 0;

// Running program
ProgramData program_data;

// ISR Data
volatile PumpFlowRate pump_flow;
volatile PumpFlowRate fill_flow;

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

    set_pump_stop(1);

	Serial.begin(115200, 0);
    unsigned long t = millis();
    while(((millis() - t) < 10000) && !Serial){
        delay(1000);
    }
        
	// Set the L led hight to show we are configuring
	digitalWrite(OUTPUT_LED_L, HIGH);

	int countdownMS = Watchdog.enable(WATCHDOG_TIME);
	Serial.print("Enabled the watchdog with max countdown of ");
	Serial.print(countdownMS, DEC);
	Serial.println(" milliseconds!");
	Serial.println();

	// Setup WIFI

	Serial.println(F("Setup WIFI ...."));
	setup_wifi();
	wifi_connect();

    // Now WiFi is up, connect to the syslog console
	syslog_udp.begin(SYSLOG_PORT);
    syslog.serial(&Serial);
    //syslog.serialMask(0x7F);
    //syslog.logMask(0x7F);
    syslog.serialMask(0xFF);
    syslog.logMask(0xFF);
    syslog.log(LOG_CRIT, "Start Syslog");

    // Enter a 2 minute loop to wait for programming if fault
    syslog.log(LOG_CRIT, "Entering upload window");
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
	tb_mqtt_client.setServer(TB_MQTT_SERVER, TB_MQTT_PORT);
	mqtt_client.setCallback(mqtt_callback);
	tb_mqtt_client.setCallback(tb_mqtt_callback);
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
	pump_flow.last = millis();
	pump_flow.clicks = 0;
	fill_flow.last = millis();
	fill_flow.clicks = 0;
	program_data.current = PROGRAM_RUN;
	program_data.level_target = PROGRAM_LEVEL_TARGET;
	program_data.run_pump_speed = PROGRAM_PUMP_RUN_SPEED;
	program_data.drain_pump_speed = PROGRAM_PUMP_DRAIN_SPEED;
	program_data.prime = 0;
	program_data.boost_time = last;
	program_data.boost_pump_speed = 6;
	program_data.boost_duration = 60 * 30; // 30m
	program_data.boost_counter = 0;

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

	// Set L low to show we are initialized.
	digitalWrite(OUTPUT_LED_L, LOW);

    syslog.log(LOG_INFO, "End of setup()");
}

bool eeprom_write(void)
{
	uint32_t magic = EEPROM_MAGIC;

	eeprom.writeIfDiff(EEPROM_MAGIC_DATA, (uint8_t *)(&magic), sizeof(magic));
	eeprom.writeIfDiff(EEPROM_PROGRAM_DATA, (uint8_t *)(&program_data), sizeof(program_data), true, true);

	return true;
}

bool eeprom_read(void)
{
	uint32_t magic;
	if(eeprom.read(EEPROM_MAGIC_DATA, (uint8_t*)(&magic), sizeof(magic)))
	{
		syslog.log(LOG_ERR, "Unable to read MAGIC from EEPROM");
		return false;
	}

	if(magic != EEPROM_MAGIC)
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

	// Pet the dog!
	Watchdog.reset();

	// Do WIFI and MQTT Connection
    syslog.log(LOG_DEBUG, "wifi_connect()");
	wifi_connect();
	Watchdog.reset(); // Pet the dog!
    syslog.log(LOG_DEBUG, "mqtt_connect()");
	mqtt_connect(&mqtt_client, MQTT_NAME, NULL, NULL, mqtt_subscribe);
	Watchdog.reset(); // Pet the dog!
    syslog.log(LOG_DEBUG, "mqtt_connect() (TB)");
	mqtt_connect(&tb_mqtt_client, TB_MQTT_NAME, TB_MQTT_USERNAME, "", tb_mqtt_subscribe);
	Watchdog.reset(); // Pet the dog!

	// Poll for over the air updates
    syslog.log(LOG_DEBUG, "WiFiOTA.poll() (TB)");
	WiFiOTA.poll();
	Watchdog.reset(); // Pet the dog!

	// Poll for MQTT updates
    syslog.log(LOG_DEBUG, "mqtt_client.loop()");
	mqtt_client.loop();
	Watchdog.reset(); // Pet the dog!
    syslog.log(LOG_DEBUG, "mqtt_client.loop() (TB)");
	tb_mqtt_client.loop();
	Watchdog.reset(); // Pet the dog!
    
    // Sync NTP client
    syslog.log(LOG_DEBUG, "ntp_client.update()");
    if(!ntp_client.update())
    {
        syslog.log(LOG_CRIT, "Unable to update from NTP server");
    }
	Watchdog.reset(); // Pet the dog!

	// Read the current time
    syslog.log(LOG_DEBUG, "Read RTC");
	now = rtc.now();
	Watchdog.reset(); // Pet the dog!

	if(now.hour() != last.hour())
	{
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
		set_pump_stop(1);
		restart_pump = now;
		prime_stop = 1;
		program_data.prime = 0;
	}

	// Every second call poll
	if((now - last).totalseconds() >= 1) 
	{
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
            syslog.logf("BOOST Delta = %ld", delta);
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
			int32_t _wl = get_water_sensor_level();
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
				set_pump_stop(1);
			}
		}

		if((prime_stop == 1) && ((now - restart_pump).totalseconds() > 30))
		{
			prime_stop = 2;
			set_pump_stop(0);
		}

		if((prime_stop == 2) && ((now - restart_pump).totalseconds() > 270))
		{
			prime_stop = 0;
			// store flow and pressure during prime. 
			mqtt_publish_data(mqtt_prime_pressure, now.unixtime() - NTP_OFFSET,
				(int32_t)(sensor_readings.pump_pressure), 1);
			mqtt_publish_data(mqtt_prime_flow, now.unixtime() - NTP_OFFSET,
				(int32_t)(sensor_readings.pump_flow), 1);
		}

        Watchdog.reset(); // Pet the dog!

#ifdef USE_EEPROM
        syslog.log(LOG_DEBUG, "eeprom_write()");
		eeprom_write();
        Watchdog.reset(); // Pet the dog!
#endif
        last = now;
    }

    if((now - last_telemetry).totalseconds() >= 10)
    {
        syslog.log(LOG_DEBUG, "process_telemetry()");
		process_telemetry(now.unixtime() - NTP_OFFSET);
        Watchdog.reset(); // Pet the dog!

        //print_readings(&sensor_readings);
        //Watchdog.reset(); // Pet the dog!
        last_telemetry = now;
	}
}

void process_eps_pump()
{
	static uint32_t flow_time;
	static bool flow_error = false;
	bool error;

	// Now check for fault conditions (pump running, no flow)
	bool _flsw = get_flow_switch();
	int _pump = get_pump_speed();
	int _pump_stop = get_pump_stop();

	error = ((_flsw == 0) && (_pump != 0) && (_pump_stop == 0));

	if(error && !flow_error)
	{
		// First notice, set timeout
		flow_time = millis();
		flow_error = true;
		syslog.log(LOG_ERR, "EPS : Flow error detected, timer started");
	}

	if(flow_error)
	{
		if((millis() - flow_time) > EPS_PUMP_FLOW_TIMEOUT)
		{
			if(error)
			{
				// We are in an error state
				syslog.log(LOG_ERR, "EPS : Flow error"); 
				set_pump_stop(1);
				set_pump_speed(0);
				program_data.current = PROGRAM_HALT;
			} else {
				// the error cleared. 
				syslog.log(LOG_ERR, "EPS : Resetting flow error");
				flow_error = false;
			}
		} else {
            syslog.logf(LOG_DEBUG, "EPS : Countdown = %ld", 
                     (EPS_PUMP_FLOW_TIMEOUT - (millis() - flow_time)));
		}
	}
}

void process_telemetry(uint32_t now)
{

	// Flash the L led
	digitalWrite(OUTPUT_LED_L, !digitalRead(OUTPUT_LED_L));

	// We have a new second
	// Read pool sensors
	read_sensors(&sensor_readings);
    Watchdog.reset();  // Pet the dog!

	// Publish with MQTT
	publish_readings(&sensor_readings, now);
    Watchdog.reset();  // Pet the dog!
	tb_publish_readings(&tb_mqtt_client, &sensor_readings, now);
    Watchdog.reset();  // Pet the dog!

	// Publish current program
	mqtt_publish_data(mqtt_program, now, 
			(int32_t)program_data.current, 0);
    Watchdog.reset();  // Pet the dog!
}

void read_sensors(DataReadings *readings)
{
	uint8_t fault = max_ts.readFault();
	if(fault)
	{
        syslog.logf(LOG_CRIT, "RTD Fault = %d", fault);
		max_ts.clearFault();
		readings->water_temp = 0.0;
	} else {
		uint16_t rtd = max_ts.readRTD();
		uint32_t dummy;
		
		dummy = ((uint32_t)(rtd << 1)) * 100 * ((uint32_t) floor(MAX31865_RREF)) ;
		dummy >>= 16 ;

		uint16_t ohmsx100 = (uint16_t)(dummy & 0xFFFF);
		float water_temp = PT100.celsius(ohmsx100);
		readings->water_temp = water_temp;
	}

	readings->uv_index = uv.readUV();
	readings->vis = uv.readVisible();
	readings->ir = uv.readIR();
	
	readings->water_level = get_water_sensor_level();
	readings->pump_pressure = get_pump_pressure();
	readings->flow_switch = get_flow_switch();
	readings->pump_speed = get_pump_speed();
	readings->pump_flow = get_pump_flow();
	readings->fill_flow = get_fill_flow();

	if((readings->uv_index == 0) &&
			(readings->vis == 0) && 
			(readings->ir == 0))
	{
		readings->error_count++;
        syslog.log(LOG_ALERT, "Resetting UV sensor");
		uv.begin();
		delay(100);
		readings->uv_index = uv.readUV();
		readings->vis = uv.readVisible();
		readings->ir = uv.readIR();
	}
}

bool get_flow_switch(void)
{
	return !digitalRead(FLOW_SWITCH);
}

int32_t get_fill_flow(void)
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

	int32_t _int_flow = (int32_t)(_flow * 1000);
	return _int_flow;
}	

int32_t get_pump_flow(void)
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

	int32_t _int_flow = (int32_t)(_flow * 1000);
	return _int_flow;
}	

void publish_readings(DataReadings *readings, uint32_t now)
{

	mqtt_publish_data(mqtt_water_temp, now, (int32_t)(readings->water_temp * 1000), 0);  
	mqtt_publish_data(mqtt_air_temp, now, (int32_t)(readings->air_temp * 1000), 0);
	mqtt_publish_data(mqtt_air_humidity, now, (int32_t)(readings->air_humidity * 1000), 0);
	mqtt_publish_data(mqtt_water_level, now, (int32_t)(readings->water_level), 0);
	mqtt_publish_data(mqtt_flow_switch, now, (int32_t)(readings->flow_switch), 0);
	mqtt_publish_data(mqtt_uv_index, now, (int32_t)(readings->uv_index), 0);
	mqtt_publish_data(mqtt_vis, now, (int32_t)(readings->vis), 0);
	mqtt_publish_data(mqtt_ir, now, (int32_t)(readings->ir), 0);
	mqtt_publish_data(mqtt_pump_pressure, now, (int32_t)(readings->pump_pressure), 0);
	mqtt_publish_data(mqtt_pump_flow, now, (int32_t)(readings->pump_flow), 0);
	mqtt_publish_data(mqtt_fill_flow, now, (int32_t)(readings->fill_flow), 0);
	mqtt_publish_data(mqtt_pump_speed, now, (int32_t)pumpSpeeds[readings->pump_speed], 0); 
	mqtt_publish_data(mqtt_error, now, (int32_t)readings->error_count, 0);
}

void tb_publish_readings(PubSubClient *client, DataReadings *readings, uint32_t now)
{
    StaticJsonDocument<JSON_BUFFER_LEN> root;

    JsonObject values = root.createNestedObject("values");
    values["water_temp"] = readings->water_temp;
    values["flow_switch"] = readings->flow_switch;
    values["uv_index"] = (double)readings->uv_index / 100.0;
    values["ir_light"] = readings->ir;
    values["visible_light"] = readings->vis;
    values["water_level"] = (double)readings->water_level / 100.0;
    values["pump_pressure"] = (double)readings->pump_pressure / 100.0;
    values["pump_flow"] = (double)readings->pump_flow / 1000.0;
    values["pump_speed"] = pumpSpeeds[readings->pump_speed];

    root["ts"] = String(now) + "000"; // This gets round the lack of uint64_t conversion

    char buffer[JSON_BUFFER_LEN];
    serializeJson(root, buffer, JSON_BUFFER_LEN);

    if(client->connected())
    {
        client->publish("v1/devices/me/telemetry", buffer, false);
        //syslog.logf(LOG_INFO, "thingsboard upload = %s strlen = %d", buffer, strlen(buffer));
    } else {
        syslog.log(LOG_ERR, "Unable to publish to thingsboard - not connected");
    }
}

void print_readings(DataReadings *readings)
{

    syslog.logf(LOG_DEBUG, "Water Temp             : %f degC", readings->water_temp);
    syslog.logf(LOG_DEBUG, "Water Level            : %ld", readings->water_level);
    if(readings->flow_switch)
    {
        syslog.log(LOG_DEBUG, "Flow Switch            : ON");
    } else {
        syslog.log(LOG_DEBUG, "Flow Switch            : OFF");
    }

    syslog.logf(LOG_DEBUG, "UV Sensor              : %f %ld %ld", 
            (float)readings->uv_index / 100,
            readings->vis,
            readings->ir);
    syslog.logf(LOG_DEBUG, "Speed                  : %ld (%d)", readings->pump_speed,
            pumpSpeeds[readings->pump_speed]);
    syslog.logf(LOG_DEBUG, "Flow rate              : %f GPM", (float)readings->pump_flow / 1000);

}

void setup_sensors(void)
{
	// Setup SI1145
	if(!uv.begin()) 
	{
        syslog.log(LOG_CRIT, "Unable to initialize SI1145 sensor");
	}

	// Setup MAX31865	
	if(!max_ts.begin(MAX31865_3WIRE))
	{
        syslog.log(LOG_CRIT, "Unable to initialize MAX31865 sensor");
	}
}

int32_t get_water_sensor_level(void)
{
	int32_t val = analogRead(WATER_LEVEL_PIN);

	val = (val * WATER_LEVEL_X) + WATER_LEVEL_C;

	return val;
}

int32_t get_pump_pressure(void)
{
	int32_t val;

	val = analogRead(PUMP_PRESSURE_PIN);

	// Now do conversion
	float _val = 3300 * (float)val;
	_val = 2 * _val / 1023;

	_val = _val - 335; // Zero point offset
	_val = _val * 75;

	val = (int32_t)_val;

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

int get_pump_speed(void)
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

void wifi_connect() {

	if(WiFi.status() == WL_CONNECTED)
	{
		return;
	}

	Watchdog.disable();

	int tries = 50;
	while(WiFi.status() != WL_CONNECTED)
	{
		Serial.print("Attempting to connect to WPA SSID: ");
		Serial.println(WIFI_SSID);
		WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
		if(--tries == 0)
		{
			Serial.println("Enabling watchdog");
			Watchdog.enable(WATCHDOG_TIME);
		}
		delay(5000);
	}

	// Re-enable the watchdog
	Watchdog.enable(WATCHDOG_TIME);

    // Print info
    Serial.print("IP Address = ");
    Serial.println(WiFi.localIP());
	
	// start the WiFi OTA library
	Serial.println("Setup OTA Programming.");
	WiFiOTA.begin(OTA_NAME, OTA_PASSWORD, InternalStorage);
}

void mqtt_connect(PubSubClient *client, const char* name, const char* uname, 
        const char* pass, const char* subscribe[]) {

	if(client->connected())
	{
		return;
	}

	Watchdog.disable();

    syslog.log(LOG_INFO, "Attempting MQTT connection");
    bool c;
    if(uname != NULL)
    {
        c = client->connect(name, uname, pass);
    } else {
        c = client->connect(name);
    }
    if(c)
    {
        syslog.logf(LOG_INFO, "Connected to MQTT server name %s", name);
        int i = 0;
        while(subscribe[i] != 0)
        {
            client->subscribe(subscribe[i], 1);
            i++;
        }
    } else {
        syslog.logf(LOG_CRIT, "MQTT Connection failed to %s rc=%d",
                name, client->state());
    }

	// Re-enable the watchdog
	Watchdog.enable(WATCHDOG_TIME);
}

void mqtt_publish_data(const char *pub, uint32_t timestamp, int32_t val, int persist)
{
    byte _val[8];

    _val[4] = (val >> 24) & 0xFF;
    _val[5] = (val >> 16) & 0xFF;
    _val[6] = (val >> 8) & 0xFF;
    _val[7] = val & 0xFF;

    _val[0] = (timestamp >> 24) & 0xFF;
    _val[1] = (timestamp >> 16) & 0xFF;
    _val[2] = (timestamp >> 8) & 0xFF;
    _val[3] = timestamp & 0xFF;
    
    if(mqtt_client.connected())
    {
        mqtt_client.publish(pub, _val, 8, persist);
    }
}

void tb_mqtt_callback(char* topic, byte* payload, unsigned int length)
{

    if(!length)
    {
        syslog.log(LOG_ERR, "Invalid tb_mqtt callback");
    }

    char buffer[256];
    strncpy(buffer, (char*)payload, length);
    payload[length] = 0;
    syslog.logf(LOG_INFO, "TB MQTT Message arrived [%s] %s", topic, (char *)payload);

    StaticJsonDocument<JSON_BUFFER_LEN> json;
    DeserializationError error = deserializeJson(json, (char*)payload);

    if(error)
    {
        syslog.logf(LOG_ERR, "deserializeJson() failed: %s", error.c_str());
        return;
    } 

    const char* method = json["method"];

    if(!strcmp(method, "setPumpSpeed"))
    {
        int speed = json["params"];
        syslog.logf(LOG_INFO, "Setting pump speed to %d", speed);
        program_data.run_pump_speed = speed;
    } else if(!strcmp(method, "setRunProgramValue")) {
        int val = json["params"];
        if(val)
        {
            program_data.current = PROGRAM_RUN;
            syslog.logf(LOG_INFO, "Setting program to %d", program_data.current);
        }
    } else if(!strcmp(method, "setHaltProgramValue")) {
        int val = json["params"];
        if(val)
        {
            program_data.current = PROGRAM_HALT;
            syslog.logf(LOG_INFO, "Setting program to %d", program_data.current);
        }
    } else if(!strcmp(method, "setDrainProgramValue")) {
        int val = json["params"];
        if(val)
        {
            program_data.current = PROGRAM_DRAIN;
            syslog.logf(LOG_INFO, "Setting program to %d", program_data.current);
        }
    }

}

void mqtt_callback(char* topic, byte* payload, unsigned int length)
{
    syslog.logf(LOG_INFO, "MQTT Message arrived [%s]", topic);

	if(!strcmp(topic, mqtt_pump_speed_sp))
	{
		if(length == 1)
		{
			program_data.run_pump_speed = payload[0];
		}
	} else if (!strcmp(topic, mqtt_program_sp)) {
		if(length == 1)
		{
			program_data.current = (int)(payload[0]);
		}
	} else if(!strcmp(topic, mqtt_reset)) {
		if(length == 1)
		{
			if(payload[0])
			{
				error(F("RESET"));
			}
		}
	} else if(!strcmp(topic, mqtt_prime_pump)) {
		if(length == 1)
		{
			if(payload[0])
			{
				program_data.prime = 1;
			}
		}
	} else if(!strcmp(topic, mqtt_boost_sp)) {
		if(length == 1) 
		{
			if(payload[0])
			{
				program_data.boost_program = program_data.current;
				program_data.current = PROGRAM_BOOST;
				program_data.boost_time = rtc.now() + 
					TimeSpan(program_data.boost_duration);
			}
		}
	} else if(!strcmp(topic, mqtt_boost_time_sp)) {
		if(length == 4)
		{
			uint32_t _val = 0;
			_val |= (payload[0] >> 24);
			_val |= (payload[1] >> 16);
			_val |= (payload[2] >> 8);
			_val |= payload[3];
			program_data.boost_duration = _val;
		}
	} else if(!strcmp(topic, mqtt_boost_speed_sp)) {
		if(length == 1)
		{
			program_data.boost_pump_speed = payload[0];
		}
	}

}

void wifi_list_networks()
{
	// scan for nearby networks:
	Serial.println("** Scan Networks **");
	byte numSsid = WiFi.scanNetworks();

	// print the list of networks seen:
	Serial.print("number of available networks:");
	Serial.println(numSsid);

	// print the network number and name for each network found:
	for (int thisNet = 0; thisNet<numSsid; thisNet++) {
		Serial.print(thisNet);
		Serial.print(") ");
		Serial.print(WiFi.SSID(thisNet));
		Serial.print("\tSignal: ");
		Serial.print(WiFi.RSSI(thisNet));
		Serial.print(" dBm");
		Serial.print("\tEncryption: ");
		Serial.println(WiFi.encryptionType(thisNet));
	}
}

