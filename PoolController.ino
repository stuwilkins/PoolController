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
#include <avr/pgmspace.h>
#include <WiFi101.h>
#include <WiFi101OTA.h>
#include <Adafruit_SleepyDog.h>
#include <Adafruit_HTU21DF.h>
#include <Adafruit_SI1145.h>
#include <Adafruit_MAX31865.h>
#include <pt100rtd.h>
#include <RTClib.h>
#include <PubSubClient.h>
#include <NTPClient.h>
#include <eeprom_i2c.h>

#include "auth.h"
#include "PoolController.h"
#include "timer.h"

// Setup other sensors
Adafruit_HTU21DF htu = Adafruit_HTU21DF();
Adafruit_SI1145 uv = Adafruit_SI1145();
Adafruit_MAX31865 max_ts = Adafruit_MAX31865(MAX31865_CS);
pt100rtd PT100 = pt100rtd();

// Realtime Clock (RTClib)
RTC_DS3231 rtc;

// EEPROM
EEPROM_I2C eeprom = EEPROM_I2C(0x50);

// WIFI and MQTT setup
WiFiClient wifi_client;
PubSubClient mqtt_client(wifi_client);

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
static const char* mqtt_loop_time      = "homeauto/pool/loop_time";
static const char* mqtt_uptime         = "homeauto/pool/uptime";
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

static const char* ota_name            = "pool_controller_ota";
static const char* ota_password        = "bHmkHvDM3Ka*^nQz";

WiFiUDP ntp_udp;
NTPClient ntp_client(ntp_udp, NTP_ADDRESS, NTP_OFFSET, NTP_INTERVAL);

// Data Readings
DataReadings sensor_readings;
DateTime last;
DateTime restart_pump;
int prime_stop = 0;
uint32_t uptime;

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

	Serial.begin(115200);

	// Set the L led hight to show we are configuring
	digitalWrite(OUTPUT_LED_L, HIGH);

	int countdownMS = Watchdog.enable(WATCHDOG_TIME);
	Serial.print("Enabled the watchdog with max countdown of ");
	Serial.print(countdownMS, DEC);
	Serial.println(" milliseconds!");
	Serial.println();

	// Setup Sensors
	Serial.println(F("Setup Sensors ...."));
	setup_sensors();
	Watchdog.reset();  // Pet the dog!

	// Setup eeprom
	Serial.println(F("Setup eeprom ...."));
	eeprom.begin();

	// Setup WIFI

	Serial.println(F("Setup WIFI ...."));
	setup_wifi();
	wifi_connect();
	Watchdog.reset();  // Pet the dog!

	Serial.println(F("Setup MQTT ...."));
	mqtt_client.setServer(MQTT_SERVER, MQTT_PORT);
	mqtt_client.setCallback(mqtt_callback);
	mqtt_connect();
	Watchdog.reset();  // Pet the dog!

	Serial.println(F("Setup NTP for time ...."));
	ntp_client.begin();
	Watchdog.reset();

	// Setup RTC
	Serial.print(F("Setup RTC ...."));
	setup_clock();
	Serial.println(F(" DONE"));
	Watchdog.reset();  // Pet the dog!
	uptime = rtc.now().unixtime() - NTP_OFFSET;
	mqtt_client.publish(mqtt_uptime, (uint8_t*)(&uptime), sizeof(uptime), 1); 

	// Set defaults
	last = rtc.now();
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

	// Now restore from memory
	eeprom_read();

	// Setup ISR
	attachInterrupt(digitalPinToInterrupt(PUMP_FLOW_PIN), 
					pump_flow_ISR, FALLING);	
	attachInterrupt(digitalPinToInterrupt(FILL_FLOW_PIN), 
					fill_flow_ISR, FALLING);	

	// Set L low to show we are initialized.
	digitalWrite(OUTPUT_LED_L, LOW);
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
	if(!eeprom.read(EEPROM_MAGIC_DATA, (uint8_t*)(&magic), sizeof(magic)))
	{
		return false;
	}

	if(magic == EEPROM_MAGIC)
	{
		return false;
	}

	return true;
}

void loop() {
	DateTime now;
	unsigned long start_millis = millis();

	// Pet the dog!
	Watchdog.reset();

	// Do WIFI and MQTT Connection
	wifi_connect();
	mqtt_connect();

	// Poll for over the air updates
	WiFiOTA.poll();

	// Poll for MQTT updates
	mqtt_client.loop();

	// Read the current time
	now = rtc.now();

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
	if(now.second() != last.second())
	{
		// Now check for running programs

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
			if(delta < 0)
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
				eeprom_write();
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
			eeprom_write();
		}

		if(program_data.current == PROGRAM_RUN) {
			// Running RUN program
			process_eps_pump();
			if(get_pump_speed() != program_data.run_pump_speed)
			{
				set_pump_speed(program_data.run_pump_speed);
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

		process_telemetry(now.unixtime() - NTP_OFFSET);

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

		mqtt_publish_data(mqtt_loop_time, now.unixtime() - NTP_OFFSET, 
				(int32_t)(millis() - start_millis), 0);

	}

	// Store last time
	last = now;
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
		Serial.println(F("*** EPS : Flow error detected, timer started"));
	}

	if(flow_error)
	{
		if((millis() - flow_time) > EPS_PUMP_FLOW_TIMEOUT)
		{
			if(error)
			{
				// We are in an error state
				Serial.println(F("*** EPS : Flow error")); 
				set_pump_stop(1);
				set_pump_speed(0);
				program_data.current = PROGRAM_HALT;
			} else {
				// the error cleared. 
				Serial.println(F("*** EPS : Resetting flow error"));
				flow_error = false;
			}
		} else {
			Serial.print(F("** EPS : Countdown = "));
			Serial.println(EPS_PUMP_FLOW_TIMEOUT - (millis() - flow_time));
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

	//print_readings(&sensor_readings);

	// Publish with MQTT
	publish_readings(&sensor_readings, now);

	// Publish current program
	mqtt_publish_data(mqtt_program, now, 
			(int32_t)program_data.current, 0);
}

void read_sensors(DataReadings *readings)
{
	//readings->air_temp = htu.readTemperature();
	//readings->air_humidity = htu.readHumidity();

	uint8_t fault = max_ts.readFault();
	if(fault)
	{
		Serial.print("Fault 0x"); Serial.println(fault, HEX);
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
		Serial.println(F("**** Resetting UV sensor"));
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

void print_readings(DataReadings *readings)
{

    Serial.print(F("Water Temp : "));
    Serial.print(readings->water_temp);
    Serial.println(F(" degC\t\t"));
    Serial.print(F("Air Temperature "));
    Serial.print(readings->air_temp);
    Serial.println(F(" degC\t\t"));
    Serial.print(F("Air Humidity "));
    Serial.print(readings->air_humidity);
    Serial.println(F("%\t\t"));
    Serial.print(F("Water Level "));
    Serial.print(readings->water_level);
    Serial.println(F("%\t\t"));
    Serial.print(F("Flow "));

    if(readings->flow_switch)
    {
        Serial.print(F("ON "));
    } else {
        Serial.print(F("OFF"));
    }

    Serial.println("");
    Serial.print(F("UV "));
    Serial.print(((float)readings->uv_index) / 100);
    Serial.println("");
    Serial.print(F("VIS "));
    Serial.print(readings->vis);
    Serial.println("");
    Serial.print(F("IR "));
    Serial.print(readings->ir);
    Serial.println("");
    Serial.print(F("Speed "));
    Serial.print(readings->pump_speed);
    Serial.print(F(" "));
    Serial.println(pumpSpeeds[readings->pump_speed]);
	Serial.print(F("Pump Flow Rate "));
	Serial.print(readings->pump_flow);
	Serial.println(F(" *1000 GPM"));

}

void setup_sensors(void)
{
	// Setup HTU21D-F

	//if (!htu.begin()) {
	//	error(F("Could not find HTU21D-F"));
	//}

	// Setup SI1145
	if(!uv.begin()) 
	{
		//error(F("Could not find SI1145"));
	}

	// Setup MAX31865	
	if(!max_ts.begin(MAX31865_3WIRE))
	{
		error(F("Unable to set MAX31865"));
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
	if(!rtc.begin()) {
		error(F("Couldn't find RTC"));
	}

	if(rtc.lostPower())
	{
		Serial.println(F("**** RTC Lost Power"));
		set_clock();
	}
}

void set_clock(void)
{
	ntp_client.update();
	DateTime now = rtc.now();
	unsigned long epoch_ntp = ntp_client.getEpochTime();
	unsigned long epoch_rtc = now.unixtime();

	long diff = epoch_ntp - epoch_rtc;

	Serial.print(F("**** RTC reports time as "));
	Serial.println(epoch_rtc, HEX);
	Serial.print(F("**** NTP reports time as "));
	Serial.println(epoch_ntp, HEX);
	Serial.print(F("**** RTC vs NTP diff is "));
	Serial.println(diff, HEX);

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
	Serial.println(F("Waiting 5s for descovery of networks"));
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
		Serial.print(F("Attempting to connect to WPA SSID: "));
		Serial.println(wifi_ssid);
		WiFi.begin(wifi_ssid, wifi_password);
		if(--tries == 0)
		{
			Serial.println(F("Enabling watchdog"));
			Watchdog.enable(WATCHDOG_TIME);
		}
		delay(5000);
	}

	// Re-enable the watchdog
	Watchdog.enable(WATCHDOG_TIME);
	
	// start the WiFi OTA library
	Serial.println(F("Setup OTA Programming ....."));
	WiFiOTA.begin(ota_name, ota_password, InternalStorage);
}

void mqtt_connect() {

	if(mqtt_client.connected())
	{
		return;
	}

	Watchdog.disable();

	int tries = 50;
	while(!mqtt_client.connected())
	{
		Serial.print("Attempting MQTT connection...");
		if(mqtt_client.connect(MQTT_CLIENT_NAME))
		{
			Serial.println(F("Connected"));
			int i = 0;
			while(mqtt_subscribe[i] != 0)
			{
				mqtt_client.subscribe(mqtt_subscribe[i], 1);
				i++;
			}
		} else {
			Serial.print(F("Connection failed, rc="));
			Serial.print(mqtt_client.state());
			Serial.println(F(" try again in 5 seconds"));
			// Wait 5 seconds before retrying
			delay(5000);
			if(--tries == 0)
			{
				Serial.println(F("Enabling watchdog"));
				Watchdog.enable(WATCHDOG_TIME);
			}
		}
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

    mqtt_client.publish(pub, _val, 8, persist);
}

void mqtt_callback(char* topic, byte* payload, unsigned int length)
{
	Serial.print(F("**** Message arrived ["));
	Serial.print(topic);
	Serial.print(F("] "));

	if(!strcmp(topic, mqtt_pump_speed_sp))
	{
		Serial.println(F("Setting pump speed"));
		if(length == 1)
		{
			program_data.run_pump_speed = payload[0];
			eeprom_write();
		}
	} else if (!strcmp(topic, mqtt_program_sp)) {
		Serial.println(F("Setting program"));
		if(length == 1)
		{
			program_data.current = (int)(payload[0]);
			eeprom_write();
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

