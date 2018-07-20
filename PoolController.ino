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
#include <OneWire.h>
#include <DallasTemperature.h>
#include <avr/pgmspace.h>
#include <WiFi101.h>
#include <WiFi101OTA.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#include <Adafruit_SleepyDog.h>
#include <Adafruit_HTU21DF.h>
#include <Adafruit_SI1145.h>
#include <RTClib.h>
#include "eeprom_i2c.h"

#include "auth.h"
#include "PoolController.h"
#include "timer.h"

// One Wire Bus Definition
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature oneWireSensors(&oneWire);
DeviceAddress waterThermometer = {0x28, 0x8F, 0x3B, 0xE1, 0x08, 0x00, 0x00, 0xC7};
 
// Setup other sensors
Adafruit_HTU21DF htu = Adafruit_HTU21DF();
Adafruit_SI1145 uv = Adafruit_SI1145();

// Realtime Clock (RTClib)
RTC_DS3231 rtc;

// EEPROM
EEProm_I2C eeprom = EEProm_I2C(0x50);

// WIFI and MQTT setup
WiFiClient wifi_client;
Adafruit_MQTT_Client mqtt_client(&wifi_client, "192.168.1.2", 1883, "", "");

Adafruit_MQTT_Publish mqtt_water_temp = Adafruit_MQTT_Publish(&mqtt_client, "homeauto/pool/water_temp", MQTT_QOS_1);
Adafruit_MQTT_Publish mqtt_air_temp = Adafruit_MQTT_Publish(&mqtt_client, "homeauto/pool/air_temp", MQTT_QOS_1);
Adafruit_MQTT_Publish mqtt_air_humidity = Adafruit_MQTT_Publish(&mqtt_client, "homeauto/pool/air_humidity", MQTT_QOS_1);
Adafruit_MQTT_Publish mqtt_water_level = Adafruit_MQTT_Publish(&mqtt_client, "homeauto/pool/water_level", MQTT_QOS_1);
Adafruit_MQTT_Publish mqtt_flow_switch = Adafruit_MQTT_Publish(&mqtt_client, "homeauto/pool/flow_switch", MQTT_QOS_1);
Adafruit_MQTT_Publish mqtt_uv_index = Adafruit_MQTT_Publish(&mqtt_client, "homeauto/pool/uv_index", MQTT_QOS_1);
Adafruit_MQTT_Publish mqtt_vis = Adafruit_MQTT_Publish(&mqtt_client, "homeauto/pool/vis_light", MQTT_QOS_1);
Adafruit_MQTT_Publish mqtt_ir = Adafruit_MQTT_Publish(&mqtt_client, "homeauto/pool/ir_light", MQTT_QOS_1);
Adafruit_MQTT_Publish mqtt_pump_speed = Adafruit_MQTT_Publish(&mqtt_client, "homeauto/pool/pump_speed", MQTT_QOS_1);
Adafruit_MQTT_Publish mqtt_error = Adafruit_MQTT_Publish(&mqtt_client, "homeauto/pool/error", MQTT_QOS_1);
Adafruit_MQTT_Publish mqtt_pump_pressure = Adafruit_MQTT_Publish(&mqtt_client, "homeauto/pool/pump_pressure", MQTT_QOS_1);
Adafruit_MQTT_Publish mqtt_pump_flow = Adafruit_MQTT_Publish(&mqtt_client, "homeauto/pool/pump_flow", MQTT_QOS_1);
Adafruit_MQTT_Publish mqtt_current_program = Adafruit_MQTT_Publish(&mqtt_client, "homeauto/pool/current_program", MQTT_QOS_1);

Adafruit_MQTT_Subscribe mqtt_pump_speed_cmd = Adafruit_MQTT_Subscribe(&mqtt_client, "homeauto/pool/pump_speed_cmd");
Adafruit_MQTT_Subscribe mqtt_set_time_cmd = Adafruit_MQTT_Subscribe(&mqtt_client, "homeauto/pool/set_time_cmd");
Adafruit_MQTT_Subscribe mqtt_drain_cmd = Adafruit_MQTT_Subscribe(&mqtt_client, "homeauto/pool/set_drain_cmd");
Adafruit_MQTT_Subscribe mqtt_fill_cmd = Adafruit_MQTT_Subscribe(&mqtt_client, "homeauto/pool/set_fill_cmd");

// Data Readings
DataReadings sensor_readings;
DateTime last;

// Running program
ProgramData program_data;

// ISR Data
volatile PumpFlowRate pump_flow;

// Error handler which pauses and flashes the L led. 
void error(const __FlashStringHelper*err) {
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
	//if (millis() - pump_flow.last > PUMP_FLOW_BOUNCE)
	//{
	pump_flow.last = millis();
	pump_flow.clicks++;
	//}
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

	Serial.begin(115200);

	int i = 0;
	for(i=0;i<100;i++)
	{
		digitalWrite(OUTPUT_LED_L, !digitalRead(OUTPUT_LED_L));
		delay(100);
	}

	// Set the L led hight to show we are configuring
	digitalWrite(OUTPUT_LED_L, HIGH);

	int countdownMS = Watchdog.enable(WATCHDOG_TIME);
	Serial.print("Enabled the watchdog with max countdown of ");
	Serial.print(countdownMS, DEC);
	Serial.println(" milliseconds!");
	Serial.println();

	// Setup Sensors
	Serial.println(F("Setup Sensors ...."));
	setupSensors();
	Watchdog.reset();  // Pet the dog!

	// Setup RTC
	Serial.println(F("Setup RTC ...."));
	setup_clock();
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
	mqtt_client.subscribe(&mqtt_pump_speed_cmd);
	mqtt_pump_speed_cmd.setCallback(pump_callback);
	mqtt_connect();
	Watchdog.reset();  // Pet the dog!

	// Now restore from memory
	//eeprom_restore();

	// Set defaults
	sensor_readings.error_count = 0;
	pump_flow.last = millis();
	pump_flow.clicks = 0;
	program_data.current = PROGRAM_HALT;

	// Setup ISR
	attachInterrupt(digitalPinToInterrupt(PUMP_FLOW_PIN), 
					pump_flow_ISR, RISING);	

	// Set L low to show we are initialized.
	digitalWrite(OUTPUT_LED_L, LOW);
}

void eeprom_restore(void)
{
	uint8_t pump_speed;
	eeprom.read(EEPROM_PUMP_SPEED, &pump_speed, 1);
	Serial.print(F("EEPROM Pump speed = "));
	Serial.println(pump_speed);
	set_pump_speed(pump_speed);
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
	//mqtt_poll(1000);
	mqtt_client.processPackets(100);

	// Read the current time
	now = rtc.now();

	// Every second call poll
	if((now.second() != last.second()) && !(now.second() % 10))
	{
		// Now check for running programs

		if(program_data.current == PROGRAM_TIMER)
		{
			// Running TIMER Progra//m
		} else if(program_data.current == PROGRAM_DRAIN) {
			// Running DRAIN Program
		} else if(program_data.current == PROGRAM_FILL) {
			// Running FILL Program
		} else if(program_data.current == PROGRAM_HALT) {
			// Program Halted
		} else {
			Serial.println(F("*** ERROR Unknown Program ***"));
		}

		// Run EPS routines

		process_eps_pump();
		process_telemetry(now.unixtime());
		
	}

	if(now.hour() != last.hour())
	{
		// We have a new hour
		if((now.hour() % 6) == 0)
		{
			// We should now stop and start the pump to maintain prime
			set_pump_stop(1);

			// Delay 10 seconds to be nice on the pump
			// We just need to keep petting the dog during this process!
			for(int i=0;i<10;i++)
			{
				Watchdog.reset();
				delay(1000);
			}

			set_pump_stop(0);
		}
	}
	// Store last time
	//Serial.print(F("Loop time = "));
	//Serial.println(millis() - start_millis);
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
	mqtt_publish_data(&mqtt_current_program, now, 
			(int32_t)program_data.current);
}

void read_sensors(DataReadings *readings)
{
	bool error = false;

	readings->water_temp = get_one_wire_temp(oneWireSensors, waterThermometer);

	//readings->air_temp = htu.readTemperature();
	//readings->air_humidity = htu.readHumidity();

	readings->uv_index = uv.readUV();
	readings->vis = uv.readVisible();
	readings->ir = uv.readIR();
	
	readings->water_level = get_water_sensor_level();
	readings->pump_pressure = get_pump_pressure();
	readings->flow_switch = get_flow_switch();
	readings->pump_speed = get_pump_speed();
	readings->pump_flow = get_pump_flow();

	if((readings->uv_index == 0) &&
			(readings->vis == 0) && 
			(readings->ir == 0))
	{
		readings->error_count++;
		Serial.println(F("**** Resetting UV sensor"));
		uv.begin();
	}
}

bool get_flow_switch(void)
{
	return !digitalRead(FLOW_SWITCH);
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

	mqtt_publish_data(&mqtt_water_temp, now, (int32_t)(readings->water_temp * 1000));  
	mqtt_publish_data(&mqtt_air_temp, now, (int32_t)(readings->air_temp * 1000));
	mqtt_publish_data(&mqtt_air_humidity, now, (int32_t)(readings->air_humidity * 1000));
	mqtt_publish_data(&mqtt_water_level, now, (int32_t)(readings->water_level * 1000));
	mqtt_publish_data(&mqtt_flow_switch, now, (int32_t)(readings->flow_switch));
	mqtt_publish_data(&mqtt_uv_index, now, (int32_t)(readings->uv_index));
	mqtt_publish_data(&mqtt_vis, now, (int32_t)(readings->vis));
	mqtt_publish_data(&mqtt_ir, now, (int32_t)(readings->ir));
	mqtt_publish_data(&mqtt_pump_pressure, now, (int32_t)(readings->pump_pressure));
	mqtt_publish_data(&mqtt_pump_flow, now, (int32_t)(readings->pump_flow));

	if((readings->pump_speed > 1) && (readings->flow_switch == 1))
	{
		mqtt_publish_data(&mqtt_pump_speed, now, (int32_t)pumpSpeeds[readings->pump_speed]); 
	} else {
		mqtt_publish_data(&mqtt_pump_speed, now, 0);
	}

	mqtt_publish_data(&mqtt_error, now, (int32_t)readings->error_count);
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

void setupSensors(void)
{
	// Setup HTU21D-F

	//if (!htu.begin()) {
	//	error(F("Could not find HTU21D-F"));
	//}

	// Setup SI1145
	if(!uv.begin()) {
		error(F("Could not find SI1145"));
	}

	// Setup 1-Wire sensors

	oneWireSensors.begin();

	Serial.print(F("Found "));
	Serial.print(oneWireSensors.getDeviceCount(), DEC);
	Serial.println(F(" device\(s\)."));

	Serial.print("OneWire Parasitic power is: "); 
	if (oneWireSensors.isParasitePowerMode())
	{
		Serial.println(F("ON"));
	} else {
		Serial.println(F("OFF"));
	}

	int c = 0;
	if(!oneWire.search(waterThermometer))
	{
		error(F("Unable to find address for waterThermometer"));
	}

	oneWireSensors.setResolution(waterThermometer, 12);
}

float get_one_wire_temp(DallasTemperature sensor, DeviceAddress address)
{
	oneWireSensors.requestTemperatures();
	float tempC = sensor.getTempC(address);
	return tempC;
}

int32_t get_water_sensor_level(void)
{
	uint64_t sum = 0;
	int32_t val;
	int i;

	for(i=0;i<5;i++)
	{
		sum += analogRead(WATER_LEVEL_PIN);
		delay(1);
	}

	val = (int32_t)(sum / 5);

	val = (val * WATER_LEVEL_X) + WATER_LEVEL_C;

	return val;
}

int32_t get_pump_pressure(void)
{
	uint64_t sum = 0;
	int32_t val;
	int i;

	for(i=0;i<5;i++)
	{
		sum += analogRead(PUMP_PRESSURE_PIN);
		delay(1);
	}

	val = (int32_t)(sum / 5);

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

	if (rtc.lostPower()) {
		Serial.println(F("RTC lost power, setting time from compilation time of sketch."));
		// following line sets the RTC to the date & time this sketch was compiled
		DateTime compile_time = DateTime(F(__DATE__), F(__TIME__));
		rtc.adjust(compile_time + TimeSpan(0, UTC_TIME_OFFSET, 0, 0));
	}
}

void set_pump_speed(uint8_t speed)
{
	// Set the pump bits
	digitalWrite(PUMP_BIT_0, speed & 0x1);
	digitalWrite(PUMP_BIT_1, speed & 0x2);
	digitalWrite(PUMP_BIT_2, speed & 0x4);

	// If the pump is stopped, start it.
	if(get_pump_stop)
	{
		set_pump_stop(0);
	}

	// Now store the pump speed in EEPROM
	eeprom.write(EEPROM_PUMP_SPEED, (uint8_t *)(&speed), 1); // Store only 1 byte
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
	eeprom.write(EEPROM_PUMP_STOP, (uint8_t *)(&stop), 1); // Store only 1 byte
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
	
	Serial.println(F("Waiting 10s for WIFI to connect"));
	delay(10000);

	// start the WiFi OTA library with SD based storage
	Serial.println(F("Setup OTA Programming ....."));
	WiFiOTA.begin(OTA_CONNECTION_NAME, ota_password, InternalStorage);

	// Re-enable the watchdog
	Watchdog.enable(WATCHDOG_TIME);
}

void mqtt_connect() {

	if(mqtt_client.connected())
	{
		return;
	}

	Serial.println(F("Connecting to MQTT Server ....."));

	Watchdog.disable();
	int8_t ret;
	int tries = 50;
	while((ret = mqtt_client.connect()) != 0) { // connect will return 0 for connected
		Serial.println(mqtt_client.connectErrorString(ret));
		Serial.print("Retrying MQTT connection ... tries = ");
		Serial.println(tries);
		mqtt_client.disconnect();

		// Check if WiFi Is connected.
		if(WiFi.status() != WL_CONNECTED)
		{
			// We need to start WiFi
			wifi_connect();
			Watchdog.disable();
		}
		if(--tries == 0)
		{
			Serial.println(F("Enabling watchdog"));
			Watchdog.enable(WATCHDOG_TIME);
		}

		delay(10000);
	}

	Serial.println("MQTT Connected!");
	// Re-enable the watchdog
	Watchdog.enable(WATCHDOG_TIME);
}

void mqtt_publish_data(Adafruit_MQTT_Publish *pub, uint32_t timestamp, int32_t val)
{
    uint8_t _val[8];

    _val[4] = (val >> 24) & 0xFF;
    _val[5] = (val >> 16) & 0xFF;
    _val[6] = (val >> 8) & 0xFF;
    _val[7] = val & 0xFF;

    _val[0] = (timestamp >> 24) & 0xFF;
    _val[1] = (timestamp >> 16) & 0xFF;
    _val[2] = (timestamp >> 8) & 0xFF;
    _val[3] = timestamp & 0xFF;

    pub->publish(_val, 8);
}

void wifi_list_networks() {
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

void pump_callback(char *data, uint16_t len) 
{
	if(len != 1)
	{
		return;
	}
	uint8_t _val = data[0];
	Serial.print(F("**** Setting pump speed to "));
	Serial.println(_val);
	set_pump_speed(_val);
}
