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
#include <Adafruit_HTU21DF.h>
#include <Adafruit_SI1145.h>
#include <Adafruit_ATParser.h>
#include <Adafruit_BluefruitLE_SPI.h>
#include <Adafruit_BLEMIDI.h>
#include <Adafruit_BLEBattery.h>
#include <Adafruit_BLEGatt.h>
#include <Adafruit_BLEEddystone.h>
#include <Adafruit_BLE.h>
#include <Adafruit_BluefruitLE_UART.h>
#include <RTClib.h>

#include "BluefruitConfig.h"
#include "BluetoothConfig.h"
#define MINIMUM_FIRMWARE_VERSION   "0.7.0"

// One Wire Bus Definition
#define ONE_WIRE_BUS 1
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature oneWireSensors(&oneWire);
DeviceAddress waterThermometer = {0x28, 0x8F, 0x3B, 0xE1, 0x08, 0x00, 0x00, 0xC7};
 
// Analog input for level meter
#define WATER_LEVEL_PIN   A0
#define WATER_LEVEL_X     0.7969
#define WATER_LEVEL_C     -364.55

// Other digital pins
#define OUTPUT_LED_L      13
#define FLOW_SWITCH       0
#define PUMP_BIT_0        10
#define PUMP_BIT_1        9
#define PUMP_BIT_2        6
#define PUMP_BIT_3        5

// Setup other sensors
Adafruit_HTU21DF htu = Adafruit_HTU21DF();
Adafruit_SI1145 uv = Adafruit_SI1145();

// Realtime Clock (RTClib)
RTC_DS3231 rtc;
//const char daysOfTheWeek_0[12] PROGMEM = "Sunday";
//const char daysOfTheWeek_1[12] PROGMEM = "Monday";
//const char daysOfTheWeek_2[12] PROGMEM = "Tuesday";
//const char daysOfTheWeek_3[12] PROGMEM = "Wednesday";
//const char daysOfTheWeek_4[12] PROGMEM = "Thursday";
//const char daysOfTheWeek_5[12] PROGMEM = "Friday";
//const char daysOfTheWeek_6[12] PROGMEM = "Saturday";
//const char* const daysOfTheWeek[] PROGMEM = {daysOfTheWeek_0,
//											 daysOfTheWeek_1,
//											 daysOfTheWeek_2,
//											 daysOfTheWeek_3,
//											 daysOfTheWeek_4,
//											 daysOfTheWeek_5,
//											 daysOfTheWeek_6};


/* The service information */

int32_t environmentServiceId;
int32_t outputServiceId;
int32_t waterTempId;
int32_t airTempId;
int32_t humidityId;
int32_t waterLevelId;
int32_t flowSwitchId;
int32_t uvIndexId;
int32_t visId;
int32_t irId;
int32_t timeId;
int32_t pumpSpeedId;
int32_t pumpSpeedCmdId;
int32_t errorId;

// Create the bluefruit object
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);
Adafruit_BLEGatt  gatt = Adafruit_BLEGatt(ble);

// Enable for debug
#define DEBUG

// Pump Speeds
uint16_t pumpSpeeds[] = {0, 0, 600, 1075, 1550, 2025, 2500, 2975, 3450};
uint16_t pumpSpeedBits[] = {0x1, 0x0, 0x2, 0x4, 0x6, 0x8, 0xA, 0xC, 0xE}; 
uint16_t pumpSpeedVals[] = {1, 0, 2, 3, 3, 5, 4, 7, 5, -1, 6, -1, 7, -1, 8};

// Error handler which pauses and flashes the L led. 
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while(1)
  {
    digitalWrite(OUTPUT_LED_L, HIGH);
    delay(200);
    digitalWrite(OUTPUT_LED_L, LOW);
    delay(200);
  }
}

void setup() {
  boolean success;

  Serial.begin(115200);

  // Set digital pins 
  pinMode(OUTPUT_LED_L, OUTPUT);   
  pinMode(PUMP_BIT_0, OUTPUT);  
  pinMode(PUMP_BIT_1, OUTPUT);
  pinMode(PUMP_BIT_2, OUTPUT);
  pinMode(PUMP_BIT_3, OUTPUT);
  pinMode(FLOW_SWITCH, INPUT_PULLUP);
 
  // Set the L led hight to show we are configuring
  digitalWrite(OUTPUT_LED_L, HIGH);
	// Stop the pump
	setPumpSpeed(0);
  
  delay(5000);

  setupBluetooth();
  setupBluetoothLE();
  setupBluetoothCallbacks();

  // Setup Sensors
  setupSensors();

  // Setup RTC
  setupClock();
  
  // Set L low to show we are initialized.
  digitalWrite(OUTPUT_LED_L, LOW);
}

void loop() {
  readSensors();
  ble.update(1000);
  //delay(1000);
}

void readSensors(void)
{
  float waterTemp = 0;
  float airTemp = 0;
  float humidity = 0;
  float waterLevel = 0;
  int16_t uvIndex = 0, vis = 0, ir = 0;
  uint8_t flowSwitch[1] = { 0 };
	uint8_t pumpSpeed = 0;
  uint32_t _now = 0;
	bool error = false;

  waterTemp = getOneWireTemp(oneWireSensors, waterThermometer);
  airTemp = htu.readTemperature();
  humidity = htu.readHumidity();
  uvIndex = uv.readUV();
  vis = uv.readVisible();
  ir = uv.readIR();
  waterLevel = getWaterSensorLevel();
  _now = rtc.now().unixtime();
  flowSwitch[0] = !digitalRead(FLOW_SWITCH);
	pumpSpeed = getPumpSpeed();

  setFloatChar(waterTempId, waterTemp * 1000);  
  setFloatChar(airTempId, airTemp * 1000);
  setFloatChar(humidityId, humidity * 1000);
  setFloatChar(waterLevelId, waterLevel * 1000);
  gatt.setChar(flowSwitchId, flowSwitch, 1);
  setInt16Char(uvIndexId, uvIndex);
  setInt16Char(visId, vis);
  setInt16Char(irId, ir);
  setInt32Char(timeId, _now);
	if((pumpSpeed > 1) && (flowSwitch[0] == 1))
	{
		setInt16Char(pumpSpeedId, pumpSpeeds[pumpSpeed]); 
	} else {
		int16_t _zero = 0;
		setInt16Char(pumpSpeedId, _zero);
		error = true;
	}

	if(error)
	{
		uint8_t _zero[1] = { 0 };
		gatt.setChar(errorId, _zero, 1);
	} else {
		uint8_t _zero[1] = { 1 };
		gatt.setChar(errorId, _zero, 1);
	}

#ifdef DEBUG
  Serial.print(F("Water Temp : "));
  Serial.print(waterTemp);
  Serial.print(F(" degC\t\t"));
  Serial.print(F("Air Temperature "));
  Serial.print(airTemp);
  Serial.print(F(" degC\t\t"));
  Serial.print(F("Humidity "));
  Serial.print(humidity);
  Serial.print(F("%\t\t"));
  Serial.print(F("Water Level "));
  Serial.print(waterLevel);
  Serial.print(F("%\t\t"));
  Serial.print(F("Flow "));
  if(flowSwitch[0])
  {
    Serial.print(F("ON "));
  } else {
    Serial.print(F("OFF"));
  }

  Serial.print(F("\t\t"));
  Serial.print(F(" UV "));
  Serial.print(((float)uvIndex) / 100);
  Serial.print(F("\t\t"));
  Serial.print(F(" VIS "));
  Serial.print(vis);
  Serial.print(F("\t\t"));
  Serial.print(F(" IR "));
  Serial.print(ir);
  Serial.print(F("\t\t"));
	Serial.print(F(" Speed "));
	Serial.print(pumpSpeed);
	Serial.print(F(" "));
	Serial.print(pumpSpeeds[pumpSpeed]);

  Serial.println("");

  Serial.print(F("Time = 0x"));
  Serial.print(_now, HEX);
  Serial.println("");

#endif
}

void setupBluetooth(void)
{
  if (!ble.begin(VERBOSE_MODE))
  {
    error(F("Couldn't find Bluefruit\n"));
  }

  if (! ble.factoryReset() ){
    error(F("Couldn't factory reset\n"));
  }

  // Disable command echo from Bluefruit
  ble.echo(false);
  ble.verbose(false);

  // Set name of device
  if (!ble.sendCommandCheckOK(F("AT+GAPDEVNAME=Pool Controller")))
	{
    error(F("Could not set device name."));
  }

  if (!ble.sendCommandCheckOK(F("AT+BLEPOWERLEVEL=4")))
	{
    error(F("Could not set output power"));
  }

  if (!ble.sendCommandCheckOK(F("AT+HWMODELED=SPI")) ){
    error(F("Could not set led output"));
  }
}

void setupBluetoothLE(void)
{
                               
  environmentServiceId = gatt.addService(environmentServiceUUID);
  if(!environmentServiceId)
  {
    error(F("Could not add pool service"));
  }

  waterTempId = gatt.addCharacteristic(waterTempCharUUID, GATT_CHARS_PROPERTIES_READ, 4, 4, BLE_DATATYPE_BYTEARRAY);
  if (waterTempId == 0) {
    error(F("Could not add characteristic"));
  }

  airTempId = gatt.addCharacteristic(airTempCharUUID, GATT_CHARS_PROPERTIES_READ, 4, 4, BLE_DATATYPE_BYTEARRAY);
  if(!airTempId)
  {
    error(F("Could not add characteristic"));
  }
  
  humidityId = gatt.addCharacteristic(humidityCharUUID, GATT_CHARS_PROPERTIES_READ, 4, 4, BLE_DATATYPE_BYTEARRAY);
  if (humidityId == 0) {
    error(F("Could not add characteristic"));
  }

  waterLevelId = gatt.addCharacteristic(waterLevelCharUUID, GATT_CHARS_PROPERTIES_READ, 4, 4, BLE_DATATYPE_BYTEARRAY);
  if (waterLevelId == 0) {
    error(F("Could not add characteristic"));
  }

  flowSwitchId = gatt.addCharacteristic(flowSwitchCharUUID, GATT_CHARS_PROPERTIES_READ, 1, 1, BLE_DATATYPE_BYTEARRAY);
  if (flowSwitchId == 0) {
    error(F("Could not add characteristic"));
  }

  uvIndexId = gatt.addCharacteristic(uvIndexCharUUID, GATT_CHARS_PROPERTIES_READ, 2, 2, BLE_DATATYPE_BYTEARRAY);
  if (uvIndexId == 0) {
    error(F("Could not add characteristic"));
  }

  visId = gatt.addCharacteristic(visCharUUID, GATT_CHARS_PROPERTIES_READ, 2, 2, BLE_DATATYPE_BYTEARRAY);
  if (visId == 0) {
    error(F("Could not add characteristic"));
  }

  irId = gatt.addCharacteristic(irCharUUID, GATT_CHARS_PROPERTIES_READ, 2, 2, BLE_DATATYPE_BYTEARRAY);
  if (irId == 0) {
    error(F("Could not add characteristic"));
  }

  timeId = gatt.addCharacteristic(timeCharUUID, GATT_CHARS_PROPERTIES_READ, 4, 4, BLE_DATATYPE_BYTEARRAY);
  if (timeId == 0) {
    error(F("Could not add characteristic"));
  }
		
  pumpSpeedId = gatt.addCharacteristic(pumpSpeedUUID, GATT_CHARS_PROPERTIES_READ, 2, 2, BLE_DATATYPE_BYTEARRAY);
  if (pumpSpeedId == 0) {
    error(F("Could not add characteristic"));
  }

  pumpSpeedCmdId = gatt.addCharacteristic(pumpSpeedCmdUUID, GATT_CHARS_PROPERTIES_WRITE, 1, 1, BLE_DATATYPE_BYTEARRAY);
  if (pumpSpeedCmdId == 0) {
    error(F("Could not add characteristic"));
  }

  errorId = gatt.addCharacteristic(errorUUID, GATT_CHARS_PROPERTIES_READ, 1, 1, BLE_DATATYPE_BYTEARRAY);
  if (errorId == 0) {
    error(F("Could not add characteristic"));
  }
  ble.reset();
}

void setupBluetoothCallbacks(void)
{
  /* Set callbacks */
  //ble.setBleUartRxCallback(BleUartRX);
	ble.setBleGattRxCallback(pumpSpeedCmdId, BleGattRX);
}

//void BleUartRX(char data[], uint16_t len)
//{
//  Serial.print( F("[BLE UART RX]" ) );
//  Serial.write(data, len);
//  Serial.println();
//}

void BleGattRX(int32_t chars_id, uint8_t data[], uint16_t len)
{
  if(chars_id == pumpSpeedCmdId)
  {  
		if(len == 1){
#ifdef DEBUG
			Serial.print("Setting pump speed to ");
			Serial.println(data[0]);
#endif
			setPumpSpeed(data[0]);
		} else {
			Serial.println(F("Error setting pump speed"));
		}
  }
}
void setupSensors(void)
{
  // Setup HTU21D-F
  
  if (!htu.begin()) {
    error(F("Could not find HTU21D-F"));
  }

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

float getOneWireTemp(DallasTemperature sensor, DeviceAddress address)
{
  oneWireSensors.requestTemperatures();
  float tempC = sensor.getTempC(address);
  return tempC;
}

void setFloatChar(int32_t gattID, float val)
{
  uint8_t _val[4];
  int32_t _intVal;

  _intVal = (int32_t)val;
  
  _val[0] = (_intVal >> 24) & 0xFF;
  _val[1] = (_intVal >> 16) & 0xFF;
  _val[2] = (_intVal >> 8) & 0xFF;
  _val[3] = _intVal & 0xFF;
  
  gatt.setChar(gattID, _val, 4);
}

void setInt16Char(int32_t gattID, int16_t val)
{
  uint8_t _val[2];
  _val[0] = (val >> 8) & 0xFF;
  _val[1] = val & 0xFF;

  gatt.setChar(gattID, _val, 2);
}

void setInt32Char(int32_t gattID, int32_t val)
{
  uint8_t _val[4];
  _val[0] = (val >> 24) & 0xFF;
  _val[1] = (val >> 16) & 0xFF;
  _val[2] = (val >> 8) & 0xFF;
  _val[3] = val & 0xFF;

  gatt.setChar(gattID, _val, 4);
}

float getWaterSensorLevel(void)
{
  uint64_t sum = 0;
  float val;
  int i;
  
  for(i=0;i<500;i++)
  {
    sum += analogRead(WATER_LEVEL_PIN);
    delay(2); // Wait 2 ms betwen each read
  }

  val = (float)sum / 500;

  // Now do conversion
#ifdef DEBUG
	Serial.print(F("ADC Value = "));
	Serial.println(val);
#endif
  val = (val * WATER_LEVEL_X) + WATER_LEVEL_C;
  
#ifdef DEBUG
	Serial.print(F("Calibrated Value = "));
	Serial.println(val);
#endif

  return val;
}

void setupClock(void)
{ 
  if(!rtc.begin()) {
    error(F("Couldn't find RTC"));
  }

  if (rtc.lostPower()) {
    Serial.println(F("RTC lost power, setting time from compilation time of sketch."));
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }
}

void setPumpSpeed(int speed)
{
	// First check if we want to stop
	if(!speed)
	{
		digitalWrite(PUMP_BIT_0, 1);
		digitalWrite(PUMP_BIT_1, 0);
		digitalWrite(PUMP_BIT_2, 0);
		digitalWrite(PUMP_BIT_3, 0);
		return;
	}

	// LSB is the stop bit and is inverted
	digitalWrite(PUMP_BIT_0, 0);
	digitalWrite(PUMP_BIT_1, (pumpSpeedBits[speed] & 0x2) == 0x2);
	digitalWrite(PUMP_BIT_2, (pumpSpeedBits[speed] & 0x4) == 0x4);
	digitalWrite(PUMP_BIT_3, (pumpSpeedBits[speed] & 0x8) == 0x8);

}

int getPumpSpeed(void)
{
	if(digitalRead(PUMP_BIT_0))
	{
		return 0;
	}
	int bits = 0;
	
	bits |= digitalRead(PUMP_BIT_0);
	bits |= (digitalRead(PUMP_BIT_1) << 1);
	bits |= (digitalRead(PUMP_BIT_2) << 2);
	bits |= (digitalRead(PUMP_BIT_3) << 3);
	return pumpSpeedVals[bits];
}
