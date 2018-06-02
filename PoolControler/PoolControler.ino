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
#include <Adafruit_BLEGatt.h>
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
#define WATER_LEVEL_X     79.69
#define WATER_LEVEL_C     -36455

// Other digital pins
#define OUTPUT_LED_L      13
#define FLOW_SWITCH       0
#define PUMP_STOP_BIT     10
#define PUMP_BIT_0        9
#define PUMP_BIT_1        6
#define PUMP_BIT_2        5

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
//                                           daysOfTheWeek_1,
//                                           daysOfTheWeek_2,
//                                           daysOfTheWeek_3,
//                                           daysOfTheWeek_4,
//                                           daysOfTheWeek_5,
//                                           daysOfTheWeek_6};


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

// Error information
int8_t error_count = 0;

// Create the bluefruit object
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);
Adafruit_BLEGatt  gatt = Adafruit_BLEGatt(ble);

// Enable for debug
#define DEBUG

// Pump Speeds
uint16_t pumpSpeeds[] = {0, 600, 1075, 1550, 2025, 2500, 2975, 3450};

// Error handler which pauses and flashes the L led. 
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while(1)
  {
    digitalWrite(OUTPUT_LED_L, !digitalRead(OUTPUT_LED_L));
    delay(100);
  }
}

void setup() {
  // Set digital pins 
  pinMode(OUTPUT_LED_L, OUTPUT);   
  pinMode(PUMP_STOP_BIT, OUTPUT);  
  pinMode(PUMP_BIT_0, OUTPUT);  
  pinMode(PUMP_BIT_1, OUTPUT);
  pinMode(PUMP_BIT_2, OUTPUT);
  pinMode(FLOW_SWITCH, INPUT_PULLUP);
  boolean success;

  Serial.begin(115200);

  int i = 0;
  for(i=0;i<100;i++)
  {
    digitalWrite(OUTPUT_LED_L, !digitalRead(OUTPUT_LED_L));
	delay(100);
  }

  // Set digital pins 
  pinMode(OUTPUT_LED_L, OUTPUT);   
  pinMode(PUMP_STOP_BIT, OUTPUT);  
  pinMode(PUMP_BIT_0, OUTPUT);  
  pinMode(PUMP_BIT_1, OUTPUT);
  pinMode(PUMP_BIT_2, OUTPUT);
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
  delay(5000);
}

void readSensors(void)
{
  float waterTemp = 0;
  float airTemp = 0;
  float humidity = 0;
  float waterLevel = 0;
  int32_t uvIndex = 0, vis = 0, ir = 0;
  int32_t flowSwitch = 0;
  int32_t pumpSpeed = 0;
  int32_t _now = 0;
  bool error = false;

  digitalWrite(OUTPUT_LED_L, !digitalRead(OUTPUT_LED_L));
  waterTemp = getOneWireTemp(oneWireSensors, waterThermometer);
  airTemp = htu.readTemperature();
  humidity = htu.readHumidity();
  uvIndex = uv.readUV();
  vis = uv.readVisible();
  ir = uv.readIR();
  waterLevel = getWaterSensorLevel();
  _now = rtc.now().unixtime();
  flowSwitch = !digitalRead(FLOW_SWITCH);
  pumpSpeed = getPumpSpeed();

  if((uvIndex == 0) &&
     (vis == 0) && (ir == 0))
  {
	error_count++;
    uv.begin();
  }

  setChar(waterTempId, _now, (int32_t)(waterTemp * 1000));  
  setChar(airTempId, _now, (int32_t)(airTemp * 1000));
  setChar(humidityId, _now, (int32_t)(humidity * 1000));
  setChar(waterLevelId, _now, (int32_t)(waterLevel * 1000));
  setChar(flowSwitchId, _now, (int32_t)(flowSwitch));
  setChar(uvIndexId, _now, (int32_t)(uvIndex));
  setChar(visId, _now, (int32_t)(vis));
  setChar(irId, _now, (int32_t)(ir));

  if((pumpSpeed > 1) && (flowSwitch == 1))
  {
    setChar(pumpSpeedId, _now, (int32_t)pumpSpeeds[pumpSpeed]); 
  } else {
    setChar(pumpSpeedId, _now, 0);
    error = true;
  }
  
  setChar(errorId, _now, (int32_t)error_count);

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
  if(flowSwitch)
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

  waterTempId = gatt.addCharacteristic(waterTempCharUUID, GATT_CHARS_PROPERTIES_NOTIFY, 8, 8, BLE_DATATYPE_BYTEARRAY);
  if (waterTempId == 0) {
    error(F("Could not add characteristic"));
  }

  airTempId = gatt.addCharacteristic(airTempCharUUID, GATT_CHARS_PROPERTIES_NOTIFY, 8, 8, BLE_DATATYPE_BYTEARRAY);
  if(!airTempId)
  {
    error(F("Could not add characteristic"));
  }
  
  humidityId = gatt.addCharacteristic(humidityCharUUID, GATT_CHARS_PROPERTIES_NOTIFY, 8, 8, BLE_DATATYPE_BYTEARRAY);
  if (humidityId == 0) {
    error(F("Could not add characteristic"));
  }

  waterLevelId = gatt.addCharacteristic(waterLevelCharUUID, GATT_CHARS_PROPERTIES_NOTIFY, 8, 8, BLE_DATATYPE_BYTEARRAY);
  if (waterLevelId == 0) {
    error(F("Could not add characteristic"));
  }

  flowSwitchId = gatt.addCharacteristic(flowSwitchCharUUID, GATT_CHARS_PROPERTIES_NOTIFY, 8, 8, BLE_DATATYPE_BYTEARRAY);
  if (flowSwitchId == 0) {
    error(F("Could not add characteristic"));
  }

  uvIndexId = gatt.addCharacteristic(uvIndexCharUUID, GATT_CHARS_PROPERTIES_NOTIFY, 8, 8, BLE_DATATYPE_BYTEARRAY);
  if (uvIndexId == 0) {
    error(F("Could not add characteristic"));
  }

  visId = gatt.addCharacteristic(visCharUUID, GATT_CHARS_PROPERTIES_NOTIFY, 8, 8, BLE_DATATYPE_BYTEARRAY);
  if (visId == 0) {
    error(F("Could not add characteristic"));
  }

  irId = gatt.addCharacteristic(irCharUUID, GATT_CHARS_PROPERTIES_NOTIFY, 8, 8, BLE_DATATYPE_BYTEARRAY);
  if (irId == 0) {
    error(F("Could not add characteristic"));
  }

  timeId = gatt.addCharacteristic(timeCharUUID, GATT_CHARS_PROPERTIES_NOTIFY, 8, 8, BLE_DATATYPE_BYTEARRAY);
  if (timeId == 0) {
    error(F("Could not add characteristic"));
  }
        
  pumpSpeedId = gatt.addCharacteristic(pumpSpeedUUID, GATT_CHARS_PROPERTIES_NOTIFY, 8, 8, BLE_DATATYPE_BYTEARRAY);
  if (pumpSpeedId == 0) {
    error(F("Could not add characteristic"));
  }

  pumpSpeedCmdId = gatt.addCharacteristic(pumpSpeedCmdUUID, GATT_CHARS_PROPERTIES_WRITE, 1, 1, BLE_DATATYPE_BYTEARRAY);
  if (pumpSpeedCmdId == 0) {
    error(F("Could not add characteristic"));
  }

  errorId = gatt.addCharacteristic(errorUUID, GATT_CHARS_PROPERTIES_NOTIFY, 8, 8, BLE_DATATYPE_BYTEARRAY);
  if (errorId == 0) {
    error(F("Could not add characteristic"));
  }

  ble.reset();
}

void setupBluetoothCallbacks(void)
{
  /* Set callbacks */
  ble.setBleUartRxCallback(BleUartRX);
  ble.setBleGattRxCallback(pumpSpeedCmdId, BleGattRX);
}

void BleUartRX(char data[], uint16_t len)
{
  Serial.print( F("[BLE UART RX]" ) );
  Serial.write(data, len);
  Serial.println();
}

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

void setChar(int32_t gattID, uint32_t timestamp, int32_t val)
{
  uint8_t _val[8];
  int i;

  _val[4] = (val >> 24) & 0xFF;
  _val[5] = (val >> 16) & 0xFF;
  _val[6] = (val >> 8) & 0xFF;
  _val[7] = val & 0xFF;

  _val[0] = (timestamp >> 24) & 0xFF;
  _val[1] = (timestamp >> 16) & 0xFF;
  _val[2] = (timestamp >> 8) & 0xFF;
  _val[3] = timestamp & 0xFF;

  gatt.setChar(gattID, _val, 8);
}

int32_t getWaterSensorLevel(void)
{
  uint64_t sum = 0;
  int32_t val;
  int i;
  
  for(i=0;i<100;i++)
  {
    sum += analogRead(WATER_LEVEL_PIN);
    delay(1);
  }

  val = (int32_t)(sum / 100);

  // Now do conversion
#ifdef DEBUG
    Serial.print(F("ADC Value = "));
    Serial.println(val);
#endif

  val = (val * WATER_LEVEL_X) + WATER_LEVEL_C;
  
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
  }
}

void setPumpSpeed(int speed)
{
    // First stop the pump
    digitalWrite(PUMP_STOP_BIT, 1);
    delay(1000);

    digitalWrite(PUMP_BIT_0, speed & 0x1);
    digitalWrite(PUMP_BIT_1, speed & 0x2);
    digitalWrite(PUMP_BIT_2, speed & 0x4);
    delay(1000);

    // Restart the pump
    digitalWrite(PUMP_STOP_BIT, 0);
}

int getPumpSpeed(void)
{
    int bits = 0;
    bits |= digitalRead(PUMP_BIT_0);
    bits |= (digitalRead(PUMP_BIT_1) << 1);
    bits |= (digitalRead(PUMP_BIT_2) << 2);
    return bits;
}
