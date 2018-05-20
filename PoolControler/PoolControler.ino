#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
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

#include "BluefruitConfig.h"

#include "BluetoothConfig.h"

#define MINIMUM_FIRMWARE_VERSION   "0.7.0"
#define ONE_WIRE_BUS 2
#define WATER_SERIES_RESISTOR 560    
 
// What pin to connect the sensor to
#define WATER_SENSOR_PIN A0
#define WATER_REFERENCE_PIN A1

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature oneWireSensors(&oneWire);
//DeviceAddress waterThermometer = {0x28, 0x1D, 0x2C, 0xE1, 0x08, 0x00, 0x00, 0x35};
DeviceAddress waterThermometer = {0x28, 0x8F, 0x3B, 0xE1, 0x08, 0x00, 0x00, 0xC7};

// Setup other sensors
Adafruit_HTU21DF htu = Adafruit_HTU21DF();
Adafruit_SI1145 uv = Adafruit_SI1145();

/* The service information */

int32_t environmentServiceId;
int32_t waterTempId;
int32_t airTempId;
int32_t humidityId;
int32_t waterLevelId;
int32_t flowSwitchId;
int32_t outputServiceId;
int32_t outputId;

/* Output pins */
int outputLEDL = 13;
int flowSwitchInput = 3;

// Create the bluefruit object
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);
Adafruit_BLEGatt  gatt = Adafruit_BLEGatt(ble);

// Enable for debug
//#define DEBUG

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while(1)
  {
    digitalWrite(outputLEDL, HIGH);
    delay(250);
    digitalWrite(outputLEDL, LOW);
    delay(250);
  }
}

void setup() {
  boolean success;

  Serial.begin(115200);

  // Input output pin setup. 
  pinMode(outputLEDL, OUTPUT);   

  // Set pump (flow) switch
  pinMode(flowSwitchInput, INPUT_PULLUP);

  // Set L to high to show we are in setup stage
  digitalWrite(outputLEDL, HIGH);
  delay(5000);

  setupBluetooth();
  setupBluetoothLE();
  setupBluetoothCallbacks();

  //Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  //ble.info();

  // Setup Sensors
  setupSensors();

  // Set L low to show we are initialized. 
  digitalWrite(outputLEDL, LOW);
}

void loop() {
  digitalWrite(outputLEDL, HIGH);

  readSensors();

  /* Delay before next measurement update */            
  digitalWrite(outputLEDL, LOW);
  ble.update(1000);
  delay(1000);
}

void readSensors(void)
{
  float waterTemp, airTemp, humidity, waterLevel;
  uint8_t flowSwitch[1];
  
  waterTemp = getOneWireTemp(oneWireSensors, waterThermometer);
  //airTemp = getAirTemp();
  //humidity = getHumidity();
  //waterLevel = getWaterSensorLevel();
  flowSwitch[0] = !digitalRead(flowSwitchInput);

  airTemp = 0.0;
  humidity = 0.0;
  waterLevel = 0.0;

  setFloatChar(waterTempId, waterTemp * 1000);  
  setFloatChar(airTempId, airTemp * 1000);
  setFloatChar(humidityId, humidity * 1000);
  setFloatChar(waterLevelId, waterLevel * 1000);
  gatt.setChar(flowSwitchId, flowSwitch, 1);

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
    Serial.print(F("ON"));
  } else {
    Serial.print(F("OFF"));
  }

  Serial.println(F(""));
  
}

void setupBluetooth(void)
{
  Serial.print(F("Initialising the Bluefruit LE module..."));
  if (!ble.begin(VERBOSE_MODE))
  {
    error(F("Couldn't find Bluefruit\n"));
  }
  Serial.println( F("OK!") );

  Serial.println(F("Performing a factory reset..."));
  if (! ble.factoryReset() ){
    error(F("Couldn't factory reset\n"));
  }

  if ( !ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    error( F("Callbacks requires at least 0.7.0") );
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);
  ble.verbose(false);

  /* Set name of device */
  if (! ble.sendCommandCheckOK(F("AT+GAPDEVNAME=Pool Controller")) ) {
    error(F("Could not set device name?"));
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

  outputServiceId = gatt.addService(outputServiceUUID);
  if(!outputServiceId)
  {
    error(F("Could not add Service"));
  }

  outputId = gatt.addCharacteristic(0x2345, GATT_CHARS_PROPERTIES_WRITE, 1, 4, BLE_DATATYPE_INTEGER);
  if(!outputId)
  {
    error(F("Could not create characteristic"));
  }

  Serial.println("Performing a SW reset");
  ble.reset();
}

void setupBluetoothCallbacks(void)
{
  /* Set callbacks */
  ble.setConnectCallback(connected);
  ble.setDisconnectCallback(disconnected);
  ble.setBleUartRxCallback(BleUartRX);
  ble.setBleGattRxCallback(outputId, BleGattRX);
}

void BleGattRX(int32_t chars_id, uint8_t data[], uint16_t len)
{
  Serial.print( F("[BLE GATT RX] (" ) );
  Serial.print(chars_id);
  Serial.print(") len = ");
  Serial.print(len);
  
  if (chars_id == outputId)
  {  
    Serial.write(data, len);
    Serial.println();
  } 
}

void BleUartRX(char data[], uint16_t len)
{
  Serial.print( F("[BLE UART RX]" ) );
  Serial.write(data, len);
  Serial.println();
}

void connected(void)
{
  Serial.println( F("Connected") );
}

void disconnected(void)
{
  Serial.println( F("Disconnected") );
}

void setupSensors(void)
{
  // Setup HTU21D-F
  
  //if (!htu.begin()) {
  //  error(F("Could not find HTU21D-F"));
  //}

  // Setup 1-Wire sensors

  oneWireSensors.begin();
  
  Serial.print(F("Found "));
  Serial.print(oneWireSensors.getDeviceCount(), DEC);
  Serial.println(F(" device\(s\)."));
  
  Serial.print("OneWire Parasitic power is: "); 
  if (oneWireSensors.isParasitePowerMode())
  {
    Serial.println("ON");
  } else {
    Serial.println("OFF");
  }

  if (!oneWire.search(waterThermometer))
  {
    error(F("Unable to find address for waterThermometer"));
  }

  oneWireSensors.setResolution(waterThermometer, 12);
  
}

// function to print a device address
void printOneWireAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

float getOneWireTemp(DallasTemperature sensor, DeviceAddress address)
{
  oneWireSensors.requestTemperatures();
  float tempC = sensor.getTempC(address);
  return tempC;
}

float getAirTemp(void)
{
  float temp;
  temp = htu.readTemperature();
  return temp;
}

float getHumidity(void)
{
  float hu;
  hu = htu.readHumidity();
  return hu;
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
  
#ifdef DEBUG
  Serial.print(F("Setting GattID "));
  Serial.print(gattID);
  Serial.print(F(" to val "));
  Serial.print(_intVal);
  Serial.print(F(" 0x"));
  int i;
  for(i=0;i<4;i++)
  {
    crPrintHEX(_val[i], 2);
  }
  Serial.println("");
#endif
}

//---------------------------------------------------------------------------------
// crPrintHEX
//---------------------------------------------------------------------------------

void crPrintHEX(unsigned long DATA, unsigned char numChars) {
  unsigned long mask  = 0x0000000F;
  mask = mask << 4*(numChars-1);
    
  for (unsigned int i=numChars; i>0; --i) 
  {
    Serial.print(((DATA & mask) >> (i-1)*4),HEX);
    mask = mask >> 4;
  }
}

float getWaterSensorLevel(void)
{
  float reading, reference;
  reading = analogRead(WATER_SENSOR_PIN);
  reference = analogRead(WATER_REFERENCE_PIN);
  
#ifdef DEBUG  
  Serial.print(F("Water analog reading ")); 
  Serial.print(reading);
  Serial.print(F(" reference "));
  Serial.print(reference);
#endif

  // convert the value to resistance
  reading = (1023 / reading)  - 1;
  reading = WATER_SERIES_RESISTOR / reading;
  reference = (1023 / reference)  - 1;
  reference = WATER_SERIES_RESISTOR / reference;
  
#ifdef DEBUG 
  Serial.print(" sensor res. "); 
  Serial.print(reading);
  Serial.print(F(" reference "));
  Serial.println(reference);
#endif

  return 0.0;
}

