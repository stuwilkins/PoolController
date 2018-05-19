#include <Wire.h>
#include <OneWire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
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

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

/* The service information */

int32_t waterTemperatureServiceId;
int32_t waterTemperatureTempId;
int32_t airTemperatureServiceId;
int32_t airTemperatureTempId;
int32_t outputServiceId;
int32_t outputId;

/* Output pins */
int outputLEDL = 13;

/* One-Wire setup */
OneWire ds(10);  // on pin 10

// Create the bluefruit object
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);
Adafruit_BLEGatt  gatt = Adafruit_BLEGatt(ble);

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while(1)
  {
    digitalWrite(outputLEDL, HIGH);
    delay(2000);
    digitalWrite(outputLEDL, LOW);
    delay(2000);
  }
}

void setup() {
  boolean success;

  Serial.begin(115200);

  // Input output pin setup. 
  pinMode(outputLEDL, OUTPUT);   
  sensors.begin();

  // Set L to high to show we are in setup stage
  digitalWrite(outputLEDL, HIGH);
  delay(10000);

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
    error( F("Callback requires at least 0.7.0") );
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);
  ble.verbose(false);

  /* Set name of device */
  if (! ble.sendCommandCheckOK(F("AT+GAPDEVNAME=Pool Controller")) ) {
    error(F("Could not set device name?"));
  }

  setupBluetoothLE();

  Serial.println("Performing a SW reset");
  ble.reset();

    /* Set callbacks */
  ble.setConnectCallback(connected);
  ble.setDisconnectCallback(disconnected);
  ble.setBleUartRxCallback(BleUartRX);
  ble.setBleGattRxCallback(outputId, BleGattRX);


  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  // Set L low to show we are initialized. 
  digitalWrite(outputLEDL, LOW);
}

void loop() {
  digitalWrite(outputLEDL, HIGH);
  
  double temp = random(0, 100) / 10.0;

  Serial.print(F("Updating Temperature value to "));
  Serial.print(temp);
  Serial.print(F(" Celcuis "));

  // https://developer.bluetooth.org/gatt/characteristics/Pages/CharacteristicViewer.aspx?u=org.bluetooth.characteristic.temperature_measurement.xml
  // Chars value is 1 flag + 4 float value. Tempearature is in Fahrenheit unit
  
  uint8_t temp_measurement[2];
  int16_t _temp;
  _temp = (temp * 100);
  Serial.print(_temp, HEX);
  Serial.print(F(" "));
  
  temp_measurement[0] = _temp >> 8;
  temp_measurement[1] = _temp & 0xFF;

  Serial.print(temp_measurement[1], HEX);
  Serial.print(F(" "));
  Serial.print(temp_measurement[0], HEX);
  Serial.print(F(" "));

  Serial.println("");
  
  gatt.setChar(waterTemperatureTempId, temp_measurement, 2);
  gatt.setChar(airTemperatureTempId, temp_measurement, 2);

  Serial.print("Requesting temperatures...");
  sensors.requestTemperatures(); // Send the command to get temperatures
  Serial.println("DONE");
  // After we got the temperatures, we can print them here.
  // We use the function ByIndex, and as an example get the temperature from the first sensor only.
  Serial.print("Temperature for the device 1 (index 0) is: ");
  Serial.println(sensors.getTempCByIndex(0));  

  /* Delay before next measurement update */            
  digitalWrite(outputLEDL, LOW);
  ble.update(1000);
  delay(1000);
}

void setupBluetoothLE(void)
{
                               
  waterTemperatureServiceId = gatt.addService(waterTemperatureServiceUUID);
  if(!waterTemperatureServiceId)
  {
    error(F("Could not add pool service"));
  }

  waterTemperatureTempId = gatt.addCharacteristic(0x2A6E, GATT_CHARS_PROPERTIES_INDICATE, 2, 2, BLE_DATATYPE_BYTEARRAY);
  if (waterTemperatureTempId == 0) {
    error(F("Could not add Temperature characteristic"));
  }

  airTemperatureServiceId = gatt.addService(airTemperatureServiceUUID);
  if(!airTemperatureServiceId)
  {
    error(F("Could not add service"));
  }
  
  airTemperatureTempId = gatt.addCharacteristic(0x2A6E, GATT_CHARS_PROPERTIES_READ, 2, 2, BLE_DATATYPE_BYTEARRAY);
  if (airTemperatureTempId == 0) {
    error(F("Could not add Temperature characteristic"));
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

