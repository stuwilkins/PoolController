#include <SPI.h>
#include <Wire.h>
#include <CurieBLE.h>
#include <Adafruit_ILI9341.h>
#include <Adafruit_STMPE610.h>
#include <Adafruit_GFX.h>
#include <fonts/FreeSans12pt7b.h>

#include <Fonts/FreeMonoBold12pt7b.h>
#include "BluetoothConfig.h"

// This is calibration data for the raw touch data to the screen coordinates
#define TS_MINX 150
#define TS_MINY 130
#define TS_MAXX 3800
#define TS_MAXY 4000

#define STMPE_CS 8
Adafruit_STMPE610 ts = Adafruit_STMPE610(STMPE_CS);
#define TFT_CS 10
#define TFT_DC 9
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);

struct data {
  float water_temp;
  float air_temp;
  float humidity;
  float uv_index;
  float water_level;
	uint16_t pump_speed;
};

void setup() {
  // put your setup code here, t  o run once:
  Serial.begin(115200);

  delay(10000);
  Serial.println("Booting ....");
  
  tft.begin();
  if (!ts.begin()) { 
    Serial.println("Unable to start touchscreen.");
  } 
  else { 
    Serial.println("Touchscreen started."); 
  }

  tft.fillScreen(ILI9341_BLACK);
  tft.setRotation(1);

  BLE.begin();

  // start scanning for peripherals
  BLE.scanForName("Pool Controller");
}

void loop() {

  data reading;
  bool reading_valid = false;

  Serial.println("Starting loop ....");

  // check if a peripheral has been discovered
  BLEDevice peripheral = BLE.available();

  if (peripheral) {
    // discovered a peripheral, print out address, local name, and advertised service
    Serial.print("Found ");
    Serial.print(peripheral.address());
    Serial.print(" '");
    Serial.print(peripheral.localName());
    Serial.print("' ");
    Serial.print(peripheral.advertisedServiceUuid());
    Serial.println();

    // stop scanning
    BLE.stopScan();

    if(connectToController(&peripheral))
    {
      reading_valid = readController(&peripheral, &reading);
    }
    
    // peripheral disconnected, start scanning again
    peripheral.disconnect();
    Serial.println("Peripheral disconnected");
    BLE.scanForName("Pool Controller");
  }

  if(reading_valid)
  {
    tft.fillScreen(ILI9341_BLACK);
    tft.setFont(&FreeMonoBold12pt7b);
  
    tft.setTextColor(0xFFFF, 0x0000);
    tft.setTextSize(1);
    tft.setCursor(5,20);
    tft.println("Water Temp");
    tft.setCursor(5,45);
    tft.println("Air Temp");
    tft.setCursor(5,70);
    tft.println("Humidity");
    tft.setCursor(5,95);
    tft.println("UV Index");
    tft.setCursor(5,120);
    tft.println("Water Lvl");
    tft.setCursor(5,145);
    tft.println("Pump Spd");
  
    tft.setCursor(180,20);
    tft.println(reading.water_temp);
    tft.setCursor(180,45);
    tft.println(reading.air_temp);
    tft.setCursor(180,70);
    tft.println(reading.humidity);
    tft.setCursor(180,95);
    tft.println(reading.uv_index);
    tft.setCursor(180,120);
    tft.println((int)reading.water_level);
    tft.setCursor(180,145);
    tft.println((int)reading.pump_speed);
  
    tft.setCursor(260,20);
    tft.print("C");
    tft.setCursor(260,45);
    tft.print("C");
    tft.setCursor(260,70);
    tft.print("%");
    tft.setCursor(260,120);
    tft.print("mm");
    tft.setCursor(260,145);
    tft.print("rpm");

  }

  delay(60000);
}


bool connectToController(BLEDevice *peripheral) {

  // connect to the peripheral
  Serial.println("Connecting ...");

  if (peripheral->connect()) {
    Serial.println("Connected");
  } else {
    Serial.println("Failed to connect!");
    return false;
  }

  // discover peripheral attributes
  Serial.println("Discovering attributes ...");
  if (peripheral->discoverAttributes()) {
    Serial.println("Attributes discovered");
  } else {
    Serial.println("Attribute discovery failed!");
    return false;
  }
    
  return true;
}

bool readController(BLEDevice *peripheral, data *reading)
{
  int32_t _val;

  readCharacteristic(peripheral, "1001", &_val, 4);
  reading->water_temp = (float)_val / 1000;
  readCharacteristic(peripheral, "1002", &_val, 4);
  reading->air_temp = (float)_val / 1000;
  readCharacteristic(peripheral, "1003", &_val, 4);
  reading->humidity = (float)_val / 1000;
  readCharacteristic(peripheral, "1006", &_val, 2);
  reading->uv_index = (float)_val / 100;
  readCharacteristic(peripheral, "1005", &_val, 4);
  reading->water_level = (float)_val / 1000;
  readCharacteristic(peripheral, "1010", &_val, 2);
  reading->pump_speed = (uint16_t)_val;

  return true;
  
}

bool readCharacteristic(BLEDevice *peripheral, const char* characteristic, int32_t *val, int size)
{
  int32_t _val = 0;
  // retrieve the LED characteristic
  BLECharacteristic c = peripheral->characteristic(characteristic);

  if(c)
  {
    
    c.read();

    int i;
    for(i=0;i<size;i++)
    {
      _val |= (c[i] & 0xFF) << ((size - i - 1) * 8);
    }
    
    *val = _val;

    Serial.print("Read characteristic ");
    Serial.print(characteristic);
    Serial.print(" value = ");
    Serial.print(*val);
    Serial.println("");

    return true;
  }

  return false;
}

