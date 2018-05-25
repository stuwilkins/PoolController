#include <SPI.h>
#include <Wire.h>
#include <CurieBLE.h>
#include <Adafruit_ILI9341.h>
#include <Adafruit_STMPE610.h>
#include <Adafruit_GFX.h>
#include <fonts/FreeSans12pt7b.h>

#include "ArialRoundedMTBold_14.h"
#include "ArialRoundedMTBold_36.h"
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
  tft.setRotation(2);

  BLE.begin();

  // start scanning for peripherals
  BLE.scanForName("Pool Controller");
}

void loop() {

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

    controlLed(peripheral);

    delay(15000);

    // peripheral disconnected, start scanning again
    BLE.scanForName("Pool Controller");
  }
  
  tft.setCursor(0,20);
  tft.setFont(&FreeSans12pt7b);
  tft.setTextColor(0xFFFF, 0x0000);
  tft.setTextSize(1);
  tft.println("Hello World");

  tft.setFont(&ArialRoundedMTBold_36);
  tft.setCursor(0,100);
  tft.println("Hello Arial World");

  delay(10000);
}


void controlLed(BLEDevice peripheral) {
  int16_t _val = 0;
  float val = 0;
  
  // connect to the peripheral
  Serial.println("Connecting ...");

  if (peripheral.connect()) {
    Serial.println("Connected");
  } else {
    Serial.println("Failed to connect!");
    return;
  }

  // discover peripheral attributes
  Serial.println("Discovering attributes ...");
  if (peripheral.discoverAttributes()) {
    Serial.println("Attributes discovered");
  } else {
    Serial.println("Attribute discovery failed!");
    peripheral.disconnect();
    return;
  }

  // retrieve the LED characteristic
  BLECharacteristic ledCharacteristic = peripheral.characteristic("67ff8149-1199-4700-a494-00f721975a41");

  if (!ledCharacteristic) {
    Serial.println("Peripheral does not have LED characteristic!");
    goto error;
  } else if (!ledCharacteristic.canRead()) {
    Serial.println("Peripheral does not have a readable LED characteristic!");
    peripheral.disconnect();
    return;
  }
  
  ledCharacteristic.read();

  _val |= (ledCharacteristic[0] & 0xF) << 12;
  _val |= (ledCharacteristic[1] & 0xF) << 8;
  _val |= (ledCharacteristic[2] & 0xF) << 4;
  _val |= ledCharacteristic[3];

  val = _val / 100.0;
  
  Serial.print(val);
  Serial.println("");

error:
  peripheral.disconnect();
  Serial.println("Peripheral disconnected");
}

