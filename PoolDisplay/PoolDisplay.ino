#include <Adafruit_GFX.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_ILI9341.h>
#include <Adafruit_STMPE610.h>
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
  
  tft.begin();
  if (!ts.begin()) { 
    Serial.println("Unable to start touchscreen.");
  } 
  else { 
    Serial.println("Touchscreen started."); 
  }

  tft.fillScreen(ILI9341_BLACK);
  tft.setRotation(2);
}

void loop() {
  tft.setCursor(0,20);
  tft.setFont(&FreeSans12pt7b);
  tft.setTextColor(0xFFFF, 0x0000);
  tft.setTextSize(1);
  tft.println("Hello World");

  tft.setFont(&ArialRoundedMTBold_36);
  tft.setCursor(0,100);
  tft.println("Hello Arial World");
  while(1);
}
