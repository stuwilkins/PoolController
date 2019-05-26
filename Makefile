#  
#  Copyright (c) 2018 Stuart B. Wilkins
#  
#  Permission is hereby granted, free of charge, to any person obtaining a copy
#  of this software and associated documentation files (the "Software"), to deal
#  in the Software without restriction, including without limitation the rights
#  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#  copies of the Software, and to permit persons to whom the Software is
#  furnished to do so, subject to the following conditions:
#  
#  The above copyright notice and this permission notice shall be included in all
#  copies or substantial portions of the Software.
#  
#  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#  SOFTWARE.
# 


OTA_PASSWORD = $(shell cat ota.passwd)
VERSION=\"$(shell git describe --tags --always --dirty --long 2> /dev/null)\"

ARDUINO_DIR   = /Applications/Arduino.app/Contents/Java
ARDUINO_PACKAGE_DIR = $(HOME)/Library/Arduino15/packages
ALTERNATE_CORE_PATH = $(HOME)/Library/Arduino15/packages/adafruit/hardware/samd/1.4.1
CMSIS_DIR = $(ARDUINO_PACKAGE_DIR)/arduino/tools/CMSIS/4.5.0/CMSIS
CMSIS_ATMEL_DIR = $(ARDUINO_PACKAGE_DIR)/arduino/tools/CMSIS-Atmel/1.2.0/CMSIS
ARCHITECTURE = sam
BOARD_TAG     = adafruit_feather_m0
CXXFLAGS_STD = -DARDUINO_ARCH_SAMD -DVERSION=$(VERSION)
ARDUINO_OTA = $(ARDUINO_PACKAGE_DIR)/arduino/tools/arduinoOTA/1.2.1/

ARDUINO_LIBS += Wire pt100rtd SPI Adafruit_MAX31865 \
				uCRC16Lib RTClib WiFi101 WiFi101OTA \
				SD Adafruit_SleepyDog stulib \
				PubSubClient NTPClient Adafruit_ASFcore eeprom_i2c \
				Syslog SerialFlash ArduinoJson Adafruit-MCP23008-library


include /usr/local/opt/arduino-mk/Sam.mk

#CC = arm-none-eabi-g++
#CXX = arm-none-eabi-g++

.PHONY: test
test: all
	/usr/local/bin/ard-reset-arduino --zero $(DEVICE_PATH)
	sleep 5
	../BOSSA/bin/bossac --erase --write --verify --info --reset \
		                --port=$(DEVICE_PATH) --offset=0x2000 \
						build-$(BOARD_TAG)/PoolController.bin 
	sleep 5
	screen $(DEVICE_PATH) $(MONITOR_BAUDRATE)

upload-ota : all
	$(ARDUINO_OTA)/bin/arduinoOTA -address pool-controller.lan -port 65280 \
	                              -username arduino -password $(OTA_PASSWORD) \
								  -sketch build-adafruit_feather_m0/PoolController.bin \
								  -upload /sketch -b -v
