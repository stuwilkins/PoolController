ARDUINO_DIR   = /Applications/Arduino.app/Contents/Java
ARDUINO_PACKAGE_DIR = $(HOME)/Library/Arduino15/packages
ALTERNATE_CORE_PATH = $(HOME)/Library/Arduino15/packages/adafruit/hardware/samd/1.2.1
CMSIS_DIR = $(ARDUINO_PACKAGE_DIR)/arduino/tools/CMSIS/4.5.0/CMSIS
CMSIS_ATMEL_DIR = $(ARDUINO_PACKAGE_DIR)/arduino/tools/CMSIS-Atmel/1.2.0/CMSIS
ARCHITECTURE = sam
BOARD_TAG     = adafruit_feather_m0
CXXFLAGS_STD = -DARDUINO_ARCH_SAMD
ARDUINO_OTA = $(ARDUINO_PACKAGE_DIR)/arduino/tools/arduinoOTA/1.2.0/

ARDUINO_LIBS += Wire pt100rtd SPI Adafruit_MAX31865_library \
				uCRC16Lib RTClib-master WiFi101 WiFi101OTA \
				SD Adafruit_SleepyDog_Library \
				Adafruit_HTU21DF_Library Adafruit_SI1145_Library \
				PubSubClient NTPClient Adafruit_ASFcore eeprom_i2c \
				RemoteConsole

include /usr/local/opt/arduino-mk/Sam.mk

#CC = arm-none-eabi-g++
CXX = arm-none-eabi-g++
VERSION=$(shell git describe --tags --always --dirty 2> /dev/null)
$(info VAR="$(VERSION)")

upload-ota : all
	$(ARDUINO_OTA)/bin/arduinoOTA -address 192.168.1.20 -port 65280 \
	                              -username arduino -password bHmkHvDM3Ka*^nQz \
								  -sketch build-adafruit_feather_m0/PoolController.bin \
								  -upload /sketch -b -v
