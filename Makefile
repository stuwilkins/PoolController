ARDUINO_DIR   = /Applications/Arduino.app/Contents/Java
ARDUINO_PACKAGE_DIR = $(HOME)/Library/Arduino15/packages
ALTERNATE_CORE_PATH = $(HOME)/Library/Arduino15/packages/adafruit/hardware/samd/1.4.1
CMSIS_DIR = $(ARDUINO_PACKAGE_DIR)/arduino/tools/CMSIS/4.5.0/CMSIS
CMSIS_ATMEL_DIR = $(ARDUINO_PACKAGE_DIR)/arduino/tools/CMSIS-Atmel/1.2.0/CMSIS
ARCHITECTURE = sam
BOARD_TAG     = adafruit_feather_m0
CXXFLAGS_STD = -DARDUINO_ARCH_SAMD
ARDUINO_OTA = $(ARDUINO_PACKAGE_DIR)/arduino/tools/arduinoOTA/1.2.1/

ARDUINO_LIBS += Wire pt100rtd SPI Adafruit_MAX31865 \
				uCRC16Lib RTClib WiFi101 WiFi101OTA \
				SD Adafruit_SleepyDog \
				Adafruit_HTU21DF_Library Adafruit_SI1145_Library \
				PubSubClient NTPClient Adafruit_ASFcore eeprom_i2c \
				Syslog SerialFlash ArduinoJson

OTA_PASSWORD = $(shell cat ota.passwd)
VERSION=$(shell git describe --tags --always --dirty 2> /dev/null)

include /usr/local/opt/arduino-mk/Sam.mk

#CC = arm-none-eabi-g++
#CXX = arm-none-eabi-g++
$(info VAR="$(VERSION)")
$(info OTA_PASSWORD="$(OTA_PASSWORD)")

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
