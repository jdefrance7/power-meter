//===================================================
// LIBRARIES
//===================================================

//---------------------------------------------------
// Core Libraries

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

//---------------------------------------------------
// Custom Libraries

//#include "battery.h"
#include "bluetooth.h"
#include "button.h"
#include "datalogger.h"
#include "display.h"
#include "led.h"
#include "relay.h"
#include "rtc.h"
#include "sensor.h"

//===================================================
// CONFIGS
//===================================================

//---------------------------------------------------
// Update Rate

#define UPDATE_RATE_MS 1000

//---------------------------------------------------
// Serial Baudrate

#define SERIAL_BAUDRATE 115200

//---------------------------------------------------
// Battery Pin

#define BATTERY_PIN A7
#define BATTERY_LOW 3.3

//---------------------------------------------------
// Sensor Settings

#define SENSOR_AVERAGING_COUNT  INA260_COUNT_16
#define VOLTAGE_CONVERSION_TIME INA260_TIME_140_us
#define CURRENT_CONVERSION_TIME INA260_TIME_140_us

//---------------------------------------------------
// Bluefruit Pins

#define BLUEFRUIT_SPI_CS  8
#define BLUEFRUIT_SPI_IRQ 7
#define BLUEFRUIT_SPI_RST 4

//---------------------------------------------------
// Datalogger Pin

#define DATALOGGER_CS 10

//---------------------------------------------------
// Relay Pin

#define RELAY_PIN 12

//---------------------------------------------------
// Button Pins

#define BUTTON_PIN_A  9
#define BUTTON_PIN_B  6
#define BUTTON_PIN_C  5

//===================================================
// OJBECTS
//===================================================

//---------------------------------------------------
// Bluetooth Object

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

//---------------------------------------------------
// Display Object

Adafruit_FeatherOLED oled = Adafruit_FeatherOLED();

//---------------------------------------------------
// RTC Object

RTC_PCF8523 rtc = RTC_PCF8523();

//---------------------------------------------------
// Sensor Object

Adafruit_INA260 sensor = Adafruit_INA260();

//---------------------------------------------------
// File Object

File logfile;

//===================================================
// FUNCTIONS
//===================================================

float getBatteryVoltage(int pin)
{
  float measuredvbat = analogRead(pin);
  measuredvbat *= 2;
  measuredvbat *= 3.3;
  measuredvbat /= 1024;
  return measuredvbat;
}

//===================================================
// SETUP
//===================================================

void setup()
{    
    //-----------------------------------------------
    // Serial
    
    Serial.begin(SERIAL_BAUDRATE);
    Serial.println("Timestamp | Voltage | Current | Power | Battery");

    //-----------------------------------------------
    // Display
    
    oled.init();
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setTextColor(SSD1306_WHITE);
    oled.setBatteryVisible(true);
    oled.setBattery(getBatteryVoltage(BATTERY_PIN));
    oled.renderBattery();
    oled.setCursor(0,0);

    //-----------------------------------------------
    // Bluetooth
    
    oled.print("BLE: ");
    if(ble.begin(true))
    {
      ble.echo(false);
      ble.verbose(false);
      ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
      ble.setMode(BLUEFRUIT_MODE_DATA);
      ble.println("Timestamp | Voltage | Current | Power | Battery");
      oled.println("OK");
      oled.display();
    }
    else
    {
      oled.println("FAIL");
      oled.display();
      while(1);
    }

    //-----------------------------------------------
    // Datalogger
    
    oled.print("SD: ");  
    if(SD.begin(DATALOGGER_CS))
    {
      oled.println("OK");
      oled.display();
    }
    else
    {
      oled.println("FAIL");
      oled.display();
      while(1);
    }    

    //-----------------------------------------------
    // Real Time Clock
    
    oled.print("RTC: ");  
    if(rtc.begin())
    {
      oled.println("OK");
      oled.display();
    }
    else
    {
      oled.println("FAIL");
      oled.display();
      while(1);
    }

    //-----------------------------------------------
    // Power Sensor
    
    oled.print("INA: ");  
    if(sensor.begin())
    {
      sensor.setAveragingCount(SENSOR_AVERAGING_COUNT);
      sensor.setVoltageConversionTime(VOLTAGE_CONVERSION_TIME);    
      sensor.setCurrentConversionTime(CURRENT_CONVERSION_TIME);
      oled.println("OK");  
      oled.display();
    }
    else
    {
      oled.println("FAIL");
      oled.display();
      while(1);
    }

    //-----------------------------------------------
    // Logfile
    
    char filename[24];
    DateTime start = rtc.now();
    
    sprintf(filename, "T-%02u%02u%02u.txt", start.hour(), start.minute(), start.second());
    
    oled.print(filename);
    oled.display();
    
    logfile = SD.open(filename, FILE_WRITE);
    if(logfile)
    {
      logfile.println("Timestamp | Voltage | Current | Power | Battery");
      offLED(BUILTIN_LED);
    }
    else
    {
      onLED(BUILTIN_LED);
      while(1);
    }
    
    //-----------------------------------------------
    // Relay
    
    closeRelay(RELAY_PIN);

    //-----------------------------------------------
}

//===================================================
// LOOP
//===================================================

void loop()
{
    static unsigned long update = millis();

    if((millis() - update) > UPDATE_RATE_MS)
    {
        DateTime time = rtc.now();

        float sensorVoltage = sensor.readBusVoltage();
        float sensorCurrent = sensor.readCurrent();
        float sensorPower = sensor.readPower();
        
        float batteryVoltage = getBatteryVoltage(BATTERY_PIN);

        char line[64];
        sprintf(
            line, 
            "%04u-%02u-%02u %02u:%02u:%02u | %4.2f | %4.2f | %4.2f | %4.2f", 
            time.year(), 
            time.month(), 
            time.day(), 
            time.hour(), 
            time.minute(), 
            time.second(), 
            sensorVoltage, 
            sensorCurrent, 
            sensorPower, 
            batteryVoltage
        );

        if(ble.isConnected())
        {
            ble.println(line);
            ble.waitForOK();
        }

        if(Serial)
        {
            Serial.println(line);
        }

        oled.clearDisplay();
        oled.setCursor(0,0);
        sprintf(line, "T: %02d:%02d:%02d", time.hour(), time.minute(), time.second());
        oled.println(line);
        sprintf(line, "V: %.2f V", sensorVoltage);
        oled.println(line);
        sprintf(line, "I: %.2f mA", sensorCurrent);
        oled.println(line);
        sprintf(line, "P: %.2f mW", sensorPower);
        oled.println(line);
        oled.setBattery(batteryVoltage);
        oled.renderBattery();
        oled.display();

        update = millis();
    }
}
