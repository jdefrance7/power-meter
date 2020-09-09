//===================================================
// ARDUINO
//===================================================

// Core Arduino Library
#include <Arduino.h>

// Core SPI Library
#include <SPI.h>

// Core I2C Library
#include <Wire.h>

//===================================================
// BLUETOOTH
//===================================================

// Bluetooth Core Library
#include <Adafruit_BLE.h>

// Bluetooth SPI Library
#include <Adafruit_BluefruitLE_SPI.h>

// Bluetooth Configs
#define BUFSIZE                     128
#define VERBOSE_MODE                true
#define BLUEFRUIT_SPI_CS            A5
#define BLUEFRUIT_SPI_IRQ           A4
#define BLUEFRUIT_SPI_RST           -1
#define FACTORYRESET_ENABLE         0 // {0:Run 1:Test}
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
#define MODE_LED_BEHAVIOUR          "MODE"

// Bluetooth Object
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// Bluetooth Initialization
int initBluetooth()
{
  // Attempts communication with bluetooth module
  if(!ble.begin(VERBOSE_MODE))
  {
    return -1;
  }

  // Checks for factory reset (should be disabled for production)
  if(FACTORYRESET_ENABLE)
  {
    if(!ble.factoryReset())
    {
      return -1;
    }
  }

  // Turns off echo for send/recieve
  ble.echo(false);

  // Turns off verbose output mode
  ble.verbose(false);

  // Checks bluetooth module version for LED behavior
  if(ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION))
  {
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Sets bluetooth module to data mode
  ble.setMode(BLUEFRUIT_MODE_DATA);

  return 0;
}

//===================================================
// DISPLAY
//===================================================

// Graphics Library
#include <Adafruit_GFX.h>

// SSD1306 Library
#include <Adafruit_SSD1306.h>

// Display Library
#include <Adafruit_FeatherOLED.h>

// Display Object
Adafruit_SSD1306 oled = Adafruit_FeatherOLED();

// OLED Initializer
int initDisplay()
{
  oled.init();
  oled.setBatteryVisible(true);
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.clearDisplay();
  oled.display();

  return 0;
}

//===================================================
// RTC
//===================================================

// RTC Library
#include <RTClib.h>

// RTC Object
RTC_PCF8523 rtc;

// RTC Initializer
int initRTC()
{
  if(!rtc.begin())
  {
    return -1;
  }
  else if(!rtc.initialized())
  {
    return -1;
  }
  else
  {
    return 0;
  }
}

//===================================================
// SENSOR
//===================================================

// INA260 Library
#include <Adafruit_INA260.h>

// Sensor Configs
#define AVERAGING_COUNT           INA260_COUNT_16
#define VOLTAGE_CONVERSION_TIME   INA260_TIME_140_us
#define CURRENT_CONVERSION_TIME   INA260_TIME_140_us

// Sensor Object
Adafruit_INA260 sensor = Adafruit_INA260();

// Sensor Initialization
int initSensor()
{
  if(!sensor.begin())
  {
    return -1;
  }
  else
  {
    sensor.setAveragingCount(AVERAGING_COUNT);
    sensor.setVoltageConversionTime(INA260_TIME_140_us);
    sensor.setCurrentConversionTime(INA260_TIME_140_us);
    return 0;
  }
}

//===================================================
// SERIAL
//===================================================

// Serial Baudrate
#define SERIAL_BAUDRATE 115200

// Serial Timeout
#define SERIAL_TIMEOUT 1000

// Serial Initialization
int initSerial()
{
  // Begins serial module
  Serial.begin(SERIAL_BAUDRATE);

  // No timeout
  if(SERIAL_TIMEOUT == 0)
  {
    return 0;
  }

  // Timeout loop for module initialization
  long counter = millis();
  while(!Serial)
  {
    if(millis() - counter > SERIAL_TIMEOUT)
    {
      return -1;
    }
  }

  return 0;
}

//===================================================
// BATTERY
//===================================================

// Battery Pin
#define BATTERY_PIN A7

// Battery Low Voltage
#define BATTERY_LOW 3.3

// Battery Class
class Battery
{
public:
    Battery()
    {
        _pin = BATTERY_PIN;
    }
    Battery(int pin)
    {
        _pin = pin;
    }
    void init()
    {
        pinMode(_pin, INPUT);
    }
    int pin()
    {
        return _pin;
    }
    float voltage()
    {
        float measuredvbat = analogRead(_pin);
        measuredvbat *= 2;    // we divided by 2, so multiply back
        measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
        measuredvbat /= 1024; // convert to voltage
        return measuredvbat;
    }
private:
    int _pin;
}

// Battery Object
Battery battery = Battery();

// Battery Initialization
int initBattery()
{
    battery.init();
    return 0;
}

//===================================================
// SD
//===================================================

// Datalogger Chip Select
#define SD_CS 4

// Datalogger Initialization
int initSD()
{
    pinMode(SD_CS, OUTPUT);
    if(!SD.begin(SD_CS))
    {
        return -1;
    }
    else
    {
        return 0;
    }
}

//===================================================
// RELAY
//===================================================

// Relay Pin
#define RELAY_PIN 12

// Relay Values
#define RELAY_OPEN    0
#define RELAY_CLOSED  1

// Relay Class
class Relay
{
public:
    Relay()
    {
        _pin = RELAY_PIN;
        _status = false;
    }
    Relay(int pin)
    {
        _pin = pin;
    }
    void init()
    {
        pinMode(_pin, OUTPUT);
        open();
    }
    void open()
    {
        _status = false;
        digitalWrite(_pin, _status);
    }
    void close()
    {
        _status = true;
        digitalWrite(_pin, _status);
    }
    void toggle()
    {
        _status = !status;
        digitalWrite(_pin, _status);
    }
    int pin()
    {
        return _pin;
    }
    bool status()
    {
        return _status;
    }
    bool isOpen()
    {
        return !_status;
    }
    bool isClosed()
    {
        return _status;
    }
private:
    int _pin;
    bool _status;
}

// Relay Object
Relay relay = Relay();

// Relay Initializer
int initRelay()
{
    relay.init();
    return 0;
}

//===================================================
// BUTTONS
//===================================================

// Button Pins
#define BUTTON_PIN_A  9
#define BUTTON_PIN_B  6
#define BUTTON_PIN_C  5

// Button Class
class Button
{
public:
    Button()
    {
        _pin = -1;
    }
    Button(int pin)
    {
        _pin = pin;
    }
    void init()
    {
        pinMode(_pin, INPUT_PULLUP);
    }
    int pin()
    {
        return _pin;
    }
    bool isPressed()
    {
        return !digitalRead(_pin);
    }
private:
    int _pin;
}

// Button Objects
Button bA = Button(BUTTON_PIN_A);
Button bB = Button(BUTTON_PIN_B);
Button bC = Button(BUTTON_PIN_C);

// Buttons Initializer
int initButtons()
{
    bA.init();
    bB.init();
    bC.init();
    return 0;
}

//===================================================
// LED
//===================================================

#define BUILTIN_LED 13

class LED
{
public:
    LED()
    {
        _pin = BUILTIN_LED; 
        _status = false;
    }
    LED(int pin)
    {
        _pin = pin; 
        _status = false;
    }
    void on()
    {
        _status = true;
        digitalWrite(_pin, HIGH);
    }
    void off()
    {
        _status = false;
        digitalWrite(_pin, LOW);
    }
    void toggle()
    {
        _status = !status;
        digitalwrite(_pin, _status);
    }
    int pin()
    {
        return _pin;
    }
    bool status()
    {
        return _status;
    }
    bool isOn()
    {
        return _status;
    }
    bool isOff()
    {
        return !_status;
    }
private:
    int _pin;
    bool _status;
}

int initLED()
{
    pinMode(led.pin(), INPUT);
    led.off();
    return 0;
}

LED led = LED();

//===================================================
// GLOBALS
//===================================================

// Time Reference
unsigned long time_reference;

// Power Readings
float sensor_voltage;
float sensor_current;
float sensor_power;

// Battery Voltage
float battery_voltage;

// Log File
File logfile;
char log_line[64];

// Display Line
char display_line[24];

//===================================================
// SETUP
//===================================================

void setup()
{
    // Main Board
    initLED();
    initBluetooth();
    initBattery();

    // Relay Board
    initRelay();

    // Sensor Board
    initSensor();

    // Datalogger Board
    initRTC();
    initSD();

    // Display Board
    initButtons();
    initDisplay();
}

//===================================================
// MAIN
//===================================================

void main()
{
    // TODO: Update Time Reference
    time_reference = rtc.now().secondstime(); // Seconds since 2000-01-01
    time_reference = (unsigned long)(millis() / 1000); // Seconds since power up

    // TODO: Update Sensor Readings
    sensor_voltage = sensor.readBusVoltage();
    sensor_current = sensor.readCurrent();
    sensor_power = sensor.readPower();

    // TODO: Update Battery Voltage
    battery_voltage = battery.voltage();

    // TODO: Update Logfile
    if(logfile)
    {
        
    }

    // TODO: Update Display
    oled.clearDisplay();
    oled.setCursor(0,0);
    oled.setBattery(battery_voltage);
    oled.renderBattery();

    memset(display_line, 0, 24);
    if(relay.isOpen())
    {
        sprintf(display_line, "Relay: OPEN");
    }
    else
    {
        sprintf(display_line, "Relay: CLOSED");
    }
    oled.println(display_line);

    memset(display_line, 0, 24);
    sprintf(display_line, "Voltage: %fV", voltage);
    oled.println(display_line);

    memset(display_line, 0, 24);
    sprintf(display_line, "Current: %fmA", current);
    oled.println(display_line);

    memset(display_line, 0, 24);
    sprintf(display_line, "Power: %fmW", power);
    oled.println(display_line);

    // TODO: Read Bluetooth Commands
    if(ble.isConnected())
    {
        if(ble.available())
        {

        }
    }

    // TODO: Read Buttons
    if(bA.isPressed())
    {

    }
    if(bB.isPressed())
    {

    }
    if(bC.isPressed())
    {

    }
}