#ifndef BATTERY_H
#define BATTERY_H

// Arduino Library
#include <Arduino.h>

// Battery Low Voltage
#define BATTERY_LOW 3.3

// Get Battery Voltage
float getBatteryVoltage(int pin);

#endif // BATTERY_H
