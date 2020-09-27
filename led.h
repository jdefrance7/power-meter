#ifndef LED_H
#define LED_H

// Arduino Library
#include <Arduino.h>

// Default LED Pin
#define BUILTIN_LED 13

void onLED(int pin);
void offLED(int pin);

#endif // LED_H
