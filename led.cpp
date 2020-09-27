#include "led.h"

void onLED(int pin)
{
  digitalWrite(pin, HIGH);
}

void offLED(int pin)
{
  digitalWrite(pin, LOW);
}
