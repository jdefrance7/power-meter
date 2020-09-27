#include "button.h"

bool isPressed(int pin)
{
  pinMode(pin, INPUT_PULLUP);
  return !digitalRead(pin);
}
