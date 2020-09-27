#include "battery.h"

float getBatteryVoltage(int pin)
{
  float measuredvbat = analogRead(pin);
  measuredvbat *= 2;
  measuredvbat *= 3.3;
  measuredvbat /= 1024;
  return measuredvbat;
}
