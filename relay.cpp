#include "relay.h"

void openRelay(int pin)
{
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

void closeRelay(int pin)
{
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
}
