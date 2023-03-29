#ifndef POTENTIOMETER_H
#define POTENTIOMETER_H
#include "mbed.h"
using namespace std;
#define ANALOG_MAX 4096
class Potentiometer
{
  public:
  Potentiometer(byte potValuePin);
  float getValuePercent();

  private:
  byte _potValuePin;
  const byte _potValueMargin = 20;
  
};

#endif