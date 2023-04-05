#ifndef POTENTIOMETER_H
#define POTENTIOMETER_H
#include "mbed.h"
using namespace std;
#define ANALOG_MAX 4096
#define ANALOG_VAL_100 3840
#define ANALOG_VAL_0 10
class Potentiometer
{
  public:
  Potentiometer(byte potValuePin);
  float getValue();
  float getValueProp();
  float getValuePercent();

  private:
  byte _potValuePin;
  const byte _potValueMargin = 20;
  
};

#endif