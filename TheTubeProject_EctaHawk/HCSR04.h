#ifndef HCSR04_H
#define HCSR04_H

#include "mbed.h"
using namespace std;

#define meas_factor 0.034 / 2
#define meas_timout 10000 //10ms (il faut environ 8ms pour une mesure de 1m (mesuré))

/*You can calculate the range through the time interval between sending trigger signal and
receiving echo signal. Formula: uS / 58 = centimeters or uS / 148 =inch; or: the
range = high level time * velocity (340M/S) / 2; we suggest to use over 60ms
measurement cycle, in order to prevent trigger signal to the echo signal. */

class HCSR04
{
  public:
  HCSR04(byte trigPin, byte echoPin);
  float measureDistance();

  private:  
  byte _trigPin;
  byte _echoPin;
  unsigned long _duration; 
};

#endif